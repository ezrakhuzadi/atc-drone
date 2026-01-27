//! Terrain sampling utilities for server-side routing.

use crate::cache;
use crate::compliance::RoutePoint;
use crate::config::Config;
use atc_core::spatial::{meters_per_deg_lat, meters_per_deg_lon};
use dashmap::DashMap;
use reqwest::Client;
use serde::{Deserialize, Serialize};
use std::sync::{Arc, OnceLock};
use std::time::{Duration, Instant};
use tokio::sync::Mutex;
use tokio::time::sleep;

#[derive(Debug, Clone)]
pub struct TerrainGrid {
    min_lat: f64,
    min_lon: f64,
    max_lat: f64,
    max_lon: f64,
    lat_step_deg: f64,
    lon_step_deg: f64,
    rows: usize,
    cols: usize,
    elevations_m: Vec<f64>,
}

#[derive(Debug, Clone)]
struct TerrainCacheEntry {
    fetched_at: Instant,
    grid: TerrainGrid,
}

impl cache::CacheEntry for TerrainCacheEntry {
    fn fetched_at(&self) -> Instant {
        self.fetched_at
    }
}

fn terrain_cache() -> &'static DashMap<String, TerrainCacheEntry> {
    static CACHE: OnceLock<DashMap<String, TerrainCacheEntry>> = OnceLock::new();
    CACHE.get_or_init(DashMap::new)
}

fn terrain_inflight() -> &'static DashMap<String, Arc<Mutex<()>>> {
    static INFLIGHT: OnceLock<DashMap<String, Arc<Mutex<()>>>> = OnceLock::new();
    INFLIGHT.get_or_init(DashMap::new)
}

const TERRAIN_CACHE_MAX_ENTRIES: usize = 256;

impl TerrainGrid {
    pub fn sample(&self, lat: f64, lon: f64) -> f64 {
        if !lat.is_finite() || !lon.is_finite() {
            return 0.0;
        }
        let clamped_lat = lat.clamp(self.min_lat, self.max_lat);
        let clamped_lon = lon.clamp(self.min_lon, self.max_lon);

        let lat_step = self.lat_step_deg.max(1e-9);
        let lon_step = self.lon_step_deg.max(1e-9);

        let mut y = (clamped_lat - self.min_lat) / lat_step;
        let mut x = (clamped_lon - self.min_lon) / lon_step;

        if !y.is_finite() || !x.is_finite() {
            return 0.0;
        }

        let max_y = (self.rows - 1) as f64;
        let max_x = (self.cols - 1) as f64;
        if y < 0.0 {
            y = 0.0;
        } else if y > max_y {
            y = max_y;
        }
        if x < 0.0 {
            x = 0.0;
        } else if x > max_x {
            x = max_x;
        }

        let y0 = y.floor() as usize;
        let x0 = x.floor() as usize;
        let y1 = (y0 + 1).min(self.rows - 1);
        let x1 = (x0 + 1).min(self.cols - 1);
        let v00 = self.value_at(y0, x0);
        let v10 = self.value_at(y0, x1);
        let v01 = self.value_at(y1, x0);
        let v11 = self.value_at(y1, x1);

        // Conservative sampling avoids undercutting peaks between grid points.
        v00.max(v10).max(v01).max(v11)
    }

    fn value_at(&self, row: usize, col: usize) -> f64 {
        let idx = row.saturating_mul(self.cols) + col.min(self.cols - 1);
        self.elevations_m.get(idx).copied().unwrap_or(0.0)
    }
}

#[derive(Debug)]
struct TerrainBounds {
    min_lat: f64,
    max_lat: f64,
    min_lon: f64,
    max_lon: f64,
}

#[derive(Debug, Deserialize)]
struct OpenMeteoElevationResponse {
    elevation: Option<Vec<Option<f64>>>,
}

#[derive(Debug, Serialize)]
struct TerrainBatchRequest<'a> {
    latitude: &'a [f64],
    longitude: &'a [f64],
}

pub async fn fetch_terrain_grid(
    client: &Client,
    config: &Config,
    points: &[RoutePoint],
    grid_spacing_m: f64,
) -> Result<Option<TerrainGrid>, String> {
    let started_at = Instant::now();
    if config.terrain_provider_url.trim().is_empty() {
        return Err("terrain provider URL is empty".to_string());
    }

    let core_bounds = match bounds_from_points(points) {
        Some(bounds) => bounds,
        None => return Ok(None),
    };
    let bounds = expand_bounds(&core_bounds, 0.2);
    let cache_ttl = Duration::from_secs(config.terrain_cache_ttl_s.max(30));
    let cache = terrain_cache();
    let mut stale_cache: Option<TerrainGrid> = None;

    if !config.terrain_provider_url.is_empty() && config.terrain_max_grid_points == 0 {
        return Err("terrain_max_grid_points must be > 0".to_string());
    }

    let spacing_target_m = config.terrain_sample_spacing_m.max(5.0);
    let mean_lat = (bounds.min_lat + bounds.max_lat) / 2.0;
    let meters_per_deg_lat = meters_per_deg_lat(mean_lat);
    let meters_per_deg_lon = meters_per_deg_lon(mean_lat).max(1.0);

    let (rows, cols, lat_step_deg, lon_step_deg, spacing_m) = resolve_grid_dims(
        &bounds,
        spacing_target_m,
        meters_per_deg_lat,
        meters_per_deg_lon,
        config.terrain_max_grid_points,
    );

    let cache_key = terrain_cache_key(&bounds, spacing_m);
    if let Some(entry) = cache.get(&cache_key) {
        let age = entry.fetched_at.elapsed();
        if age <= cache_ttl {
            return Ok(Some(entry.grid.clone()));
        }
        if age <= cache_ttl.saturating_mul(2) {
            stale_cache = Some(entry.grid.clone());
        }
    }

    // Coalesce concurrent fetches for the same grid so many route-plans don’t stampede terrain-api.
    let inflight_lock = terrain_inflight()
        .entry(cache_key.clone())
        .or_insert_with(|| Arc::new(Mutex::new(())))
        .clone();
    let _guard = inflight_lock.lock().await;

    // Another request may have populated the cache while we waited.
    if let Some(entry) = cache.get(&cache_key) {
        let age = entry.fetched_at.elapsed();
        if age <= cache_ttl {
            return Ok(Some(entry.grid.clone()));
        }
        if stale_cache.is_none() && age <= cache_ttl.saturating_mul(2) {
            stale_cache = Some(entry.grid.clone());
        }
    }

    let total = rows.saturating_mul(cols);
    if total == 0 {
        return Ok(None);
    }

    let mut latitudes = Vec::with_capacity(total);
    let mut longitudes = Vec::with_capacity(total);
    for row in 0..rows {
        let lat = bounds.min_lat + row as f64 * lat_step_deg;
        for col in 0..cols {
            let lon = bounds.min_lon + col as f64 * lon_step_deg;
            latitudes.push(lat);
            longitudes.push(lon);
        }
    }

    let max_points = config.terrain_max_points_per_request.max(1);
    let timeout = Duration::from_secs(config.terrain_request_timeout_s.max(3));
    let min_interval = Duration::from_millis(config.terrain_request_min_interval_ms);
    let max_attempts = config.terrain_request_retries.saturating_add(1);
    let backoff_base_ms = config.terrain_request_backoff_ms.max(1);
    let total_requests = total.div_ceil(max_points);
    if config.terrain_max_requests > 0 && total_requests > config.terrain_max_requests {
        return Err(format!(
            "terrain request count {} exceeds terrain_max_requests {}",
            total_requests, config.terrain_max_requests
        ));
    }

    tracing::info!(
        provider = %config.terrain_provider_url,
        grid_spacing_m = grid_spacing_m,
        spacing_m = spacing_m,
        rows = rows,
        cols = cols,
        points = total,
        max_points_per_request = max_points,
        requests = total_requests,
        min_interval_ms = config.terrain_request_min_interval_ms,
        "Terrain fetch start"
    );

    let mut elevations = vec![0.0; total];

    let mut start = 0usize;
    let mut last_request_at: Option<Instant> = None;
    while start < total {
        let request_idx = start / max_points;
        if total_requests >= 25 && request_idx > 0 && request_idx.is_multiple_of(25) {
            tracing::info!(
                request = request_idx,
                total_requests = total_requests,
                elapsed_ms = started_at.elapsed().as_millis() as u64,
                "Terrain fetch progress"
            );
        }
        let end = (start + max_points).min(total);
        let lat_slice = &latitudes[start..end];
        let lon_slice = &longitudes[start..end];

        if let Some(last_request_at) = last_request_at {
            let elapsed = last_request_at.elapsed();
            if elapsed < min_interval {
                sleep(min_interval - elapsed).await;
            }
        }

        let (url, request_body) = if config.terrain_use_post {
            (
                config.terrain_provider_url.clone(),
                Some(TerrainBatchRequest {
                    latitude: lat_slice,
                    longitude: lon_slice,
                }),
            )
        } else {
            let lat_param = join_params(lat_slice);
            let lon_param = join_params(lon_slice);
            (
                build_provider_url(&config.terrain_provider_url, &lat_param, &lon_param),
                None,
            )
        };
        let mut payload: Option<OpenMeteoElevationResponse> = None;
        let mut last_err: Option<String> = None;

        for attempt in 0..max_attempts {
            let response = if let Some(body) = request_body.as_ref() {
                client.post(&url).timeout(timeout).json(body).send().await
            } else {
                client.get(&url).timeout(timeout).send().await
            };

            match response {
                Ok(response) => {
                    if !response.status().is_success() {
                        last_err = Some(format!("terrain provider HTTP {}", response.status()));
                    } else {
                        match response.json().await {
                            Ok(parsed) => {
                                payload = Some(parsed);
                                break;
                            }
                            Err(err) => {
                                last_err = Some(err.to_string());
                            }
                        }
                    }
                }
                Err(err) => {
                    last_err = Some(err.to_string());
                }
            }

            if attempt + 1 < max_attempts {
                let delay_ms = backoff_base_ms.saturating_mul(attempt.saturating_add(1) as u64);
                sleep(Duration::from_millis(delay_ms)).await;
            }
        }

        let payload = match payload {
            Some(payload) => payload,
            None => {
                let err = last_err.unwrap_or_else(|| "terrain request failed".to_string());
                if let Some(stale) = stale_cache {
                    tracing::warn!("Terrain fetch failed, using stale cache: {}", err);
                    return Ok(Some(stale));
                }
                return Err(err);
            }
        };

        let chunk = payload
            .elevation
            .ok_or_else(|| "terrain provider missing elevation".to_string())?;

        if chunk.len() != lat_slice.len() {
            return Err("terrain provider returned unexpected sample count".to_string());
        }

        let mut missing_required = 0usize;
        for (idx, maybe_value) in chunk.into_iter().enumerate() {
            let value = maybe_value.filter(|value| value.is_finite());
            if let Some(value) = value {
                elevations[start + idx] = value;
                continue;
            }

            let lat = lat_slice[idx];
            let lon = lon_slice[idx];
            let in_core = lat >= core_bounds.min_lat
                && lat <= core_bounds.max_lat
                && lon >= core_bounds.min_lon
                && lon <= core_bounds.max_lon;
            if in_core {
                missing_required += 1;
            }
        }

        if missing_required > 0 {
            tracing::warn!(
                missing_required = missing_required,
                requested = lat_slice.len(),
                "Terrain provider returned missing samples"
            );
        }

        if missing_required > 0 && config.terrain_require {
            if let Some(stale) = stale_cache {
                tracing::warn!(
                    missing_required = missing_required,
                    "Terrain provider returned missing required samples; using stale cache"
                );
                return Ok(Some(stale));
            }
            return Err(format!(
                "terrain provider returned {} missing required samples",
                missing_required
            ));
        }

        last_request_at = Some(Instant::now());
        start = end;
    }

    let grid = TerrainGrid {
        min_lat: bounds.min_lat,
        min_lon: bounds.min_lon,
        max_lat: bounds.max_lat,
        max_lon: bounds.max_lon,
        lat_step_deg,
        lon_step_deg,
        rows,
        cols,
        elevations_m: elevations,
    };

    cache.insert(
        cache_key,
        TerrainCacheEntry {
            fetched_at: Instant::now(),
            grid: grid.clone(),
        },
    );
    cache::prune_cache(
        cache,
        TERRAIN_CACHE_MAX_ENTRIES,
        cache_ttl.saturating_mul(2),
    );

    tracing::info!(
        rows = rows,
        cols = cols,
        points = total,
        requests = total_requests,
        elapsed_ms = started_at.elapsed().as_millis() as u64,
        "Terrain fetch complete"
    );

    Ok(Some(grid))
}

fn bounds_from_points(points: &[RoutePoint]) -> Option<TerrainBounds> {
    let mut min_lat = f64::INFINITY;
    let mut max_lat = f64::NEG_INFINITY;
    let mut min_lon = f64::INFINITY;
    let mut max_lon = f64::NEG_INFINITY;
    for point in points {
        if !point.lat.is_finite() || !point.lon.is_finite() {
            continue;
        }
        min_lat = min_lat.min(point.lat);
        max_lat = max_lat.max(point.lat);
        min_lon = min_lon.min(point.lon);
        max_lon = max_lon.max(point.lon);
    }
    if !min_lat.is_finite() || !min_lon.is_finite() {
        return None;
    }
    Some(TerrainBounds {
        min_lat,
        max_lat,
        min_lon,
        max_lon,
    })
}

fn expand_bounds(bounds: &TerrainBounds, pad_ratio: f64) -> TerrainBounds {
    let lat_span = bounds.max_lat - bounds.min_lat;
    let lon_span = bounds.max_lon - bounds.min_lon;
    let pad_lat = (lat_span * pad_ratio).max(0.0015);
    let pad_lon = (lon_span * pad_ratio).max(0.0015);
    TerrainBounds {
        min_lat: bounds.min_lat - pad_lat,
        max_lat: bounds.max_lat + pad_lat,
        min_lon: bounds.min_lon - pad_lon,
        max_lon: bounds.max_lon + pad_lon,
    }
}

fn resolve_grid_dims(
    bounds: &TerrainBounds,
    spacing_m: f64,
    meters_per_deg_lat: f64,
    meters_per_deg_lon: f64,
    max_points: usize,
) -> (usize, usize, f64, f64, f64) {
    let mut spacing = spacing_m.max(5.0);
    let max_points = max_points.max(1000);

    loop {
        let spacing_used = spacing;
        let lat_step_deg = spacing / meters_per_deg_lat;
        let lon_step_deg = spacing / meters_per_deg_lon;
        let rows = ((bounds.max_lat - bounds.min_lat) / lat_step_deg)
            .ceil()
            .max(1.0) as usize
            + 1;
        let cols = ((bounds.max_lon - bounds.min_lon) / lon_step_deg)
            .ceil()
            .max(1.0) as usize
            + 1;
        let total = rows.saturating_mul(cols);
        if total <= max_points {
            return (rows, cols, lat_step_deg, lon_step_deg, spacing_used);
        }

        let scale = ((total as f64) / (max_points as f64)).sqrt().max(1.1);
        spacing *= scale;
        if spacing > 2000.0 {
            return (rows, cols, lat_step_deg, lon_step_deg, spacing_used);
        }
    }
}

fn join_params(values: &[f64]) -> String {
    let mut buf = String::new();
    for (idx, value) in values.iter().enumerate() {
        if idx > 0 {
            buf.push(',');
        }
        buf.push_str(&format!("{:.6}", value));
    }
    buf
}

fn build_provider_url(base: &str, latitudes: &str, longitudes: &str) -> String {
    let separator = if base.contains('?') { "&" } else { "?" };
    format!(
        "{}{}latitude={}&longitude={}",
        base, separator, latitudes, longitudes
    )
}

fn terrain_cache_key(bounds: &TerrainBounds, spacing_m: f64) -> String {
    format!(
        "terrain:{:.4}:{:.4}:{:.4}:{:.4}:{:.1}",
        bounds.min_lat, bounds.min_lon, bounds.max_lat, bounds.max_lon, spacing_m
    )
}
