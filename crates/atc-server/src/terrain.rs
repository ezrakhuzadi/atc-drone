//! Terrain sampling utilities for server-side routing.

use crate::config::Config;
use crate::compliance::RoutePoint;
use dashmap::DashMap;
use reqwest::Client;
use serde::Deserialize;
use std::sync::OnceLock;
use std::time::{Duration, Instant};

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

fn terrain_cache() -> &'static DashMap<String, TerrainCacheEntry> {
    static CACHE: OnceLock<DashMap<String, TerrainCacheEntry>> = OnceLock::new();
    CACHE.get_or_init(DashMap::new)
}

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
        let dy = y - y0 as f64;
        let dx = x - x0 as f64;

        let v00 = self.value_at(y0, x0);
        let v10 = self.value_at(y0, x1);
        let v01 = self.value_at(y1, x0);
        let v11 = self.value_at(y1, x1);

        let v0 = v00 + (v10 - v00) * dx;
        let v1 = v01 + (v11 - v01) * dx;
        v0 + (v1 - v0) * dy
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
    elevation: Option<Vec<f64>>,
}

pub async fn fetch_terrain_grid(
    client: &Client,
    config: &Config,
    points: &[RoutePoint],
    grid_spacing_m: f64,
) -> Result<Option<TerrainGrid>, String> {
    if config.terrain_provider_url.trim().is_empty() {
        return Err("terrain provider URL is empty".to_string());
    }

    let bounds = match bounds_from_points(points) {
        Some(bounds) => expand_bounds(&bounds, 0.2),
        None => return Ok(None),
    };
    let cache_key = terrain_cache_key(&bounds, grid_spacing_m);
    let cache_ttl = Duration::from_secs(config.terrain_cache_ttl_s.max(30));
    let cache = terrain_cache();
    let mut stale_cache: Option<TerrainGrid> = None;
    if let Some(entry) = cache.get(&cache_key) {
        let age = entry.fetched_at.elapsed();
        if age <= cache_ttl {
            return Ok(Some(entry.grid.clone()));
        }
        if age <= cache_ttl.saturating_mul(2) {
            stale_cache = Some(entry.grid.clone());
        }
    }

    if !config.terrain_provider_url.is_empty() && config.terrain_max_grid_points == 0 {
        return Err("terrain_max_grid_points must be > 0".to_string());
    }

    let mut spacing_m = config.terrain_sample_spacing_m.max(5.0);
    let max_grid_spacing = grid_spacing_m.max(2.0) * 2.0;
    if spacing_m > max_grid_spacing {
        spacing_m = max_grid_spacing;
    }

    let mean_lat = ((bounds.min_lat + bounds.max_lat) / 2.0).to_radians();
    let meters_per_deg_lat = 111_320.0;
    let meters_per_deg_lon = 111_320.0 * mean_lat.cos().max(0.01);

    let (rows, cols, lat_step_deg, lon_step_deg) =
        resolve_grid_dims(&bounds, spacing_m, meters_per_deg_lat, meters_per_deg_lon, config.terrain_max_grid_points);

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
    let mut elevations = vec![0.0; total];

    let mut start = 0usize;
    while start < total {
        let end = (start + max_points).min(total);
        let lat_slice = &latitudes[start..end];
        let lon_slice = &longitudes[start..end];
        let lat_param = join_params(lat_slice);
        let lon_param = join_params(lon_slice);

        let url = build_provider_url(&config.terrain_provider_url, &lat_param, &lon_param);
    let response = client
        .get(url)
        .timeout(timeout)
        .send()
        .await
        .map_err(|err| err.to_string());

    let response = match response {
        Ok(response) => response,
        Err(err) => {
            if let Some(stale) = stale_cache {
                tracing::warn!("Terrain fetch failed, using stale cache: {}", err);
                return Ok(Some(stale));
            }
            return Err(err);
        }
    };

    if !response.status().is_success() {
        if let Some(stale) = stale_cache {
            tracing::warn!(
                "Terrain provider HTTP {}, using stale cache",
                response.status()
            );
            return Ok(Some(stale));
        }
        return Err(format!("terrain provider HTTP {}", response.status()));
    }

        let payload: OpenMeteoElevationResponse = match response.json().await {
            Ok(payload) => payload,
            Err(err) => {
                if let Some(stale) = stale_cache {
                    tracing::warn!("Terrain parse failed, using stale cache: {}", err);
                    return Ok(Some(stale));
                }
                return Err(err.to_string());
            }
        };
        let chunk = payload
            .elevation
            .ok_or_else(|| "terrain provider missing elevation".to_string())?;

        if chunk.len() != lat_slice.len() {
            return Err("terrain provider returned unexpected sample count".to_string());
        }

        for (idx, value) in chunk.into_iter().enumerate() {
            elevations[start + idx] = if value.is_finite() { value } else { 0.0 };
        }

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
) -> (usize, usize, f64, f64) {
    let mut spacing = spacing_m.max(5.0);
    let max_points = max_points.max(1000);

    loop {
        let lat_step_deg = spacing / meters_per_deg_lat;
        let lon_step_deg = spacing / meters_per_deg_lon;
        let rows = ((bounds.max_lat - bounds.min_lat) / lat_step_deg).ceil().max(1.0) as usize + 1;
        let cols = ((bounds.max_lon - bounds.min_lon) / lon_step_deg).ceil().max(1.0) as usize + 1;
        let total = rows.saturating_mul(cols);
        if total <= max_points {
            return (rows, cols, lat_step_deg, lon_step_deg);
        }

        let scale = ((total as f64) / (max_points as f64)).sqrt().max(1.1);
        spacing *= scale;
        if spacing > 2000.0 {
            return (rows, cols, lat_step_deg, lon_step_deg);
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
        bounds.min_lat,
        bounds.min_lon,
        bounds.max_lat,
        bounds.max_lon,
        spacing_m
    )
}
