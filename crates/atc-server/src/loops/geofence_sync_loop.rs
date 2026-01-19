//! Sync local geofences to Flight Blender.
//!
//! Keeps Blender aware of ATC-defined no-fly zones so compliance and
//! DSS workflows see the same constraints.

use std::collections::{HashMap, HashSet};
use std::hash::{Hash, Hasher};
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::Duration;

use chrono::{DateTime, Duration as ChronoDuration, NaiveDateTime, Utc};
use serde::{Deserialize, Serialize};
use serde_json::json;
use tokio::sync::broadcast;
use tokio::fs;
use tokio::time::interval;

use anyhow::Result;
use atc_blender::BlenderClient;
use atc_core::models::{Geofence, GeofenceType};

use crate::config::Config;
use crate::state::AppState;

const GEOFENCE_SYNC_SECS: u64 = 15;
const GEOFENCE_TTL_HOURS: i64 = 6;
const GEOFENCE_REFRESH_GRACE_SECS: i64 = 600;

#[derive(Debug, Clone, Serialize, Deserialize)]
struct BlenderGeofenceState {
    blender_id: String,
    fingerprint: u64,
    #[serde(default)]
    expires_at: i64,
}

/// Start the geofence sync loop.
pub async fn run_geofence_sync_loop(
    state: Arc<AppState>,
    config: Config,
    mut shutdown: broadcast::Receiver<()>,
) {
    let blender = BlenderClient::new(
        &config.blender_url,
        &config.blender_session_id,
        &config.blender_auth_token,
    );

    let mut ticker = interval(Duration::from_secs(GEOFENCE_SYNC_SECS));
    let state_path = PathBuf::from(config.geofence_sync_state_path.clone());
    let mut tracked = match load_tracking_state(&state_path).await {
        Ok(existing) => existing,
        Err(err) => {
            tracing::warn!("Geofence sync state load failed: {}", err);
            HashMap::new()
        }
    };

    loop {
        tokio::select! {
            _ = shutdown.recv() => {
                tracing::info!("Geofence sync loop shutting down");
                break;
            }
            _ = ticker.tick() => {
                if config.pull_blender_geofences {
                    if let Err(err) = sync_external_geofences(&blender, state.as_ref(), &tracked, &config).await {
                        tracing::warn!("Failed to pull Blender geofences: {}", err);
                    }
                }

                let geofences = state.get_local_geofences();
                let active_geofences: Vec<Geofence> = geofences
                    .into_iter()
                    .filter(|geofence| geofence.active)
                    .collect();
                let active_ids: HashSet<String> = active_geofences
                    .iter()
                    .map(|geofence| geofence.id.clone())
                    .collect();
                let mut dirty = false;

                // Remove deleted/inactive geofences from Blender
                let to_remove: Vec<String> = tracked
                    .keys()
                    .filter(|id| !active_ids.contains(*id))
                    .cloned()
                    .collect();
                for local_id in to_remove {
                    if let Some(existing) = tracked.remove(&local_id) {
                        dirty = true;
                        if let Err(err) = blender.delete_geofence(&existing.blender_id).await {
                            tracing::warn!("Failed to delete Blender geofence {}: {}", local_id, err);
                        }
                    }
                }

                let now = Utc::now();
                let refresh_deadline = now.timestamp() + GEOFENCE_REFRESH_GRACE_SECS;

                for geofence in active_geofences {
                    let fingerprint = fingerprint_geofence(&geofence);
                    let needs_refresh = tracked
                        .get(&geofence.id)
                        .map(|entry| entry.expires_at <= refresh_deadline)
                        .unwrap_or(true);
                    let needs_update = tracked
                        .get(&geofence.id)
                        .map(|entry| entry.fingerprint != fingerprint)
                        .unwrap_or(true);

                    if !needs_update && !needs_refresh {
                        continue;
                    }

                    if let Some(existing) = tracked.remove(&geofence.id) {
                        dirty = true;
                        if let Err(err) = blender.delete_geofence(&existing.blender_id).await {
                            tracing::warn!("Failed to delete Blender geofence {}: {}", geofence.id, err);
                        }
                    }

                    let payload = build_geofence_payload(&geofence, now);
                    match blender.create_geofence(&payload).await {
                        Ok(blender_id) => {
                            let expires_at = (now + ChronoDuration::hours(GEOFENCE_TTL_HOURS)).timestamp();
                            tracked.insert(
                                geofence.id.clone(),
                                BlenderGeofenceState {
                                    blender_id,
                                    fingerprint,
                                    expires_at,
                                },
                            );
                            dirty = true;
                        }
                        Err(err) => {
                            tracing::warn!("Failed to sync geofence {}: {}", geofence.id, err);
                        }
                    }
                }

                if dirty {
                    if let Err(err) = persist_tracking_state(&state_path, &tracked).await {
                        tracing::warn!("Failed to persist geofence sync state: {}", err);
                    }
                }
            }
        }
    }
}

struct ParsedBlenderGeofence {
    blender_id: String,
    geofence: Geofence,
}

async fn sync_external_geofences(
    blender: &BlenderClient,
    state: &AppState,
    tracked: &HashMap<String, BlenderGeofenceState>,
    config: &Config,
) -> Result<()> {
    let view = state.get_rid_view_bbox();
    let view = if view.trim().is_empty() {
        config.rid_view_bbox.clone()
    } else {
        view
    };
    let view_param = if view.trim().is_empty() { None } else { Some(view.as_str()) };

    let raw_geofences = blender.fetch_geofences(view_param).await?;
    let ignored_ids: HashSet<String> = tracked
        .values()
        .map(|entry| entry.blender_id.clone())
        .collect();
    let conflict_ids = state.get_conflict_geofence_ids();
    let now = Utc::now();
    let mut external_geofences = Vec::new();

    for entry in raw_geofences {
        let parsed = match parse_blender_geofence(&entry, now) {
            Some(parsed) => parsed,
            None => continue,
        };
        if ignored_ids.contains(&parsed.blender_id) {
            continue;
        }
        if conflict_ids.contains(&parsed.blender_id) {
            continue;
        }
        if is_conflict_geofence(&parsed.geofence) {
            continue;
        }
        external_geofences.push(parsed.geofence);
    }

    state.set_external_geofences(external_geofences);
    Ok(())
}

fn parse_blender_geofence(entry: &serde_json::Value, now: DateTime<Utc>) -> Option<ParsedBlenderGeofence> {
    let blender_id = entry.get("id")?.as_str()?.to_string();

    let raw_geo = extract_json(entry.get("raw_geo_fence"));
    let geozone = extract_json(entry.get("geozone"));
    let polygon = raw_geo
        .as_ref()
        .and_then(extract_geojson_polygon)
        .or_else(|| extract_geozone_polygon(geozone.as_ref()?))?;

    if polygon.len() < 3 {
        return None;
    }

    let name = entry
        .get("name")
        .and_then(|v| v.as_str())
        .map(|s| s.to_string())
        .or_else(|| {
            raw_geo
                .as_ref()
                .and_then(|geo| geo.get("features"))
                .and_then(|f| f.as_array())
                .and_then(|arr| arr.first())
                .and_then(|f| f.get("properties"))
                .and_then(|p| p.get("name"))
                .and_then(|n| n.as_str())
                .map(|s| s.to_string())
        })
        .unwrap_or_else(|| format!("External Geofence {}", blender_id));

    let upper = extract_f64(entry.get("upper_limit")).unwrap_or(120.0);
    let lower = extract_f64(entry.get("lower_limit")).unwrap_or(0.0);

    let status = entry.get("status").and_then(|v| v.as_i64());
    let start_time = parse_datetime(entry.get("start_datetime"))
        .or_else(|| raw_geo.as_ref().and_then(|geo| extract_geojson_time(geo, "start_time")));
    let end_time = parse_datetime(entry.get("end_datetime"))
        .or_else(|| raw_geo.as_ref().and_then(|geo| extract_geojson_time(geo, "end_time")));

    let mut active = !matches!(status, Some(4 | 5 | 6));
    if let (Some(start), Some(end)) = (start_time, end_time) {
        if now < start || now > end {
            active = false;
        }
    }

    let created_at = parse_datetime(entry.get("created_at")).unwrap_or(now);

    Some(ParsedBlenderGeofence {
        blender_id: blender_id.clone(),
        geofence: Geofence {
            id: format!("blender:{}", blender_id),
            name,
            geofence_type: GeofenceType::TemporaryRestriction,
            polygon,
            lower_altitude_m: lower.min(upper),
            upper_altitude_m: upper.max(lower),
            active,
            created_at,
        },
    })
}

fn extract_json(value: Option<&serde_json::Value>) -> Option<serde_json::Value> {
    let value = value?;
    if value.is_null() {
        return None;
    }
    if let Some(text) = value.as_str() {
        return serde_json::from_str(text).ok();
    }
    Some(value.clone())
}

fn extract_f64(value: Option<&serde_json::Value>) -> Option<f64> {
    let value = value?;
    if let Some(num) = value.as_f64() {
        return Some(num);
    }
    if let Some(num) = value.as_i64() {
        return Some(num as f64);
    }
    if let Some(text) = value.as_str() {
        return text.parse::<f64>().ok();
    }
    None
}

fn parse_datetime(value: Option<&serde_json::Value>) -> Option<DateTime<Utc>> {
    let text = value?.as_str()?;
    if let Ok(dt) = DateTime::parse_from_rfc3339(text) {
        return Some(dt.with_timezone(&Utc));
    }
    if let Ok(dt) = DateTime::parse_from_str(text, "%Y-%m-%dT%H:%M:%S%.f%z") {
        return Some(dt.with_timezone(&Utc));
    }
    if let Ok(dt) = NaiveDateTime::parse_from_str(text, "%Y-%m-%d %H:%M:%S%.f") {
        return Some(DateTime::<Utc>::from_naive_utc_and_offset(dt, Utc));
    }
    None
}

fn extract_geojson_time(geojson: &serde_json::Value, field: &str) -> Option<DateTime<Utc>> {
    let value = geojson
        .get("features")?
        .as_array()?
        .first()?
        .get("properties")?
        .get(field)?;
    parse_datetime(Some(value))
}

fn extract_geojson_polygon(geojson: &serde_json::Value) -> Option<Vec<[f64; 2]>> {
    let geometry = if geojson.get("type").is_some() && geojson.get("coordinates").is_some() {
        geojson
    } else if let Some(geometry) = geojson.get("geometry") {
        geometry
    } else if let Some(feature) = geojson.get("features").and_then(|f| f.as_array()).and_then(|arr| arr.first()) {
        feature.get("geometry")?
    } else {
        return None;
    };

    let geometry_type = geometry.get("type")?.as_str()?;
    let coordinates = geometry.get("coordinates")?;
    let ring = match geometry_type {
        "Polygon" => coordinates.get(0)?,
        "MultiPolygon" => coordinates.get(0)?.get(0)?,
        _ => return None,
    };
    let points = ring.as_array()?;
    let mut polygon = Vec::with_capacity(points.len() + 1);
    for point in points {
        let point = point.as_array()?;
        if point.len() < 2 {
            continue;
        }
        let lon = point[0].as_f64()?;
        let lat = point[1].as_f64()?;
        polygon.push([lat, lon]);
    }
    if polygon.len() < 3 {
        return None;
    }
    if let (Some(first), Some(last)) = (polygon.first().cloned(), polygon.last().cloned()) {
        if first != last {
            polygon.push(first);
        }
    }
    Some(polygon)
}

fn extract_geozone_polygon(geozone: &serde_json::Value) -> Option<Vec<[f64; 2]>> {
    let features = geozone.get("features")?.as_array()?;
    for feature in features {
        let geometries = feature.get("geometry").and_then(|v| v.as_array())?;
        for geometry in geometries {
            if let Some(polygon) = extract_geojson_polygon_from_projection(geometry) {
                return Some(polygon);
            }
        }
    }
    None
}

fn extract_geojson_polygon_from_projection(geometry: &serde_json::Value) -> Option<Vec<[f64; 2]>> {
    if let Some(polygon) = extract_geojson_polygon(geometry) {
        return Some(polygon);
    }
    if let Some(projection) = geometry.get("horizontalProjection") {
        return extract_geojson_polygon(projection);
    }
    None
}

fn is_conflict_geofence(geofence: &Geofence) -> bool {
    let name = geofence.name.to_lowercase();
    name.starts_with("conflict:")
        || name.contains("conflict zone")
        || name.contains("conflict")
        || geofence.id.contains("CONFLICT_")
}

fn build_geofence_payload(geofence: &Geofence, start_time: chrono::DateTime<Utc>) -> serde_json::Value {
    let mut coordinates: Vec<[f64; 2]> = geofence
        .polygon
        .iter()
        .map(|point| [point[1], point[0]])
        .collect();

    if let (Some(first), Some(last)) = (coordinates.first().cloned(), coordinates.last().cloned()) {
        if first != last {
            coordinates.push(first);
        }
    }

    let end_time = (start_time + ChronoDuration::hours(GEOFENCE_TTL_HOURS)).to_rfc3339();
    let start_time = start_time.to_rfc3339();

    json!({
        "type": "FeatureCollection",
        "features": [{
            "type": "Feature",
            "properties": {
                "name": geofence.name,
                "upper_limit": geofence.upper_altitude_m.round() as i32,
                "lower_limit": geofence.lower_altitude_m.round() as i32,
                "start_time": start_time,
                "end_time": end_time
            },
            "geometry": {
                "type": "Polygon",
                "coordinates": [coordinates]
            }
        }]
    })
}

fn fingerprint_geofence(geofence: &Geofence) -> u64 {
    let mut hasher = std::collections::hash_map::DefaultHasher::new();
    geofence.id.hash(&mut hasher);
    geofence.name.hash(&mut hasher);
    format!("{:?}", geofence.geofence_type).hash(&mut hasher);
    for point in &geofence.polygon {
        point[0].to_bits().hash(&mut hasher);
        point[1].to_bits().hash(&mut hasher);
    }
    geofence.lower_altitude_m.to_bits().hash(&mut hasher);
    geofence.upper_altitude_m.to_bits().hash(&mut hasher);
    geofence.active.hash(&mut hasher);
    hasher.finish()
}

async fn load_tracking_state(
    path: &Path,
) -> Result<HashMap<String, BlenderGeofenceState>, Box<dyn std::error::Error + Send + Sync>> {
    if !path.exists() {
        return Ok(HashMap::new());
    }
    let bytes = fs::read(path).await?;
    let map = serde_json::from_slice::<HashMap<String, BlenderGeofenceState>>(&bytes)?;
    Ok(map)
}

async fn persist_tracking_state(
    path: &Path,
    tracked: &HashMap<String, BlenderGeofenceState>,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    if let Some(parent) = path.parent() {
        if !parent.as_os_str().is_empty() {
            fs::create_dir_all(parent).await?;
        }
    }
    let payload = serde_json::to_vec_pretty(tracked)?;
    fs::write(path, payload).await?;
    Ok(())
}
