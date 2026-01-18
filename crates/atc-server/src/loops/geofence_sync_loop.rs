//! Sync local geofences to Flight Blender.
//!
//! Keeps Blender aware of ATC-defined no-fly zones so compliance and
//! DSS workflows see the same constraints.

use std::collections::{HashMap, HashSet};
use std::hash::{Hash, Hasher};
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::Duration;

use chrono::{Duration as ChronoDuration, Utc};
use serde::{Deserialize, Serialize};
use serde_json::json;
use tokio::fs;
use tokio::time::interval;

use atc_blender::BlenderClient;
use atc_core::models::Geofence;

use crate::config::Config;
use crate::state::AppState;

const GEOFENCE_SYNC_SECS: u64 = 15;

#[derive(Debug, Clone, Serialize, Deserialize)]
struct BlenderGeofenceState {
    blender_id: String,
    fingerprint: u64,
}

/// Start the geofence sync loop.
pub async fn run_geofence_sync_loop(state: Arc<AppState>, config: Config) {
    let blender = BlenderClient::new(
        &config.blender_url,
        &config.blender_session_id,
        &config.blender_auth_token,
    );

    let mut ticker = interval(Duration::from_secs(GEOFENCE_SYNC_SECS));
    let state_path = PathBuf::from(config.geofence_sync_state_path);
    let mut tracked = match load_tracking_state(&state_path).await {
        Ok(existing) => existing,
        Err(err) => {
            tracing::warn!("Geofence sync state load failed: {}", err);
            HashMap::new()
        }
    };

    loop {
        ticker.tick().await;

        let geofences = state.get_geofences();
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

        for geofence in active_geofences {
            let fingerprint = fingerprint_geofence(&geofence);
            let needs_update = tracked
                .get(&geofence.id)
                .map(|entry| entry.fingerprint != fingerprint)
                .unwrap_or(true);

            if !needs_update {
                continue;
            }

            if let Some(existing) = tracked.remove(&geofence.id) {
                dirty = true;
                if let Err(err) = blender.delete_geofence(&existing.blender_id).await {
                    tracing::warn!("Failed to delete Blender geofence {}: {}", geofence.id, err);
                }
            }

            let payload = build_geofence_payload(&geofence);
            match blender.create_geofence(&payload).await {
                Ok(blender_id) => {
                    tracked.insert(
                        geofence.id.clone(),
                        BlenderGeofenceState {
                            blender_id,
                            fingerprint,
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

fn build_geofence_payload(geofence: &Geofence) -> serde_json::Value {
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

    let start_time = Utc::now().to_rfc3339();
    let end_time = (Utc::now() + ChronoDuration::hours(1)).to_rfc3339();

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
