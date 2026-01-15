//! Continuous conflict detection loop.
//!
//! Runs in the background, periodically checking for conflicts
//! and broadcasting updates as geofences to Blender.

use std::sync::Arc;
use std::time::Duration;
use tokio::time::interval;

use crate::config::Config;
use crate::state::AppState;
use atc_blender::{BlenderClient, conflict_to_geofence};

/// Start the conflict detection loop.
pub async fn run_conflict_loop(state: Arc<AppState>, config: Config) {
    let mut ticker = interval(Duration::from_secs(1));

    // Create Blender client for geofence sync
    let blender = BlenderClient::new(
        &config.blender_url,
        &config.blender_session_id,
        "", // Token is generated per-request
    );

    loop {
        ticker.tick().await;

        let conflicts = state.get_conflicts();
        if conflicts.is_empty() {
            continue;
        }

        tracing::warn!(
            "Detected {} conflict(s)",
            conflicts.len()
        );

        // Get current drone positions for geofence generation
        let drones = state.get_all_drones();

        // Transform conflicts to geofences
        let mut geofences = Vec::with_capacity(conflicts.len());
        for conflict in &conflicts {
            // Find drone positions
            let drone1_pos = drones.iter()
                .find(|d| d.drone_id == conflict.drone1_id)
                .map(|d| (d.lat, d.lon, d.altitude_m));
            let drone2_pos = drones.iter()
                .find(|d| d.drone_id == conflict.drone2_id)
                .map(|d| (d.lat, d.lon, d.altitude_m));

            let gf = conflict_to_geofence(conflict, drone1_pos, drone2_pos);
            
            tracing::warn!(
                "  [{:?}] {} <-> {} @ {}m -> Geofence {}",
                conflict.severity,
                conflict.drone1_id,
                conflict.drone2_id,
                conflict.distance_m as i32,
                gf.id
            );
            
            geofences.push(gf);
        }

        // Send to Blender (non-blocking error handling)
        if let Err(e) = blender.send_conflict_geofences(&geofences).await {
            tracing::error!("Failed to sync conflict geofences to Blender: {}", e);
        } else {
            tracing::debug!("Synced {} conflict geofences to Blender", geofences.len());
        }
    }
}

