//! Continuous conflict detection loop.
//!
//! Runs in the background, periodically checking for conflicts
//! and broadcasting updates as geofences to Blender.
//! Also issues HOLD commands when critical conflicts are detected.

use std::sync::Arc;
use std::time::Duration;
use tokio::time::interval;
use chrono::{Duration as ChronoDuration, Utc};

use crate::config::Config;
use crate::state::AppState;
use atc_blender::{BlenderClient, conflict_to_geofence};
use atc_core::{ConflictSeverity, models::{Command, CommandType}};

/// Cooldown in seconds before issuing another command to the same drone.
const COMMAND_COOLDOWN_SECS: u64 = 10;

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

        // Transform conflicts to geofences + issue HOLD commands
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

            // === AUTO-HOLD COMMAND DISPATCH ===
            // Issue HOLD commands for Critical or Warning conflicts
            if matches!(conflict.severity, ConflictSeverity::Critical | ConflictSeverity::Warning) {
                let now = Utc::now();
                
                // Issue HOLD to drone1 if cooldown passed
                if state.can_issue_command(&conflict.drone1_id, COMMAND_COOLDOWN_SECS) {
                    let cmd = Command {
                        command_id: format!("HOLD-{}-{}", &conflict.drone1_id, now.timestamp()),
                        drone_id: conflict.drone1_id.clone(),
                        command_type: CommandType::Hold { duration_secs: 15 },
                        issued_at: now,
                        expires_at: Some(now + ChronoDuration::seconds(30)),
                        acknowledged: false,
                    };
                    state.enqueue_command(cmd);
                    state.mark_command_issued(&conflict.drone1_id);
                    tracing::info!("Auto-issued HOLD to {} (conflict)", conflict.drone1_id);
                }

                // Issue HOLD to drone2 if cooldown passed
                if state.can_issue_command(&conflict.drone2_id, COMMAND_COOLDOWN_SECS) {
                    let cmd = Command {
                        command_id: format!("HOLD-{}-{}", &conflict.drone2_id, now.timestamp()),
                        drone_id: conflict.drone2_id.clone(),
                        command_type: CommandType::Hold { duration_secs: 15 },
                        issued_at: now,
                        expires_at: Some(now + ChronoDuration::seconds(30)),
                        acknowledged: false,
                    };
                    state.enqueue_command(cmd);
                    state.mark_command_issued(&conflict.drone2_id);
                    tracing::info!("Auto-issued HOLD to {} (conflict)", conflict.drone2_id);
                }
            }
        }

        // Send geofences to Blender (non-blocking error handling)
        if let Err(e) = blender.send_conflict_geofences(&geofences).await {
            tracing::error!("Failed to sync conflict geofences to Blender: {}", e);
        } else {
            tracing::debug!("Synced {} conflict geofences to Blender", geofences.len());
        }
    }
}
