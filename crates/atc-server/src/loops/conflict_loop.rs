//! Continuous conflict detection loop.
//!
//! Runs in the background, periodically checking for conflicts
//! and broadcasting updates as geofences to Blender.
//! Issues REROUTE commands when critical conflicts are detected.

use std::sync::Arc;
use std::time::Duration;
use tokio::time::interval;
use chrono::{Duration as ChronoDuration, Utc};

use crate::config::Config;
use crate::state::AppState;
use atc_blender::{BlenderClient, conflict_to_geofence};
use atc_core::{
    ConflictSeverity, 
    models::{Command, CommandType, Waypoint},
    generate_avoidance_route, select_avoidance_type,
};

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

        // Check for timed-out drones first
        let lost_drones = state.check_timeouts();
        for drone_id in &lost_drones {
            tracing::warn!("Drone {} marked LOST (no telemetry)", drone_id);
        }

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

        // Transform conflicts to geofences + issue REROUTE commands
        let mut geofences = Vec::with_capacity(conflicts.len());
        for conflict in &conflicts {
            // Find drone positions
            let drone1 = drones.iter().find(|d| d.drone_id == conflict.drone1_id);
            let drone2 = drones.iter().find(|d| d.drone_id == conflict.drone2_id);
            
            let drone1_pos = drone1.map(|d| (d.lat, d.lon, d.altitude_m));
            let drone2_pos = drone2.map(|d| (d.lat, d.lon, d.altitude_m));

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

            // === AUTO-REROUTE COMMAND DISPATCH ===
            // Issue REROUTE to the lower-priority drone (higher ID = newer = gives way)
            if matches!(conflict.severity, ConflictSeverity::Critical | ConflictSeverity::Warning) {
                let now = Utc::now();
                
                // Determine which drone should give way
                let (give_way_id, priority_drone) = if conflict.drone1_id < conflict.drone2_id {
                    (&conflict.drone2_id, drone1)
                } else {
                    (&conflict.drone1_id, drone2)
                };
                
                let give_way_drone = if give_way_id == &conflict.drone1_id { drone1 } else { drone2 };
                
                // Only issue if we can find both drones and cooldown has passed
                if state.can_issue_command(give_way_id, COMMAND_COOLDOWN_SECS) {
                    if let (Some(gw), Some(pri)) = (give_way_drone, priority_drone) {
                        // Create conflict point waypoint (midpoint between drones)
                        let conflict_point = Waypoint {
                            lat: (gw.lat + pri.lat) / 2.0,
                            lon: (gw.lon + pri.lon) / 2.0,
                            altitude_m: gw.altitude_m,
                            speed_mps: Some(gw.speed_mps),
                        };
                        
                        // Create current position waypoint
                        let current_pos = Waypoint {
                            lat: gw.lat,
                            lon: gw.lon,
                            altitude_m: gw.altitude_m,
                            speed_mps: Some(gw.speed_mps),
                        };
                        
                        // Destination: continue in current heading direction
                        // For now, use a point ~500m ahead
                        let heading_rad = gw.heading_deg.to_radians();
                        let offset_deg = 0.005; // ~500m
                        let destination = Waypoint {
                            lat: gw.lat + heading_rad.cos() * offset_deg,
                            lon: gw.lon + heading_rad.sin() * offset_deg,
                            altitude_m: gw.altitude_m,
                            speed_mps: Some(gw.speed_mps),
                        };
                        
                        // Select avoidance type based on altitude context
                        let avoidance_type = select_avoidance_type(
                            gw.altitude_m,
                            pri.altitude_m,
                            gw.altitude_m > 100.0, // Basic ceiling check
                        );
                        
                        // Generate avoidance route
                        let avoidance_waypoints = generate_avoidance_route(
                            &current_pos,
                            &destination,
                            &conflict_point,
                            avoidance_type,
                        );
                        
                        let cmd = Command {
                            command_id: format!("REROUTE-{}-{}", give_way_id, now.timestamp()),
                            drone_id: give_way_id.clone(),
                            command_type: CommandType::Reroute { 
                                waypoints: avoidance_waypoints,
                                reason: Some(format!(
                                    "Conflict avoidance ({:?}) with {}",
                                    avoidance_type,
                                    if give_way_id == &conflict.drone1_id { 
                                        &conflict.drone2_id 
                                    } else { 
                                        &conflict.drone1_id 
                                    }
                                )),
                            },
                            issued_at: now,
                            expires_at: Some(now + ChronoDuration::seconds(60)),
                            acknowledged: false,
                        };
                        state.enqueue_command(cmd);
                        state.mark_command_issued(give_way_id);
                        tracing::info!(
                            "Auto-issued REROUTE ({:?}) to {} (gives way to {})", 
                            avoidance_type,
                            give_way_id,
                            if give_way_id == &conflict.drone1_id { &conflict.drone2_id } else { &conflict.drone1_id }
                        );
                    } else {
                        // Fallback: issue HOLD if we can't compute reroute
                        let cmd = Command {
                            command_id: format!("HOLD-{}-{}", give_way_id, now.timestamp()),
                            drone_id: give_way_id.clone(),
                            command_type: CommandType::Hold { duration_secs: 15 },
                            issued_at: now,
                            expires_at: Some(now + ChronoDuration::seconds(30)),
                            acknowledged: false,
                        };
                        state.enqueue_command(cmd);
                        state.mark_command_issued(give_way_id);
                        tracing::info!("Fallback: issued HOLD to {}", give_way_id);
                    }
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

