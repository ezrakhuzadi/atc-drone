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
/// Set to 30s to cover typical reroute duration (~25s).
const COMMAND_COOLDOWN_SECS: u64 = 30;

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
                        // === HOLD-AWARE LOGIC ===
                        // If the priority drone is holding/rerouting, check if it's actually blocking
                        let priority_id = if give_way_id == &conflict.drone1_id { 
                            &conflict.drone2_id 
                        } else { 
                            &conflict.drone1_id 
                        };
                        
                        if state.has_active_command(priority_id) {
                            // Priority drone is on HOLD/REROUTE - check if it's in our path
                            // Use heading to predict: will give_way drone fly past the held drone?
                            use atc_core::spatial::offset_by_bearing;
                            let heading_rad = gw.heading_deg.to_radians();
                            
                            // Project give_way drone's position 30 seconds ahead (~300m at 10m/s)
                            let (future_lat, future_lon) = offset_by_bearing(
                                gw.lat, gw.lon, 
                                gw.speed_mps * 30.0,  // distance = speed * time
                                heading_rad
                            );
                            
                            // Check if priority drone is between current and future position
                            let lat_between = (pri.lat > gw.lat.min(future_lat)) && (pri.lat < gw.lat.max(future_lat));
                            let lon_between = (pri.lon > gw.lon.min(future_lon)) && (pri.lon < gw.lon.max(future_lon));
                            
                            if !lat_between && !lon_between {
                                // Priority drone is NOT in our path - skip reroute
                                tracing::info!(
                                    "Skipping reroute for {} - {} is holding but not blocking path",
                                    give_way_id, priority_id
                                );
                                continue;
                            }
                        }
                        // Create conflict point waypoint using predicted CPA (not current midpoint)
                        let conflict_point = Waypoint {
                            lat: conflict.cpa_lat,
                            lon: conflict.cpa_lon,
                            altitude_m: conflict.cpa_altitude_m,
                            speed_mps: Some(gw.speed_mps),
                        };
                        
                        // Create current position waypoint
                        let current_pos = Waypoint {
                            lat: gw.lat,
                            lon: gw.lon,
                            altitude_m: gw.altitude_m,
                            speed_mps: Some(gw.speed_mps),
                        };
                        
                        // Destination: continue in current heading direction (~500m ahead)
                        // Using proper ENU conversion instead of degree offsets
                        use atc_core::spatial::offset_by_bearing;
                        let heading_rad = gw.heading_deg.to_radians();
                        let (dest_lat, dest_lon) = offset_by_bearing(
                            gw.lat, gw.lon, 
                            500.0,  // 500 meters ahead
                            heading_rad
                        );
                        let destination = Waypoint {
                            lat: dest_lat,
                            lon: dest_lon,
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

