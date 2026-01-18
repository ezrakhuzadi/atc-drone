//! Continuous conflict detection loop.
//!
//! Runs in the background, periodically checking for conflicts
//! and broadcasting updates as geofences to Blender.
//! Issues REROUTE commands when critical conflicts are detected.

use std::collections::{HashMap, HashSet};
use std::sync::Arc;
use std::time::Duration;
use tokio::time::interval;
use chrono::{Duration as ChronoDuration, Utc};

use crate::config::Config;
use crate::state::AppState;
use atc_blender::{BlenderClient, conflict_to_geofence};
use atc_core::{
    ConflictSeverity, 
    models::{Command, CommandType, DaaAdvisory, DaaSeverity, Waypoint},
    generate_avoidance_route, select_avoidance_type,
};

/// Cooldown in seconds before issuing another command to the same drone.
/// Set to 60s to cover full reroute execution (physics-based ~30-45s) plus buffer.
const COMMAND_COOLDOWN_SECS: u64 = 60;

/// Start the conflict detection loop.
pub async fn run_conflict_loop(state: Arc<AppState>, config: Config) {
    let mut ticker = interval(Duration::from_secs(1));

    // Create Blender client for geofence sync
    let blender = BlenderClient::new(
        &config.blender_url,
        &config.blender_session_id,
        &config.blender_auth_token,
    );

    loop {
        ticker.tick().await;

        // Check for timed-out drones first
        let lost_drones = state.check_timeouts();
        for drone_id in &lost_drones {
            tracing::warn!("Drone {} marked LOST (no telemetry)", drone_id);
        }
        
        // Purge expired commands to prevent them from hiding drones
        state.purge_expired_commands();

        let conflicts = state.get_conflicts();
        let mut active_daa_ids: HashSet<String> = HashSet::new();
        if conflicts.is_empty() {
            resolve_inactive_conflict_advisories(state.as_ref(), &active_daa_ids);
            continue;
        }

        tracing::warn!(
            "Detected {} conflict(s)",
            conflicts.len()
        );

        // Get current drone positions for geofence generation
        let drones = state.get_all_drones();
        let external_tracks = state.get_external_traffic();
        let external_by_id: HashMap<String, _> = external_tracks
            .into_iter()
            .map(|track| (track.traffic_id.clone(), track))
            .collect();

        // Transform conflicts to geofences + issue REROUTE commands
        let mut geofences = Vec::with_capacity(conflicts.len());
        for conflict in &conflicts {
            let conflict_key = format!("{}-{}", conflict.drone1_id, conflict.drone2_id);
            let now = Utc::now();
            // Find drone positions
            let drone1 = drones.iter().find(|d| d.drone_id == conflict.drone1_id);
            let drone2 = drones.iter().find(|d| d.drone_id == conflict.drone2_id);
            let external1 = external_by_id.get(&conflict.drone1_id);
            let external2 = external_by_id.get(&conflict.drone2_id);
            
            let drone1_pos = drone1
                .map(|d| (d.lat, d.lon, d.altitude_m))
                .or_else(|| external1.map(|t| (t.lat, t.lon, t.altitude_m)));
            let drone2_pos = drone2
                .map(|d| (d.lat, d.lon, d.altitude_m))
                .or_else(|| external2.map(|t| (t.lat, t.lon, t.altitude_m)));

            let gf = conflict_to_geofence(conflict, drone1_pos, drone2_pos);
            
            tracing::warn!(
                "  [{:?}] {} <-> {} @ {}m -> Geofence {}",
                conflict.severity,
                conflict.drone1_id,
                conflict.drone2_id,
                conflict.distance_m as i32,
                gf.id
            );

            let (severity, action) = match conflict.severity {
                ConflictSeverity::Critical => (DaaSeverity::Critical, "reroute"),
                ConflictSeverity::Warning => (DaaSeverity::Warning, "reroute"),
                ConflictSeverity::Info => (DaaSeverity::Advisory, "monitor"),
            };

            if let Some(drone) = drone1 {
                let advisory_id = format!("conflict-{}-{}", drone.drone_id, conflict.drone2_id);
                active_daa_ids.insert(advisory_id.clone());
                state.set_daa_advisory(DaaAdvisory {
                    advisory_id,
                    drone_id: drone.drone_id.clone(),
                    owner_id: drone.owner_id.clone(),
                    source: "conflict".to_string(),
                    severity,
                    action: action.to_string(),
                    description: format!(
                        "Conflict with {} ({:.0}m separation)",
                        conflict.drone2_id,
                        conflict.distance_m
                    ),
                    related_id: Some(conflict_key.clone()),
                    record: None,
                    created_at: now,
                    updated_at: now,
                    resolved: false,
                });
            }

            if let Some(drone) = drone2 {
                let advisory_id = format!("conflict-{}-{}", drone.drone_id, conflict.drone1_id);
                active_daa_ids.insert(advisory_id.clone());
                state.set_daa_advisory(DaaAdvisory {
                    advisory_id,
                    drone_id: drone.drone_id.clone(),
                    owner_id: drone.owner_id.clone(),
                    source: "conflict".to_string(),
                    severity,
                    action: action.to_string(),
                    description: format!(
                        "Conflict with {} ({:.0}m separation)",
                        conflict.drone1_id,
                        conflict.distance_m
                    ),
                    related_id: Some(conflict_key.clone()),
                    record: None,
                    created_at: now,
                    updated_at: now,
                    resolved: false,
                });
            }
            
            geofences.push(gf);

            // === AUTO-REROUTE COMMAND DISPATCH ===
            // Issue REROUTE to the lower-priority drone (higher ID = newer = gives way)
            if matches!(conflict.severity, ConflictSeverity::Critical | ConflictSeverity::Warning) {
                let now = Utc::now();

                let drone1_external = drone1.is_none() && external1.is_some();
                let drone2_external = drone2.is_none() && external2.is_some();
                if drone1_external ^ drone2_external {
                    let local_drone = if drone1_external { drone2 } else { drone1 };
                    let external = if drone1_external { external1 } else { external2 };

                    if let Some(gw) = local_drone {
                        let give_way_id = gw.drone_id.clone();
                        if state.can_issue_command(&give_way_id, COMMAND_COOLDOWN_SECS) {
                            let conflict_point = Waypoint {
                                lat: conflict.cpa_lat,
                                lon: conflict.cpa_lon,
                                altitude_m: conflict.cpa_altitude_m,
                                speed_mps: Some(gw.speed_mps),
                            };

                            let current_pos = Waypoint {
                                lat: gw.lat,
                                lon: gw.lon,
                                altitude_m: gw.altitude_m,
                                speed_mps: Some(gw.speed_mps),
                            };

                            use atc_core::spatial::offset_by_bearing;
                            let heading_rad = gw.heading_deg.to_radians();
                            let (dest_lat, dest_lon) = offset_by_bearing(
                                gw.lat, gw.lon,
                                500.0,
                                heading_rad
                            );
                            let destination = Waypoint {
                                lat: dest_lat,
                                lon: dest_lon,
                                altitude_m: gw.altitude_m,
                                speed_mps: Some(gw.speed_mps),
                            };

                            let priority_alt = external
                                .map(|traffic| traffic.altitude_m)
                                .unwrap_or(conflict.cpa_altitude_m);
                            let avoidance_type = select_avoidance_type(
                                gw.altitude_m,
                                priority_alt,
                                gw.altitude_m > 100.0,
                            );

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
                                    reason: Some("Conflict avoidance (external traffic)".to_string()),
                                },
                                issued_at: now,
                                expires_at: Some(now + ChronoDuration::seconds(60)),
                                acknowledged: false,
                            };
                            state.enqueue_command(cmd);
                            state.mark_command_issued(&give_way_id);
                            tracing::info!(
                                "Auto-issued REROUTE to {} due to external traffic",
                                give_way_id
                            );
                        }
                    }
                    continue;
                }
                
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
                            // Priority drone is on HOLD/REROUTE - check if it's blocking our path
                            // Use proper distance-to-segment calculation instead of bounding box
                            use atc_core::spatial::{offset_by_bearing, distance_to_segment_m};
                            let heading_rad = gw.heading_deg.to_radians();
                            
                            // Project give_way drone's position ~30 seconds ahead
                            let projection_distance = gw.speed_mps * 30.0;
                            let (future_lat, future_lon) = offset_by_bearing(
                                gw.lat, gw.lon, 
                                projection_distance,
                                heading_rad
                            );
                            
                            // Calculate distance from priority drone to our projected path
                            let distance_to_path = distance_to_segment_m(
                                pri.lat, pri.lon,
                                gw.lat, gw.lon,
                                future_lat, future_lon,
                            );
                            
                            // Use horizontal separation threshold (with buffer)
                            let blocking_threshold = state.rules().min_horizontal_separation_m * 2.0;
                            
                            if distance_to_path > blocking_threshold {
                                // Priority drone is NOT in our path - skip reroute
                                tracing::info!(
                                    "Skipping reroute for {} - {} is {:.0}m from path (threshold: {:.0}m)",
                                    give_way_id, priority_id, distance_to_path, blocking_threshold
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

        resolve_inactive_conflict_advisories(state.as_ref(), &active_daa_ids);
    }
}

fn resolve_inactive_conflict_advisories(state: &AppState, active_ids: &HashSet<String>) {
    for advisory in state.get_daa_advisories() {
        if advisory.source != "conflict" {
            continue;
        }
        if active_ids.contains(&advisory.advisory_id) {
            continue;
        }
        if !advisory.resolved {
            state.resolve_daa_advisory(&advisory.advisory_id);
        }
    }
}
