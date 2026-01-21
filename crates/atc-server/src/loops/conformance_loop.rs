//! Periodic conformance monitoring loop.
//!
//! Polls Flight Blender for conformance status and issues HOLD commands
//! for non-conforming drones.

use std::collections::HashMap;
use std::sync::Arc;
use std::time::Duration;

use chrono::{Duration as ChronoDuration, Utc};
use tokio::sync::broadcast;
use tokio::time::interval;

use atc_blender::BlenderClient;
use atc_core::models::{
    Command,
    CommandType,
    ConformanceRecord,
    ConformanceStatus,
    DaaAdvisory,
    DaaSeverity,
    DroneState,
    Geofence,
    Waypoint,
};

use crate::config::Config;
use crate::blender_auth::BlenderAuthManager;
use crate::state::AppState;

const CONFORMANCE_POLL_SECS: u64 = 10;
const CONFORMANCE_COMMAND_COOLDOWN_SECS: u64 = 120;
const CONFORMANCE_HOLD_SECS: u32 = 60;
const GEOFENCE_EXIT_BUFFER_M: f64 = 50.0;

/// Start the conformance monitoring loop.
pub async fn run_conformance_loop(
    state: Arc<AppState>,
    config: Config,
    mut shutdown: broadcast::Receiver<()>,
) {
    let auth = BlenderAuthManager::new(&config);
    let mut blender = BlenderClient::new(
        &config.blender_url,
        &config.blender_session_id,
        &config.blender_auth_token,
    );

    let mut ticker = interval(Duration::from_secs(CONFORMANCE_POLL_SECS));
    let mut last_status: HashMap<String, String> = HashMap::new();

    loop {
        tokio::select! {
            _ = shutdown.recv() => {
                tracing::info!("Conformance loop shutting down");
                break;
            }
            _ = ticker.tick() => {
                if let Err(err) = auth.apply(&mut blender).await {
                    tracing::warn!("Conformance loop Blender auth refresh failed: {}", err);
                    continue;
                }
                let drones = state.get_all_drones();
                if drones.is_empty() {
                    continue;
                }

                for drone in drones {
                    let response = blender.fetch_conformance_status(&drone.drone_id).await;
                    let Ok(payload) = response else {
                        tracing::debug!(
                            "Conformance status fetch failed for {}",
                            drone.drone_id
                        );
                        continue;
                    };

                    let status = ConformanceStatus {
                        drone_id: drone.drone_id.clone(),
                        owner_id: drone.owner_id.clone(),
                        status: payload.status.clone(),
                        last_checked: Utc::now(),
                        record: payload.record.clone(),
                    };

                    state.set_conformance_status(status.clone());

                    let previous = last_status.insert(drone.drone_id.clone(), status.status.clone());
                    let record = status.record.as_ref();
                    let advisory_id = format!("conformance-{}", drone.drone_id);

                    if status.status == "nonconforming" && requires_hold(record) {
                        let now = Utc::now();
                        let geofence_exit = record
                            .filter(|entry| entry.geofence_breach)
                            .and_then(|entry| entry.geofence_id.as_deref())
                            .and_then(|geofence_id| state.get_geofence(geofence_id))
                            .and_then(|geofence| compute_geofence_exit(&drone, &geofence));
                        let description = record
                            .map(|entry| entry.description.clone())
                            .unwrap_or_else(|| "Conformance issue detected".to_string());
                        state.set_daa_advisory(DaaAdvisory {
                            advisory_id,
                            drone_id: drone.drone_id.clone(),
                            owner_id: drone.owner_id.clone(),
                            source: "conformance".to_string(),
                            severity: DaaSeverity::Critical,
                            action: if geofence_exit.is_some() { "reroute" } else { "hold" }.to_string(),
                            description,
                            related_id: record.and_then(|entry| entry.geofence_id.clone()),
                            record: status.record.clone(),
                            created_at: now,
                            updated_at: now,
                            resolved: false,
                        });

                        if !state.has_pending_command(&drone.drone_id)
                            && state.can_issue_command(&drone.drone_id, CONFORMANCE_COMMAND_COOLDOWN_SECS)
                        {
                            let now = Utc::now();
                            let cmd = if let Some(exit_waypoint) = geofence_exit {
                                Command {
                                    command_id: format!("CONFORMANCE-EXIT-{}-{}", drone.drone_id, now.timestamp()),
                                    drone_id: drone.drone_id.clone(),
                                    command_type: CommandType::Reroute {
                                        waypoints: vec![exit_waypoint],
                                        reason: Some("Geofence breach: exit route".to_string()),
                                    },
                                    issued_at: now,
                                    expires_at: Some(now + ChronoDuration::seconds(CONFORMANCE_HOLD_SECS as i64)),
                                    acknowledged: false,
                                }
                            } else {
                                Command {
                                    command_id: format!("CONFORMANCE-HOLD-{}-{}", drone.drone_id, now.timestamp()),
                                    drone_id: drone.drone_id.clone(),
                                    command_type: CommandType::Hold {
                                        duration_secs: CONFORMANCE_HOLD_SECS,
                                    },
                                    issued_at: now,
                                    expires_at: Some(now + ChronoDuration::seconds(CONFORMANCE_HOLD_SECS as i64)),
                                    acknowledged: false,
                                }
                            };
                            if let Err(err) = state.enqueue_command(cmd).await {
                                tracing::warn!(
                                    "Failed to enqueue conformance recovery for {}: {}",
                                    drone.drone_id,
                                    err
                                );
                            } else {
                                state.mark_command_issued(&drone.drone_id);
                                tracing::warn!(
                                    "Issued conformance recovery for {}",
                                    drone.drone_id,
                                );
                            }
                        }
                    } else if status.status == "nonconforming" {
                        let now = Utc::now();
                        let description = record
                            .map(|entry| entry.description.clone())
                            .unwrap_or_else(|| "Conformance issue detected".to_string());
                        state.set_daa_advisory(DaaAdvisory {
                            advisory_id,
                            drone_id: drone.drone_id.clone(),
                            owner_id: drone.owner_id.clone(),
                            source: "conformance".to_string(),
                            severity: DaaSeverity::Warning,
                            action: "monitor".to_string(),
                            description,
                            related_id: record.and_then(|entry| entry.geofence_id.clone()),
                            record: status.record.clone(),
                            created_at: now,
                            updated_at: now,
                            resolved: false,
                        });
                    } else if status.status == "conforming" && previous.as_deref() == Some("nonconforming") {
                        state.resolve_daa_advisory(&advisory_id);
                        if !state.has_active_command(&drone.drone_id)
                            && state.can_issue_command(&drone.drone_id, CONFORMANCE_COMMAND_COOLDOWN_SECS)
                        {
                            let now = Utc::now();
                            let cmd = Command {
                                command_id: format!("CONFORMANCE-RESUME-{}-{}", drone.drone_id, now.timestamp()),
                                drone_id: drone.drone_id.clone(),
                                command_type: CommandType::Resume,
                                issued_at: now,
                                expires_at: Some(now + ChronoDuration::seconds(CONFORMANCE_HOLD_SECS as i64)),
                                acknowledged: false,
                            };
                            if let Err(err) = state.enqueue_command(cmd).await {
                                tracing::warn!(
                                    "Failed to enqueue RESUME for {}: {}",
                                    drone.drone_id,
                                    err
                                );
                            } else {
                                state.mark_command_issued(&drone.drone_id);
                                tracing::info!(
                                    "Issued RESUME for {} after conformance restored",
                                    drone.drone_id
                                );
                            }
                        }
                    }
                }
            }
        }
    }
}

fn requires_hold(record: Option<&ConformanceRecord>) -> bool {
    let Some(record) = record else {
        return true;
    };

    if record.resolved {
        return false;
    }

    if record.geofence_breach {
        return true;
    }

    match record.conformance_state_code.as_deref() {
        Some("C7a") => true, // Flight out of bounds
        Some("C7b") => true, // Flight altitude out of bounds
        Some("C8") => true,  // Geofence breached
        _ => false,
    }
}

fn compute_geofence_exit(drone: &DroneState, geofence: &Geofence) -> Option<Waypoint> {
    if geofence.polygon.len() < 2 {
        return None;
    }
    if !geofence.contains_point(drone.lat, drone.lon, drone.altitude_m) {
        return None;
    }

    let ref_lat = drone.lat;
    let ref_lon = drone.lon;
    let mut best: Option<(f64, f64, f64)> = None;

    for edge in geofence.polygon.windows(2) {
        let p1 = edge[0];
        let p2 = edge[1];
        let (x1, y1) = project_xy(p1[0], p1[1], ref_lat, ref_lon);
        let (x2, y2) = project_xy(p2[0], p2[1], ref_lat, ref_lon);
        let (cx, cy) = closest_point_on_segment(0.0, 0.0, x1, y1, x2, y2);
        let dist_sq = cx * cx + cy * cy;
        match best {
            Some((_, _, best_dist_sq)) if dist_sq >= best_dist_sq => {}
            _ => {
                best = Some((cx, cy, dist_sq));
            }
        }
    }

    let Some((cx, cy, dist_sq)) = best else {
        return None;
    };

    let distance = dist_sq.sqrt();
    let (dir_x, dir_y) = if distance > f64::EPSILON {
        (cx / distance, cy / distance)
    } else {
        (1.0, 0.0)
    };

    let exit_distance = distance + GEOFENCE_EXIT_BUFFER_M;
    let exit_x = exit_distance * dir_x;
    let exit_y = exit_distance * dir_y;
    let (exit_lat, exit_lon) = unproject_xy(exit_x, exit_y, ref_lat, ref_lon);

    Some(Waypoint {
        lat: exit_lat,
        lon: exit_lon,
        altitude_m: drone.altitude_m,
        speed_mps: Some(drone.speed_mps),
    })
}

fn project_xy(lat: f64, lon: f64, ref_lat: f64, ref_lon: f64) -> (f64, f64) {
    const METERS_PER_DEG_LAT: f64 = 111_320.0;
    let x = (lon - ref_lon) * METERS_PER_DEG_LAT * ref_lat.to_radians().cos();
    let y = (lat - ref_lat) * METERS_PER_DEG_LAT;
    (x, y)
}

fn unproject_xy(x: f64, y: f64, ref_lat: f64, ref_lon: f64) -> (f64, f64) {
    const METERS_PER_DEG_LAT: f64 = 111_320.0;
    let lat = ref_lat + (y / METERS_PER_DEG_LAT);
    let lon = ref_lon + (x / (METERS_PER_DEG_LAT * ref_lat.to_radians().cos()));
    (lat, lon)
}

fn closest_point_on_segment(
    px: f64,
    py: f64,
    x1: f64,
    y1: f64,
    x2: f64,
    y2: f64,
) -> (f64, f64) {
    let dx = x2 - x1;
    let dy = y2 - y1;
    let len_sq = dx * dx + dy * dy;
    if len_sq <= f64::EPSILON {
        return (x1, y1);
    }
    let t = ((px - x1) * dx + (py - y1) * dy) / len_sq;
    let t = t.clamp(0.0, 1.0);
    (x1 + t * dx, y1 + t * dy)
}
