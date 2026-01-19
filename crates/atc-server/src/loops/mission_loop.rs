//! Mission execution loop.
//!
//! Activates approved flight plans and marks them complete based on telemetry.

use std::sync::Arc;
use std::time::Duration;
use tokio::sync::broadcast;
use tokio::time::interval;
use chrono::Utc;
use uuid::Uuid;

use crate::state::AppState;
use atc_core::haversine_distance;
use atc_core::models::{Command, CommandType, DroneStatus, FlightStatus};

const LOOP_INTERVAL_SECS: u64 = 2;
const COMMAND_COOLDOWN_SECS: u64 = 10;
const ARRIVAL_DISTANCE_M: f64 = 20.0;
const ARRIVAL_ALTITUDE_M: f64 = 15.0;

pub async fn run_mission_loop(state: Arc<AppState>, mut shutdown: broadcast::Receiver<()>) {
    let mut ticker = interval(Duration::from_secs(LOOP_INTERVAL_SECS));

    loop {
        tokio::select! {
            _ = shutdown.recv() => {
                tracing::info!("Mission loop shutting down");
                break;
            }
            _ = ticker.tick() => {
                let now = Utc::now();

                for mut entry in state.flight_plans.iter_mut() {
                    let plan = entry.value_mut();

                    match plan.status {
                        FlightStatus::Approved | FlightStatus::Pending => {
                            if plan.waypoints.is_empty() {
                                continue;
                            }

                            if plan.departure_time > now {
                                continue;
                            }

                            let drone = match state.get_drone(&plan.drone_id) {
                                Some(drone) => drone,
                                None => continue,
                            };

                            if matches!(drone.status, DroneStatus::Lost | DroneStatus::Inactive) {
                                continue;
                            }

                            if state.has_active_command(&plan.drone_id) {
                                continue;
                            }

                            if !state.can_issue_command(&plan.drone_id, COMMAND_COOLDOWN_SECS) {
                                continue;
                            }

                            let cmd = Command {
                                command_id: Uuid::new_v4().to_string(),
                                drone_id: plan.drone_id.clone(),
                                command_type: CommandType::Reroute {
                                    waypoints: plan.waypoints.clone(),
                                    reason: Some("Mission plan execution".to_string()),
                                },
                                issued_at: now,
                                expires_at: None,
                                acknowledged: false,
                            };

                            state.enqueue_command(cmd);
                            state.mark_command_issued(&plan.drone_id);
                            plan.status = FlightStatus::Active;
                        }
                        FlightStatus::Active => {
                            let drone = match state.get_drone(&plan.drone_id) {
                                Some(drone) => drone,
                                None => continue,
                            };

                            if matches!(drone.status, DroneStatus::Lost | DroneStatus::Inactive) {
                                plan.status = FlightStatus::Cancelled;
                                continue;
                            }

                            let last_wp = match plan.waypoints.last() {
                                Some(wp) => wp,
                                None => continue,
                            };

                            let distance = haversine_distance(drone.lat, drone.lon, last_wp.lat, last_wp.lon);
                            let altitude_delta = (drone.altitude_m - last_wp.altitude_m).abs();

                            if distance <= ARRIVAL_DISTANCE_M && altitude_delta <= ARRIVAL_ALTITUDE_M {
                                plan.status = FlightStatus::Completed;
                                plan.arrival_time = Some(now);
                            }
                        }
                        _ => {}
                    }
                }
            }
        }
    }
}
