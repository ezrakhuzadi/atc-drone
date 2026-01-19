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
};

use crate::config::Config;
use crate::state::AppState;

const CONFORMANCE_POLL_SECS: u64 = 10;
const CONFORMANCE_COMMAND_COOLDOWN_SECS: u64 = 120;
const CONFORMANCE_HOLD_SECS: u32 = 60;

/// Start the conformance monitoring loop.
pub async fn run_conformance_loop(
    state: Arc<AppState>,
    config: Config,
    mut shutdown: broadcast::Receiver<()>,
) {
    let blender = BlenderClient::new(
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
                        let description = record
                            .map(|entry| entry.description.clone())
                            .unwrap_or_else(|| "Conformance issue detected".to_string());
                        state.set_daa_advisory(DaaAdvisory {
                            advisory_id,
                            drone_id: drone.drone_id.clone(),
                            owner_id: drone.owner_id.clone(),
                            source: "conformance".to_string(),
                            severity: DaaSeverity::Critical,
                            action: "hold".to_string(),
                            description,
                            related_id: record.and_then(|entry| entry.geofence_id.clone()),
                            record: status.record.clone(),
                            created_at: now,
                            updated_at: now,
                            resolved: false,
                        });

                        if !state.has_active_command(&drone.drone_id)
                            && state.can_issue_command(&drone.drone_id, CONFORMANCE_COMMAND_COOLDOWN_SECS)
                        {
                            let now = Utc::now();
                            let cmd = Command {
                                command_id: format!("CONFORMANCE-HOLD-{}-{}", drone.drone_id, now.timestamp()),
                                drone_id: drone.drone_id.clone(),
                                command_type: CommandType::Hold {
                                    duration_secs: CONFORMANCE_HOLD_SECS,
                                },
                                issued_at: now,
                                expires_at: Some(now + ChronoDuration::seconds(CONFORMANCE_HOLD_SECS as i64)),
                                acknowledged: false,
                            };
                            state.enqueue_command(cmd);
                            state.mark_command_issued(&drone.drone_id);
                            tracing::warn!(
                                "Issued HOLD for {} due to nonconformance",
                                drone.drone_id
                            );
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
                            state.enqueue_command(cmd);
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
