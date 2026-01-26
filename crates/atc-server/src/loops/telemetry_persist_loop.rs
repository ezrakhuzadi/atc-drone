//! Telemetry persistence loop.
//!
//! Coalesces high-frequency telemetry into periodic DB writes.

use std::collections::HashMap;
use std::time::Duration;

use anyhow::Result;
use tokio::sync::{broadcast, mpsc};
use tokio::time::interval;

use atc_core::models::DroneState;

use crate::state::AppState;
use crate::persistence::{Database, drones as drones_db};

const TELEMETRY_FLUSH_SECS: u64 = 1;

pub async fn run_telemetry_persist_loop(
    db: Database,
    app_state: std::sync::Arc<AppState>,
    mut rx: mpsc::Receiver<DroneState>,
    mut shutdown: broadcast::Receiver<()>,
) {
    let mut ticker = interval(Duration::from_secs(TELEMETRY_FLUSH_SECS));
    let mut pending: HashMap<String, DroneState> = HashMap::new();

    loop {
        tokio::select! {
            _ = shutdown.recv() => {
                tracing::info!("Telemetry persistence loop shutting down");
                break;
            }
            maybe_state = rx.recv() => {
                match maybe_state {
                    Some(state) => {
                        pending.insert(state.drone_id.clone(), state);
                        drain_queue(&mut pending, &mut rx);
                        merge_overflow(&app_state, &mut pending);
                    }
                    None => {
                        tracing::info!("Telemetry persistence channel closed");
                        break;
                    }
                }
            }
            _ = ticker.tick() => {
                merge_overflow(&app_state, &mut pending);
                if let Err(err) = flush_pending(&db, &mut pending).await {
                    tracing::warn!("Telemetry persistence flush failed: {}", err);
                }
            }
        }
    }

    merge_overflow(&app_state, &mut pending);
    if let Err(err) = flush_pending(&db, &mut pending).await {
        tracing::warn!("Telemetry persistence final flush failed: {}", err);
    }
}

fn drain_queue(
    pending: &mut HashMap<String, DroneState>,
    rx: &mut mpsc::Receiver<DroneState>,
) {
    while let Ok(state) = rx.try_recv() {
        pending.insert(state.drone_id.clone(), state);
    }
}

fn merge_overflow(state: &AppState, pending: &mut HashMap<String, DroneState>) {
    let overflow = state.take_telemetry_overflow();
    if overflow.is_empty() {
        return;
    }
    for (drone_id, snapshot) in overflow {
        pending.insert(drone_id, snapshot);
    }
}

async fn flush_pending(db: &Database, pending: &mut HashMap<String, DroneState>) -> Result<()> {
    if pending.is_empty() {
        return Ok(());
    }

    let batch = std::mem::take(pending);
    let mut tx = match db.pool().begin().await {
        Ok(tx) => tx,
        Err(err) => {
            pending.extend(batch);
            return Err(err.into());
        }
    };

    let mut write_error: Option<anyhow::Error> = None;
    for (_, state) in batch.iter() {
        if let Err(err) = drones_db::upsert_drone_tx(&mut tx, state).await {
            write_error = Some(err);
            break;
        }
    }

    if let Some(err) = write_error {
        tx.rollback().await.ok();
        pending.extend(batch);
        return Err(err);
    }

    if let Err(err) = tx.commit().await {
        pending.extend(batch);
        return Err(err.into());
    }

    Ok(())
}
