//! Operational intent expiry loop.
//!
//! Cancels expired reserved intents so they don't occupy schedule slots indefinitely.

use std::sync::Arc;
use std::time::Duration;

use chrono::{DateTime, Utc};
use tokio::sync::broadcast;
use tokio::time::interval;

use atc_core::models::FlightStatus;

use crate::persistence::flight_plans as flight_plans_db;
use crate::state::AppState;

const LOOP_INTERVAL_SECS: u64 = 5;

pub async fn run_operational_intent_expiry_loop(
    state: Arc<AppState>,
    mut shutdown: broadcast::Receiver<()>,
) {
    let Some(db) = state.database().cloned() else {
        tracing::warn!("Operational intent expiry loop disabled (no database)");
        return;
    };

    let pool = db.pool().clone();
    let mut ticker = interval(Duration::from_secs(LOOP_INTERVAL_SECS));
    state.mark_loop_heartbeat("oi-expiry");

    'main: loop {
        tokio::select! {
            _ = shutdown.recv() => {
                tracing::info!("Operational intent expiry loop shutting down");
                break;
            }
            _ = ticker.tick() => {
                state.mark_loop_heartbeat("oi-expiry");
                let now = Utc::now();
                let mut tx = match pool.begin().await {
                    Ok(tx) => tx,
                    Err(err) => {
                        tracing::warn!("Operational intent expiry begin tx failed: {}", err);
                        continue;
                    }
                };

                if let Err(err) = flight_plans_db::lock_scheduler(&mut tx).await {
                    tracing::warn!("Operational intent expiry lock failed: {}", err);
                    tx.rollback().await.ok();
                    continue;
                }

                let plans = match flight_plans_db::load_all_flight_plans_tx(&mut tx).await {
                    Ok(plans) => plans,
                    Err(err) => {
                        tracing::warn!("Operational intent expiry load failed: {}", err);
                        tx.rollback().await.ok();
                        continue;
                    }
                };

                let mut expired: Vec<atc_core::models::FlightPlan> = Vec::new();
                for plan in plans {
                    if plan.status != FlightStatus::Reserved {
                        continue;
                    }
                    if !is_expired(&plan, now) {
                        continue;
                    }
                    let mut updated = plan;
                    updated.status = FlightStatus::Cancelled;
                    expired.push(updated);
                }

                if expired.is_empty() {
                    tx.rollback().await.ok();
                    continue 'main;
                }

                for plan in &expired {
                    if let Err(err) = flight_plans_db::upsert_flight_plan_tx(&mut tx, plan).await {
                        tracing::warn!("Operational intent expiry persist failed: {}", err);
                        tx.rollback().await.ok();
                        continue 'main;
                    }
                }

                if let Err(err) = tx.commit().await {
                    tracing::warn!("Operational intent expiry commit failed: {}", err);
                    continue 'main;
                }

                for plan in expired {
                    state.flight_plans.insert(plan.flight_id.clone(), plan);
                }
            }
        }
    }
}

fn is_expired(plan: &atc_core::models::FlightPlan, now: DateTime<Utc>) -> bool {
    let Some(meta) = plan.metadata.as_ref() else {
        return false;
    };
    let Some(raw) = meta.reservation_expires_at.as_deref() else {
        return false;
    };
    chrono::DateTime::parse_from_rfc3339(raw)
        .map(|dt| dt.with_timezone(&Utc))
        .map(|expires| expires <= now)
        .unwrap_or(false)
}
