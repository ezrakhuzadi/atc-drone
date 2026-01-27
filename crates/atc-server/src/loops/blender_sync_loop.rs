//! Loop to sync drone tracks to Flight Blender.

use std::sync::Arc;
use std::time::{Duration, Instant};

use chrono::{DateTime, Utc};
use std::collections::{HashMap, HashSet};
use tokio::sync::broadcast;
use tokio::time::interval;

use crate::blender_auth::BlenderAuthManager;
use crate::config::Config;
use crate::state::AppState;
use atc_blender::BlenderClient;

const BLENDER_SYNC_INTERVAL_SECS: u64 = 2;
const BLENDER_FULL_SYNC_SECS: u64 = 30;

/// Start the Blender sync loop.
pub async fn run_blender_loop(
    state: Arc<AppState>,
    config: Config,
    mut shutdown: broadcast::Receiver<()>,
) {
    let auth = BlenderAuthManager::new(&config);
    let mut client = BlenderClient::new(
        config.blender_url,
        config.blender_session_id,
        config.blender_auth_token,
    );

    let mut ticker = interval(Duration::from_secs(BLENDER_SYNC_INTERVAL_SECS));
    let mut last_sent: HashMap<String, DateTime<Utc>> = HashMap::new();
    let mut last_full_sync = Instant::now()
        .checked_sub(Duration::from_secs(BLENDER_FULL_SYNC_SECS + 1))
        .unwrap_or_else(Instant::now);
    state.mark_loop_heartbeat("blender-sync");

    loop {
        tokio::select! {
            _ = shutdown.recv() => {
                tracing::info!("Blender sync loop shutting down");
                break;
            }
            _ = ticker.tick() => {
                state.mark_loop_heartbeat("blender-sync");
                if let Err(err) = auth.apply(&mut client).await {
                    tracing::warn!("Blender sync auth refresh failed: {}", err);
                    continue;
                }
                let full_sync = last_full_sync.elapsed().as_secs() >= BLENDER_FULL_SYNC_SECS;
                let mut to_send = Vec::new();
                let mut current_ids: Option<HashSet<String>> = None;

                for drone in state.get_all_drones() {
                    let id = drone.drone_id.clone();
                    let last_update = drone.last_update;
                    let changed = full_sync
                        || last_sent
                            .get(&id)
                            .map(|prev| *prev != last_update)
                            .unwrap_or(true);

                    last_sent.insert(id.clone(), last_update);
                    if changed {
                        to_send.push(drone);
                    }

                    if full_sync {
                        current_ids
                            .get_or_insert_with(HashSet::new)
                            .insert(id);
                    }
                }

                if full_sync {
                    last_full_sync = Instant::now();
                    if let Some(ids) = current_ids {
                        last_sent.retain(|id, _| ids.contains(id));
                    }
                }

                if to_send.is_empty() {
                    continue;
                }

                if let Err(e) = client.send_snapshot(&to_send).await {
                    tracing::error!("Blender Sync Error: {}", e);
                } else {
                    tracing::debug!("Synced {} drones to Blender", to_send.len());
                }
            }
        }
    }
}
