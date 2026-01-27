//! Loop to sync drone tracks to Flight Blender.

use std::sync::Arc;
use std::time::Duration;
use tokio::sync::broadcast;
use tokio::time::interval;

use crate::state::AppState;
use crate::config::Config;
use crate::blender_auth::BlenderAuthManager;
use atc_blender::BlenderClient;

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

    let mut ticker = interval(Duration::from_millis(1000));
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
                let drones = state.get_all_drones();
                if drones.is_empty() {
                    continue;
                }

                if let Err(e) = client.send_snapshot(&drones).await {
                    tracing::error!("Blender Sync Error: {}", e);
                } else {
                    tracing::debug!("Synced {} drones to Blender", drones.len());
                }
            }
        }
    }
}
