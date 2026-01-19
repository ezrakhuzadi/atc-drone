//! Loop to sync drone tracks to Flight Blender.

use std::sync::Arc;
use std::time::Duration;
use tokio::sync::broadcast;
use tokio::time::interval;

use crate::state::AppState;
use crate::config::Config;
use atc_blender::BlenderClient;

/// Start the Blender sync loop.
pub async fn run_blender_loop(
    state: Arc<AppState>,
    config: Config,
    mut shutdown: broadcast::Receiver<()>,
) {
    let client = BlenderClient::new(
        config.blender_url,
        config.blender_session_id,
        config.blender_auth_token,
    );

    let mut ticker = interval(Duration::from_millis(1000));

    loop {
        tokio::select! {
            _ = shutdown.recv() => {
                tracing::info!("Blender sync loop shutting down");
                break;
            }
            _ = ticker.tick() => {
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
