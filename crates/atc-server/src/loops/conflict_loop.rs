//! Continuous conflict detection loop.
//!
//! Runs in the background, periodically checking for conflicts
//! and broadcasting updates.

use std::sync::Arc;
use std::time::Duration;
use tokio::time::interval;

use crate::state::AppState;

/// Start the conflict detection loop.
pub async fn run_conflict_loop(state: Arc<AppState>) {
    let mut ticker = interval(Duration::from_secs(1));

    loop {
        ticker.tick().await;

        let conflicts = state.get_conflicts();
        if !conflicts.is_empty() {
            tracing::warn!(
                "Detected {} conflict(s)",
                conflicts.len()
            );
            for conflict in &conflicts {
                tracing::warn!(
                    "  [{:?}] {} <-> {} @ {}m",
                    conflict.severity,
                    conflict.drone1_id,
                    conflict.drone2_id,
                    conflict.distance_m as i32
                );
            }
        }
    }
}
