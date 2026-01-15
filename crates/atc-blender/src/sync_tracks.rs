//! Sync drone tracks to Blender flight feed.

use crate::BlenderClient;
use anyhow::Result;
use atc_core::models::DroneState;

/// Sync drone states to Blender.
pub async fn sync_tracks(client: &BlenderClient, drones: &[DroneState]) -> Result<()> {
    for drone in drones {
        client
            .send_observation(
                &drone.drone_id,
                drone.lat,
                drone.lon,
                drone.altitude_m,
                drone.heading_deg,
                drone.speed_mps,
            )
            .await?;
    }
    Ok(())
}
