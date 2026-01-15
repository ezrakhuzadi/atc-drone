//! Telemetry streaming helpers.

use crate::AtcClient;
use anyhow::Result;
use atc_core::models::Telemetry;
use chrono::Utc;

impl AtcClient {
    /// Send telemetry to the ATC server.
    /// Send position update to the ATC server.
    pub async fn send_position(
        &self,
        lat: f64,
        lon: f64,
        altitude_m: f64,
        heading_deg: f64,
        speed_mps: f64,
    ) -> Result<()> {
        let drone_id = self
            .drone_id()
            .ok_or_else(|| anyhow::anyhow!("Not registered"))?;

        // Calculate velocity components from heading and speed
        // heading_deg: 0 = North, 90 = East (clockwise from North)
        let heading_rad = heading_deg.to_radians();
        let velocity_x = speed_mps * heading_rad.sin();  // East component
        let velocity_y = speed_mps * heading_rad.cos();  // North component
        let velocity_z = 0.0; // Assuming level flight (could add climb rate later)

        let telemetry = Telemetry {
            drone_id: drone_id.to_string(),
            lat,
            lon,
            altitude_m,
            velocity_x,
            velocity_y,
            velocity_z,
            heading_deg,
            speed_mps,
            timestamp: Utc::now(),
        };

        self.send_telemetry(&telemetry).await
    }
}
