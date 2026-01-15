//! Telemetry streaming helpers.

use crate::AtcClient;
use anyhow::Result;
use atc_core::models::Telemetry;
use chrono::Utc;

impl AtcClient {
    /// Send telemetry to the ATC server.
    pub async fn send_telemetry(
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

        let url = format!("{}/v1/telemetry", self.base_url);

        let telemetry = Telemetry {
            drone_id: drone_id.to_string(),
            lat,
            lon,
            altitude_m,
            velocity_x: 0.0,
            velocity_y: 0.0,
            velocity_z: 0.0,
            heading_deg,
            speed_mps,
            timestamp: Utc::now(),
        };

        self.client
            .post(&url)
            .json(&telemetry)
            .send()
            .await?;

        Ok(())
    }
}
