//! Blender API HTTP client.

use anyhow::{Context, Result};
use chrono::Utc;
use reqwest::Client;
use serde::{Deserialize, Serialize};
use std::time::Duration;

/// HTTP client for Flight Blender API.
pub struct BlenderClient {
    client: Client,
    base_url: String,
    session_id: String,
    token: String,
}

#[derive(Debug, Serialize)]
struct Observation {
    lat_dd: f64,
    lon_dd: f64,
    altitude_mm: i64,
    icao_address: String,
    traffic_source: i32,
    source_type: i32,
    timestamp: i64,
    metadata: ObservationMetadata,
}

#[derive(Debug, Serialize)]
struct ObservationMetadata {
    heading: f64,
    speed_mps: f64,
    aircraft_type: String,
}

#[derive(Debug, Serialize)]
struct ObservationRequest {
    observations: Vec<Observation>,
}

impl BlenderClient {
    /// Create a new Blender client.
    pub fn new(
        base_url: impl Into<String>,
        session_id: impl Into<String>,
        token: impl Into<String>,
    ) -> Self {
        Self {
            client: Client::builder()
                .timeout(Duration::from_secs(10))
                .build()
                .expect("Failed to create HTTP client"),
            base_url: base_url.into(),
            session_id: session_id.into(),
            token: token.into(),
        }
    }

    /// Send telemetry observation to Blender.
    pub async fn send_observation(
        &self,
        drone_id: &str,
        lat: f64,
        lon: f64,
        altitude_m: f64,
        heading: f64,
        speed_mps: f64,
    ) -> Result<u16> {
        let url = format!(
            "{}/flight_stream/set_air_traffic/{}",
            self.base_url, self.session_id
        );

        let request = ObservationRequest {
            observations: vec![Observation {
                lat_dd: lat,
                lon_dd: lon,
                altitude_mm: (altitude_m * 1000.0) as i64,
                icao_address: drone_id.to_string(),
                traffic_source: 1,
                source_type: 1,
                timestamp: Utc::now().timestamp(),
                metadata: ObservationMetadata {
                    heading,
                    speed_mps,
                    aircraft_type: "UAV".to_string(),
                },
            }],
        };

        let response = self
            .client
            .post(&url)
            .header("Content-Type", "application/json")
            .header("Authorization", format!("Bearer {}", self.token))
            .json(&request)
            .send()
            .await
            .context("Failed to send observation")?;

        Ok(response.status().as_u16())
    }
}
