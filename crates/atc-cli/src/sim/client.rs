//! HTTP client for Flight Blender API.

use anyhow::{Context, Result};
use reqwest::blocking::Client;
use serde::{Deserialize, Serialize};
use std::time::{SystemTime, UNIX_EPOCH};

/// Observation sent to Flight Blender.
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

/// Response from Flight Blender API.
#[derive(Debug, Deserialize)]
pub struct BlenderResponse {
    #[serde(default)]
    pub message: Option<String>,
}

/// HTTP client for sending telemetry to Flight Blender.
pub struct BlenderClient {
    client: Client,
    base_url: String,
    session_id: String,
    token: String,
}

impl BlenderClient {
    /// Create a new Blender client.
    ///
    /// # Arguments
    /// * `base_url` - Base URL of Flight Blender (e.g., "http://localhost:8000")
    /// * `session_id` - Session UUID
    /// * `token` - JWT bearer token
    pub fn new(base_url: impl Into<String>, session_id: impl Into<String>, token: impl Into<String>) -> Self {
        Self {
            client: Client::new(),
            base_url: base_url.into(),
            session_id: session_id.into(),
            token: token.into(),
        }
    }

    /// Send a single observation to Flight Blender.
    ///
    /// # Arguments
    /// * `drone_id` - Unique drone identifier (ICAO address)
    /// * `lat` - Latitude in decimal degrees
    /// * `lon` - Longitude in decimal degrees
    /// * `altitude_m` - Altitude in meters
    /// * `heading` - Heading in degrees (0 = North)
    /// * `speed_mps` - Speed in meters per second
    ///
    /// # Returns
    /// HTTP status code
    pub fn send_observation(
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

        let timestamp = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .context("Failed to get current time")?
            .as_secs() as i64;

        let request = ObservationRequest {
            observations: vec![Observation {
                lat_dd: lat,
                lon_dd: lon,
                altitude_mm: (altitude_m * 1000.0) as i64,
                icao_address: drone_id.to_string(),
                traffic_source: 1, // ADS-B
                source_type: 1,
                timestamp,
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
            .context("Failed to send observation")?;

        Ok(response.status().as_u16())
    }
}
