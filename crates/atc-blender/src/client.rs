//! Blender API HTTP client.

use anyhow::{Context, Result};
use chrono::Utc;
use reqwest::Client;
use serde::{Deserialize, Serialize};
use std::time::Duration;
use base64::{Engine as _, engine::general_purpose::URL_SAFE_NO_PAD};

/// Generate a dummy JWT token that Blender will accept.
/// Blender with BYPASS_AUTH_TOKEN_VERIFICATION=1 still validates:
/// - Token format (must be header.payload.signature)
/// - Issuer (must be a valid URL)
/// - Scopes (must include flightblender.write)
fn generate_dummy_jwt() -> String {
    let header = serde_json::json!({"alg": "HS256", "typ": "JWT"});
    let now = Utc::now().timestamp();
    let payload = serde_json::json!({
        "iss": "https://atc-server.local",
        "sub": "atc-server",
        "aud": "testflight.flightblender.com",
        "scope": "flightblender.read flightblender.write",
        "exp": now + 3600,
        "iat": now
    });
    
    let header_b64 = URL_SAFE_NO_PAD.encode(header.to_string().as_bytes());
    let payload_b64 = URL_SAFE_NO_PAD.encode(payload.to_string().as_bytes());
    let signature_b64 = URL_SAFE_NO_PAD.encode(b"dummy");
    
    format!("{}.{}.{}", header_b64, payload_b64, signature_b64)
}

/// HTTP client for Flight Blender API.
pub struct BlenderClient {
    client: Client,
    base_url: String,
    session_id: String,
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
    /// Note: Token is not needed as we generate a dummy JWT for each request.
    pub fn new(
        base_url: impl Into<String>,
        session_id: impl Into<String>,
        _token: impl Into<String>, // Kept for API compatibility, but unused
    ) -> Self {
        Self {
            client: Client::builder()
                .timeout(Duration::from_secs(10))
                .build()
                .expect("Failed to create HTTP client"),
            base_url: base_url.into(),
            session_id: session_id.into(),
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

        self.send_request(&url, &request).await
    }

    /// Send a snapshot of all drones to Blender.
    pub async fn send_snapshot(&self, drones: &[atc_core::models::DroneState]) -> Result<u16> {
        if drones.is_empty() {
            return Ok(200);
        }

        let url = format!(
            "{}/flight_stream/set_air_traffic/{}",
            self.base_url, self.session_id
        );

        let observations = drones.iter().map(|d| Observation {
            lat_dd: d.lat,
            lon_dd: d.lon,
            altitude_mm: (d.altitude_m * 1000.0) as i64,
            icao_address: d.drone_id.clone(),
            traffic_source: 1,
            source_type: 1,
            timestamp: Utc::now().timestamp(),
            metadata: ObservationMetadata {
                heading: d.heading_deg,
                speed_mps: d.speed_mps,
                aircraft_type: "UAV".to_string(),
            },
        }).collect();

        let request = ObservationRequest { observations };
        self.send_request(&url, &request).await
    }

    async fn send_request(&self, url: &str, request: &ObservationRequest) -> Result<u16> {
        // Generate a fresh JWT for each request (handles token expiration)
        let token = generate_dummy_jwt();
        
        let response = self
            .client
            .post(url)
            .header("Content-Type", "application/json")
            .header("Authorization", format!("Bearer {}", token))
            .json(request)
            .send()
            .await
            .context("Failed to send observation")?;

        Ok(response.status().as_u16())
    }
}
