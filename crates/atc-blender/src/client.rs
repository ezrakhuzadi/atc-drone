//! Blender API HTTP client.

use anyhow::{Context, Result};
use base64::{engine::general_purpose::URL_SAFE_NO_PAD, Engine as _};
use chrono::Utc;
use reqwest::Client;
use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::time::Duration;

/// Generate a dummy JWT token that Blender will accept.
/// Blender with BYPASS_AUTH_TOKEN_VERIFICATION=1 still validates:
/// - Token format (must be header.payload.signature)
/// - Issuer (must be a valid URL)
/// - Scopes (must include flightblender.write)
pub(crate) fn generate_dummy_jwt() -> String {
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
    pub(crate) client: Client,
    pub(crate) base_url: String,
    pub(crate) session_id: String,
    pub(crate) auth_token: Option<String>,
    pub(crate) request_id: Option<String>,
}

#[derive(Debug, Deserialize)]
pub struct BlenderConformanceStatusResponse {
    pub status: String,
    pub record: Option<atc_core::models::ConformanceRecord>,
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
    /// If a token is provided, it will be used for all requests; otherwise a dummy JWT is generated.
    pub fn new(
        base_url: impl Into<String>,
        session_id: impl Into<String>,
        token: impl Into<String>,
    ) -> Self {
        let token = token.into();
        let auth_token = if token.trim().is_empty() {
            None
        } else {
            Some(token)
        };
        Self {
            client: Client::builder()
                .timeout(Duration::from_secs(10))
                .build()
                .expect("Failed to create HTTP client"),
            base_url: base_url.into(),
            session_id: session_id.into(),
            auth_token,
            request_id: None,
        }
    }

    fn apply_request_id(&self, request: reqwest::RequestBuilder) -> reqwest::RequestBuilder {
        match self.request_id.as_deref() {
            Some(value) if !value.is_empty() => request.header("X-Request-ID", value),
            _ => request,
        }
    }

    pub(crate) fn auth_header(&self) -> String {
        let token = self.auth_token.clone().unwrap_or_else(generate_dummy_jwt);
        format!("Bearer {}", token)
    }

    /// Update auth token at runtime (OAuth refresh, rotation, etc.).
    pub fn set_auth_token(&mut self, token: Option<String>) {
        self.auth_token = token
            .map(|value| value.trim().to_string())
            .filter(|value| !value.is_empty());
    }

    pub fn set_request_id(&mut self, request_id: Option<String>) {
        self.request_id = request_id
            .map(|value| value.trim().to_string())
            .filter(|value| !value.is_empty());
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

        let observations = drones
            .iter()
            .map(|d| Observation {
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
            })
            .collect();

        let request = ObservationRequest { observations };
        self.send_request(&url, &request).await
    }

    async fn send_request(&self, url: &str, request: &ObservationRequest) -> Result<u16> {
        // Resolve auth token (configured or dummy)
        let auth_header = self.auth_header();

        let response = self
            .apply_request_id(
                self.client
                    .post(url)
                    .header("Content-Type", "application/json")
                    .header("Authorization", auth_header)
                    .json(request),
            )
            .send()
            .await
            .context("Failed to send observation")?;

        Ok(response.status().as_u16())
    }

    /// Fetch conformance status for a specific aircraft.
    pub async fn fetch_conformance_status(
        &self,
        aircraft_id: &str,
    ) -> Result<BlenderConformanceStatusResponse> {
        let url = format!(
            "{}/conformance_monitoring_operations/conformance_status/?aircraft_id={}",
            self.base_url, aircraft_id
        );

        let auth_header = self.auth_header();

        let response = self
            .apply_request_id(self.client.get(&url).header("Authorization", auth_header))
            .send()
            .await
            .context("Failed to fetch conformance status")?;

        if !response.status().is_success() {
            let status = response.status();
            let body = response.text().await.unwrap_or_default();
            return Err(anyhow::anyhow!(
                "Conformance status request failed: {} {}",
                status,
                body
            ));
        }

        let payload = response
            .json::<BlenderConformanceStatusResponse>()
            .await
            .context("Failed to parse conformance status response")?;

        Ok(payload)
    }

    /// Create a DSS Remote ID subscription for a viewport.
    pub async fn create_rid_subscription(&self, view: &str) -> Result<String> {
        let url = format!("{}/rid/create_dss_subscription", self.base_url);

        let auth_header = self.auth_header();

        let response = self
            .apply_request_id(
                self.client
                    .put(&url)
                    .header("Authorization", auth_header)
                    .query(&[("view", view)]),
            )
            .send()
            .await
            .context("Failed to create RID subscription")?;

        let status = response.status();
        let payload: Value = response
            .json()
            .await
            .context("Failed to parse RID subscription response")?;

        if !status.is_success() {
            return Err(anyhow::anyhow!(
                "RID subscription request failed: {} {}",
                status,
                payload
            ));
        }

        let subscription_id = payload
            .get("dss_subscription_response")
            .and_then(|value| value.get("dss_subscription_id"))
            .and_then(|value| value.as_str())
            .or_else(|| {
                payload
                    .get("dss_subscription_id")
                    .and_then(|value| value.as_str())
            })
            .ok_or_else(|| anyhow::anyhow!("RID subscription missing ID"))?;

        Ok(subscription_id.to_string())
    }

    /// Fetch RID data for a DSS subscription.
    pub async fn fetch_rid_data(&self, subscription_id: &str) -> Result<Value> {
        let url = format!("{}/rid/get_rid_data/{}", self.base_url, subscription_id);

        let auth_header = self.auth_header();

        let response = self
            .apply_request_id(self.client.get(&url).header("Authorization", auth_header))
            .send()
            .await
            .context("Failed to fetch RID data")?;

        if response.status().as_u16() == 404 {
            return Ok(Value::Array(Vec::new()));
        }

        if !response.status().is_success() {
            let status = response.status();
            let body = response.text().await.unwrap_or_default();
            return Err(anyhow::anyhow!(
                "RID data request failed: {} {}",
                status,
                body
            ));
        }

        let payload: Value = response
            .json()
            .await
            .context("Failed to parse RID data response")?;

        Ok(payload)
    }

    /// Create a geofence in Flight Blender and return its ID.
    pub async fn create_geofence(&self, payload: &Value) -> Result<String> {
        let url = format!("{}/geo_fence_ops/set_geo_fence", self.base_url);
        let auth_header = self.auth_header();

        let response = self
            .apply_request_id(
                self.client
                    .put(&url)
                    .header("Content-Type", "application/json")
                    .header("Authorization", auth_header)
                    .json(payload),
            )
            .send()
            .await
            .context("Failed to create geofence")?;

        let status = response.status();
        let body: Value = response
            .json()
            .await
            .context("Failed to parse geofence response")?;

        if !status.is_success() {
            return Err(anyhow::anyhow!(
                "Geofence create failed: {} {}",
                status,
                body
            ));
        }

        let geofence_id = body
            .get("id")
            .and_then(|value| value.as_str())
            .ok_or_else(|| anyhow::anyhow!("Geofence response missing ID"))?;

        Ok(geofence_id.to_string())
    }

    /// Fetch geofences from Flight Blender (optionally filtered by view bbox).
    pub async fn fetch_geofences(&self, view: Option<&str>) -> Result<Vec<Value>> {
        let auth_header = self.auth_header();
        let mut results: Vec<Value> = Vec::new();
        let mut next_url = Some(format!("{}/geo_fence_ops/geo_fence", self.base_url));
        let mut first = true;

        while let Some(url) = next_url {
            let mut request = self
                .client
                .get(&url)
                .header("Authorization", auth_header.clone());

            if first {
                if let Some(view) = view {
                    if !view.trim().is_empty() {
                        request = request.query(&[("view", view)]);
                    }
                }
                first = false;
            }

            let response = self
                .apply_request_id(request)
                .send()
                .await
                .context("Failed to fetch geofences")?;

            if !response.status().is_success() {
                let status = response.status();
                let body = response.text().await.unwrap_or_default();
                return Err(anyhow::anyhow!(
                    "Geofence fetch failed: {} {}",
                    status,
                    body
                ));
            }

            let payload: Value = response
                .json()
                .await
                .context("Failed to parse geofence response")?;

            if let Some(entries) = payload.as_array() {
                results.extend(entries.iter().cloned());
                next_url = None;
                continue;
            }

            if let Some(entries) = payload.get("results").and_then(|v| v.as_array()) {
                results.extend(entries.iter().cloned());
                next_url = payload
                    .get("next")
                    .and_then(|v| v.as_str())
                    .map(|s| s.to_string());
                continue;
            }

            next_url = None;
        }

        Ok(results)
    }

    /// Fetch flight declarations from Flight Blender.
    pub async fn fetch_flight_declarations(&self) -> Result<Vec<Value>> {
        let auth_header = self.auth_header();
        let mut results: Vec<Value> = Vec::new();
        let mut next_url = Some(format!(
            "{}/flight_declaration_ops/flight_declaration",
            self.base_url
        ));

        while let Some(url) = next_url {
            let response = self
                .apply_request_id(
                    self.client
                        .get(&url)
                        .header("Authorization", auth_header.clone()),
                )
                .send()
                .await
                .context("Failed to fetch flight declarations")?;

            if !response.status().is_success() {
                let status = response.status();
                let body = response.text().await.unwrap_or_default();
                return Err(anyhow::anyhow!(
                    "Flight declarations fetch failed: {} {}",
                    status,
                    body
                ));
            }

            let payload: Value = response
                .json()
                .await
                .context("Failed to parse flight declarations response")?;

            if let Some(entries) = payload.as_array() {
                results.extend(entries.iter().cloned());
                next_url = None;
                continue;
            }

            if let Some(entries) = payload.get("results").and_then(|v| v.as_array()) {
                results.extend(entries.iter().cloned());
                next_url = payload
                    .get("next")
                    .and_then(|v| v.as_str())
                    .map(|s| s.to_string());
                continue;
            }

            next_url = None;
        }

        Ok(results)
    }

    /// Check whether a flight declaration exists in Flight Blender.
    pub async fn flight_declaration_exists(&self, declaration_id: &str) -> Result<bool> {
        let url = format!(
            "{}/flight_declaration_ops/flight_declaration/{}",
            self.base_url, declaration_id
        );
        let auth_header = self.auth_header();

        let response = self
            .apply_request_id(self.client.get(&url).header("Authorization", auth_header))
            .send()
            .await
            .context("Failed to fetch flight declaration")?;

        if response.status().as_u16() == 404 {
            return Ok(false);
        }

        if !response.status().is_success() {
            let status = response.status();
            let body = response.text().await.unwrap_or_default();
            return Err(anyhow::anyhow!(
                "Flight declaration lookup failed: {} {}",
                status,
                body
            ));
        }

        Ok(true)
    }

    /// Delete a geofence in Flight Blender by ID.
    pub async fn delete_geofence(&self, geofence_id: &str) -> Result<()> {
        let url = format!(
            "{}/geo_fence_ops/geo_fence/{}/delete",
            self.base_url, geofence_id
        );
        let auth_header = self.auth_header();

        let response = self
            .apply_request_id(
                self.client
                    .delete(&url)
                    .header("Authorization", auth_header),
            )
            .send()
            .await
            .context("Failed to delete geofence")?;

        if response.status().as_u16() == 404 {
            return Ok(());
        }

        if !response.status().is_success() {
            let status = response.status();
            let body = response.text().await.unwrap_or_default();
            return Err(anyhow::anyhow!(
                "Geofence delete failed: {} {}",
                status,
                body
            ));
        }

        Ok(())
    }
}
