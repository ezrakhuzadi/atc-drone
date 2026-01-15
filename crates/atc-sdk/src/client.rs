//! ATC SDK client for drone registration and communication.

use anyhow::Result;
use serde::{Deserialize, Serialize};

/// Client for connecting to the ATC server.
pub struct AtcClient {
    pub(crate) base_url: String,
    pub(crate) drone_id: Option<String>,
    pub(crate) client: reqwest::Client,
}

#[derive(Debug, Serialize)]
pub struct RegisterRequest {
    pub drone_id: Option<String>,
    pub drone_type: String,
}

#[derive(Debug, Deserialize)]
pub struct RegisterResponse {
    pub drone_id: String,
    pub session_token: String,
}

impl AtcClient {
    /// Create a new ATC client.
    pub fn new(base_url: impl Into<String>) -> Self {
        Self {
            base_url: base_url.into(),
            drone_id: None,
            client: reqwest::Client::new(),
        }
    }

    /// Register this drone with the ATC server.
    pub async fn register(&mut self, drone_id: Option<&str>) -> Result<RegisterResponse> {
        let url = format!("{}/v1/drones/register", self.base_url);
        
        let request = RegisterRequest {
            drone_id: drone_id.map(|s| s.to_string()),
            drone_type: "UAV".to_string(),
        };

        let response: RegisterResponse = self
            .client
            .post(&url)
            .json(&request)
            .send()
            .await?
            .json()
            .await?;

        self.drone_id = Some(response.drone_id.clone());
        Ok(response)
    }

    /// Get the current drone ID.
    pub fn drone_id(&self) -> Option<&str> {
        self.drone_id.as_deref()
    }

    /// Send telemetry update to the ATC server.
    pub async fn send_telemetry(&self, telemetry: &atc_core::models::Telemetry) -> Result<()> {
        let url = format!("{}/v1/telemetry", self.base_url);
        
        let response = self
            .client
            .post(&url)
            .json(telemetry)
            .send()
            .await?;
            
        if !response.status().is_success() {
            anyhow::bail!("Failed to send telemetry: {}", response.status());
        }

        Ok(())
    }
}
