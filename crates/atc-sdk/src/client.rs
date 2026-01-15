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

#[derive(Debug, Serialize)]
struct AckRequest {
    command_id: String,
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

    /// Create a flight plan for the drone.
    pub async fn create_flight_plan(&self, request: &atc_core::models::FlightPlanRequest) -> Result<atc_core::models::FlightPlan> {
        let url = format!("{}/v1/flights/plan", self.base_url);
        
        // Ensure the request has the correct drone_id
        let mut req = request.clone();
        if let Some(drone_id) = &self.drone_id {
            req.drone_id = drone_id.clone();
        }

        let response: atc_core::models::FlightPlan = self
            .client
            .post(&url)
            .json(&req)
            .send()
            .await?
            .json()
            .await?;

        Ok(response)
    }

    // ========== COMMAND HANDLING ==========

    /// Poll for the next pending command for this drone.
    /// Returns None if no command is pending.
    pub async fn get_next_command(&self) -> Result<Option<atc_core::models::Command>> {
        let drone_id = self.drone_id.as_ref()
            .ok_or_else(|| anyhow::anyhow!("Drone not registered"))?;
        
        let url = format!("{}/v1/commands/next?drone_id={}", self.base_url, drone_id);
        
        let response: Option<atc_core::models::Command> = self
            .client
            .get(&url)
            .send()
            .await?
            .json()
            .await?;

        Ok(response)
    }

    /// Acknowledge a command by ID (marks it as executed).
    pub async fn ack_command(&self, command_id: &str) -> Result<()> {
        let url = format!("{}/v1/commands/ack", self.base_url);
        
        let response = self
            .client
            .post(&url)
            .json(&AckRequest { command_id: command_id.to_string() })
            .send()
            .await?;
            
        if !response.status().is_success() {
            anyhow::bail!("Failed to ack command: {}", response.status());
        }

        Ok(())
    }
}

