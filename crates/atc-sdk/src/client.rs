//! ATC SDK client for drone registration and communication.

use anyhow::Result;
use futures_util::StreamExt;
use reqwest::Url;
use serde::{Deserialize, Serialize};
use tokio::net::TcpStream;
use tokio_tungstenite::tungstenite::client::IntoClientRequest;
use tokio_tungstenite::tungstenite::http::HeaderValue;
use tokio_tungstenite::tungstenite::Message;
use tokio_tungstenite::{connect_async, MaybeTlsStream, WebSocketStream};

/// Client for connecting to the ATC server.
pub struct AtcClient {
    pub(crate) base_url: String,
    pub(crate) drone_id: Option<String>,
    pub(crate) owner_id: Option<String>,
    pub(crate) session_token: Option<String>,
    pub(crate) registration_token: Option<String>,
    pub(crate) client: reqwest::Client,
}

#[derive(Debug, Serialize)]
pub struct RegisterRequest {
    pub drone_id: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub owner_id: Option<String>,
    pub drone_type: String,
}

#[derive(Debug, Deserialize)]
pub struct RegisterResponse {
    pub drone_id: String,
    pub session_token: String,
}

/// WebSocket command stream for a drone.
pub struct CommandStream {
    socket: WebSocketStream<MaybeTlsStream<TcpStream>>,
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
            owner_id: None,
            session_token: None,
            registration_token: None,
            client: reqwest::Client::new(),
        }
    }

    /// Set the shared registration token for /v1/drones/register.
    pub fn set_registration_token(&mut self, token: Option<String>) {
        self.registration_token = token;
    }

    /// Register this drone with the ATC server.
    pub async fn register(&mut self, drone_id: Option<&str>) -> Result<RegisterResponse> {
        self.register_with_owner(drone_id, None).await
    }

    /// Register this drone with the ATC server, including an owner ID.
    pub async fn register_with_owner(
        &mut self,
        drone_id: Option<&str>,
        owner_id: Option<&str>,
    ) -> Result<RegisterResponse> {
        let url = format!("{}/v1/drones/register", self.base_url);

        let request = RegisterRequest {
            drone_id: drone_id.map(|s| s.to_string()),
            owner_id: owner_id.map(|s| s.to_string()),
            drone_type: "UAV".to_string(),
        };

        let mut builder = self.client.post(&url).json(&request);
        if let Some(token) = self.registration_token.as_deref() {
            builder = builder.header("X-Registration-Token", token);
        }

        let response: RegisterResponse = builder.send().await?.json().await?;

        self.drone_id = Some(response.drone_id.clone());
        if let Some(owner_id) = owner_id {
            self.owner_id = Some(owner_id.to_string());
        }
        self.session_token = Some(response.session_token.clone());
        Ok(response)
    }

    /// Get the current drone ID.
    pub fn drone_id(&self) -> Option<&str> {
        self.drone_id.as_deref()
    }

    /// Get the owner ID associated with this client.
    pub fn owner_id(&self) -> Option<&str> {
        self.owner_id.as_deref()
    }

    /// Get the current session token.
    pub fn session_token(&self) -> Option<&str> {
        self.session_token.as_deref()
    }

    /// Set or update the owner ID for this client.
    pub fn set_owner_id(&mut self, owner_id: Option<String>) {
        self.owner_id = owner_id;
    }

    /// Send telemetry update to the ATC server.
    pub async fn send_telemetry(&self, telemetry: &atc_core::models::Telemetry) -> Result<()> {
        let url = format!("{}/v1/telemetry", self.base_url);
        let auth = self
            .session_token
            .as_deref()
            .ok_or_else(|| anyhow::anyhow!("Drone not registered"))?;
        
        let response = self
            .client
            .post(&url)
            .header("Authorization", format!("Bearer {}", auth))
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
        let auth = self
            .session_token
            .as_deref()
            .ok_or_else(|| anyhow::anyhow!("Drone not registered"))?;
        
        let url = format!("{}/v1/commands/next?drone_id={}", self.base_url, drone_id);
        
        let response: Option<atc_core::models::Command> = self
            .client
            .get(&url)
            .header("Authorization", format!("Bearer {}", auth))
            .send()
            .await?
            .json()
            .await?;

        Ok(response)
    }

    /// Acknowledge a command by ID (marks it as executed).
    pub async fn ack_command(&self, command_id: &str) -> Result<()> {
        let url = format!("{}/v1/commands/ack", self.base_url);
        let auth = self
            .session_token
            .as_deref()
            .ok_or_else(|| anyhow::anyhow!("Drone not registered"))?;
        
        let response = self
            .client
            .post(&url)
            .header("Authorization", format!("Bearer {}", auth))
            .json(&AckRequest { command_id: command_id.to_string() })
            .send()
            .await?;
            
        if !response.status().is_success() {
            anyhow::bail!("Failed to ack command: {}", response.status());
        }

        Ok(())
    }

    /// Connect to the command WebSocket stream.
    pub async fn connect_command_stream(&self) -> Result<CommandStream> {
        let drone_id = self
            .drone_id
            .as_deref()
            .ok_or_else(|| anyhow::anyhow!("Drone not registered"))?;
        let auth = self
            .session_token
            .as_deref()
            .ok_or_else(|| anyhow::anyhow!("Drone not registered"))?;

        let url = build_ws_url(&self.base_url, "/v1/commands/ws", drone_id)?;
        let mut request = url.as_str().into_client_request()?;
        request.headers_mut().insert(
            "Authorization",
            HeaderValue::from_str(&format!("Bearer {}", auth))?,
        );

        let (socket, _) = connect_async(request).await?;
        Ok(CommandStream { socket })
    }
}

impl CommandStream {
    /// Read the next command from the stream (returns None on close).
    pub async fn next_command(&mut self) -> Result<Option<atc_core::models::Command>> {
        while let Some(msg) = self.socket.next().await {
            let msg = msg?;
            match msg {
                Message::Text(text) => {
                    let command = serde_json::from_str(&text)?;
                    return Ok(Some(command));
                }
                Message::Binary(data) => {
                    if let Ok(text) = String::from_utf8(data) {
                        if let Ok(command) = serde_json::from_str(&text) {
                            return Ok(Some(command));
                        }
                    }
                }
                Message::Close(_) => return Ok(None),
                _ => {}
            }
        }
        Ok(None)
    }
}

fn build_ws_url(base: &str, path: &str, drone_id: &str) -> Result<Url> {
    let mut url = Url::parse(base)?;
    let scheme = match url.scheme() {
        "http" => "ws",
        "https" => "wss",
        other => other,
    }.to_string();
    
    url.set_scheme(&scheme)
        .map_err(|_| anyhow::anyhow!("Invalid base URL scheme"))?;
    url.set_path(path);
    url.query_pairs_mut().append_pair("drone_id", drone_id);
    Ok(url)
}
