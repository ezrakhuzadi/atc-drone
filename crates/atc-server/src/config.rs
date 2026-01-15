//! Server configuration from environment.

use std::env;

#[derive(Debug, Clone)]
pub struct Config {
    pub server_port: u16,
    pub blender_url: String,
    pub blender_session_id: String,
}

impl Config {
    pub fn from_env() -> Self {
        Self {
            server_port: env::var("ATC_PORT")
                .ok()
                .and_then(|s| s.parse().ok())
                .unwrap_or(3000),
            blender_url: env::var("BLENDER_URL")
                .unwrap_or_else(|_| "http://localhost:8000".to_string()),
            blender_session_id: env::var("BLENDER_SESSION_ID")
                .unwrap_or_else(|_| "00000000-0000-0000-0000-000000000001".to_string()),
        }
    }
}
