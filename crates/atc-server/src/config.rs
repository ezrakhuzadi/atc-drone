//! Server configuration from environment.

use std::env;

#[derive(Debug, Clone)]
pub struct Config {
    pub server_port: u16,
    pub blender_url: String,
    pub blender_session_id: String,
    pub rid_view_bbox: String,
    pub geofence_sync_state_path: String,
    pub blender_auth_token: String,
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
            rid_view_bbox: env::var("RID_VIEW_BBOX")
                .unwrap_or_else(|_| "33.654600,-117.856500,33.714600,-117.796500".to_string()),
            geofence_sync_state_path: env::var("GEOFENCE_SYNC_STATE_PATH")
                .unwrap_or_else(|_| "data/geofence_sync.json".to_string()),
            blender_auth_token: env::var("BLENDER_AUTH_TOKEN").unwrap_or_default(),
        }
    }
}
