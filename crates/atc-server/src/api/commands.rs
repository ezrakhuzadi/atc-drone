//! Command dispatch API endpoints.
//!
//! Allows ATC to issue commands to drones and drones to poll/ack them.

use axum::{
    extract::{Query, State},
    http::StatusCode,
    Json,
};
use chrono::{Duration, Utc};
use serde::{Deserialize, Serialize};
use std::sync::Arc;

use atc_core::models::{Command, CommandType};
use crate::state::AppState;

/// Request to issue a new command.
#[derive(Debug, Deserialize)]
pub struct IssueCommandRequest {
    pub drone_id: String,
    /// Owner/operator ID for access enforcement
    pub owner_id: Option<String>,
    #[serde(flatten)]
    pub command_type: CommandType,
    /// Command expiry in seconds (default: 60)
    pub expires_in_secs: Option<u32>,
}

/// Response after issuing a command.
#[derive(Debug, Serialize)]
pub struct IssueCommandResponse {
    pub command_id: String,
    pub drone_id: String,
    pub status: String,
}

/// Query params for getting next command.
#[derive(Debug, Deserialize)]
pub struct NextCommandQuery {
    pub drone_id: String,
}

/// Request to acknowledge a command.
#[derive(Debug, Deserialize)]
pub struct AckCommandRequest {
    pub command_id: String,
}

/// Issue a new command to a drone.
/// POST /v1/commands
pub async fn issue_command(
    State(state): State<Arc<AppState>>,
    Json(request): Json<IssueCommandRequest>,
) -> Result<Json<IssueCommandResponse>, StatusCode> {
    let drone = state.get_drone(&request.drone_id).ok_or(StatusCode::NOT_FOUND)?;
    if let Some(expected_owner) = drone.owner_id {
        if request.owner_id.as_deref() != Some(expected_owner.as_str()) {
            return Err(StatusCode::FORBIDDEN);
        }
    }

    let now = Utc::now();
    let expires_in = request.expires_in_secs.unwrap_or(60);
    
    let command = Command {
        command_id: format!("CMD-{}", uuid::Uuid::new_v4().to_string()[..8].to_uppercase()),
        drone_id: request.drone_id.clone(),
        command_type: request.command_type,
        issued_at: now,
        expires_at: Some(now + Duration::seconds(expires_in as i64)),
        acknowledged: false,
    };

    let command_id = command.command_id.clone();
    state.enqueue_command(command);
    state.mark_command_issued(&request.drone_id);

    tracing::info!("Issued command {} to drone {}", command_id, request.drone_id);

    Ok(Json(IssueCommandResponse {
        command_id,
        drone_id: request.drone_id,
        status: "queued".to_string(),
    }))
}

/// Get the next pending command for a drone.
/// GET /v1/commands/next?drone_id=DRONE001
pub async fn get_next_command(
    State(state): State<Arc<AppState>>,
    Query(query): Query<NextCommandQuery>,
) -> Result<Json<Option<Command>>, StatusCode> {
    let command = state.peek_command(&query.drone_id);
    Ok(Json(command))
}

/// Acknowledge (and remove) a command.
/// POST /v1/commands/ack
pub async fn ack_command(
    State(state): State<Arc<AppState>>,
    Json(request): Json<AckCommandRequest>,
) -> Result<Json<serde_json::Value>, StatusCode> {
    let found = state.ack_command(&request.command_id);
    
    if found {
        tracing::info!("Command {} acknowledged", request.command_id);
        Ok(Json(serde_json::json!({
            "status": "acknowledged",
            "command_id": request.command_id
        })))
    } else {
        Ok(Json(serde_json::json!({
            "status": "not_found",
            "command_id": request.command_id
        })))
    }
}

/// Get all pending commands (for debugging/UI).
/// GET /v1/commands
pub async fn get_all_commands(
    State(state): State<Arc<AppState>>,
) -> Json<Vec<Command>> {
    Json(state.get_all_pending_commands())
}
