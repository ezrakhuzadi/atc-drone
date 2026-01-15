//! REST API routes.

use axum::{
    extract::State,
    http::StatusCode,
    routing::{get, post},
    Json, Router,
};
use serde::{Deserialize, Serialize};
use std::sync::Arc;

use crate::state::AppState;
use atc_core::models::Telemetry;

/// Create the API router.
pub fn create_router() -> Router {
    let state = Arc::new(AppState::new());

    Router::new()
        .route("/v1/drones/register", post(register_drone))
        .route("/v1/telemetry", post(receive_telemetry))
        .route("/v1/drones", get(list_drones))
        .route("/v1/conflicts", get(list_conflicts))
        .with_state(state)
}

// === Request/Response types ===

#[derive(Debug, Deserialize)]
pub struct RegisterRequest {
    pub drone_id: Option<String>,
    pub drone_type: Option<String>,
}

#[derive(Debug, Serialize)]
pub struct RegisterResponse {
    pub drone_id: String,
    pub session_token: String,
}

// === Handlers ===

async fn register_drone(
    State(state): State<Arc<AppState>>,
    Json(req): Json<RegisterRequest>,
) -> (StatusCode, Json<RegisterResponse>) {
    let drone_id = req.drone_id.unwrap_or_else(|| {
        format!("DRONE{:04}", state.next_drone_id())
    });

    state.register_drone(&drone_id);

    (
        StatusCode::CREATED,
        Json(RegisterResponse {
            drone_id,
            session_token: "dummy-token".to_string(),
        }),
    )
}

async fn receive_telemetry(
    State(state): State<Arc<AppState>>,
    Json(telemetry): Json<Telemetry>,
) -> StatusCode {
    state.update_telemetry(telemetry);
    StatusCode::ACCEPTED
}

async fn list_drones(
    State(state): State<Arc<AppState>>,
) -> Json<Vec<atc_core::models::DroneState>> {
    Json(state.get_all_drones())
}

async fn list_conflicts(
    State(state): State<Arc<AppState>>,
) -> Json<Vec<atc_core::Conflict>> {
    Json(state.get_conflicts())
}
