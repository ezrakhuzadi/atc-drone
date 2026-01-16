//! REST API routes.

use axum::{
    extract::State,
    http::StatusCode,
    routing::{get, post, delete},
    Json, Router,
};
use serde::{Deserialize, Serialize};
use std::sync::Arc;

use crate::api::{commands, flights, geofences};
use crate::state::AppState;
use atc_core::models::Telemetry;

/// Create the API router.
pub fn create_router() -> Router<Arc<AppState>> {
    Router::new()
        .route("/v1/drones/register", post(register_drone))
        .route("/v1/telemetry", post(receive_telemetry))
        .route("/v1/drones", get(list_drones))
        .route("/v1/conflicts", get(list_conflicts))
        .route("/v1/flights/plan", post(flights::create_flight_plan))
        .route("/v1/flights", get(flights::get_flight_plans))
        // Command dispatch routes
        .route("/v1/commands", post(commands::issue_command))
        .route("/v1/commands", get(commands::get_all_commands))
        .route("/v1/commands/next", get(commands::get_next_command))
        .route("/v1/commands/ack", post(commands::ack_command))
        // Geofence routes
        .route("/v1/geofences", post(geofences::create_geofence))
        .route("/v1/geofences", get(geofences::list_geofences))
        .route("/v1/geofences/:id", get(geofences::get_geofence))
        .route("/v1/geofences/:id", delete(geofences::delete_geofence))
        .route("/v1/geofences/check", get(geofences::check_point))
        .route("/v1/geofences/check-route", post(geofences::check_route))
        // Admin routes
        .route("/v1/admin/reset", post(admin_reset))
}

// === Request/Response types ===

#[derive(Debug, Deserialize)]
pub struct RegisterRequest {
    pub drone_id: Option<String>,
    #[allow(dead_code)] // Reserved for future drone type handling
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

// === Admin Handlers ===

/// Reset all state for demo purposes.
async fn admin_reset(
    State(state): State<Arc<AppState>>,
) -> StatusCode {
    state.clear_all();
    StatusCode::OK
}

