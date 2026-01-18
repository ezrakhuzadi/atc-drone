//! REST API routes.

use axum::{
    extract::{State, Query},
    http::StatusCode,
    routing::{get, post, delete, put},
    Json, Router,
};
use serde::{Deserialize, Serialize};
use std::collections::HashSet;
use std::sync::Arc;

use crate::api::{commands, flights, geofences, daa, ws};
use crate::state::{AppState, ExternalTraffic};
use atc_core::models::{Telemetry, ConformanceStatus, DroneStatus};

/// Create the API router.
pub fn create_router() -> Router<Arc<AppState>> {
    Router::new()
        .route("/v1/drones/register", post(register_drone))
        .route("/v1/telemetry", post(receive_telemetry))
        .route("/v1/drones", get(list_drones))
        .route("/v1/traffic", get(list_traffic))
        .route("/v1/conflicts", get(list_conflicts))
        .route("/v1/conformance", get(list_conformance))
        .route("/v1/daa", get(daa::list_daa))
        .route("/v1/rid/view", post(update_rid_view))
        .route("/v1/flights/plan", post(flights::create_flight_plan))
        .route("/v1/flights", get(flights::get_flight_plans).post(flights::create_flight_plan_compat))
        // Command dispatch routes
        .route("/v1/commands", post(commands::issue_command))
        .route("/v1/commands", get(commands::get_all_commands))
        .route("/v1/commands/next", get(commands::get_next_command))
        .route("/v1/commands/ack", post(commands::ack_command))
        // Geofence routes
        .route("/v1/geofences", post(geofences::create_geofence))
        .route("/v1/geofences", get(geofences::list_geofences))
        .route("/v1/geofences/:id", get(geofences::get_geofence))
        .route("/v1/geofences/:id", put(geofences::update_geofence))
        .route("/v1/geofences/:id", delete(geofences::delete_geofence))
        .route("/v1/geofences/check", get(geofences::check_point))
        .route("/v1/geofences/check-route", post(geofences::check_route))
        // WebSocket streaming
        .route("/v1/ws", get(ws::ws_handler))
        // Admin routes
        .route("/v1/admin/reset", post(admin_reset))
}

// === Request/Response types ===

#[derive(Debug, Deserialize)]
pub struct RegisterRequest {
    pub drone_id: Option<String>,
    /// Owner/operator ID for user-specific tracking
    pub owner_id: Option<String>,
    #[allow(dead_code)] // Reserved for future drone type handling
    pub drone_type: Option<String>,
}

#[derive(Debug, Deserialize)]
pub struct ListDronesQuery {
    /// Filter drones by owner ID
    pub owner_id: Option<String>,
}

#[derive(Debug, Deserialize)]
pub struct ConformanceQuery {
    /// Filter conformance statuses by owner ID
    pub owner_id: Option<String>,
}

#[derive(Debug, Deserialize)]
pub struct TrafficQuery {
    /// Filter drones by owner ID
    pub owner_id: Option<String>,
    /// Include external RID/DSS tracks
    pub include_external: Option<bool>,
}

#[derive(Debug, Serialize)]
pub struct TrafficState {
    pub drone_id: String,
    pub owner_id: Option<String>,
    pub lat: f64,
    pub lon: f64,
    pub altitude_m: f64,
    pub heading_deg: f64,
    pub speed_mps: f64,
    pub last_update: chrono::DateTime<chrono::Utc>,
    pub status: DroneStatus,
    pub traffic_source: String,
}

#[derive(Debug, Deserialize)]
pub struct ConflictQuery {
    /// Filter conflicts by owner ID
    pub owner_id: Option<String>,
}

#[derive(Debug, Deserialize)]
pub struct RidViewRequest {
    pub min_lat: f64,
    pub min_lon: f64,
    pub max_lat: f64,
    pub max_lon: f64,
}

#[derive(Debug, Serialize)]
pub struct RidViewResponse {
    pub view: String,
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

    state.register_drone(&drone_id, req.owner_id.clone());

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
    Query(query): Query<ListDronesQuery>,
) -> Json<Vec<atc_core::models::DroneState>> {
    let all_drones = state.get_all_drones();
    
    // Filter by owner_id if provided
    let filtered = if let Some(owner_id) = query.owner_id {
        all_drones.into_iter()
            .filter(|d| d.owner_id.as_ref() == Some(&owner_id))
            .collect()
    } else {
        all_drones
    };
    
    Json(filtered)
}

async fn list_traffic(
    State(state): State<Arc<AppState>>,
    Query(query): Query<TrafficQuery>,
) -> Json<Vec<TrafficState>> {
    let mut traffic: Vec<TrafficState> = Vec::new();

    let drones = state.get_all_drones();
    let filtered = if let Some(owner_id) = query.owner_id.clone() {
        drones
            .into_iter()
            .filter(|d| d.owner_id.as_ref() == Some(&owner_id))
            .collect::<Vec<_>>()
    } else {
        drones
    };

    traffic.extend(filtered.into_iter().map(|drone| TrafficState {
        drone_id: drone.drone_id,
        owner_id: drone.owner_id,
        lat: drone.lat,
        lon: drone.lon,
        altitude_m: drone.altitude_m,
        heading_deg: drone.heading_deg,
        speed_mps: drone.speed_mps,
        last_update: drone.last_update,
        status: drone.status,
        traffic_source: "local".to_string(),
    }));

    if query.include_external.unwrap_or(false) {
        let external = state.get_external_traffic();
        traffic.extend(external.into_iter().map(external_to_traffic));
    }

    Json(traffic)
}

fn external_to_traffic(external: ExternalTraffic) -> TrafficState {
    TrafficState {
        drone_id: external.traffic_id,
        owner_id: None,
        lat: external.lat,
        lon: external.lon,
        altitude_m: external.altitude_m,
        heading_deg: external.heading_deg,
        speed_mps: external.speed_mps,
        last_update: external.last_update,
        status: DroneStatus::Active,
        traffic_source: external.source,
    }
}

async fn list_conflicts(
    State(state): State<Arc<AppState>>,
    Query(query): Query<ConflictQuery>,
) -> Json<Vec<atc_core::Conflict>> {
    let conflicts = state.get_conflicts();

    if let Some(owner_id) = query.owner_id {
        let owner_drone_ids: HashSet<String> = state.get_all_drones()
            .into_iter()
            .filter(|drone| drone.owner_id.as_ref() == Some(&owner_id))
            .map(|drone| drone.drone_id)
            .collect();
        let filtered = conflicts
            .into_iter()
            .filter(|conflict| {
                owner_drone_ids.contains(&conflict.drone1_id)
                    || owner_drone_ids.contains(&conflict.drone2_id)
            })
            .collect();
        return Json(filtered);
    }

    Json(conflicts)
}

async fn list_conformance(
    State(state): State<Arc<AppState>>,
    Query(query): Query<ConformanceQuery>,
) -> Json<Vec<ConformanceStatus>> {
    let mut statuses = state.get_conformance_statuses();

    if let Some(owner_id) = query.owner_id {
        statuses.retain(|status| status.owner_id.as_ref() == Some(&owner_id));
    }

    Json(statuses)
}

async fn update_rid_view(
    State(state): State<Arc<AppState>>,
    Json(req): Json<RidViewRequest>,
) -> Result<Json<RidViewResponse>, (StatusCode, Json<serde_json::Value>)> {
    if req.min_lat >= req.max_lat || req.min_lon >= req.max_lon {
        return Err((
            StatusCode::BAD_REQUEST,
            Json(serde_json::json!({
                "error": "Invalid bounding box",
                "details": "min_lat/min_lon must be less than max_lat/max_lon"
            }))
        ));
    }
    if !(req.min_lat >= -90.0 && req.max_lat <= 90.0 && req.min_lon >= -180.0 && req.max_lon <= 180.0) {
        return Err((
            StatusCode::BAD_REQUEST,
            Json(serde_json::json!({
                "error": "Bounding box out of range",
                "details": "lat must be within [-90, 90], lon within [-180, 180]"
            }))
        ));
    }

    let view = format!("{},{},{},{}", req.min_lat, req.min_lon, req.max_lat, req.max_lon);
    state.set_rid_view_bbox(view.clone());

    Ok(Json(RidViewResponse { view }))
}

// === Admin Handlers ===

/// Reset all state for demo purposes.
async fn admin_reset(
    State(state): State<Arc<AppState>>,
) -> StatusCode {
    state.clear_all();
    StatusCode::OK
}
