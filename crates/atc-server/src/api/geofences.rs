//! Geofence API endpoints.
//! 
//! Provides CRUD operations for no-fly zones and restricted areas.

use axum::{
    extract::{Path, State},
    http::StatusCode,
    Json,
};
use std::sync::Arc;
use chrono::Utc;
use uuid::Uuid;

use crate::state::AppState;
use atc_core::{Geofence, CreateGeofenceRequest};

/// Create a new geofence.
pub async fn create_geofence(
    State(state): State<Arc<AppState>>,
    Json(req): Json<CreateGeofenceRequest>,
) -> (StatusCode, Json<Geofence>) {
    let geofence = Geofence {
        id: Uuid::new_v4().to_string(),
        name: req.name,
        geofence_type: req.geofence_type,
        polygon: req.polygon,
        lower_altitude_m: req.lower_altitude_m.unwrap_or(0.0),
        upper_altitude_m: req.upper_altitude_m.unwrap_or(120.0), // Default 120m ceiling
        active: true,
        created_at: Utc::now(),
    };
    
    state.add_geofence(geofence.clone());
    tracing::info!("Created geofence '{}' ({})", geofence.name, geofence.id);
    
    (StatusCode::CREATED, Json(geofence))
}

/// List all geofences.
pub async fn list_geofences(
    State(state): State<Arc<AppState>>,
) -> Json<Vec<Geofence>> {
    Json(state.get_geofences())
}

/// Get a specific geofence by ID.
pub async fn get_geofence(
    State(state): State<Arc<AppState>>,
    Path(id): Path<String>,
) -> Result<Json<Geofence>, StatusCode> {
    state.get_geofence(&id)
        .map(Json)
        .ok_or(StatusCode::NOT_FOUND)
}

/// Delete a geofence by ID.
pub async fn delete_geofence(
    State(state): State<Arc<AppState>>,
    Path(id): Path<String>,
) -> StatusCode {
    if state.remove_geofence(&id) {
        tracing::info!("Deleted geofence {}", id);
        StatusCode::NO_CONTENT
    } else {
        StatusCode::NOT_FOUND
    }
}

/// Check if a point is inside any active geofence.
#[derive(serde::Deserialize)]
pub struct PointCheckQuery {
    pub lat: f64,
    pub lon: f64,
    pub altitude_m: Option<f64>,
}

#[derive(serde::Serialize)]
pub struct PointCheckResponse {
    pub inside_geofence: bool,
    pub geofence_ids: Vec<String>,
}

pub async fn check_point(
    State(state): State<Arc<AppState>>,
    axum::extract::Query(query): axum::extract::Query<PointCheckQuery>,
) -> Json<PointCheckResponse> {
    let altitude = query.altitude_m.unwrap_or(50.0);
    let geofences = state.get_geofences();
    
    let matching: Vec<String> = geofences
        .iter()
        .filter(|g| g.active && g.contains_point(query.lat, query.lon, altitude))
        .map(|g| g.id.clone())
        .collect();
    
    Json(PointCheckResponse {
        inside_geofence: !matching.is_empty(),
        geofence_ids: matching,
    })
}
