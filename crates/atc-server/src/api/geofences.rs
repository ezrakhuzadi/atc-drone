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
use atc_core::{Geofence, CreateGeofenceRequest, UpdateGeofenceRequest};

/// Create a new geofence.
pub async fn create_geofence(
    State(state): State<Arc<AppState>>,
    Json(req): Json<CreateGeofenceRequest>,
) -> Result<(StatusCode, Json<Geofence>), (StatusCode, Json<serde_json::Value>)> {
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
    
    // Validate geofence before saving
    let errors = geofence.validate();
    if !errors.is_empty() {
        return Err((
            StatusCode::BAD_REQUEST,
            Json(serde_json::json!({
                "error": "Invalid geofence",
                "validation_errors": errors
            }))
        ));
    }
    
    state.add_geofence(geofence.clone());
    tracing::info!("Created geofence '{}' ({})", geofence.name, geofence.id);
    
    Ok((StatusCode::CREATED, Json(geofence)))
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

/// Update a geofence by ID.
pub async fn update_geofence(
    State(state): State<Arc<AppState>>,
    Path(id): Path<String>,
    Json(req): Json<UpdateGeofenceRequest>,
) -> Result<Json<Geofence>, (StatusCode, Json<serde_json::Value>)> {
    let mut geofence = match state.get_geofence(&id) {
        Some(existing) => existing,
        None => {
            return Err((
                StatusCode::NOT_FOUND,
                Json(serde_json::json!({
                    "error": "Geofence not found",
                    "id": id
                }))
            ));
        }
    };

    if let Some(name) = req.name {
        geofence.name = name;
    }
    if let Some(geofence_type) = req.geofence_type {
        geofence.geofence_type = geofence_type;
    }
    if let Some(polygon) = req.polygon {
        geofence.polygon = polygon;
    }
    if let Some(lower_altitude_m) = req.lower_altitude_m {
        geofence.lower_altitude_m = lower_altitude_m;
    }
    if let Some(upper_altitude_m) = req.upper_altitude_m {
        geofence.upper_altitude_m = upper_altitude_m;
    }
    if let Some(active) = req.active {
        geofence.active = active;
    }

    let errors = geofence.validate();
    if !errors.is_empty() {
        return Err((
            StatusCode::BAD_REQUEST,
            Json(serde_json::json!({
                "error": "Invalid geofence",
                "validation_errors": errors
            }))
        ));
    }

    state.add_geofence(geofence.clone());
    tracing::info!("Updated geofence '{}' ({})", geofence.name, geofence.id);

    Ok(Json(geofence))
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

/// Check if a route conflicts with any active geofences.
#[derive(serde::Deserialize)]
pub struct RouteCheckRequest {
    pub waypoints: Vec<atc_core::Waypoint>,
}

#[derive(serde::Serialize)]
pub struct RouteCheckResponse {
    pub conflicts: bool,
    pub conflicting_geofences: Vec<GeofenceConflict>,
}

#[derive(serde::Serialize)]
pub struct GeofenceConflict {
    pub geofence_id: String,
    pub geofence_name: String,
    pub segment_index: usize,
}

pub async fn check_route(
    State(state): State<Arc<AppState>>,
    Json(req): Json<RouteCheckRequest>,
) -> Json<RouteCheckResponse> {
    let geofences = state.get_geofences();
    let mut conflicts = Vec::new();
    
    // Check each segment of the route against all active geofences
    for i in 0..req.waypoints.len().saturating_sub(1) {
        let wp1 = &req.waypoints[i];
        let wp2 = &req.waypoints[i + 1];
        
        for geofence in geofences.iter().filter(|g| g.active) {
            if geofence.intersects_segment(
                wp1.lat, wp1.lon, wp1.altitude_m,
                wp2.lat, wp2.lon, wp2.altitude_m,
            ) {
                conflicts.push(GeofenceConflict {
                    geofence_id: geofence.id.clone(),
                    geofence_name: geofence.name.clone(),
                    segment_index: i,
                });
            }
        }
    }
    
    Json(RouteCheckResponse {
        conflicts: !conflicts.is_empty(),
        conflicting_geofences: conflicts,
    })
}
