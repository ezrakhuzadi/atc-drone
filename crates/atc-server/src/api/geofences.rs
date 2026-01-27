//! Geofence API endpoints.
//!
//! Provides CRUD operations for no-fly zones and restricted areas.

use axum::{
    extract::{Path, State},
    http::StatusCode,
    Json,
};
use chrono::Utc;
use std::sync::Arc;
use uuid::Uuid;

use crate::altitude::altitude_to_amsl;
use crate::state::AppState;
use atc_core::{CreateGeofenceRequest, Geofence, GeofenceType, UpdateGeofenceRequest};

/// Create a new geofence.
pub async fn create_geofence(
    State(state): State<Arc<AppState>>,
    Json(req): Json<CreateGeofenceRequest>,
) -> Result<(StatusCode, Json<Geofence>), (StatusCode, Json<serde_json::Value>)> {
    let config = state.config();
    let lower_altitude_m = altitude_to_amsl(
        req.lower_altitude_m.unwrap_or(0.0),
        config.altitude_reference,
        config.geoid_offset_m,
    );
    let upper_altitude_m = altitude_to_amsl(
        req.upper_altitude_m.unwrap_or(120.0),
        config.altitude_reference,
        config.geoid_offset_m,
    );
    let geofence = Geofence {
        id: Uuid::new_v4().to_string(),
        name: req.name,
        geofence_type: req.geofence_type,
        polygon: req.polygon,
        lower_altitude_m,
        upper_altitude_m, // Default 120m ceiling
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
            })),
        ));
    }

    if let Err(err) = state.add_geofence(geofence.clone()).await {
        tracing::error!("Failed to persist geofence {}: {}", geofence.id, err);
        return Err((
            StatusCode::INTERNAL_SERVER_ERROR,
            Json(serde_json::json!({
                "error": "Failed to create geofence",
                "id": geofence.id
            })),
        ));
    }
    tracing::info!("Created geofence '{}' ({})", geofence.name, geofence.id);

    Ok((StatusCode::CREATED, Json(geofence)))
}

/// List all geofences.
pub async fn list_geofences(State(state): State<Arc<AppState>>) -> Json<Vec<Geofence>> {
    Json(state.get_geofences())
}

/// Get a specific geofence by ID.
pub async fn get_geofence(
    State(state): State<Arc<AppState>>,
    Path(id): Path<String>,
) -> Result<Json<Geofence>, StatusCode> {
    state
        .get_geofence(&id)
        .map(Json)
        .ok_or(StatusCode::NOT_FOUND)
}

/// Update a geofence by ID.
pub async fn update_geofence(
    State(state): State<Arc<AppState>>,
    Path(id): Path<String>,
    Json(req): Json<UpdateGeofenceRequest>,
) -> Result<Json<Geofence>, (StatusCode, Json<serde_json::Value>)> {
    if state.is_external_geofence(&id) {
        return Err((
            StatusCode::FORBIDDEN,
            Json(serde_json::json!({
                "error": "External geofences are read-only",
                "id": id
            })),
        ));
    }

    let mut geofence = match state.get_geofence(&id) {
        Some(existing) => existing,
        None => {
            return Err((
                StatusCode::NOT_FOUND,
                Json(serde_json::json!({
                    "error": "Geofence not found",
                    "id": id
                })),
            ));
        }
    };

    let config = state.config();
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
        geofence.lower_altitude_m = altitude_to_amsl(
            lower_altitude_m,
            config.altitude_reference,
            config.geoid_offset_m,
        );
    }
    if let Some(upper_altitude_m) = req.upper_altitude_m {
        geofence.upper_altitude_m = altitude_to_amsl(
            upper_altitude_m,
            config.altitude_reference,
            config.geoid_offset_m,
        );
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
            })),
        ));
    }

    if let Err(err) = state.add_geofence(geofence.clone()).await {
        tracing::error!("Failed to persist geofence {}: {}", geofence.id, err);
        return Err((
            StatusCode::INTERNAL_SERVER_ERROR,
            Json(serde_json::json!({
                "error": "Failed to update geofence",
                "id": geofence.id
            })),
        ));
    }
    tracing::info!("Updated geofence '{}' ({})", geofence.name, geofence.id);

    Ok(Json(geofence))
}

/// Delete a geofence by ID.
pub async fn delete_geofence(
    State(state): State<Arc<AppState>>,
    Path(id): Path<String>,
) -> StatusCode {
    if state.is_external_geofence(&id) {
        return StatusCode::FORBIDDEN;
    }

    match state.remove_geofence(&id).await {
        Ok(true) => {
            tracing::info!("Deleted geofence {}", id);
            StatusCode::NO_CONTENT
        }
        Ok(false) => StatusCode::NOT_FOUND,
        Err(err) => {
            tracing::error!("Failed to delete geofence {}: {}", id, err);
            StatusCode::INTERNAL_SERVER_ERROR
        }
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
    let config = state.config();
    let altitude = altitude_to_amsl(
        query.altitude_m.unwrap_or(50.0),
        config.altitude_reference,
        config.geoid_offset_m,
    );

    let matching = state.check_point_in_geofences(query.lat, query.lon, altitude);

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
    let config = state.config();
    let waypoints: Vec<atc_core::Waypoint> = req
        .waypoints
        .into_iter()
        .map(|wp| atc_core::Waypoint {
            altitude_m: altitude_to_amsl(
                wp.altitude_m,
                config.altitude_reference,
                config.geoid_offset_m,
            ),
            ..wp
        })
        .collect();
    let geofences = state.get_geofences();
    let mut conflicts = Vec::new();

    // Check each segment of the route against all active geofences
    for i in 0..waypoints.len().saturating_sub(1) {
        let wp1 = &waypoints[i];
        let wp2 = &waypoints[i + 1];

        for geofence in geofences
            .iter()
            .filter(|g| g.active && g.geofence_type != GeofenceType::Advisory)
        {
            if geofence.intersects_segment(
                wp1.lat,
                wp1.lon,
                wp1.altitude_m,
                wp2.lat,
                wp2.lon,
                wp2.altitude_m,
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
