use crate::state::store::AppState;
use atc_core::models::{FlightPlan, FlightPlanMetadata, FlightPlanRequest, FlightStatus, GeofenceType, TrajectoryPoint, Waypoint};
use atc_core::routing::generate_random_route;
use axum::{extract::{State, Query}, http::StatusCode, response::IntoResponse, Json};
use chrono::Utc;
use serde::Deserialize;
use serde_json::json;
use std::collections::HashSet;
use std::sync::Arc;
use uuid::Uuid;

#[derive(Debug, Deserialize)]
pub(crate) struct PlannerWaypoint {
    lat: f64,
    lon: f64,
    #[serde(alias = "altitude_m")]
    alt: f64,
    #[serde(default, alias = "time_offset_s")]
    time_offset: Option<f64>,
}

#[derive(Debug, Clone, Deserialize)]
pub(crate) struct PlannerMetadata {
    #[serde(default)]
    drone_speed_mps: Option<f64>,
    #[serde(default)]
    drone_id: Option<String>,
    #[serde(default)]
    total_distance_m: Option<f64>,
    #[serde(default)]
    total_flight_time_s: Option<f64>,
    #[serde(default)]
    trajectory_points: Option<u64>,
    #[serde(default)]
    planned_altitude_m: Option<f64>,
    #[serde(default)]
    max_obstacle_height_m: Option<f64>,
    #[serde(default)]
    faa_compliant: Option<bool>,
    #[serde(default)]
    submitted_at: Option<String>,
    #[serde(default)]
    blender_declaration_id: Option<String>,
}

#[derive(Debug, Deserialize)]
pub(crate) struct PlannerFlightRequest {
    flight_id: String,
    waypoints: Vec<PlannerWaypoint>,
    #[serde(default)]
    trajectory_log: Option<Vec<PlannerWaypoint>>,
    #[serde(default)]
    owner_id: Option<String>,
    #[serde(default)]
    metadata: Option<PlannerMetadata>,
}

#[derive(Debug, Deserialize)]
#[serde(untagged)]
pub(crate) enum FlightPlanSubmission {
    Atc(FlightPlanRequest),
    Planner(PlannerFlightRequest),
}

#[derive(Debug, Deserialize)]
pub struct FlightPlansQuery {
    pub owner_id: Option<String>,
}

#[derive(Clone, Copy)]
struct RoutePoint {
    lat: f64,
    lon: f64,
    altitude_m: f64,
}

pub async fn create_flight_plan(
    State(state): State<Arc<AppState>>,
    Json(payload): Json<FlightPlanRequest>,
) -> Result<(StatusCode, Json<FlightPlan>), (StatusCode, Json<serde_json::Value>)> {
    enforce_owner_for_drone(state.as_ref(), &payload.drone_id, payload.owner_id.as_deref())?;
    let violations = validate_flight_plan(state.as_ref(), &payload);
    if !violations.is_empty() {
        return Err((
            StatusCode::UNPROCESSABLE_ENTITY,
            Json(json!({
                "error": "Flight plan rejected",
                "violations": violations
            })),
        ));
    }
    let plan = build_plan(state.as_ref(), payload, None);
    Ok((StatusCode::CREATED, Json(plan)))
}

pub async fn create_flight_plan_compat(
    State(state): State<Arc<AppState>>,
    Json(payload): Json<FlightPlanSubmission>,
) -> Result<(StatusCode, Json<FlightPlan>), (StatusCode, Json<serde_json::Value>)> {
    let (request, requested_flight_id) = match payload {
        FlightPlanSubmission::Atc(request) => (request, None),
        FlightPlanSubmission::Planner(request) => {
            let flight_id = request.flight_id.clone();
            (planner_to_request(request), Some(flight_id))
        }
    };

    enforce_owner_for_drone(state.as_ref(), &request.drone_id, request.owner_id.as_deref())?;
    let violations = validate_flight_plan(state.as_ref(), &request);
    if !violations.is_empty() {
        return Err((
            StatusCode::UNPROCESSABLE_ENTITY,
            Json(json!({
                "error": "Flight plan rejected",
                "violations": violations
            })),
        ));
    }
    let plan = build_plan(state.as_ref(), request, requested_flight_id);
    Ok((StatusCode::CREATED, Json(plan)))
}

fn planner_to_request(request: PlannerFlightRequest) -> FlightPlanRequest {
    let metadata = request.metadata.clone();
    let speed_mps = metadata
        .as_ref()
        .and_then(|metadata| metadata.drone_speed_mps)
        .unwrap_or(15.0);
    let drone_id = metadata
        .as_ref()
        .and_then(|metadata| metadata.drone_id.clone())
        .unwrap_or_else(|| format!("PLANNER-{}", request.flight_id));

    let waypoints = request
        .waypoints
        .into_iter()
        .map(|wp| Waypoint {
            lat: wp.lat,
            lon: wp.lon,
            altitude_m: wp.alt,
            speed_mps: Some(speed_mps),
        })
        .collect();

    FlightPlanRequest {
        drone_id,
        owner_id: request.owner_id,
        waypoints: Some(waypoints),
        trajectory_log: planner_trajectory_to_log(request.trajectory_log),
        metadata: metadata.map(planner_metadata_to_flight_metadata),
        origin: None,
        destination: None,
        departure_time: None,
    }
}

fn planner_metadata_to_flight_metadata(metadata: PlannerMetadata) -> FlightPlanMetadata {
    FlightPlanMetadata {
        drone_speed_mps: metadata.drone_speed_mps,
        total_distance_m: metadata.total_distance_m,
        total_flight_time_s: metadata.total_flight_time_s,
        trajectory_points: metadata.trajectory_points,
        planned_altitude_m: metadata.planned_altitude_m,
        max_obstacle_height_m: metadata.max_obstacle_height_m,
        faa_compliant: metadata.faa_compliant,
        submitted_at: metadata.submitted_at,
        blender_declaration_id: metadata.blender_declaration_id,
    }
}

fn planner_trajectory_to_log(
    trajectory: Option<Vec<PlannerWaypoint>>,
) -> Option<Vec<TrajectoryPoint>> {
    trajectory.map(|points| {
        points
            .into_iter()
            .map(|wp| TrajectoryPoint {
                lat: wp.lat,
                lon: wp.lon,
                altitude_m: wp.alt,
                time_offset_s: wp.time_offset,
            })
            .collect()
    })
}

fn extract_route_points(request: &FlightPlanRequest) -> Vec<RoutePoint> {
    if let Some(log) = request.trajectory_log.as_ref() {
        if log.len() >= 2 {
            return log
                .iter()
                .map(|point| RoutePoint {
                    lat: point.lat,
                    lon: point.lon,
                    altitude_m: point.altitude_m,
                })
                .collect();
        }
    }

    request
        .waypoints
        .as_ref()
        .map(|waypoints| {
            waypoints
                .iter()
                .map(|point| RoutePoint {
                    lat: point.lat,
                    lon: point.lon,
                    altitude_m: point.altitude_m,
                })
                .collect()
        })
        .unwrap_or_default()
}

fn validate_flight_plan(state: &AppState, request: &FlightPlanRequest) -> Vec<serde_json::Value> {
    let mut violations = Vec::new();
    let points = extract_route_points(request);
    if points.is_empty() {
        return violations;
    }

    if points.len() < 2 {
        violations.push(json!({
            "type": "route",
            "message": "At least 2 waypoints are required"
        }));
        return violations;
    }

    let rules = state.rules();
    for (idx, point) in points.iter().enumerate() {
        if !point.altitude_m.is_finite() {
            violations.push(json!({
                "type": "altitude",
                "point_index": idx,
                "altitude_m": point.altitude_m,
                "message": "Altitude must be a finite number"
            }));
            continue;
        }

        if point.altitude_m > rules.max_altitude_m {
            violations.push(json!({
                "type": "altitude",
                "point_index": idx,
                "altitude_m": point.altitude_m,
                "max_m": rules.max_altitude_m,
                "message": format!(
                    "Altitude {:.1}m exceeds max altitude {:.1}m",
                    point.altitude_m,
                    rules.max_altitude_m
                )
            }));
        }

        if point.altitude_m < rules.min_altitude_m {
            violations.push(json!({
                "type": "altitude",
                "point_index": idx,
                "altitude_m": point.altitude_m,
                "min_m": rules.min_altitude_m,
                "message": format!(
                    "Altitude {:.1}m is below min altitude {:.1}m",
                    point.altitude_m,
                    rules.min_altitude_m
                )
            }));
        }
    }

    let geofences = state.get_geofences();
    for i in 0..points.len().saturating_sub(1) {
        let start = points[i];
        let end = points[i + 1];
        for geofence in geofences.iter().filter(|g| g.active && g.geofence_type != GeofenceType::Advisory) {
            if geofence.intersects_segment(
                start.lat,
                start.lon,
                start.altitude_m,
                end.lat,
                end.lon,
                end.altitude_m,
            ) {
                violations.push(json!({
                    "type": "geofence",
                    "segment_index": i,
                    "geofence_id": geofence.id,
                    "geofence_name": geofence.name,
                    "geofence_type": geofence.geofence_type,
                    "message": format!("Route intersects geofence '{}'", geofence.name)
                }));
            }
        }
    }

    violations
}

fn build_plan(
    state: &AppState,
    payload: FlightPlanRequest,
    requested_flight_id: Option<String>,
) -> FlightPlan {
    let flight_id = requested_flight_id.unwrap_or_else(|| Uuid::new_v4().to_string());
    let FlightPlanRequest {
        drone_id,
        owner_id,
        waypoints,
        trajectory_log,
        metadata,
        origin,
        destination,
        departure_time,
    } = payload;
    let departure = departure_time.unwrap_or_else(Utc::now);

    // Gather existing active plans for deconfliction
    let existing_plans = state.get_flight_plans();
    let active_plans: Vec<FlightPlan> = existing_plans
        .into_iter()
        .filter(|p| p.status == FlightStatus::Approved || p.status == FlightStatus::Active)
        .collect();

    let has_custom_waypoints = waypoints.is_some();

    // Determine candidate routes
    let candidates = if let Some(wp) = waypoints {
         // User provided specific route - wrap as single option
         vec![atc_core::routing::RouteOption {
             option_id: "user".to_string(),
             name: "User Defined".to_string(),
             description: "Custom route".to_string(),
             waypoints: wp,
             estimated_duration_secs: 0,
             conflict_risk: atc_core::routing::ConflictRisk::High,
         }]
    } else if let (Some(origin), Some(dest)) = (origin, destination) {
        atc_core::routing::generate_route_options(origin, dest, 50.0)
    } else {
        // Fallback to random
        vec![atc_core::routing::RouteOption {
            option_id: "random".to_string(),
            name: "Random".to_string(),
            description: "Randomly generated".to_string(),
            waypoints: generate_random_route(),
            estimated_duration_secs: 0,
            conflict_risk: atc_core::routing::ConflictRisk::High,
        }]
    };

    // Select the first safe route
    let mut selected_waypoints = None;
    let mut flight_status = FlightStatus::Rejected;

    let candidate_log = if has_custom_waypoints {
        trajectory_log.clone()
    } else {
        None
    };

    for option in &candidates {
        // Create a temp plan to test against
        let test_plan = FlightPlan {
            flight_id: flight_id.clone(),
            drone_id: drone_id.clone(),
            owner_id: owner_id.clone(),
            waypoints: option.waypoints.clone(),
            trajectory_log: candidate_log.clone(),
            metadata: metadata.clone(),
            status: FlightStatus::Pending,
            departure_time: departure,
            arrival_time: None,
            created_at: Utc::now(),
        };

        let has_conflict = active_plans.iter().any(|existing| {
            atc_core::spatial::check_plan_conflict(&test_plan, existing)
        });

        if !has_conflict {
            selected_waypoints = Some(option.waypoints.clone());
            flight_status = FlightStatus::Approved;
            break; // Found a good one!
        }
    }
    
    // If approved, use selected. If rejected, use first option (but mark rejected) or empty?
    // We should probably return the rejected plan so user sees why (or which path failed).
    let final_waypoints = selected_waypoints.unwrap_or_else(|| {
        // If rejected, just take the first candidate to show what failed
        candidates.first().map(|c| c.waypoints.clone()).unwrap_or_default()
    });

    // Estimate arrival time based on route distance
    let arrival_time = estimate_arrival_time(&final_waypoints, trajectory_log.as_ref(), departure);

    let plan = FlightPlan {
        flight_id: flight_id.clone(),
        drone_id,
        owner_id,
        waypoints: final_waypoints,
        trajectory_log,
        metadata,
        status: flight_status,
        departure_time: departure,
        arrival_time,
        created_at: Utc::now(),
    };
    
    state.add_flight_plan(plan.clone());

    plan
}

fn enforce_owner_for_drone(
    state: &AppState,
    drone_id: &str,
    owner_id: Option<&str>,
) -> Result<(), (StatusCode, Json<serde_json::Value>)> {
    let drone = match state.get_drone(drone_id) {
        Some(drone) => drone,
        None => return Ok(()),
    };

    if let Some(expected_owner) = drone.owner_id.as_deref() {
        if owner_id != Some(expected_owner) {
            return Err((
                StatusCode::FORBIDDEN,
                Json(json!({
                    "error": "Forbidden",
                    "message": "Owner does not match drone ownership",
                    "drone_id": drone_id
                })),
            ));
        }
    }

    Ok(())
}

pub async fn get_flight_plans(
    State(state): State<Arc<AppState>>,
    Query(query): Query<FlightPlansQuery>,
) -> impl IntoResponse {
    let plans = state.get_flight_plans();
    if let Some(owner_id) = query.owner_id {
        let owner_drone_ids: HashSet<String> = state.get_all_drones()
            .into_iter()
            .filter(|drone| drone.owner_id.as_ref() == Some(&owner_id))
            .map(|drone| drone.drone_id)
            .collect();

        let filtered: Vec<FlightPlan> = plans
            .into_iter()
            .filter(|plan| {
                plan.owner_id.as_ref() == Some(&owner_id)
                    || owner_drone_ids.contains(&plan.drone_id)
            })
            .collect();
        return Json(filtered);
    }

    Json(plans)
}

/// Estimate arrival time based on waypoint distances and assumed cruise speed.
fn estimate_arrival_time(
    waypoints: &[atc_core::models::Waypoint],
    trajectory_log: Option<&Vec<TrajectoryPoint>>,
    departure: chrono::DateTime<chrono::Utc>,
) -> Option<chrono::DateTime<chrono::Utc>> {
    if let Some(log) = trajectory_log {
        let last_offset = log
            .iter()
            .filter_map(|point| point.time_offset_s)
            .filter(|value| value.is_finite() && *value >= 0.0)
            .max_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        if let Some(offset) = last_offset {
            return Some(departure + chrono::Duration::milliseconds((offset * 1000.0) as i64));
        }
    }

    if waypoints.len() < 2 {
        return None;
    }
    
    // Calculate total route distance
    let mut total_distance_m = 0.0;
    for i in 0..(waypoints.len() - 1) {
        total_distance_m += haversine_distance(
            waypoints[i].lat, waypoints[i].lon,
            waypoints[i + 1].lat, waypoints[i + 1].lon,
        );
    }
    
    // Assume 10 m/s cruise speed (could use waypoint.speed_mps if provided)
    const CRUISE_SPEED_MPS: f64 = 10.0;
    let duration_secs = (total_distance_m / CRUISE_SPEED_MPS) as i64;
    
    Some(departure + chrono::Duration::seconds(duration_secs))
}

// Use shared implementation
use atc_core::haversine_distance;
