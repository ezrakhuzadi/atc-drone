use crate::altitude::altitude_to_amsl;
use crate::blender_auth::BlenderAuthManager;
use crate::compliance::{self, ComplianceEvaluation, RoutePoint};
use crate::config::Config;
use crate::state::store::AppState;
use atc_blender::BlenderClient;
use atc_core::models::{
    FlightPlan, FlightPlanMetadata, FlightPlanRequest, FlightStatus, GeofenceType, TrajectoryPoint,
    Waypoint,
};
use atc_core::routing::generate_random_route;
use axum::{
    extract::{Query, State},
    http::StatusCode,
    response::IntoResponse,
    Json,
};
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
    #[allow(dead_code)]
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
    #[serde(default)]
    operation_type: Option<u8>,
    #[serde(default)]
    battery_capacity_min: Option<f64>,
    #[serde(default)]
    battery_reserve_min: Option<f64>,
    #[serde(default)]
    clearance_m: Option<f64>,
    #[serde(default)]
    compliance_override_enabled: Option<bool>,
    #[serde(default)]
    compliance_override_notes: Option<String>,
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

pub async fn create_flight_plan(
    State(state): State<Arc<AppState>>,
    Json(payload): Json<FlightPlanRequest>,
) -> Result<(StatusCode, Json<FlightPlan>), (StatusCode, Json<serde_json::Value>)> {
    let mut payload = payload;
    enforce_owner_for_drone(
        state.as_ref(),
        &payload.drone_id,
        payload.owner_id.as_deref(),
    )?;
    normalize_flight_plan_request(&mut payload, state.config());
    let validation = validate_flight_plan(state.as_ref(), &payload).await;
    if !validation.violations.is_empty() {
        return Err((
            StatusCode::UNPROCESSABLE_ENTITY,
            Json(json!({
                "error": "Flight plan rejected",
                "violations": validation.violations
            })),
        ));
    }
    apply_compliance_metadata(&mut payload.metadata, validation.compliance.as_ref());
    let plan = build_plan(state.as_ref(), payload, None)
        .await
        .map_err(|err| {
            tracing::error!("Failed to persist flight plan: {}", err);
            (
                StatusCode::INTERNAL_SERVER_ERROR,
                Json(json!({
                    "error": "Failed to persist flight plan"
                })),
            )
        })?;
    Ok((StatusCode::CREATED, Json(plan)))
}

pub(crate) async fn create_flight_plan_compat(
    State(state): State<Arc<AppState>>,
    Json(payload): Json<FlightPlanSubmission>,
) -> Result<(StatusCode, Json<FlightPlan>), (StatusCode, Json<serde_json::Value>)> {
    let (mut request, requested_flight_id, requires_trajectory) = match payload {
        FlightPlanSubmission::Atc(request) => (request, None, false),
        FlightPlanSubmission::Planner(request) => {
            let flight_id = request.flight_id.clone();
            (planner_to_request(request), Some(flight_id), true)
        }
    };

    if requires_trajectory {
        let trajectory_len = request
            .trajectory_log
            .as_ref()
            .map(|log| log.len())
            .unwrap_or(0);
        if trajectory_len < 2 {
            return Err((
                StatusCode::UNPROCESSABLE_ENTITY,
                Json(json!({
                    "error": "Flight plan rejected",
                    "violations": [{
                        "type": "trajectory",
                        "message": "Trajectory log is required for planner submissions"
                    }]
                })),
            ));
        }
    }

    enforce_owner_for_drone(
        state.as_ref(),
        &request.drone_id,
        request.owner_id.as_deref(),
    )?;
    normalize_flight_plan_request(&mut request, state.config());
    let validation = validate_flight_plan(state.as_ref(), &request).await;
    if !validation.violations.is_empty() {
        return Err((
            StatusCode::UNPROCESSABLE_ENTITY,
            Json(json!({
                "error": "Flight plan rejected",
                "violations": validation.violations
            })),
        ));
    }
    apply_compliance_metadata(&mut request.metadata, validation.compliance.as_ref());
    let plan = build_plan(state.as_ref(), request, requested_flight_id)
        .await
        .map_err(|err| {
            tracing::error!("Failed to persist flight plan: {}", err);
            (
                StatusCode::INTERNAL_SERVER_ERROR,
                Json(json!({
                    "error": "Failed to persist flight plan"
                })),
            )
        })?;
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
        operation_type: metadata.operation_type,
        battery_capacity_min: metadata.battery_capacity_min,
        battery_reserve_min: metadata.battery_reserve_min,
        clearance_m: metadata.clearance_m,
        compliance_override_enabled: metadata.compliance_override_enabled,
        compliance_override_notes: metadata.compliance_override_notes,
        compliance_report: None,
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

fn normalize_flight_plan_request(request: &mut FlightPlanRequest, config: &Config) {
    if let Some(waypoints) = request.waypoints.as_mut() {
        for waypoint in waypoints {
            normalize_waypoint(waypoint, config);
        }
    }

    if let Some(log) = request.trajectory_log.as_mut() {
        for point in log {
            point.altitude_m = altitude_to_amsl(
                point.altitude_m,
                config.altitude_reference,
                config.geoid_offset_m,
            );
        }
    }

    if let Some(origin) = request.origin.as_mut() {
        normalize_waypoint(origin, config);
    }

    if let Some(destination) = request.destination.as_mut() {
        normalize_waypoint(destination, config);
    }

    if let Some(metadata) = request.metadata.as_mut() {
        if let Some(altitude_m) = metadata.planned_altitude_m.as_mut() {
            *altitude_m = altitude_to_amsl(
                *altitude_m,
                config.altitude_reference,
                config.geoid_offset_m,
            );
        }
    }
}

fn normalize_waypoint(waypoint: &mut Waypoint, config: &Config) {
    waypoint.altitude_m = altitude_to_amsl(
        waypoint.altitude_m,
        config.altitude_reference,
        config.geoid_offset_m,
    );
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

    let mut points = request
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
                .collect::<Vec<_>>()
        })
        .unwrap_or_default();

    if points.is_empty() {
        if let (Some(origin), Some(destination)) = (&request.origin, &request.destination) {
            points.push(RoutePoint {
                lat: origin.lat,
                lon: origin.lon,
                altitude_m: origin.altitude_m,
            });
            points.push(RoutePoint {
                lat: destination.lat,
                lon: destination.lon,
                altitude_m: destination.altitude_m,
            });
        }
    }

    points
}

struct ValidationOutcome {
    violations: Vec<serde_json::Value>,
    compliance: Option<ComplianceEvaluation>,
}

async fn validate_flight_plan(state: &AppState, request: &FlightPlanRequest) -> ValidationOutcome {
    let mut violations = Vec::new();
    let points = extract_route_points(request);
    if points.is_empty() {
        violations.push(json!({
            "type": "route",
            "message": "Route is required for compliance checks"
        }));
        return ValidationOutcome {
            violations,
            compliance: None,
        };
    }

    if points.len() < 2 {
        violations.push(json!({
            "type": "route",
            "message": "At least 2 waypoints are required"
        }));
        return ValidationOutcome {
            violations,
            compliance: None,
        };
    }

    let rules = state.rules();
    for (idx, point) in points.iter().enumerate() {
        if !point.lat.is_finite() || !point.lon.is_finite() {
            violations.push(json!({
                "type": "coordinate",
                "point_index": idx,
                "lat": point.lat,
                "lon": point.lon,
                "message": "Latitude/longitude must be finite numbers"
            }));
            continue;
        }

        if !(-90.0..=90.0).contains(&point.lat) || !(-180.0..=180.0).contains(&point.lon) {
            violations.push(json!({
                "type": "coordinate",
                "point_index": idx,
                "lat": point.lat,
                "lon": point.lon,
                "message": "Latitude/longitude out of range"
            }));
        }

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
        for geofence in geofences
            .iter()
            .filter(|g| g.active && g.geofence_type != GeofenceType::Advisory)
        {
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

    if let Some(log) = request.trajectory_log.as_ref() {
        let mut last_offset: Option<f64> = None;
        for (idx, point) in log.iter().enumerate() {
            if let Some(offset) = point.time_offset_s {
                if !offset.is_finite() || offset < 0.0 {
                    violations.push(json!({
                        "type": "trajectory",
                        "point_index": idx,
                        "time_offset_s": offset,
                        "message": "Trajectory time_offset_s must be a non-negative finite number"
                    }));
                }
                if let Some(prev) = last_offset {
                    if offset < prev {
                        violations.push(json!({
                            "type": "trajectory",
                            "point_index": idx,
                            "time_offset_s": offset,
                            "message": "Trajectory time_offset_s must be non-decreasing"
                        }));
                    }
                }
                last_offset = Some(offset);
            }
        }
    }

    let require_blender = state.config().require_blender_declaration;
    if violations.is_empty() && require_blender {
        let blender_id = request
            .metadata
            .as_ref()
            .and_then(|metadata| metadata.blender_declaration_id.as_deref())
            .map(str::trim)
            .filter(|id| !id.is_empty());

        match blender_id {
            Some(declaration_id) => {
                let auth = BlenderAuthManager::new(state.config());
                let mut blender = BlenderClient::new(
                    &state.config().blender_url,
                    &state.config().blender_session_id,
                    &state.config().blender_auth_token,
                );
                if let Err(err) = auth.apply(&mut blender).await {
                    violations.push(json!({
                        "type": "blender",
                        "message": format!("Failed to refresh Blender auth: {}", err),
                        "blender_declaration_id": declaration_id,
                    }));
                } else {
                    match blender.flight_declaration_exists(declaration_id).await {
                        Ok(true) => {}
                        Ok(false) => violations.push(json!({
                            "type": "blender",
                            "message": "Blender declaration not found",
                            "blender_declaration_id": declaration_id,
                        })),
                        Err(err) => violations.push(json!({
                            "type": "blender",
                            "message": format!("Failed to validate Blender declaration: {}", err),
                            "blender_declaration_id": declaration_id,
                        })),
                    }
                }
            }
            None => violations.push(json!({
                "type": "blender",
                "message": "Missing Blender declaration ID for flight plan",
            })),
        }
    }

    if !violations.is_empty() {
        return ValidationOutcome {
            violations,
            compliance: None,
        };
    }

    let compliance = compliance::evaluate_compliance(state.config(), request, &points).await;
    if !compliance.ok {
        let report = serde_json::to_value(&compliance.report).unwrap_or_else(|_| json!({}));
        violations.push(json!({
            "type": "compliance",
            "message": "Compliance checks failed",
            "blocking_checks": compliance.blocking.clone(),
            "report": report
        }));
    }

    ValidationOutcome {
        violations,
        compliance: Some(compliance),
    }
}

fn apply_compliance_metadata(
    metadata: &mut Option<FlightPlanMetadata>,
    compliance: Option<&ComplianceEvaluation>,
) {
    let compliant = compliance.map(|c| c.ok).unwrap_or(false);
    if let Some(meta) = metadata.as_mut() {
        meta.faa_compliant = Some(compliant);
        if let Some(report) = compliance.map(|c| &c.report) {
            meta.compliance_report = serde_json::to_value(report).ok();
        }
    } else if compliance.is_some() {
        let report = compliance.and_then(|c| serde_json::to_value(&c.report).ok());
        *metadata = Some(FlightPlanMetadata {
            faa_compliant: Some(compliant),
            compliance_report: report,
            ..Default::default()
        });
    }
}

pub(crate) async fn build_plan(
    state: &AppState,
    payload: FlightPlanRequest,
    requested_flight_id: Option<String>,
) -> anyhow::Result<FlightPlan> {
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
    let mut metadata = metadata;
    let _booking_guard = state.flight_plan_booking_lock().lock().await;

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
    let mut selected_log: Option<Vec<TrajectoryPoint>> = None;
    let mut selected_departure = departure;

    let max_delay_secs = if state.config().strategic_scheduling_enabled {
        state.config().strategic_max_delay_secs
    } else {
        0
    };
    let delay_step_secs = state.config().strategic_delay_step_secs.max(1);

    let mut delay_secs = 0u64;
    'schedule: while delay_secs <= max_delay_secs {
        let scheduled_departure = departure + chrono::Duration::seconds(delay_secs as i64);
        for option in &candidates {
            let candidate_log = resolve_candidate_trajectory(
                &option.waypoints,
                trajectory_log.as_ref(),
                metadata.as_ref(),
                has_custom_waypoints,
            );
            // Create a temp plan to test against
            let test_plan = FlightPlan {
                flight_id: flight_id.clone(),
                drone_id: drone_id.clone(),
                owner_id: owner_id.clone(),
                waypoints: option.waypoints.clone(),
                trajectory_log: candidate_log.clone(),
                metadata: metadata.clone(),
                status: FlightStatus::Pending,
                departure_time: scheduled_departure,
                arrival_time: None,
                created_at: Utc::now(),
            };

            let has_conflict = active_plans.iter().any(|existing| {
                atc_core::spatial::check_plan_conflict_with_rules(
                    &test_plan,
                    existing,
                    state.rules(),
                )
            });

            if !has_conflict {
                selected_waypoints = Some(option.waypoints.clone());
                selected_log = candidate_log;
                selected_departure = scheduled_departure;
                flight_status = FlightStatus::Approved;
                break 'schedule; // Found earliest available slot!
            }
        }

        delay_secs = delay_secs.saturating_add(delay_step_secs);
    }

    // If approved, use selected. If rejected, use first option (but mark rejected) or empty?
    // We should probably return the rejected plan so user sees why (or which path failed).
    let final_waypoints = selected_waypoints.unwrap_or_else(|| {
        // If rejected, just take the first candidate to show what failed
        candidates
            .first()
            .map(|c| c.waypoints.clone())
            .unwrap_or_default()
    });
    let final_log = selected_log.or_else(|| {
        resolve_candidate_trajectory(
            &final_waypoints,
            trajectory_log.as_ref(),
            metadata.as_ref(),
            has_custom_waypoints,
        )
    });

    fill_flight_metadata(&mut metadata, &final_waypoints, final_log.as_ref());

    // Estimate arrival time based on route distance
    let arrival_time = estimate_arrival_time(
        &final_waypoints,
        final_log.as_ref(),
        metadata.as_ref(),
        selected_departure,
    );

    let plan = FlightPlan {
        flight_id: flight_id.clone(),
        drone_id,
        owner_id,
        waypoints: final_waypoints,
        trajectory_log: final_log,
        metadata,
        status: flight_status,
        departure_time: selected_departure,
        arrival_time,
        created_at: Utc::now(),
    };

    state.add_flight_plan(plan.clone()).await?;

    Ok(plan)
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
            let message = if owner_id.is_none() {
                "Owner_id is required for this drone"
            } else {
                "Owner does not match drone ownership"
            };
            return Err((
                StatusCode::FORBIDDEN,
                Json(json!({
                    "error": "Forbidden",
                    "message": message,
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
        let owner_drone_ids: HashSet<String> = state
            .get_all_drones()
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

/// Estimate arrival time based on trajectory timing or waypoint distances.
fn estimate_arrival_time(
    waypoints: &[atc_core::models::Waypoint],
    trajectory_log: Option<&Vec<TrajectoryPoint>>,
    metadata: Option<&FlightPlanMetadata>,
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
    let total_distance_m = total_distance_from_waypoints(waypoints);
    let speed_mps = resolve_default_speed_mps(metadata);
    let duration_secs = if speed_mps > 0.0 {
        total_distance_m / speed_mps
    } else {
        0.0
    };

    Some(departure + chrono::Duration::milliseconds((duration_secs * 1000.0) as i64))
}

// Use shared implementation
use atc_core::haversine_distance;

const DEFAULT_TRAJECTORY_SPEED_MPS: f64 = 10.0;
const MIN_TRAJECTORY_SPEED_MPS: f64 = 1.0;

fn resolve_candidate_trajectory(
    waypoints: &[Waypoint],
    payload_log: Option<&Vec<TrajectoryPoint>>,
    metadata: Option<&FlightPlanMetadata>,
    allow_payload_log: bool,
) -> Option<Vec<TrajectoryPoint>> {
    if allow_payload_log {
        if let Some(log) = payload_log {
            if log.len() >= 2 {
                if trajectory_has_complete_timing(log) {
                    return Some(log.clone());
                }
                let speed_mps = resolve_default_speed_mps(metadata);
                return Some(apply_time_offsets_to_log(log, speed_mps));
            }
        }
    }

    build_trajectory_from_waypoints(waypoints, metadata)
}

fn build_trajectory_from_waypoints(
    waypoints: &[Waypoint],
    metadata: Option<&FlightPlanMetadata>,
) -> Option<Vec<TrajectoryPoint>> {
    if waypoints.len() < 2 {
        return None;
    }
    let mut points = Vec::with_capacity(waypoints.len());
    let mut offset_s = 0.0;

    let first = &waypoints[0];
    points.push(TrajectoryPoint {
        lat: first.lat,
        lon: first.lon,
        altitude_m: first.altitude_m,
        time_offset_s: Some(0.0),
    });

    for i in 1..waypoints.len() {
        let prev = &waypoints[i - 1];
        let current = &waypoints[i];
        let distance_m = haversine_distance(prev.lat, prev.lon, current.lat, current.lon);
        let speed_mps = resolve_segment_speed_mps(prev, current, metadata);
        let segment_time_s = if speed_mps > 0.0 {
            distance_m / speed_mps
        } else {
            0.0
        };
        offset_s += segment_time_s;
        points.push(TrajectoryPoint {
            lat: current.lat,
            lon: current.lon,
            altitude_m: current.altitude_m,
            time_offset_s: Some(offset_s),
        });
    }

    Some(points)
}

fn resolve_segment_speed_mps(
    start: &Waypoint,
    end: &Waypoint,
    metadata: Option<&FlightPlanMetadata>,
) -> f64 {
    let speed = start
        .speed_mps
        .or(end.speed_mps)
        .or_else(|| metadata.and_then(|m| m.drone_speed_mps))
        .unwrap_or(DEFAULT_TRAJECTORY_SPEED_MPS);
    sanitize_speed_mps(speed)
}

fn resolve_default_speed_mps(metadata: Option<&FlightPlanMetadata>) -> f64 {
    let speed = metadata
        .and_then(|m| m.drone_speed_mps)
        .unwrap_or(DEFAULT_TRAJECTORY_SPEED_MPS);
    sanitize_speed_mps(speed)
}

fn sanitize_speed_mps(speed: f64) -> f64 {
    if speed.is_finite() && speed > 0.0 {
        speed.max(MIN_TRAJECTORY_SPEED_MPS)
    } else {
        DEFAULT_TRAJECTORY_SPEED_MPS
    }
}

fn trajectory_has_complete_timing(log: &[TrajectoryPoint]) -> bool {
    let mut last_offset = None;
    for point in log {
        let Some(offset) = point.time_offset_s else {
            return false;
        };
        if !offset.is_finite() || offset < 0.0 {
            return false;
        }
        if let Some(prev) = last_offset {
            if offset < prev {
                return false;
            }
        }
        last_offset = Some(offset);
    }
    true
}

fn apply_time_offsets_to_log(log: &[TrajectoryPoint], speed_mps: f64) -> Vec<TrajectoryPoint> {
    if log.is_empty() {
        return Vec::new();
    }
    let speed_mps = sanitize_speed_mps(speed_mps);
    let mut points = Vec::with_capacity(log.len());
    let mut offset_s = 0.0;

    let first = &log[0];
    points.push(TrajectoryPoint {
        lat: first.lat,
        lon: first.lon,
        altitude_m: first.altitude_m,
        time_offset_s: Some(0.0),
    });

    for i in 1..log.len() {
        let prev = &log[i - 1];
        let current = &log[i];
        let distance_m = haversine_distance(prev.lat, prev.lon, current.lat, current.lon);
        let segment_time_s = if speed_mps > 0.0 {
            distance_m / speed_mps
        } else {
            0.0
        };
        offset_s += segment_time_s;
        points.push(TrajectoryPoint {
            lat: current.lat,
            lon: current.lon,
            altitude_m: current.altitude_m,
            time_offset_s: Some(offset_s),
        });
    }

    points
}

fn fill_flight_metadata(
    metadata: &mut Option<FlightPlanMetadata>,
    waypoints: &[Waypoint],
    trajectory_log: Option<&Vec<TrajectoryPoint>>,
) {
    if metadata.is_none() {
        *metadata = Some(FlightPlanMetadata::default());
    }
    let Some(meta) = metadata.as_mut() else {
        return;
    };

    let log_points = trajectory_log.filter(|log| log.len() >= 2);
    let total_distance_m = match log_points {
        Some(log) => total_distance_from_trajectory(log),
        None => total_distance_from_waypoints(waypoints),
    };

    let total_time_s = log_points
        .and_then(|log| max_time_offset_s(log))
        .or_else(|| {
            let speed_mps = resolve_default_speed_mps(Some(meta));
            if speed_mps > 0.0 {
                Some(total_distance_m / speed_mps)
            } else {
                None
            }
        });

    let avg_speed_mps = total_time_s.and_then(|time| {
        if time > 0.0 {
            Some(total_distance_m / time)
        } else {
            None
        }
    });

    if meta.total_distance_m.is_none() && total_distance_m.is_finite() {
        meta.total_distance_m = Some(total_distance_m);
    }
    if meta.total_flight_time_s.is_none() {
        if let Some(time) = total_time_s {
            meta.total_flight_time_s = Some(time);
        }
    }
    if meta.trajectory_points.is_none() {
        let count = log_points
            .map(|log| log.len() as u64)
            .unwrap_or(waypoints.len() as u64);
        if count > 0 {
            meta.trajectory_points = Some(count);
        }
    }
    if meta.drone_speed_mps.is_none() {
        if let Some(speed) = avg_speed_mps {
            meta.drone_speed_mps = Some(speed);
        }
    }
}

fn total_distance_from_waypoints(waypoints: &[Waypoint]) -> f64 {
    let mut total_distance_m = 0.0;
    for i in 0..waypoints.len().saturating_sub(1) {
        total_distance_m += haversine_distance(
            waypoints[i].lat,
            waypoints[i].lon,
            waypoints[i + 1].lat,
            waypoints[i + 1].lon,
        );
    }
    total_distance_m
}

fn total_distance_from_trajectory(log: &[TrajectoryPoint]) -> f64 {
    let mut total_distance_m = 0.0;
    for i in 0..log.len().saturating_sub(1) {
        total_distance_m +=
            haversine_distance(log[i].lat, log[i].lon, log[i + 1].lat, log[i + 1].lon);
    }
    total_distance_m
}

fn max_time_offset_s(log: &[TrajectoryPoint]) -> Option<f64> {
    log.iter()
        .filter_map(|point| point.time_offset_s)
        .filter(|value| value.is_finite() && *value >= 0.0)
        .max_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal))
}
