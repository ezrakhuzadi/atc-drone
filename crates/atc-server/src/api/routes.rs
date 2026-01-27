//! REST API routes.

use axum::{
    extract::{Path, Query, State},
    http::{HeaderMap, StatusCode},
    middleware,
    response::IntoResponse,
    routing::{get, post, delete, put},
    Json, Router,
};
use chrono::{DateTime, Duration, Utc};
use serde::{Deserialize, Serialize};
use serde_json::json;
use std::collections::HashSet;
use std::sync::Arc;

use crate::api::{commands, flights, geofences, daa, ws};
use crate::api::auth::{self, AdminToken, RateLimiter};
use crate::altitude::altitude_to_amsl;
use crate::compliance::{self, ComplianceReport, RoutePoint};
use crate::config::Config;
use crate::route_planner::{plan_route, RoutePlanRequest, RoutePlanResponse};
use crate::state::{AppState, ExternalTraffic};
use crate::state::store::RegisterDroneOutcome;
use atc_core::models::{Telemetry, ConformanceStatus, DroneStatus, FlightPlanRequest, FlightPlanMetadata, TrajectoryPoint, Waypoint, GeofenceType};

/// Create the API router.
pub fn create_router(config: &Config) -> Router<Arc<AppState>> {
    // Create rate limiter for telemetry
    let rate_limiter = RateLimiter::new(
        config.rate_limit_rps,
        config.rate_limit_enabled,
        config.trust_proxy,
        config.rate_limit_max_tracked_ips,
        std::time::Duration::from_secs(config.rate_limit_entry_ttl_s),
    );
    let registration_limiter = RateLimiter::new(
        config.registration_rate_limit_rps,
        config.rate_limit_enabled,
        config.trust_proxy,
        config.rate_limit_max_tracked_ips,
        std::time::Duration::from_secs(config.rate_limit_entry_ttl_s),
    );
    let expensive_limiter = RateLimiter::new(
        config.expensive_rate_limit_rps,
        config.rate_limit_enabled,
        config.trust_proxy,
        config.rate_limit_max_tracked_ips,
        std::time::Duration::from_secs(config.rate_limit_entry_ttl_s),
    );
    
    // Create admin token extractor
    let admin_token = AdminToken(Arc::new(config.admin_token.clone()));
    
    // Public routes (no auth required)
    let public_routes = Router::new()
        .route("/v1/drones", get(list_drones))
        .route("/v1/drones/:drone_id", get(get_drone))
        .route("/v1/traffic", get(list_traffic))
        .route("/v1/conflicts", get(list_conflicts))
        .route("/v1/conformance", get(list_conformance))
        .route("/v1/daa", get(daa::list_daa))
        .route("/v1/compliance/limits", get(get_compliance_limits))
        .route("/v1/rid/view", post(update_rid_view))
        .route("/v1/flights", get(flights::get_flight_plans))
        // Command polling routes
        .route("/v1/commands/next", get(commands::get_next_command))
        .route("/v1/commands/ack", post(commands::ack_command))
        .route("/v1/commands/ws", get(commands::command_stream_ws))
        // Geofence routes
        .route("/v1/geofences", post(geofences::create_geofence))
        .route("/v1/geofences", get(geofences::list_geofences))
        .route("/v1/geofences/:id", get(geofences::get_geofence))
        .route("/v1/geofences/:id", put(geofences::update_geofence))
        .route("/v1/geofences/:id", delete(geofences::delete_geofence))
        .route("/v1/geofences/check", get(geofences::check_point))
        .route("/v1/geofences/check-route", post(geofences::check_route))
        // WebSocket streaming
        .route("/v1/ws", get(ws::ws_handler));

    let registration_routes = Router::new()
        .route("/v1/drones/register", post(register_drone))
        .layer(middleware::from_fn_with_state(registration_limiter, auth::rate_limit));

    let admin_command_routes = Router::new()
        .route("/v1/commands", post(commands::issue_command))
        .route("/v1/commands", get(commands::get_all_commands))
        .layer(middleware::from_fn_with_state(admin_token.clone(), auth::require_admin));

    let admin_flight_routes = Router::new()
        .route("/v1/flights/plan", post(flights::create_flight_plan))
        .route("/v1/flights", post(flights::create_flight_plan_compat))
        .route(
            "/v1/operational_intents/reserve",
            post(flights::reserve_operational_intent),
        )
        .route(
            "/v1/operational_intents/:flight_id/confirm",
            post(flights::confirm_operational_intent),
        )
        .route(
            "/v1/operational_intents/:flight_id",
            put(flights::update_operational_intent),
        )
        .route(
            "/v1/operational_intents/:flight_id/cancel",
            post(flights::cancel_operational_intent),
        )
        .layer(middleware::from_fn_with_state(admin_token.clone(), auth::require_admin));
    
    // Rate-limited telemetry route
    let telemetry_route = Router::new()
        .route("/v1/telemetry", post(receive_telemetry))
        .layer(middleware::from_fn_with_state(rate_limiter, auth::rate_limit));

    let expensive_routes = Router::new()
        .route("/v1/compliance/evaluate", post(evaluate_compliance))
        .route("/v1/routes/plan", post(plan_route_handler))
        .layer(middleware::from_fn_with_state(expensive_limiter, auth::rate_limit));
    
    // Admin routes (require admin token)
    let admin_routes = Router::new()
        .route("/v1/admin/reset", post(admin_reset))
        .layer(middleware::from_fn_with_state(admin_token, auth::require_admin));
    
    public_routes
        .merge(registration_routes)
        .merge(telemetry_route)
        .merge(expensive_routes)
        .merge(admin_command_routes)
        .merge(admin_flight_routes)
        .merge(admin_routes)
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
    /// Filter by traffic source (local, rid, external, etc.)
    pub source: Option<String>,
    /// Return only external traffic
    pub external_only: Option<bool>,
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

#[derive(Debug, Deserialize)]
pub struct ComplianceEvaluateRequest {
    pub drone_id: Option<String>,
    pub owner_id: Option<String>,
    pub waypoints: Option<Vec<Waypoint>>,
    #[serde(default)]
    pub trajectory_log: Option<Vec<TrajectoryPoint>>,
    #[serde(default)]
    pub metadata: Option<FlightPlanMetadata>,
    pub origin: Option<Waypoint>,
    pub destination: Option<Waypoint>,
    pub departure_time: Option<chrono::DateTime<chrono::Utc>>,
}

#[derive(Debug, Serialize)]
pub struct ComplianceEvaluateResponse {
    pub ok: bool,
    pub blocking: Vec<String>,
    pub violations: Vec<serde_json::Value>,
    pub report: Option<ComplianceReport>,
}

#[derive(Debug, Serialize)]
pub struct RidViewResponse {
    pub view: String,
}



// === Handlers ===

async fn register_drone(
    State(state): State<Arc<AppState>>,
    headers: HeaderMap,
    Json(req): Json<RegisterRequest>,
) -> impl IntoResponse {
    let config = state.config();
    if config.require_registration_token {
        let expected = match config.registration_token.as_deref() {
            Some(token) => token,
            None => {
                return (
                    StatusCode::SERVICE_UNAVAILABLE,
                    Json(serde_json::json!({
                        "error": "Registration is temporarily unavailable",
                        "hint": "ATC_REGISTRATION_TOKEN is not configured"
                    })),
                );
            }
        };

        match auth::extract_registration_token(&headers) {
            Some(token) if token == expected => {}
            Some(_) => {
                return (
                    StatusCode::FORBIDDEN,
                    Json(serde_json::json!({
                        "error": "Invalid registration token"
                    })),
                );
            }
            None => {
                return (
                    StatusCode::UNAUTHORIZED,
                    Json(serde_json::json!({
                        "error": "Registration token required",
                        "hint": "Set X-Registration-Token header"
                    })),
                );
            }
        }
    }

    let drone_id = req.drone_id.unwrap_or_else(|| {
        format!("DRONE{:04}", state.next_drone_id())
    });

    let existing = state.get_drone(&drone_id);
    if existing.is_none()
        && config.max_tracked_drones > 0
        && state.drone_count() >= config.max_tracked_drones
    {
        return (
            StatusCode::SERVICE_UNAVAILABLE,
            Json(serde_json::json!({
                "error": "Drone registration capacity reached",
                "hint": "Try again later or increase ATC_MAX_TRACKED_DRONES",
                "limit": config.max_tracked_drones,
            })),
        );
    }

    if let Some(existing) = existing {
        if let Some(expected_owner) = existing.owner_id.as_deref() {
            match req.owner_id.as_deref() {
                Some(owner_id) if owner_id == expected_owner => {}
                Some(_) => {
                    return (
                        StatusCode::CONFLICT,
                        Json(serde_json::json!({
                            "error": "Drone ID already claimed",
                            "hint": "Use a different drone_id or match the existing owner_id",
                            "drone_id": drone_id
                        })),
                    );
                }
                None => {
                    return (
                        StatusCode::CONFLICT,
                        Json(serde_json::json!({
                            "error": "Drone ID already claimed",
                            "hint": "Provide owner_id to reclaim this drone_id",
                            "drone_id": drone_id
                        })),
                    );
                }
            }
        }

        if state.drone_token(&drone_id).is_some() {
            return (
                StatusCode::CONFLICT,
                Json(serde_json::json!({
                    "error": "Drone already registered",
                    "hint": "Use existing session token or register with a new drone_id",
                    "drone_id": drone_id
                })),
            );
        }
    }

    let session_token = uuid::Uuid::new_v4().to_string();
    match state
        .register_drone_with_token(&drone_id, req.owner_id.clone(), session_token.clone())
        .await
    {
        Ok(RegisterDroneOutcome::Registered) => {}
        Ok(RegisterDroneOutcome::AlreadyRegistered) => {
            return (
                StatusCode::CONFLICT,
                Json(serde_json::json!({
                    "error": "Drone already registered",
                    "hint": "Use existing session token or register with a new drone_id",
                    "drone_id": drone_id
                })),
            );
        }
        Err(err) => {
            tracing::error!("Failed to persist drone registration {}: {}", drone_id, err);
            return (
                StatusCode::INTERNAL_SERVER_ERROR,
                Json(serde_json::json!({
                    "error": "Failed to register drone",
                    "drone_id": drone_id
                })),
            );
        }
    }

    tracing::info!("Registered drone {}", drone_id);
    
    (
        StatusCode::CREATED,
        Json(serde_json::json!({
            "drone_id": drone_id,
            "session_token": session_token,
        })),
    )
}

async fn receive_telemetry(
    State(state): State<Arc<AppState>>,
    headers: HeaderMap,
    Json(telemetry): Json<Telemetry>,
) -> (StatusCode, Json<serde_json::Value>) {
    if let Err(status) = auth::authorize_drone_for(state.as_ref(), &telemetry.drone_id, &headers) {
        return (status, Json(serde_json::json!({"error": "Authorization failed"})));
    }
    let mut telemetry = telemetry;
    let now = Utc::now();
    normalize_telemetry_timestamp(&mut telemetry, state.config(), now);
    if let Err(response) = validate_telemetry(&telemetry, state.config(), now) {
        return response;
    }
    state.update_telemetry(telemetry).await;
    (StatusCode::ACCEPTED, Json(serde_json::json!({})))
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

async fn get_drone(
    State(state): State<Arc<AppState>>,
    Path(drone_id): Path<String>,
) -> Result<Json<atc_core::models::DroneState>, StatusCode> {
    state.get_drone(&drone_id)
        .map(Json)
        .ok_or(StatusCode::NOT_FOUND)
}

async fn list_traffic(
    State(state): State<Arc<AppState>>,
    Query(query): Query<TrafficQuery>,
) -> Json<Vec<TrafficState>> {
    let mut traffic: Vec<TrafficState> = Vec::new();

    let source_filter = query.source.as_ref().map(|s| s.to_lowercase());
    let external_only = query.external_only.unwrap_or(false);
    let mut include_local = !external_only;
    let mut include_external = query.include_external.unwrap_or(false);

    if let Some(source) = source_filter.as_deref() {
        match source {
            "local" => {
                include_local = true;
                include_external = false;
            }
            "external" | "rid" => {
                include_local = false;
                include_external = true;
            }
            _ => {}
        }
    }

    if include_local {
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
    }

    if include_external {
        let external = state.get_external_traffic();
        traffic.extend(external.into_iter().map(external_to_traffic));
    }

    if let Some(source) = source_filter {
        if source == "external" {
            traffic.retain(|entry| entry.traffic_source.to_lowercase() != "local");
        } else {
            traffic.retain(|entry| entry.traffic_source.eq_ignore_ascii_case(&source));
        }
    }

    Json(traffic)
}

fn bad_request(message: &str, field: Option<&str>) -> (StatusCode, Json<serde_json::Value>) {
    let mut payload = serde_json::json!({ "error": message });
    if let Some(field) = field {
        payload["field"] = serde_json::Value::String(field.to_string());
    }
    (StatusCode::BAD_REQUEST, Json(payload))
}

fn validate_telemetry(
    telemetry: &Telemetry,
    config: &Config,
    now: DateTime<Utc>,
) -> Result<(), (StatusCode, Json<serde_json::Value>)> {
    let lat = telemetry.lat;
    let lon = telemetry.lon;
    if !lat.is_finite() {
        return Err(bad_request("Latitude must be a finite number", Some("lat")));
    }
    if !lon.is_finite() {
        return Err(bad_request("Longitude must be a finite number", Some("lon")));
    }
    if lat < -90.0 || lat > 90.0 {
        return Err(bad_request("Latitude out of range", Some("lat")));
    }
    if lon < -180.0 || lon > 180.0 {
        return Err(bad_request("Longitude out of range", Some("lon")));
    }

    let altitude = telemetry.altitude_m;
    let altitude_amsl = altitude_to_amsl(
        altitude,
        config.altitude_reference,
        config.geoid_offset_m,
    );
    if !altitude_amsl.is_finite() {
        return Err(bad_request("Altitude must be a finite number", Some("altitude_m")));
    }
    if altitude_amsl < config.telemetry_min_alt_m || altitude_amsl > config.telemetry_max_alt_m {
        return Err(bad_request("Altitude out of allowed range", Some("altitude_m")));
    }

    let speed = telemetry.speed_mps;
    if !speed.is_finite() {
        return Err(bad_request("Speed must be a finite number", Some("speed_mps")));
    }
    if speed < 0.0 || speed > config.telemetry_max_speed_mps {
        return Err(bad_request("Speed out of allowed range", Some("speed_mps")));
    }

    let heading = telemetry.heading_deg;
    if !heading.is_finite() {
        return Err(bad_request("Heading must be a finite number", Some("heading_deg")));
    }
    if heading < 0.0 || heading >= 360.0 {
        return Err(bad_request("Heading out of range", Some("heading_deg")));
    }

    let max_future = Duration::seconds(config.telemetry_max_future_s);
    let max_age = Duration::seconds(config.telemetry_max_age_s);
    if telemetry.timestamp > now + max_future {
        return Err(bad_request("Telemetry timestamp is too far in the future", Some("timestamp")));
    }
    if telemetry.timestamp < now - max_age {
        return Err(bad_request("Telemetry timestamp is too old", Some("timestamp")));
    }

    Ok(())
}

fn normalize_telemetry_timestamp(telemetry: &mut Telemetry, config: &Config, now: DateTime<Utc>) {
    let max_future = Duration::seconds(config.telemetry_max_future_s);
    let max_age = Duration::seconds(config.telemetry_max_age_s);
    if telemetry.timestamp > now + max_future || telemetry.timestamp < now - max_age {
        tracing::warn!(
            "Telemetry timestamp out of bounds for {} ({}); normalizing to server time",
            telemetry.drone_id,
            telemetry.timestamp
        );
        telemetry.timestamp = now;
    }
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

async fn get_compliance_limits(
    State(state): State<Arc<AppState>>,
) -> Json<serde_json::Value> {
    let config = state.config();
    Json(serde_json::json!({
        "maxWindMps": config.compliance_max_wind_mps,
        "maxGustMps": config.compliance_max_gust_mps,
        "maxPrecipMm": config.compliance_max_precip_mm,
        "windWarnRatio": config.compliance_wind_warn_ratio,
        "batteryWarnMarginMin": config.compliance_battery_warn_margin_min,
        "populationBvlosMax": config.compliance_population_bvlos_max,
        "populationWarn": config.compliance_population_warn,
        "populationAbsoluteMax": config.compliance_population_absolute_max,
        "defaultClearanceM": config.compliance_default_clearance_m
    }))
}

async fn evaluate_compliance(
    State(state): State<Arc<AppState>>,
    Json(req): Json<ComplianceEvaluateRequest>,
) -> Json<ComplianceEvaluateResponse> {
    let request = FlightPlanRequest {
        drone_id: req
            .drone_id
            .unwrap_or_else(|| "COMPLIANCE-REQUEST".to_string()),
        owner_id: req.owner_id,
        waypoints: req.waypoints,
        trajectory_log: req.trajectory_log,
        metadata: req.metadata,
        origin: req.origin,
        destination: req.destination,
        departure_time: req.departure_time,
    };

    let points = extract_route_points(&request, state.config());
    let mut violations: Vec<serde_json::Value> = Vec::new();

    if points.is_empty() {
        violations.push(json!({
            "type": "route",
            "message": "Route is required for compliance checks"
        }));
        return Json(ComplianceEvaluateResponse {
            ok: false,
            blocking: Vec::new(),
            violations,
            report: None,
        });
    }

    if points.len() < 2 {
        violations.push(json!({
            "type": "route",
            "message": "At least 2 waypoints are required"
        }));
        return Json(ComplianceEvaluateResponse {
            ok: false,
            blocking: Vec::new(),
            violations,
            report: None,
        });
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

        if !( -90.0..=90.0).contains(&point.lat) || !( -180.0..=180.0).contains(&point.lon) {
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

    if !violations.is_empty() {
        return Json(ComplianceEvaluateResponse {
            ok: false,
            blocking: Vec::new(),
            violations,
            report: None,
        });
    }

    let compliance = compliance::evaluate_compliance(state.config(), &request, &points).await;
    Json(ComplianceEvaluateResponse {
        ok: compliance.ok,
        blocking: compliance.blocking,
        violations,
        report: Some(compliance.report),
    })
}

fn extract_route_points(request: &FlightPlanRequest, config: &Config) -> Vec<RoutePoint> {
    if let Some(log) = request.trajectory_log.as_ref() {
        if log.len() >= 2 {
            return log
                .iter()
                .map(|point| RoutePoint {
                    lat: point.lat,
                    lon: point.lon,
                    altitude_m: altitude_to_amsl(
                        point.altitude_m,
                        config.altitude_reference,
                        config.geoid_offset_m,
                    ),
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
                    altitude_m: altitude_to_amsl(
                        point.altitude_m,
                        config.altitude_reference,
                        config.geoid_offset_m,
                    ),
                })
                .collect::<Vec<_>>()
        })
        .unwrap_or_default();

    if points.is_empty() {
        if let (Some(origin), Some(destination)) = (&request.origin, &request.destination) {
            points.push(RoutePoint {
                lat: origin.lat,
                lon: origin.lon,
                altitude_m: altitude_to_amsl(
                    origin.altitude_m,
                    config.altitude_reference,
                    config.geoid_offset_m,
                ),
            });
            points.push(RoutePoint {
                lat: destination.lat,
                lon: destination.lon,
                altitude_m: altitude_to_amsl(
                    destination.altitude_m,
                    config.altitude_reference,
                    config.geoid_offset_m,
                ),
            });
        }
    }

    points
}

async fn plan_route_handler(
    State(state): State<Arc<AppState>>,
    Json(request): Json<RoutePlanRequest>,
) -> impl IntoResponse {
    let response: RoutePlanResponse = plan_route(state.as_ref(), state.config(), request).await;
    let status = if response.ok { StatusCode::OK } else { StatusCode::BAD_REQUEST };
    (status, Json(response))
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
#[derive(Debug, Deserialize)]
struct AdminResetRequest {
    /// Explicit confirmation string, must be "RESET".
    confirm: Option<String>,
    /// Require that no active/holding drones exist before reset.
    #[serde(default)]
    require_idle: Option<bool>,
}




async fn admin_reset(
    State(state): State<Arc<AppState>>,
    Json(req): Json<AdminResetRequest>,
) -> impl IntoResponse {
    let config = state.config();
    if !config.allow_admin_reset {
        return (
            StatusCode::FORBIDDEN,
            Json(serde_json::json!({
                "error": "Admin reset disabled",
                "hint": "Set ATC_ALLOW_ADMIN_RESET=1 to enable"
            })),
        );
    }

    if req.confirm.as_deref() != Some("RESET") {
        return (
            StatusCode::BAD_REQUEST,
            Json(serde_json::json!({
                "error": "Reset requires confirm=RESET"
            })),
        );
    }

    let active_drones = state.get_all_drones()
        .into_iter()
        .filter(|drone| drone.status != DroneStatus::Inactive)
        .count();

    if req.require_idle.unwrap_or(true) && active_drones > 0 {
        return (
            StatusCode::CONFLICT,
            Json(serde_json::json!({
                "error": "Active drones present",
                "active_drones": active_drones,
                "hint": "Stop all drones or set require_idle=false"
            })),
        );
    }

    if let Err(err) = state.clear_all().await {
        tracing::error!("Failed to reset state: {}", err);
        return (
            StatusCode::INTERNAL_SERVER_ERROR,
            Json(serde_json::json!({
                "error": "Failed to reset state"
            })),
        );
    }
    (
        StatusCode::OK,
        Json(serde_json::json!({
            "cleared": true,
            "active_drones": active_drones,
            "message": "State reset",
        })),
    )
}
