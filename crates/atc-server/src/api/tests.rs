use atc_core::models::{FlightPlanMetadata, FlightPlanRequest, FlightStatus, Waypoint};
use axum::{
    body::Body,
    http::{Request, StatusCode},
};
use chrono::Utc;
use serde_json::{json, Value};
use std::sync::Arc;
use tower::ServiceExt;

use crate::{api, config::Config, persistence, state::AppState};

async fn setup_app_with(overrides: impl FnOnce(&mut Config)) -> (axum::Router, Arc<AppState>) {
    let mut config = Config::from_env();
    config.database_path = std::env::temp_dir()
        .join(format!("atc-test-{}.db", uuid::Uuid::new_v4()))
        .to_string_lossy()
        .to_string();
    config.require_blender_declaration = false;
    config.require_registration_token = true;
    config.registration_token = Some("test-registration-token".to_string());
    config.allow_admin_reset = true;
    config.admin_token = "test-admin-token".to_string();

    overrides(&mut config);

    let db = persistence::init_database(&config.database_path, config.database_max_connections)
        .await
        .expect("init db");
    let state = Arc::new(AppState::with_database(db, config.clone()));
    state.load_from_database().await.expect("load db");

    let app = api::routes(&config).with_state(state.clone());
    (app, state)
}

async fn setup_app() -> (axum::Router, Arc<AppState>) {
    setup_app_with(|_config| {}).await
}

async fn read_json(response: axum::response::Response) -> Value {
    let bytes = axum::body::to_bytes(response.into_body(), usize::MAX)
        .await
        .expect("read body");
    serde_json::from_slice(&bytes).expect("parse json")
}

#[tokio::test]
async fn register_and_send_telemetry() {
    let (app, _state) = setup_app().await;

    let register_req = Request::builder()
        .method("POST")
        .uri("/v1/drones/register")
        .header("content-type", "application/json")
        .header("X-Registration-Token", "test-registration-token")
        .body(Body::from(
            json!({
                "drone_id": "DRONE_TEST",
                "owner_id": "owner-1"
            })
            .to_string(),
        ))
        .unwrap();

    let register_res = app.clone().oneshot(register_req).await.unwrap();
    assert_eq!(register_res.status(), StatusCode::CREATED);
    let register_body = read_json(register_res).await;
    let token = register_body["session_token"]
        .as_str()
        .expect("session token");
    let drone_id = register_body["drone_id"].as_str().unwrap();

    let telemetry_req = Request::builder()
        .method("POST")
        .uri("/v1/telemetry")
        .header("content-type", "application/json")
        .header("authorization", format!("Bearer {}", token))
        .body(Body::from(
            json!({
                "drone_id": drone_id,
                "owner_id": "owner-1",
                "lat": 33.6846,
                "lon": -117.8265,
                "altitude_m": 90.0,
                "heading_deg": 180.0,
                "speed_mps": 12.0,
                "timestamp": Utc::now().to_rfc3339()
            })
            .to_string(),
        ))
        .unwrap();

    let telemetry_res = app.clone().oneshot(telemetry_req).await.unwrap();
    assert_eq!(telemetry_res.status(), StatusCode::ACCEPTED);
}

#[tokio::test]
async fn create_geofence_and_check_route() {
    let (app, _state) = setup_app().await;

    let create_req = Request::builder()
        .method("POST")
        .uri("/v1/geofences")
        .header("content-type", "application/json")
        .body(Body::from(
            json!({
                "name": "Test Zone",
                "geofence_type": "no_fly_zone",
                "polygon": [
                    [33.0, -117.0],
                    [33.0, -116.9],
                    [33.1, -116.9],
                    [33.1, -117.0],
                    [33.0, -117.0]
                ],
                "lower_altitude_m": 0.0,
                "upper_altitude_m": 120.0
            })
            .to_string(),
        ))
        .unwrap();

    let create_res = app.clone().oneshot(create_req).await.unwrap();
    assert_eq!(create_res.status(), StatusCode::CREATED);

    let route_req = Request::builder()
        .method("POST")
        .uri("/v1/geofences/check-route")
        .header("content-type", "application/json")
        .body(Body::from(
            json!({
                "waypoints": [
                    { "lat": 33.05, "lon": -117.05, "altitude_m": 50.0 },
                    { "lat": 33.05, "lon": -116.95, "altitude_m": 50.0 }
                ]
            })
            .to_string(),
        ))
        .unwrap();

    let route_res = app.clone().oneshot(route_req).await.unwrap();
    assert_eq!(route_res.status(), StatusCode::OK);
    let route_body = read_json(route_res).await;
    assert_eq!(route_body["conflicts"], Value::Bool(true));
}

#[tokio::test]
async fn reject_invalid_telemetry() {
    let (app, _state) = setup_app().await;

    let register_req = Request::builder()
        .method("POST")
        .uri("/v1/drones/register")
        .header("content-type", "application/json")
        .header("X-Registration-Token", "test-registration-token")
        .body(Body::from(
            json!({
                "drone_id": "DRONE_BAD",
                "owner_id": "owner-1"
            })
            .to_string(),
        ))
        .unwrap();

    let register_res = app.clone().oneshot(register_req).await.unwrap();
    assert_eq!(register_res.status(), StatusCode::CREATED);
    let register_body = read_json(register_res).await;
    let token = register_body["session_token"]
        .as_str()
        .expect("session token");
    let drone_id = register_body["drone_id"].as_str().unwrap();

    let telemetry_req = Request::builder()
        .method("POST")
        .uri("/v1/telemetry")
        .header("content-type", "application/json")
        .header("authorization", format!("Bearer {}", token))
        .body(Body::from(
            json!({
                "drone_id": drone_id,
                "owner_id": "owner-1",
                "lat": 123.456,
                "lon": -117.8265,
                "altitude_m": 90.0,
                "heading_deg": 180.0,
                "speed_mps": 12.0,
                "timestamp": Utc::now().to_rfc3339()
            })
            .to_string(),
        ))
        .unwrap();

    let telemetry_res = app.clone().oneshot(telemetry_req).await.unwrap();
    assert_eq!(telemetry_res.status(), StatusCode::BAD_REQUEST);
}

#[tokio::test]
async fn admin_reset_requires_confirmation() {
    let (app, _state) = setup_app().await;

    let reset_req = Request::builder()
        .method("POST")
        .uri("/v1/admin/reset")
        .header("content-type", "application/json")
        .header("authorization", "Bearer test-admin-token")
        .body(Body::from("{}"))
        .unwrap();

    let reset_res = app.clone().oneshot(reset_req).await.unwrap();
    assert_eq!(reset_res.status(), StatusCode::BAD_REQUEST);
}

#[tokio::test]
async fn strategic_scheduling_delays_conflicting_flight_plan() {
    let (_app, state) = setup_app_with(|config| {
        config.strategic_scheduling_enabled = true;
        config.strategic_max_delay_secs = 30;
        config.strategic_delay_step_secs = 1;
    })
    .await;

    let departure = Utc::now() + chrono::Duration::seconds(60);
    state
        .register_drone("DRONE_A", None)
        .await
        .expect("register DRONE_A");
    state
        .register_drone("DRONE_B", None)
        .await
        .expect("register DRONE_B");
    let waypoints = vec![
        Waypoint {
            lat: 33.0,
            lon: -117.0,
            altitude_m: 50.0,
            speed_mps: None,
        },
        Waypoint {
            lat: 33.0,
            lon: -116.999,
            altitude_m: 50.0,
            speed_mps: None,
        },
    ];
    let metadata = Some(FlightPlanMetadata {
        drone_speed_mps: Some(10.0),
        ..Default::default()
    });

    let plan1 = crate::api::flights::build_plan(
        state.as_ref(),
        FlightPlanRequest {
            drone_id: "DRONE_A".to_string(),
            owner_id: None,
            waypoints: Some(waypoints.clone()),
            trajectory_log: None,
            metadata: metadata.clone(),
            origin: None,
            destination: None,
            departure_time: Some(departure),
        },
        None,
    )
    .await
    .expect("plan1");
    assert_eq!(plan1.status, FlightStatus::Approved);
    assert_eq!(plan1.departure_time, departure);

    let plan2 = crate::api::flights::build_plan(
        state.as_ref(),
        FlightPlanRequest {
            drone_id: "DRONE_B".to_string(),
            owner_id: None,
            waypoints: Some(waypoints.clone()),
            trajectory_log: None,
            metadata,
            origin: None,
            destination: None,
            departure_time: Some(departure),
        },
        None,
    )
    .await
    .expect("plan2");
    assert_eq!(plan2.status, FlightStatus::Approved);
    assert!(plan2.departure_time > departure);
    assert!(plan2.departure_time <= departure + chrono::Duration::seconds(30));
}
