use axum::{
    body::Body,
    http::{Request, StatusCode},
};
use chrono::Utc;
use serde_json::{json, Value};
use std::sync::Arc;
use tower::ServiceExt;

use crate::{api, config::Config, persistence, state::AppState};

async fn setup_app() -> (axum::Router<Arc<AppState>>, Arc<AppState>) {
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

    let db = persistence::init_database(
        &config.database_path,
        config.database_max_connections,
    )
        .await
        .expect("init db");
    let state = Arc::new(AppState::with_database(db, config.clone()));
    state.load_from_database().await.expect("load db");

    let app = api::routes(&config).with_state(state.clone());
    (app, state)
}

async fn read_json(response: axum::response::Response) -> Value {
    let bytes = hyper::body::to_bytes(response.into_body())
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
