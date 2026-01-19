//! Telemetry API integration tests.
//!
//! Run with: cargo test --test telemetry_test -- --ignored
//! 
//! Note: Requires a running ATC server at http://localhost:3000
//! or set ATC_TEST_URL environment variable.

use atc_core::models::Telemetry;
use chrono::Utc;
use reqwest::Client;

fn base_url() -> String {
    std::env::var("ATC_TEST_URL").unwrap_or_else(|_| "http://localhost:3000".to_string())
}

fn make_telemetry(drone_id: &str, lat: f64, lon: f64, altitude_m: f64, heading_deg: f64, speed_mps: f64) -> Telemetry {
    Telemetry {
        drone_id: drone_id.to_string(),
        owner_id: None,
        lat,
        lon,
        altitude_m,
        velocity_x: speed_mps * heading_deg.to_radians().sin(),
        velocity_y: speed_mps * heading_deg.to_radians().cos(),
        velocity_z: 0.0,
        heading_deg,
        speed_mps,
        timestamp: Utc::now(),
    }
}

#[tokio::test]
#[ignore] // Run only when server is running
async fn test_register_and_send_telemetry() {
    let client = Client::new();
    let base = base_url();
    
    // Register a drone
    let body = serde_json::json!({"drone_id": "TEST-TEL-001"});
    let resp = client.post(format!("{}/v1/drones/register", base))
        .json(&body)
        .send()
        .await
        .expect("Failed to register");
    let json: serde_json::Value = resp.json().await.unwrap();
    let drone_id = json["drone_id"].as_str().unwrap();
    let token = json["session_token"].as_str().unwrap();
    assert_eq!(drone_id, "TEST-TEL-001");
    
    // Send telemetry
    let telem = make_telemetry(drone_id, 33.6845, -117.8265, 100.0, 45.0, 10.0);
    client.post(format!("{}/v1/telemetry", base))
        .header("Authorization", format!("Bearer {}", token))
        .json(&telem)
        .send()
        .await
        .expect("Failed to send telemetry");
    
    // Verify drone appears in list
    let resp = client.get(format!("{}/v1/drones", base)).send().await.unwrap();
    let drones: Vec<serde_json::Value> = resp.json().await.unwrap();
    
    let found = drones.iter().any(|d| d["drone_id"].as_str() == Some("TEST-TEL-001"));
    assert!(found, "Drone should appear in list after telemetry");
}

#[tokio::test]
#[ignore]
async fn test_telemetry_updates_position() {
    let client = Client::new();
    let base = base_url();
    
    // Register drone
    let body = serde_json::json!({"drone_id": "TEST-TEL-002"});
    let resp = client.post(format!("{}/v1/drones/register", base))
        .json(&body)
        .send()
        .await
        .unwrap();
    let json: serde_json::Value = resp.json().await.unwrap();
    let drone_id = json["drone_id"].as_str().unwrap().to_string();
    let token = json["session_token"].as_str().unwrap().to_string();
    
    // Send initial position
    let telem1 = make_telemetry(&drone_id, 33.6845, -117.8265, 100.0, 0.0, 0.0);
    client.post(format!("{}/v1/telemetry", base))
        .header("Authorization", format!("Bearer {}", token))
        .json(&telem1)
        .send()
        .await
        .unwrap();
    
    // Get initial position
    let resp = client.get(format!("{}/v1/drones", base)).send().await.unwrap();
    let drones: Vec<serde_json::Value> = resp.json().await.unwrap();
    let drone = drones.iter().find(|d| d["drone_id"].as_str() == Some(&drone_id)).unwrap();
    assert!((drone["lat"].as_f64().unwrap() - 33.6845).abs() < 0.0001);
    
    // Update position
    let telem2 = make_telemetry(&drone_id, 33.6900, -117.8200, 150.0, 45.0, 15.0);
    client.post(format!("{}/v1/telemetry", base))
        .header("Authorization", format!("Bearer {}", token))
        .json(&telem2)
        .send()
        .await
        .unwrap();
    
    // Verify position updated
    let resp = client.get(format!("{}/v1/drones", base)).send().await.unwrap();
    let drones: Vec<serde_json::Value> = resp.json().await.unwrap();
    let drone = drones.iter().find(|d| d["drone_id"].as_str() == Some(&drone_id)).unwrap();
    assert!((drone["lat"].as_f64().unwrap() - 33.6900).abs() < 0.0001);
    assert!((drone["altitude_m"].as_f64().unwrap() - 150.0).abs() < 0.1);
}
