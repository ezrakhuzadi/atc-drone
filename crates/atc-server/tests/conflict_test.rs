//! Conflict detection integration tests.
//!
//! Tests the end-to-end conflict detection and command dispatch flow.
//!
//! Run with: cargo test --test conflict_test -- --ignored
//! Requires a running ATC server.

use atc_core::models::Telemetry;
use chrono::Utc;
use reqwest::Client;
use std::time::Duration;
use tokio::time::sleep;

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

/// Test that two drones on collision course generate a conflict.
#[tokio::test]
#[ignore]
async fn test_collision_course_generates_conflict() {
    let client = Client::new();
    let base = base_url();
    
    // Register two drones
    let resp = client.post(format!("{}/v1/drones/register", base))
        .json(&serde_json::json!({"drone_id": "CONFLICT-001"}))
        .send().await.unwrap();
    let drone1_resp: serde_json::Value = resp.json().await.unwrap();
    let drone1 = drone1_resp["drone_id"].as_str().unwrap().to_string();
    let token1 = drone1_resp["session_token"].as_str().unwrap().to_string();
    
    let resp = client.post(format!("{}/v1/drones/register", base))
        .json(&serde_json::json!({"drone_id": "CONFLICT-002"}))
        .send().await.unwrap();
    let drone2_resp: serde_json::Value = resp.json().await.unwrap();
    let drone2 = drone2_resp["drone_id"].as_str().unwrap().to_string();
    let token2 = drone2_resp["session_token"].as_str().unwrap().to_string();
    
    // Position drone1 heading east
    let telem1 = make_telemetry(&drone1, 33.6845, -117.8265, 100.0, 90.0, 15.0);
    client.post(format!("{}/v1/telemetry", base))
        .header("Authorization", format!("Bearer {}", token1))
        .json(&telem1)
        .send()
        .await
        .unwrap();
    
    // Position drone2 heading west (towards drone1)
    let telem2 = make_telemetry(&drone2, 33.6845, -117.8200, 100.0, 270.0, 15.0);
    client.post(format!("{}/v1/telemetry", base))
        .header("Authorization", format!("Bearer {}", token2))
        .json(&telem2)
        .send()
        .await
        .unwrap();
    
    // Wait for conflict detection loop (runs every 1 second)
    sleep(Duration::from_secs(3)).await;
    
    // Check for conflicts
    let resp = client.get(format!("{}/v1/conflicts", base)).send().await.unwrap();
    let conflicts: Vec<serde_json::Value> = resp.json().await.unwrap();
    
    // Should have at least one conflict involving our drones
    let our_conflict = conflicts.iter().any(|c| {
        let d1 = c["drone1_id"].as_str().unwrap_or("");
        let d2 = c["drone2_id"].as_str().unwrap_or("");
        (d1 == drone1 || d2 == drone1) && (d1 == drone2 || d2 == drone2)
    });
    
    assert!(our_conflict, "Should detect conflict between converging drones");
}

/// Test that parallel drones do not generate a conflict.
#[tokio::test]
#[ignore]  
async fn test_parallel_drones_no_conflict() {
    let client = Client::new();
    let base = base_url();
    
    // Register two drones
    let resp = client.post(format!("{}/v1/drones/register", base))
        .json(&serde_json::json!({"drone_id": "PARALLEL-001"}))
        .send().await.unwrap();
    let drone1_resp: serde_json::Value = resp.json().await.unwrap();
    let drone1 = drone1_resp["drone_id"].as_str().unwrap().to_string();
    let token1 = drone1_resp["session_token"].as_str().unwrap().to_string();
    
    let resp = client.post(format!("{}/v1/drones/register", base))
        .json(&serde_json::json!({"drone_id": "PARALLEL-002"}))
        .send().await.unwrap();
    let drone2_resp: serde_json::Value = resp.json().await.unwrap();
    let drone2 = drone2_resp["drone_id"].as_str().unwrap().to_string();
    let token2 = drone2_resp["session_token"].as_str().unwrap().to_string();
    
    // Position drones far apart, moving parallel
    let telem1 = make_telemetry(&drone1, 33.6845, -117.8265, 100.0, 0.0, 10.0);
    client.post(format!("{}/v1/telemetry", base))
        .header("Authorization", format!("Bearer {}", token1))
        .json(&telem1)
        .send()
        .await
        .unwrap();
    
    // 500m away, same direction
    let telem2 = make_telemetry(&drone2, 33.6890, -117.8265, 100.0, 0.0, 10.0);
    client.post(format!("{}/v1/telemetry", base))
        .header("Authorization", format!("Bearer {}", token2))
        .json(&telem2)
        .send()
        .await
        .unwrap();
    
    // Wait for detection
    sleep(Duration::from_secs(2)).await;
    
    // Check conflicts
    let resp = client.get(format!("{}/v1/conflicts", base)).send().await.unwrap();
    let conflicts: Vec<serde_json::Value> = resp.json().await.unwrap();
    
    let our_conflict = conflicts.iter().any(|c| {
        let d1 = c["drone1_id"].as_str().unwrap_or("");
        let d2 = c["drone2_id"].as_str().unwrap_or("");
        (d1 == drone1 || d2 == drone1) && (d1 == drone2 || d2 == drone2)
    });
    
    assert!(!our_conflict, "Parallel drones should not conflict");
}
