//! Geofence API integration tests.
//!
//! Run with: cargo test --test geofence_test -- --ignored

use reqwest::Client;

fn base_url() -> String {
    std::env::var("ATC_TEST_URL").unwrap_or_else(|_| "http://localhost:3000".to_string())
}

/// Test geofence CRUD operations.
#[tokio::test]
#[ignore]
async fn test_geofence_crud() {
    let client = Client::new();
    let base = base_url();
    
    // Create a geofence via direct POST
    let geofence_body = serde_json::json!({
        "name": "Test No-Fly Zone",
        "geofence_type": "no_fly_zone",
        "polygon": [
            [33.68, -117.83],
            [33.68, -117.82],
            [33.69, -117.82],
            [33.69, -117.83],
            [33.68, -117.83]
        ],
        "lower_altitude_m": 0.0,
        "upper_altitude_m": 400.0
    });
    
    let resp = client
        .post(format!("{}/v1/geofences", base))
        .json(&geofence_body)
        .send()
        .await
        .expect("Failed to create geofence");
    
    assert!(resp.status().is_success(), "Should create geofence successfully");
    let created: serde_json::Value = resp.json().await.unwrap();
    let geofence_id = created["id"].as_str().unwrap();
    
    // List geofences
    let resp = client
        .get(format!("{}/v1/geofences", base))
        .send()
        .await
        .unwrap();
    
    let geofences: Vec<serde_json::Value> = resp.json().await.unwrap();
    let found = geofences.iter().any(|g| g["id"].as_str() == Some(geofence_id));
    assert!(found, "Created geofence should appear in list");
    
    // Get single geofence
    let resp = client
        .get(format!("{}/v1/geofences/{}", base, geofence_id))
        .send()
        .await
        .unwrap();
    
    assert!(resp.status().is_success(), "Should get geofence by ID");
    let geofence: serde_json::Value = resp.json().await.unwrap();
    assert_eq!(geofence["name"].as_str(), Some("Test No-Fly Zone"));
    
    // Delete geofence
    let resp = client
        .delete(format!("{}/v1/geofences/{}", base, geofence_id))
        .send()
        .await
        .unwrap();
    
    assert!(resp.status().is_success(), "Should delete geofence");
}

