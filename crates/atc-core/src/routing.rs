//! Simple route suggestion logic ("Waze options").
//!
//! Given a start and end point, generates route alternatives.

use crate::models::Waypoint;
use serde::{Deserialize, Serialize};

/// A suggested route option.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RouteOption {
    pub option_id: String,
    pub name: String,
    pub description: String,
    pub waypoints: Vec<Waypoint>,
    pub estimated_duration_secs: u32,
    pub conflict_risk: ConflictRisk,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum ConflictRisk {
    Low,
    Medium,
    High,
}

/// Generate route options from start to end.
pub fn generate_route_options(
    start: Waypoint,
    end: Waypoint,
    altitude_m: f64,
) -> Vec<RouteOption> {
    let direct_distance = haversine_distance(start.lat, start.lon, end.lat, end.lon);
    let duration_secs = (direct_distance / 10.0) as u32; // Assume 10 m/s

    vec![
        // Option 1: Direct route
        RouteOption {
            option_id: "direct".to_string(),
            name: "Direct".to_string(),
            description: "Shortest path, straight line".to_string(),
            waypoints: vec![
                start.clone(),
                end.clone(),
            ],
            estimated_duration_secs: duration_secs,
            conflict_risk: ConflictRisk::Medium, // To be calculated
        },
        // Option 2: Higher altitude
        RouteOption {
            option_id: "high_alt".to_string(),
            name: "High Altitude".to_string(),
            description: format!("Fly at {}m for separation", altitude_m + 30.0),
            waypoints: vec![
                Waypoint { altitude_m: altitude_m + 30.0, ..start.clone() },
                Waypoint { altitude_m: altitude_m + 30.0, ..end.clone() },
            ],
            estimated_duration_secs: duration_secs + 10, // Climb time
            conflict_risk: ConflictRisk::Low,
        },
        // Option 3: Dogleg (waypoint offset)
        RouteOption {
            option_id: "dogleg".to_string(),
            name: "Dogleg".to_string(),
            description: "Route via offset waypoint".to_string(),
            waypoints: vec![
                start.clone(),
                Waypoint {
                    lat: (start.lat + end.lat) / 2.0 + 0.001,
                    lon: (start.lon + end.lon) / 2.0 + 0.001,
                    altitude_m,
                    speed_mps: None,
                },
                end.clone(),
            ],
            estimated_duration_secs: (duration_secs as f64 * 1.2) as u32,
            conflict_risk: ConflictRisk::Low,
        },
    ]
}

/// Calculate distance between two points in meters (Haversine formula).
fn haversine_distance(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    const R: f64 = 6_371_000.0;

    let phi1 = lat1.to_radians();
    let phi2 = lat2.to_radians();
    let dphi = (lat2 - lat1).to_radians();
    let dlambda = (lon2 - lon1).to_radians();

    let a = (dphi / 2.0).sin().powi(2)
        + phi1.cos() * phi2.cos() * (dlambda / 2.0).sin().powi(2);

    2.0 * R * a.sqrt().atan2((1.0 - a).sqrt())
}
