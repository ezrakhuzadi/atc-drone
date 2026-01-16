//! Simple route suggestion logic.

use crate::models::Waypoint;
use serde::{Deserialize, Serialize};
use rand::Rng;

const IRVINE_LAT: f64 = 33.6846;
const IRVINE_LON: f64 = -117.8265;
const RADIUS_DEG: f64 = 0.02; // approx 2km
const DEFAULT_ALT: f64 = 50.0;
const DEFAULT_SPEED: f64 = 10.0;

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
            conflict_risk: ConflictRisk::Medium,
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

/// Generate a random route with a start and end point near Irvine.
pub fn generate_random_route() -> Vec<Waypoint> {
    let start = random_point_near_irvine();
    let end = random_point_near_irvine();
    
    // Simple 2-point route
    vec![start, end]
}

fn random_point_near_irvine() -> Waypoint {
    let mut rng = rand::rng();
    let lat_offset = rng.random_range(-RADIUS_DEG..RADIUS_DEG);
    let lon_offset = rng.random_range(-RADIUS_DEG..RADIUS_DEG);
    
    Waypoint {
        lat: IRVINE_LAT + lat_offset,
        lon: IRVINE_LON + lon_offset,
        altitude_m: DEFAULT_ALT,
        speed_mps: Some(DEFAULT_SPEED),
    }
}

// Use shared implementation from spatial module
use crate::spatial::haversine_distance;

// ==== Conflict Avoidance ====

/// Type of avoidance maneuver to perform.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum AvoidanceType {
    /// Deviate laterally (perpendicular offset from original path)
    Lateral,
    /// Change altitude to create vertical separation
    Vertical,
    /// Combination of lateral and vertical
    Combined,
}

/// Generate avoidance waypoints around a conflict point.
/// 
/// # Arguments
/// * `current_pos` - Drone's current position
/// * `destination` - Original destination waypoint
/// * `conflict_point` - Predicted point of conflict
/// * `avoidance_type` - Type of maneuver to perform
/// * `offset_meters` - Separation offset (default ~100m lateral, ~30m vertical)
/// 
/// # Returns
/// Vector of waypoints that route around the conflict
pub fn generate_avoidance_route(
    current_pos: &Waypoint,
    destination: &Waypoint,
    conflict_point: &Waypoint,
    avoidance_type: AvoidanceType,
) -> Vec<Waypoint> {
    // Constants for avoidance
    const LATERAL_OFFSET_DEG: f64 = 0.001; // ~100m offset
    const VERTICAL_OFFSET_M: f64 = 30.0;   // 30m altitude change
    const BUFFER_RATIO: f64 = 0.1;         // 10% before/after conflict point
    
    // Calculate direction of travel (bearing)
    let delta_lat = destination.lat - current_pos.lat;
    let delta_lon = destination.lon - current_pos.lon;
    let path_length = (delta_lat.powi(2) + delta_lon.powi(2)).sqrt();
    
    // Normalize direction vector
    let (dir_lat, dir_lon) = if path_length > 0.0001 {
        (delta_lat / path_length, delta_lon / path_length)
    } else {
        (0.0, 1.0) // Default east if no movement
    };
    
    // Perpendicular vector for lateral offset (rotate 90 degrees)
    let perp_lat = -dir_lon;
    let perp_lon = dir_lat;
    
    match avoidance_type {
        AvoidanceType::Lateral => {
            // Create waypoints that curve around the conflict point
            let offset_lat = perp_lat * LATERAL_OFFSET_DEG;
            let offset_lon = perp_lon * LATERAL_OFFSET_DEG;
            
            vec![
                // Current position (start)
                current_pos.clone(),
                // Pre-conflict waypoint (offset begins)
                Waypoint {
                    lat: conflict_point.lat - dir_lat * BUFFER_RATIO + offset_lat,
                    lon: conflict_point.lon - dir_lon * BUFFER_RATIO + offset_lon,
                    altitude_m: current_pos.altitude_m,
                    speed_mps: current_pos.speed_mps,
                },
                // Offset waypoint at conflict point
                Waypoint {
                    lat: conflict_point.lat + offset_lat,
                    lon: conflict_point.lon + offset_lon,
                    altitude_m: current_pos.altitude_m,
                    speed_mps: current_pos.speed_mps,
                },
                // Post-conflict waypoint (return to path)
                Waypoint {
                    lat: conflict_point.lat + dir_lat * BUFFER_RATIO + offset_lat / 2.0,
                    lon: conflict_point.lon + dir_lon * BUFFER_RATIO + offset_lon / 2.0,
                    altitude_m: current_pos.altitude_m,
                    speed_mps: current_pos.speed_mps,
                },
                // Resume to destination
                destination.clone(),
            ]
        }
        AvoidanceType::Vertical => {
            // Climb or descend to avoid conflict
            let new_altitude = current_pos.altitude_m + VERTICAL_OFFSET_M;
            
            vec![
                current_pos.clone(),
                // Climb before conflict
                Waypoint {
                    lat: conflict_point.lat - dir_lat * BUFFER_RATIO,
                    lon: conflict_point.lon - dir_lon * BUFFER_RATIO,
                    altitude_m: new_altitude,
                    speed_mps: current_pos.speed_mps,
                },
                // Maintain altitude through conflict zone
                Waypoint {
                    lat: conflict_point.lat + dir_lat * BUFFER_RATIO,
                    lon: conflict_point.lon + dir_lon * BUFFER_RATIO,
                    altitude_m: new_altitude,
                    speed_mps: current_pos.speed_mps,
                },
                // Descend back to original altitude
                Waypoint {
                    lat: destination.lat,
                    lon: destination.lon,
                    altitude_m: destination.altitude_m,
                    speed_mps: destination.speed_mps,
                },
            ]
        }
        AvoidanceType::Combined => {
            // Both lateral offset and altitude change
            let offset_lat = perp_lat * LATERAL_OFFSET_DEG * 0.7;
            let offset_lon = perp_lon * LATERAL_OFFSET_DEG * 0.7;
            let new_altitude = current_pos.altitude_m + VERTICAL_OFFSET_M * 0.7;
            
            vec![
                current_pos.clone(),
                Waypoint {
                    lat: conflict_point.lat + offset_lat,
                    lon: conflict_point.lon + offset_lon,
                    altitude_m: new_altitude,
                    speed_mps: current_pos.speed_mps,
                },
                destination.clone(),
            ]
        }
    }
}

/// Determine the best avoidance type based on circumstances.
pub fn select_avoidance_type(
    altitude_m: f64,
    other_altitude_m: f64,
    has_ceiling: bool,
) -> AvoidanceType {
    // If significant altitude difference exists, use lateral
    let alt_diff = (altitude_m - other_altitude_m).abs();
    if alt_diff > 20.0 {
        return AvoidanceType::Lateral;
    }
    
    // If near altitude limits, use lateral
    if altitude_m > 100.0 || has_ceiling {
        return AvoidanceType::Lateral;
    }
    
    // Default: vertical is often simpler
    AvoidanceType::Vertical
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_generate_lateral_avoidance() {
        let current = Waypoint { lat: 33.68, lon: -117.82, altitude_m: 50.0, speed_mps: Some(10.0) };
        let dest = Waypoint { lat: 33.69, lon: -117.82, altitude_m: 50.0, speed_mps: Some(10.0) };
        let conflict = Waypoint { lat: 33.685, lon: -117.82, altitude_m: 50.0, speed_mps: None };
        
        let route = generate_avoidance_route(&current, &dest, &conflict, AvoidanceType::Lateral);
        
        assert!(route.len() >= 3, "Should generate multiple waypoints");
        // First waypoint should be current position
        assert!((route[0].lat - current.lat).abs() < 0.0001);
    }

    #[test]
    fn test_generate_vertical_avoidance() {
        let current = Waypoint { lat: 33.68, lon: -117.82, altitude_m: 50.0, speed_mps: Some(10.0) };
        let dest = Waypoint { lat: 33.69, lon: -117.82, altitude_m: 50.0, speed_mps: Some(10.0) };
        let conflict = Waypoint { lat: 33.685, lon: -117.82, altitude_m: 50.0, speed_mps: None };
        
        let route = generate_avoidance_route(&current, &dest, &conflict, AvoidanceType::Vertical);
        
        // Middle waypoints should have different altitude
        assert!(route[1].altitude_m != current.altitude_m, "Should change altitude");
    }
}
