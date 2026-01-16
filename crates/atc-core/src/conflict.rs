//! Conflict detection module for ATC-Drone.
//!
//! Provides real-time conflict detection with lookahead prediction
//! for multiple drones operating in the same airspace.

use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::time::{SystemTime, UNIX_EPOCH};

/// Severity levels for detected conflicts.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum ConflictSeverity {
    /// Drones on converging paths but far
    Info,
    /// Conflict predicted within lookahead window
    Warning,
    /// Immediate separation violation
    Critical,
}

/// Current position and velocity of a drone.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DronePosition {
    pub drone_id: String,
    pub lat: f64,
    pub lon: f64,
    pub altitude_m: f64,
    #[serde(default)]
    pub heading_deg: f64,
    #[serde(default)]
    pub speed_mps: f64,
    #[serde(default = "current_timestamp")]
    pub timestamp: f64,
}

fn current_timestamp() -> f64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs_f64())
        .unwrap_or(0.0)
}

impl DronePosition {
    /// Create a new drone position with only required fields.
    pub fn new(drone_id: impl Into<String>, lat: f64, lon: f64, altitude_m: f64) -> Self {
        Self {
            drone_id: drone_id.into(),
            lat,
            lon,
            altitude_m,
            heading_deg: 0.0,
            speed_mps: 0.0,
            timestamp: current_timestamp(),
        }
    }

    /// Set heading and speed.
    pub fn with_velocity(mut self, heading_deg: f64, speed_mps: f64) -> Self {
        self.heading_deg = heading_deg;
        self.speed_mps = speed_mps;
        self
    }
}

/// Detected conflict between two drones.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Conflict {
    pub drone1_id: String,
    pub drone2_id: String,
    pub severity: ConflictSeverity,
    pub distance_m: f64,
    /// Seconds until closest approach
    pub time_to_closest: f64,
    pub closest_distance_m: f64,
    /// Predicted CPA (Closest Point of Approach) coordinates
    /// Midpoint between the two drones at time of closest approach
    pub cpa_lat: f64,
    pub cpa_lon: f64,
    pub cpa_altitude_m: f64,
    pub timestamp: f64,
}

/// Real-time conflict detection engine.
///
/// Uses position extrapolation to predict conflicts within a
/// configurable lookahead window.
pub struct ConflictDetector {
    /// How far ahead to predict (seconds)
    pub lookahead_seconds: f64,
    /// Minimum horizontal separation (meters)
    pub separation_horizontal_m: f64,
    /// Minimum vertical separation (meters)
    pub separation_vertical_m: f64,
    /// Multiplier for warning threshold
    pub warning_multiplier: f64,

    /// Tracked drone positions
    drones: HashMap<String, DronePosition>,
    /// Active conflicts (keyed by sorted drone ID pair)
    active_conflicts: HashMap<(String, String), Conflict>,
}

impl Default for ConflictDetector {
    fn default() -> Self {
        Self::new(20.0, 50.0, 30.0, 2.0)
    }
}

impl ConflictDetector {
    /// Create a new conflict detector with custom parameters.
    ///
    /// # Arguments
    /// * `lookahead_seconds` - How far ahead to predict (default 20s)
    /// * `separation_horizontal_m` - Minimum horizontal separation
    /// * `separation_vertical_m` - Minimum vertical separation
    /// * `warning_multiplier` - Multiplier for warning threshold
    pub fn new(
        lookahead_seconds: f64,
        separation_horizontal_m: f64,
        separation_vertical_m: f64,
        warning_multiplier: f64,
    ) -> Self {
        Self {
            lookahead_seconds,
            separation_horizontal_m,
            separation_vertical_m,
            warning_multiplier,
            drones: HashMap::new(),
            active_conflicts: HashMap::new(),
        }
    }

    /// Update tracked position for a drone.
    pub fn update_position(&mut self, position: DronePosition) {
        self.drones.insert(position.drone_id.clone(), position);
    }

    /// Remove a drone from tracking.
    pub fn remove_drone(&mut self, drone_id: &str) {
        self.drones.remove(drone_id);
        // Remove any conflicts involving this drone
        self.active_conflicts
            .retain(|(id1, id2), _| id1 != drone_id && id2 != drone_id);
    }

    /// Get all tracked drone positions.
    pub fn get_all_positions(&self) -> Vec<&DronePosition> {
        self.drones.values().collect()
    }

    /// Get number of tracked drones.
    pub fn drone_count(&self) -> usize {
        self.drones.len()
    }

    /// Calculate horizontal distance between two points in meters (Haversine formula).
    fn haversine_distance(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
        const R: f64 = 6_371_000.0; // Earth radius in meters

        let phi1 = lat1.to_radians();
        let phi2 = lat2.to_radians();
        let dphi = (lat2 - lat1).to_radians();
        let dlambda = (lon2 - lon1).to_radians();

        let a = (dphi / 2.0).sin().powi(2)
            + phi1.cos() * phi2.cos() * (dlambda / 2.0).sin().powi(2);

        2.0 * R * a.sqrt().atan2((1.0 - a).sqrt())
    }

    /// Predict drone position after time_offset_s seconds.
    fn predict_position(drone: &DronePosition, time_offset_s: f64) -> (f64, f64, f64) {
        if drone.speed_mps <= 0.0 {
            return (drone.lat, drone.lon, drone.altitude_m);
        }

        // Distance traveled
        let distance_m = drone.speed_mps * time_offset_s;

        // Convert heading to radians (0 = North, clockwise)
        let heading_rad = drone.heading_deg.to_radians();

        // Calculate offset in meters
        let north_m = distance_m * heading_rad.cos();
        let east_m = distance_m * heading_rad.sin();

        // Convert to lat/lon offset
        let lat_offset = north_m / 111_320.0;
        let lon_offset = east_m / (111_320.0 * drone.lat.to_radians().cos());

        (
            drone.lat + lat_offset,
            drone.lon + lon_offset,
            drone.altitude_m, // Assuming constant altitude
        )
    }

    /// Check separation between two positions.
    /// Returns (horizontal_distance_m, vertical_distance_m).
    fn check_separation(
        pos1: (f64, f64, f64),
        pos2: (f64, f64, f64),
    ) -> (f64, f64) {
        let horizontal = Self::haversine_distance(pos1.0, pos1.1, pos2.0, pos2.1);
        let vertical = (pos1.2 - pos2.2).abs();
        (horizontal, vertical)
    }

    /// Find time and distance of closest approach.
    /// Returns (time_to_closest_s, closest_distance_m, cpa_lat, cpa_lon, cpa_altitude_m).
    fn find_closest_approach(
        &self,
        drone1: &DronePosition,
        drone2: &DronePosition,
    ) -> (f64, f64, f64, f64, f64) {
        let mut min_distance = f64::INFINITY;
        let mut time_of_min = 0.0;
        let mut cpa_pos1 = (drone1.lat, drone1.lon, drone1.altitude_m);
        let mut cpa_pos2 = (drone2.lat, drone2.lon, drone2.altitude_m);

        // Sample at 1-second intervals
        for t in 0..=(self.lookahead_seconds as i32) {
            let t = t as f64;
            let pos1 = Self::predict_position(drone1, t);
            let pos2 = Self::predict_position(drone2, t);
            let (h_dist, v_dist) = Self::check_separation(pos1, pos2);

            // 3D distance (weighted vertical)
            let distance = (h_dist.powi(2) + v_dist.powi(2)).sqrt();

            if distance < min_distance {
                min_distance = distance;
                time_of_min = t;
                cpa_pos1 = pos1;
                cpa_pos2 = pos2;
            }
        }

        // CPA is the midpoint between the two predicted positions
        let cpa_lat = (cpa_pos1.0 + cpa_pos2.0) / 2.0;
        let cpa_lon = (cpa_pos1.1 + cpa_pos2.1) / 2.0;
        let cpa_altitude_m = (cpa_pos1.2 + cpa_pos2.2) / 2.0;

        (time_of_min, min_distance, cpa_lat, cpa_lon, cpa_altitude_m)
    }

    /// Check all tracked drones for conflicts.
    pub fn detect_conflicts(&mut self) -> Vec<Conflict> {
        let mut conflicts = Vec::new();
        let drone_list: Vec<_> = self.drones.values().collect();

        // Check all pairs
        for i in 0..drone_list.len() {
            for j in (i + 1)..drone_list.len() {
                let drone1 = drone_list[i];
                let drone2 = drone_list[j];

                // Check current separation
                let (h_dist, v_dist) = Self::check_separation(
                    (drone1.lat, drone1.lon, drone1.altitude_m),
                    (drone2.lat, drone2.lon, drone2.altitude_m),
                );
                let current_distance = (h_dist.powi(2) + v_dist.powi(2)).sqrt();

                // Check for current violation
                if h_dist < self.separation_horizontal_m
                    && v_dist < self.separation_vertical_m
                {
                    // Current position is the CPA for immediate violations
                    let cpa_lat = (drone1.lat + drone2.lat) / 2.0;
                    let cpa_lon = (drone1.lon + drone2.lon) / 2.0;
                    let cpa_altitude_m = (drone1.altitude_m + drone2.altitude_m) / 2.0;
                    
                    conflicts.push(Conflict {
                        drone1_id: drone1.drone_id.clone(),
                        drone2_id: drone2.drone_id.clone(),
                        severity: ConflictSeverity::Critical,
                        distance_m: current_distance,
                        time_to_closest: 0.0,
                        closest_distance_m: current_distance,
                        cpa_lat,
                        cpa_lon,
                        cpa_altitude_m,
                        timestamp: current_timestamp(),
                    });
                    continue;
                }

                // Check predicted conflicts
                let (time_to_closest, closest_distance, cpa_lat, cpa_lon, cpa_altitude_m) =
                    self.find_closest_approach(drone1, drone2);

                // Warning threshold
                let warning_h = self.separation_horizontal_m * self.warning_multiplier;

                let severity = if closest_distance < self.separation_horizontal_m {
                    ConflictSeverity::Critical
                } else if closest_distance < warning_h {
                    ConflictSeverity::Warning
                } else {
                    continue; // No conflict
                };

                conflicts.push(Conflict {
                    drone1_id: drone1.drone_id.clone(),
                    drone2_id: drone2.drone_id.clone(),
                    severity,
                    distance_m: current_distance,
                    time_to_closest,
                    closest_distance_m: closest_distance,
                    cpa_lat,
                    cpa_lon,
                    cpa_altitude_m,
                    timestamp: current_timestamp(),
                });
            }
        }

        // Update active conflicts
        self.active_conflicts.clear();
        for conflict in &conflicts {
            let key = if conflict.drone1_id < conflict.drone2_id {
                (conflict.drone1_id.clone(), conflict.drone2_id.clone())
            } else {
                (conflict.drone2_id.clone(), conflict.drone1_id.clone())
            };
            self.active_conflicts.insert(key, conflict.clone());
        }

        conflicts
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_haversine_distance() {
        // Test known distance: approx 111km per degree of latitude
        let dist = ConflictDetector::haversine_distance(0.0, 0.0, 1.0, 0.0);
        assert!((dist - 111_320.0).abs() < 1000.0); // Within 1km tolerance
    }

    #[test]
    fn test_no_conflict_when_far_apart() {
        let mut detector = ConflictDetector::default();

        detector.update_position(DronePosition::new("DRONE001", 33.0, -117.0, 50.0));
        detector.update_position(DronePosition::new("DRONE002", 34.0, -118.0, 50.0));

        let conflicts = detector.detect_conflicts();
        assert!(conflicts.is_empty());
    }

    #[test]
    fn test_critical_conflict_when_close() {
        let mut detector = ConflictDetector::default();

        // Two drones at nearly the same position
        detector.update_position(DronePosition::new("DRONE001", 33.6846, -117.8265, 50.0));
        detector.update_position(DronePosition::new("DRONE002", 33.6846, -117.8265, 50.0));

        let conflicts = detector.detect_conflicts();
        assert_eq!(conflicts.len(), 1);
        assert_eq!(conflicts[0].severity, ConflictSeverity::Critical);
    }
}
