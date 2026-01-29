//! Conflict detection module for ATC-Drone.
//!
//! Provides real-time conflict detection with lookahead prediction
//! for multiple drones operating in the same airspace.

use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::time::{SystemTime, UNIX_EPOCH};

const METERS_PER_DEG_LAT: f64 = 111_320.0;
const CPA_EPS: f64 = 1e-9;

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

#[derive(Debug, Clone, Copy)]
struct ClosestApproach {
    distance_m: f64,
    time_s: f64,
    pos1: (f64, f64, f64),
    pos2: (f64, f64, f64),
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
    #[serde(default)]
    pub velocity_z: f64,
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
            velocity_z: 0.0,
            timestamp: current_timestamp(),
        }
    }

    /// Set heading and speed.
    pub fn with_velocity(mut self, heading_deg: f64, speed_mps: f64, velocity_z: f64) -> Self {
        self.heading_deg = heading_deg;
        self.speed_mps = speed_mps;
        self.velocity_z = velocity_z;
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

    // Removed duplicate haversine_distance. Using crate::spatial::haversine_distance instead.

    /// Predict drone position after time_offset_s seconds.
    fn predict_position(drone: &DronePosition, time_offset_s: f64) -> (f64, f64, f64) {
        if drone.speed_mps <= 0.0 {
            let altitude_m = drone.altitude_m + drone.velocity_z * time_offset_s;
            return (drone.lat, drone.lon, altitude_m);
        }

        // Distance traveled
        let distance_m = drone.speed_mps * time_offset_s;

        // Convert heading to radians (0 = North, clockwise)
        let heading_rad = drone.heading_deg.to_radians();

        let (lat, lon) =
            crate::spatial::offset_by_bearing(drone.lat, drone.lon, distance_m, heading_rad);

        let altitude_m = drone.altitude_m + drone.velocity_z * time_offset_s;

        (lat, lon, altitude_m)
    }

    /// Check separation between two positions.
    /// Returns (horizontal_distance_m, vertical_distance_m).
    fn check_separation(pos1: (f64, f64, f64), pos2: (f64, f64, f64)) -> (f64, f64) {
        let horizontal = crate::spatial::haversine_distance(pos1.0, pos1.1, pos2.0, pos2.1);
        let vertical = (pos1.2 - pos2.2).abs();
        (horizontal, vertical)
    }

    /// Find time and distance of closest approach.
    /// Returns (severity, time_to_closest_s, closest_distance_m, cpa_lat, cpa_lon, cpa_altitude_m).
    fn predict_conflict(
        &self,
        drone1: &DronePosition,
        drone2: &DronePosition,
        warning_horizontal_m: f64,
        warning_vertical_m: f64,
    ) -> Option<(ConflictSeverity, f64, f64, f64, f64, f64)> {
        let lookahead = self.lookahead_seconds.max(0.0);
        if lookahead <= 0.0 {
            return None;
        }

        let ref_lat = (drone1.lat + drone2.lat) / 2.0;
        let ref_lon = (drone1.lon + drone2.lon) / 2.0;

        let (d1_x, d1_y) = project_xy(drone1.lat, drone1.lon, ref_lat, ref_lon);
        let (d2_x, d2_y) = project_xy(drone2.lat, drone2.lon, ref_lat, ref_lon);

        let (v1_x, v1_y) = velocity_xy(drone1);
        let (v2_x, v2_y) = velocity_xy(drone2);

        let rel_pos_x = d2_x - d1_x;
        let rel_pos_y = d2_y - d1_y;
        let rel_pos_z = drone2.altitude_m - drone1.altitude_m;

        let rel_vel_x = v2_x - v1_x;
        let rel_vel_y = v2_y - v1_y;
        let rel_vel_z = drone2.velocity_z - drone1.velocity_z;

        let best = if let Some(window) = conflict_time_window(
            rel_pos_x,
            rel_pos_y,
            rel_vel_x,
            rel_vel_y,
            rel_pos_z,
            rel_vel_z,
            self.separation_horizontal_m,
            self.separation_vertical_m,
            lookahead,
        ) {
            (
                ConflictSeverity::Critical,
                best_approach_in_window(drone1, drone2, window),
            )
        } else if let Some(window) = conflict_time_window(
            rel_pos_x,
            rel_pos_y,
            rel_vel_x,
            rel_vel_y,
            rel_pos_z,
            rel_vel_z,
            warning_horizontal_m,
            warning_vertical_m,
            lookahead,
        ) {
            (
                ConflictSeverity::Warning,
                best_approach_in_window(drone1, drone2, window),
            )
        } else {
            return None;
        };

        let (severity, best) = best;

        let cpa_lat = (best.pos1.0 + best.pos2.0) / 2.0;
        let cpa_lon = (best.pos1.1 + best.pos2.1) / 2.0;
        let cpa_altitude_m = (best.pos1.2 + best.pos2.2) / 2.0;

        Some((
            severity,
            best.time_s,
            best.distance_m,
            cpa_lat,
            cpa_lon,
            cpa_altitude_m,
        ))
    }

    /// Check all tracked drones for conflicts.
    pub fn detect_conflicts(&mut self) -> Vec<Conflict> {
        let mut conflicts = Vec::new();
        let drone_list: Vec<DronePosition> = self.drones.values().cloned().collect();
        if drone_list.len() < 2 {
            self.active_conflicts.clear();
            return conflicts;
        }

        let max_speed = drone_list
            .iter()
            .map(|drone| drone.speed_mps)
            .fold(0.0, f64::max);
        let warning_h = self.separation_horizontal_m * self.warning_multiplier;
        let warning_v = self.separation_vertical_m * self.warning_multiplier;
        let max_threshold = self.separation_horizontal_m.max(warning_h);
        let cell_size_m = (max_threshold + max_speed * self.lookahead_seconds).max(1.0);

        let (ref_lat, ref_lon) = average_lat_lon(&drone_list);
        let mut grid: HashMap<(i32, i32), Vec<usize>> = HashMap::new();
        let mut projected: Vec<(f64, f64)> = Vec::with_capacity(drone_list.len());

        for (idx, drone) in drone_list.iter().enumerate() {
            let (x, y) = project_xy(drone.lat, drone.lon, ref_lat, ref_lon);
            projected.push((x, y));
            let cell = (
                (x / cell_size_m).floor() as i32,
                (y / cell_size_m).floor() as i32,
            );
            grid.entry(cell).or_default().push(idx);
        }

        // Check nearby pairs using a spatial grid to avoid O(N^2) scans.
        for i in 0..drone_list.len() {
            let drone1 = &drone_list[i];
            let (x, y) = projected[i];
            let cell_x = (x / cell_size_m).floor() as i32;
            let cell_y = (y / cell_size_m).floor() as i32;
            let search_radius_m =
                max_threshold + (drone1.speed_mps + max_speed) * self.lookahead_seconds;
            let search_cells = (search_radius_m / cell_size_m).ceil() as i32;

            for dx in -search_cells..=search_cells {
                for dy in -search_cells..=search_cells {
                    let Some(indices) = grid.get(&(cell_x + dx, cell_y + dy)) else {
                        continue;
                    };

                    for &j in indices {
                        if j <= i {
                            continue;
                        }
                        let drone2 = &drone_list[j];

                        // Check current separation
                        let (h_dist, v_dist) = Self::check_separation(
                            (drone1.lat, drone1.lon, drone1.altitude_m),
                            (drone2.lat, drone2.lon, drone2.altitude_m),
                        );
                        let max_possible_distance = max_threshold
                            + (drone1.speed_mps + drone2.speed_mps) * self.lookahead_seconds;
                        if h_dist > max_possible_distance {
                            continue;
                        }
                        let current_distance = (h_dist.powi(2) + v_dist.powi(2)).sqrt();

                        // Check for current violation
                        if h_dist < self.separation_horizontal_m
                            && v_dist < self.separation_vertical_m
                        {
                            // Current position is the CPA for immediate violations
                            let cpa_lat = (drone1.lat + drone2.lat) / 2.0;
                            let cpa_lon = (drone1.lon + drone2.lon) / 2.0;
                            let cpa_altitude_m = (drone1.altitude_m + drone2.altitude_m) / 2.0;
                            let (drone1_id, drone2_id) = if drone1.drone_id <= drone2.drone_id {
                                (drone1.drone_id.clone(), drone2.drone_id.clone())
                            } else {
                                (drone2.drone_id.clone(), drone1.drone_id.clone())
                            };

                            conflicts.push(Conflict {
                                drone1_id,
                                drone2_id,
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

                        let Some((
                            severity,
                            time_to_closest,
                            closest_distance,
                            cpa_lat,
                            cpa_lon,
                            cpa_altitude_m,
                        )) = self.predict_conflict(drone1, drone2, warning_h, warning_v)
                        else {
                            continue;
                        };

                        let (drone1_id, drone2_id) = if drone1.drone_id <= drone2.drone_id {
                            (drone1.drone_id.clone(), drone2.drone_id.clone())
                        } else {
                            (drone2.drone_id.clone(), drone1.drone_id.clone())
                        };

                        conflicts.push(Conflict {
                            drone1_id,
                            drone2_id,
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

fn velocity_xy(drone: &DronePosition) -> (f64, f64) {
    if drone.speed_mps.abs() <= CPA_EPS {
        return (0.0, 0.0);
    }
    let heading_rad = drone.heading_deg.to_radians();

    // Local ENU: x = east, y = north; heading is 0=north, 90=east.
    let vel_x = drone.speed_mps * heading_rad.sin();
    let vel_y = drone.speed_mps * heading_rad.cos();
    (vel_x, vel_y)
}

fn conflict_time_window(
    rel_pos_x: f64,
    rel_pos_y: f64,
    rel_vel_x: f64,
    rel_vel_y: f64,
    rel_pos_z: f64,
    rel_vel_z: f64,
    horiz_threshold_m: f64,
    vert_threshold_m: f64,
    lookahead_s: f64,
) -> Option<(f64, f64)> {
    let (h_start, h_end) = horizontal_time_window(
        rel_pos_x,
        rel_pos_y,
        rel_vel_x,
        rel_vel_y,
        horiz_threshold_m,
        lookahead_s,
    )?;
    let (v_start, v_end) =
        vertical_time_window(rel_pos_z, rel_vel_z, vert_threshold_m, lookahead_s)?;

    let start = h_start.max(v_start);
    let end = h_end.min(v_end);

    if start <= end {
        Some((start, end))
    } else {
        None
    }
}

fn horizontal_time_window(
    rel_pos_x: f64,
    rel_pos_y: f64,
    rel_vel_x: f64,
    rel_vel_y: f64,
    threshold_m: f64,
    lookahead_s: f64,
) -> Option<(f64, f64)> {
    let a = rel_vel_x * rel_vel_x + rel_vel_y * rel_vel_y;
    let b = 2.0 * (rel_pos_x * rel_vel_x + rel_pos_y * rel_vel_y);
    let c = rel_pos_x * rel_pos_x + rel_pos_y * rel_pos_y - threshold_m * threshold_m;

    if a.abs() <= CPA_EPS {
        return if c <= 0.0 {
            Some((0.0, lookahead_s))
        } else {
            None
        };
    }

    let disc = b * b - 4.0 * a * c;
    if disc < 0.0 {
        return None;
    }

    let sqrt_disc = disc.sqrt();
    let t_low = (-b - sqrt_disc) / (2.0 * a);
    let t_high = (-b + sqrt_disc) / (2.0 * a);

    let start = t_low.min(t_high).max(0.0);
    let end = t_low.max(t_high).min(lookahead_s);

    if start <= end {
        Some((start, end))
    } else {
        None
    }
}

fn vertical_time_window(
    rel_pos_z: f64,
    rel_vel_z: f64,
    threshold_m: f64,
    lookahead_s: f64,
) -> Option<(f64, f64)> {
    if rel_vel_z.abs() <= CPA_EPS {
        return if rel_pos_z.abs() <= threshold_m {
            Some((0.0, lookahead_s))
        } else {
            None
        };
    }

    let t1 = (-threshold_m - rel_pos_z) / rel_vel_z;
    let t2 = (threshold_m - rel_pos_z) / rel_vel_z;

    let start = t1.min(t2).max(0.0);
    let end = t1.max(t2).min(lookahead_s);

    if start <= end {
        Some((start, end))
    } else {
        None
    }
}

fn best_approach_in_window(
    drone1: &DronePosition,
    drone2: &DronePosition,
    window_s: (f64, f64),
) -> ClosestApproach {
    let (start_s, end_s) = window_s;
    let start_s = start_s.max(0.0);
    let end_s = end_s.max(start_s);

    let ref_lat = (drone1.lat + drone2.lat) / 2.0;
    let ref_lon = (drone1.lon + drone2.lon) / 2.0;

    let (d1_x, d1_y) = project_xy(drone1.lat, drone1.lon, ref_lat, ref_lon);
    let (d2_x, d2_y) = project_xy(drone2.lat, drone2.lon, ref_lat, ref_lon);

    let (v1_x, v1_y) = velocity_xy(drone1);
    let (v2_x, v2_y) = velocity_xy(drone2);

    let rel_pos_x = d2_x - d1_x;
    let rel_pos_y = d2_y - d1_y;
    let rel_pos_z = drone2.altitude_m - drone1.altitude_m;

    let rel_vel_x = v2_x - v1_x;
    let rel_vel_y = v2_y - v1_y;
    let rel_vel_z = drone2.velocity_z - drone1.velocity_z;

    let rel_speed_sq = rel_vel_x * rel_vel_x + rel_vel_y * rel_vel_y + rel_vel_z * rel_vel_z;
    let t_star = if rel_speed_sq.abs() <= CPA_EPS {
        start_s
    } else {
        let dot = rel_pos_x * rel_vel_x + rel_pos_y * rel_vel_y + rel_pos_z * rel_vel_z;
        (-dot / rel_speed_sq).clamp(start_s, end_s)
    };

    let candidates = [start_s, t_star, end_s];
    let mut best: Option<ClosestApproach> = None;

    for &t in &candidates {
        let distance_m = {
            let dx = rel_pos_x + rel_vel_x * t;
            let dy = rel_pos_y + rel_vel_y * t;
            let dz = rel_pos_z + rel_vel_z * t;
            (dx * dx + dy * dy + dz * dz).sqrt()
        };

        let pos1 = ConflictDetector::predict_position(drone1, t);
        let pos2 = ConflictDetector::predict_position(drone2, t);

        let replace = best
            .as_ref()
            .map(|current| distance_m < current.distance_m)
            .unwrap_or(true);
        if replace {
            best = Some(ClosestApproach {
                distance_m,
                time_s: t,
                pos1,
                pos2,
            });
        }
    }

    best.expect("candidates always yields a best")
}

fn average_lat_lon(drones: &[DronePosition]) -> (f64, f64) {
    let count = drones.len() as f64;
    let (sum_lat, sum_lon) = drones.iter().fold((0.0, 0.0), |acc, drone| {
        (acc.0 + drone.lat, acc.1 + drone.lon)
    });
    (sum_lat / count, sum_lon / count)
}

fn project_xy(lat: f64, lon: f64, ref_lat: f64, ref_lon: f64) -> (f64, f64) {
    let meters_per_deg_lon = (ref_lat.to_radians().cos().abs().max(0.01)) * METERS_PER_DEG_LAT;
    let x = (lon - ref_lon) * meters_per_deg_lon;
    let y = (lat - ref_lat) * METERS_PER_DEG_LAT;
    (x, y)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_haversine_distance() {
        use crate::spatial::haversine_distance;
        // Test known distance: approx 111km per degree of latitude
        let dist = haversine_distance(0.0, 0.0, 1.0, 0.0);
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

    #[test]
    fn test_vertical_conflict_detection() {
        let mut detector = ConflictDetector::default();

        let (lat2, lon2) =
            crate::spatial::offset_by_bearing(0.0, 0.0, 49.0, std::f64::consts::FRAC_PI_2);

        detector.update_position(DronePosition::new("DRONE001", 0.0, 0.0, 0.0));
        detector.update_position(
            DronePosition::new("DRONE002", lat2, lon2, 80.0).with_velocity(0.0, 0.0, -3.0),
        );

        let conflicts = detector.detect_conflicts();
        assert!(!conflicts.is_empty());
        assert_eq!(conflicts[0].severity, ConflictSeverity::Critical);
    }

    #[test]
    fn detects_near_miss_between_whole_seconds() {
        let mut detector = ConflictDetector::default();

        let base_lat = 0.0;
        let base_lon = 0.0;

        // Drones pass within the critical horizontal separation only between integer seconds.
        // At t=0 and t=1, the separation is > 50m; at t=0.5 it is < 50m.
        let d_lon = crate::spatial::meters_to_lon(10.0, base_lat);
        let d_lat = crate::spatial::meters_to_lat(49.0, base_lat);

        detector.update_position(
            DronePosition::new("A", base_lat, base_lon - d_lon, 50.0)
                .with_velocity(90.0, 20.0, 0.0),
        );
        detector.update_position(
            DronePosition::new("B", base_lat + d_lat, base_lon + d_lon, 50.0)
                .with_velocity(270.0, 20.0, 0.0),
        );

        let conflicts = detector.detect_conflicts();
        assert_eq!(conflicts.len(), 1, "expected conflict between A and B");
        assert_eq!(conflicts[0].severity, ConflictSeverity::Critical);
        assert!(
            (conflicts[0].time_to_closest - 0.5).abs() < 0.01,
            "expected CPA at t=0.5, got {}",
            conflicts[0].time_to_closest
        );
        assert!(
            (conflicts[0].closest_distance_m - 49.0).abs() < 0.5,
            "expected closest distance near 49m, got {}",
            conflicts[0].closest_distance_m
        );
    }
}
