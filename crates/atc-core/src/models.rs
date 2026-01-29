//! Core data models for the ATC system.

use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};

/// Telemetry data received from a drone.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Telemetry {
    pub drone_id: String,
    /// Owner/operator ID for user-specific filtering
    #[serde(default)]
    pub owner_id: Option<String>,
    pub lat: f64,
    pub lon: f64,
    pub altitude_m: f64,
    #[serde(default)]
    pub velocity_x: f64,
    #[serde(default)]
    pub velocity_y: f64,
    #[serde(default)]
    pub velocity_z: f64,
    #[serde(default)]
    pub heading_deg: f64,
    #[serde(default)]
    pub speed_mps: f64,
    pub timestamp: DateTime<Utc>,
}

/// Current state of a registered drone.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DroneState {
    pub drone_id: String,
    /// Owner/operator ID for user-specific filtering
    pub owner_id: Option<String>,
    pub lat: f64,
    pub lon: f64,
    pub altitude_m: f64,
    pub heading_deg: f64,
    pub speed_mps: f64,
    pub velocity_x: f64,
    pub velocity_y: f64,
    pub velocity_z: f64,
    pub last_update: DateTime<Utc>,
    pub status: DroneStatus,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum DroneStatus {
    /// Normal operation
    #[default]
    Active,
    /// Executing HOLD command
    Holding,
    /// Lost communication (timeout)
    Lost,
    /// Landed/inactive
    Inactive,
}

impl DroneState {
    /// Create a new DroneState from telemetry.
    pub fn from_telemetry(telemetry: &Telemetry) -> Self {
        Self {
            drone_id: telemetry.drone_id.clone(),
            owner_id: telemetry.owner_id.clone(),
            lat: telemetry.lat,
            lon: telemetry.lon,
            altitude_m: telemetry.altitude_m,
            heading_deg: telemetry.heading_deg,
            speed_mps: telemetry.speed_mps,
            velocity_x: telemetry.velocity_x,
            velocity_y: telemetry.velocity_y,
            velocity_z: telemetry.velocity_z,
            last_update: telemetry.timestamp,
            status: DroneStatus::Active,
        }
    }

    /// Update state from new telemetry.
    pub fn update(&mut self, telemetry: &Telemetry) {
        self.lat = telemetry.lat;
        self.lon = telemetry.lon;
        self.altitude_m = telemetry.altitude_m;
        self.heading_deg = telemetry.heading_deg;
        self.speed_mps = telemetry.speed_mps;
        self.velocity_x = telemetry.velocity_x;
        self.velocity_y = telemetry.velocity_y;
        self.velocity_z = telemetry.velocity_z;
        self.last_update = telemetry.timestamp;
        if telemetry.owner_id.is_some() {
            self.owner_id = telemetry.owner_id.clone();
        }
        self.status = DroneStatus::Active;
    }
}

/// A flight plan submitted by an operator.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FlightPlan {
    pub flight_id: String,
    pub drone_id: String,
    pub owner_id: Option<String>,
    pub waypoints: Vec<Waypoint>,
    #[serde(default)]
    pub trajectory_log: Option<Vec<TrajectoryPoint>>,
    #[serde(default)]
    pub metadata: Option<FlightPlanMetadata>,
    pub status: FlightStatus,
    /// Scheduled departure time
    pub departure_time: DateTime<Utc>,
    /// Estimated arrival time
    pub arrival_time: Option<DateTime<Utc>>,
    pub created_at: DateTime<Utc>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FlightPlanRequest {
    pub drone_id: String,
    pub owner_id: Option<String>,
    /// If provided, specific waypoints are used. If not, a route is generated.
    pub waypoints: Option<Vec<Waypoint>>,
    #[serde(default)]
    pub trajectory_log: Option<Vec<TrajectoryPoint>>,
    #[serde(default)]
    pub metadata: Option<FlightPlanMetadata>,
    /// Alternatively, provide origin/destination for auto-generation
    pub origin: Option<Waypoint>,
    pub destination: Option<Waypoint>,
    pub departure_time: Option<DateTime<Utc>>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Waypoint {
    pub lat: f64,
    pub lon: f64,
    pub altitude_m: f64,
    pub speed_mps: Option<f64>,
}

/// Time-stamped trajectory point for high-fidelity conflict checks.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrajectoryPoint {
    pub lat: f64,
    pub lon: f64,
    #[serde(alias = "alt")]
    pub altitude_m: f64,
    #[serde(default, alias = "time_offset")]
    pub time_offset_s: Option<f64>,
}

/// Metadata captured with a flight plan submission.
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct FlightPlanMetadata {
    #[serde(default)]
    pub drone_speed_mps: Option<f64>,
    #[serde(default)]
    pub total_distance_m: Option<f64>,
    #[serde(default)]
    pub total_flight_time_s: Option<f64>,
    #[serde(default)]
    pub trajectory_points: Option<u64>,
    #[serde(default)]
    pub planned_altitude_m: Option<f64>,
    #[serde(default)]
    pub max_obstacle_height_m: Option<f64>,
    #[serde(default)]
    pub faa_compliant: Option<bool>,
    #[serde(default)]
    pub submitted_at: Option<String>,
    #[serde(default)]
    pub blender_declaration_id: Option<String>,
    #[serde(default)]
    pub operation_type: Option<u8>,
    #[serde(default)]
    pub battery_capacity_min: Option<f64>,
    #[serde(default)]
    pub battery_reserve_min: Option<f64>,
    #[serde(default)]
    pub clearance_m: Option<f64>,
    #[serde(default)]
    pub compliance_override_enabled: Option<bool>,
    #[serde(default)]
    pub compliance_override_notes: Option<String>,
    #[serde(default)]
    pub compliance_report: Option<serde_json::Value>,
    /// Optional scheduling priority (lower = higher priority).
    #[serde(default)]
    pub scheduling_priority: Option<u32>,
    /// Original requested departure time (RFC3339) before ATC scheduling/slotting.
    #[serde(default)]
    pub requested_departure_time: Option<String>,
    /// Scheduled delay applied by ATC, in seconds.
    #[serde(default)]
    pub scheduled_delay_s: Option<u64>,
    /// If the plan is reserved, the time (RFC3339) when the reservation expires.
    #[serde(default)]
    pub reservation_expires_at: Option<String>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum FlightStatus {
    /// Reserved (slot held), not yet confirmed for execution.
    Reserved,
    /// Received but not yet approved
    Pending,
    /// Approved by ATC, ready for takeoff
    Approved,
    /// Currently flying
    Active,
    /// Flight completed successfully
    Completed,
    /// Rejected by ATC (conflict, etc.)
    Rejected,
    /// Cancelled by operator
    Cancelled,
}

/// Command issued to a drone.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Command {
    pub command_id: String,
    pub drone_id: String,
    pub command_type: CommandType,
    pub issued_at: DateTime<Utc>,
    pub expires_at: Option<DateTime<Utc>>,
    pub acknowledged: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "SCREAMING_SNAKE_CASE")]
pub enum CommandType {
    /// Hold position (loiter)
    Hold { duration_secs: u32 },
    /// Change altitude
    AltitudeChange { target_altitude_m: f64 },
    /// Reroute to new waypoints (conflict avoidance)
    Reroute {
        waypoints: Vec<Waypoint>,
        reason: Option<String>,
    },
    /// Resume normal operation
    Resume,
}

// ========== GEOFENCE MODELS ==========

/// A geographic boundary defining restricted airspace.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Geofence {
    pub id: String,
    pub name: String,
    pub geofence_type: GeofenceType,
    /// Polygon vertices as [lat, lon] pairs (closed ring - first == last)
    pub polygon: Vec<[f64; 2]>,
    /// Lower altitude limit in meters (floor)
    pub lower_altitude_m: f64,
    /// Upper altitude limit in meters (ceiling)
    pub upper_altitude_m: f64,
    /// Whether the geofence is currently active
    pub active: bool,
    pub created_at: DateTime<Utc>,
}

/// Type of geofence/restricted area.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum GeofenceType {
    /// No flights allowed
    NoFlyZone,
    /// Flights allowed with authorization
    RestrictedArea,
    /// Temporary flight restriction (TFR)
    TemporaryRestriction,
    /// Advisory only (not enforced)
    Advisory,
}

// ========== CONFORMANCE MONITORING ==========

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConformanceRecord {
    pub id: String,
    pub flight_declaration_id: String,
    pub aircraft_id: String,
    pub conformance_state: i32,
    pub conformance_state_label: String,
    pub conformance_state_code: Option<String>,
    pub timestamp: String,
    pub description: String,
    pub event_type: String,
    pub geofence_breach: bool,
    pub geofence_id: Option<String>,
    pub resolved: bool,
    pub created_at: String,
    pub updated_at: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConformanceStatus {
    pub drone_id: String,
    pub owner_id: Option<String>,
    pub status: String,
    pub last_checked: DateTime<Utc>,
    pub record: Option<ConformanceRecord>,
}

// ========== DAA (DETECT AND AVOID) ==========

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum DaaSeverity {
    Advisory,
    Warning,
    Critical,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DaaAdvisory {
    pub advisory_id: String,
    pub drone_id: String,
    pub owner_id: Option<String>,
    /// Advisory source (conformance, conflict, or other backend)
    pub source: String,
    pub severity: DaaSeverity,
    /// Recommended action (monitor, hold, reroute)
    pub action: String,
    pub description: String,
    /// Optional related identifier (conflict id, geofence id, etc.)
    pub related_id: Option<String>,
    pub record: Option<ConformanceRecord>,
    pub created_at: DateTime<Utc>,
    pub updated_at: DateTime<Utc>,
    pub resolved: bool,
}

/// Request to create a new geofence.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CreateGeofenceRequest {
    pub name: String,
    pub geofence_type: GeofenceType,
    pub polygon: Vec<[f64; 2]>,
    pub lower_altitude_m: Option<f64>,
    pub upper_altitude_m: Option<f64>,
}

/// Request to update an existing geofence.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UpdateGeofenceRequest {
    pub name: Option<String>,
    pub geofence_type: Option<GeofenceType>,
    pub polygon: Option<Vec<[f64; 2]>>,
    pub lower_altitude_m: Option<f64>,
    pub upper_altitude_m: Option<f64>,
    pub active: Option<bool>,
}

impl Geofence {
    fn contains_point_2d(&self, lat: f64, lon: f64) -> bool {
        // Ray casting: count intersections with polygon edges
        let mut inside = false;
        let n = self.polygon.len();
        if n < 3 {
            return false;
        }

        let mut j = n - 1;
        for i in 0..n {
            let yi = self.polygon[i][0];
            let xi = self.polygon[i][1];
            let yj = self.polygon[j][0];
            let xj = self.polygon[j][1];

            if ((yi > lat) != (yj > lat)) && (lon < (xj - xi) * (lat - yi) / (yj - yi) + xi) {
                inside = !inside;
            }
            j = i;
        }

        inside
    }

    /// Check if a point is inside this geofence's polygon.
    /// Uses ray casting algorithm.
    pub fn contains_point(&self, lat: f64, lon: f64, altitude_m: f64) -> bool {
        // Check altitude bounds first
        if altitude_m < self.lower_altitude_m || altitude_m > self.upper_altitude_m {
            return false;
        }

        self.contains_point_2d(lat, lon)
    }

    /// Validate geofence configuration.
    /// Returns list of validation errors (empty = valid).
    pub fn validate(&self) -> Vec<String> {
        let mut errors = Vec::new();

        // Check polygon has at least 3 points
        if self.polygon.len() < 3 {
            errors.push("Polygon must have at least 3 vertices".to_string());
        }

        // Check polygon is closed (first == last)
        if let (Some(first), Some(last)) = (self.polygon.first(), self.polygon.last()) {
            if (first[0] - last[0]).abs() > 0.0001 || (first[1] - last[1]).abs() > 0.0001 {
                errors.push("Polygon must be closed (first vertex must equal last)".to_string());
            }
        }

        // Check altitude bounds
        if self.lower_altitude_m >= self.upper_altitude_m {
            errors.push(format!(
                "Lower altitude ({}) must be less than upper altitude ({})",
                self.lower_altitude_m, self.upper_altitude_m
            ));
        }

        // Check altitude is reasonable (0-10000m)
        if self.lower_altitude_m < 0.0 {
            errors.push("Lower altitude cannot be negative".to_string());
        }
        if self.upper_altitude_m > 10000.0 {
            errors.push("Upper altitude exceeds maximum (10000m)".to_string());
        }

        errors
    }

    /// Check if geofence is valid.
    pub fn is_valid(&self) -> bool {
        self.validate().is_empty()
    }

    /// Check if a route segment intersects this geofence.
    /// Returns true if any point along the segment is inside the geofence.
    pub fn intersects_segment(
        &self,
        lat1: f64,
        lon1: f64,
        alt1: f64,
        lat2: f64,
        lon2: f64,
        alt2: f64,
    ) -> bool {
        if self.polygon.len() < 3 {
            return false;
        }

        let alt_min = self.lower_altitude_m.min(self.upper_altitude_m);
        let alt_max = self.lower_altitude_m.max(self.upper_altitude_m);

        // Restrict the segment to the portion where altitude overlaps the geofence altitude band.
        let (t_start, t_end) = if (alt2 - alt1).abs() <= f64::EPSILON {
            if alt1 < alt_min || alt1 > alt_max {
                return false;
            }
            (0.0, 1.0)
        } else {
            let t0 = (alt_min - alt1) / (alt2 - alt1);
            let t1 = (alt_max - alt1) / (alt2 - alt1);
            let enter = t0.min(t1).max(0.0);
            let exit = t0.max(t1).min(1.0);
            if enter > exit {
                return false;
            }
            (enter, exit)
        };

        let seg_lat1 = lat1 + t_start * (lat2 - lat1);
        let seg_lon1 = lon1 + t_start * (lon2 - lon1);
        let seg_lat2 = lat1 + t_end * (lat2 - lat1);
        let seg_lon2 = lon1 + t_end * (lon2 - lon1);

        // Endpoint-in-polygon checks cover:
        // - segment fully inside the polygon
        // - degenerate segments (start==end)
        if self.contains_point_2d(seg_lat1, seg_lon1) || self.contains_point_2d(seg_lat2, seg_lon2)
        {
            return true;
        }

        // Segment–polygon edge intersection (local planar ENU approximation).
        let (sum_lat, sum_lon) = self
            .polygon
            .iter()
            .fold((0.0_f64, 0.0_f64), |(acc_lat, acc_lon), point| {
                (acc_lat + point[0], acc_lon + point[1])
            });
        let poly_len = self.polygon.len() as f64;
        let poly_avg_lat = sum_lat / poly_len;
        let poly_avg_lon = sum_lon / poly_len;

        let ref_lat = (seg_lat1 + seg_lat2 + poly_avg_lat) / 3.0;
        let ref_lon = (seg_lon1 + seg_lon2 + poly_avg_lon) / 3.0;

        let seg_start_xy = (
            crate::spatial::lon_to_meters(seg_lon1 - ref_lon, ref_lat),
            crate::spatial::lat_to_meters(seg_lat1 - ref_lat, ref_lat),
        );
        let seg_end_xy = (
            crate::spatial::lon_to_meters(seg_lon2 - ref_lon, ref_lat),
            crate::spatial::lat_to_meters(seg_lat2 - ref_lat, ref_lat),
        );

        for idx in 0..self.polygon.len() {
            let next_idx = (idx + 1) % self.polygon.len();
            let edge_lat1 = self.polygon[idx][0];
            let edge_lon1 = self.polygon[idx][1];
            let edge_lat2 = self.polygon[next_idx][0];
            let edge_lon2 = self.polygon[next_idx][1];

            let edge_start_xy = (
                crate::spatial::lon_to_meters(edge_lon1 - ref_lon, ref_lat),
                crate::spatial::lat_to_meters(edge_lat1 - ref_lat, ref_lat),
            );
            let edge_end_xy = (
                crate::spatial::lon_to_meters(edge_lon2 - ref_lon, ref_lat),
                crate::spatial::lat_to_meters(edge_lat2 - ref_lat, ref_lat),
            );

            // Skip zero-length edges.
            if (edge_start_xy.0 - edge_end_xy.0).abs() <= f64::EPSILON
                && (edge_start_xy.1 - edge_end_xy.1).abs() <= f64::EPSILON
            {
                continue;
            }

            if crate::spatial::segments_intersect_2d(
                seg_start_xy,
                seg_end_xy,
                edge_start_xy,
                edge_end_xy,
            ) {
                return true;
            }
        }

        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn geofence_intersects_segment_detects_crossing_between_samples() {
        // Long segment + small fence: the old sampling implementation could miss if the fence fell between samples.
        let fence_center_lat = 0.0;
        let fence_center_lon = 0.0075;
        let d = 0.0002;

        let geofence = Geofence {
            id: "g1".to_string(),
            name: "test".to_string(),
            geofence_type: GeofenceType::NoFlyZone,
            polygon: vec![
                [fence_center_lat - d, fence_center_lon - d],
                [fence_center_lat - d, fence_center_lon + d],
                [fence_center_lat + d, fence_center_lon + d],
                [fence_center_lat + d, fence_center_lon - d],
                [fence_center_lat - d, fence_center_lon - d],
            ],
            lower_altitude_m: 0.0,
            upper_altitude_m: 100.0,
            active: true,
            created_at: chrono::Utc::now(),
        };

        assert!(geofence.intersects_segment(0.0, 0.0, 50.0, 0.0, 1.0, 50.0));
    }

    #[test]
    fn geofence_intersects_segment_requires_horizontal_and_altitude_overlap_at_same_time() {
        // The full segment crosses the fence horizontally, and the segment has altitude overlap somewhere,
        // but the altitude overlap occurs in a different part of the segment; this should not count as an intersection.
        let fence_center_lat = 0.0;
        let fence_center_lon = 0.1;
        let d = 0.0002;

        let geofence = Geofence {
            id: "g2".to_string(),
            name: "altitude_test".to_string(),
            geofence_type: GeofenceType::NoFlyZone,
            polygon: vec![
                [fence_center_lat - d, fence_center_lon - d],
                [fence_center_lat - d, fence_center_lon + d],
                [fence_center_lat + d, fence_center_lon + d],
                [fence_center_lat + d, fence_center_lon - d],
                [fence_center_lat - d, fence_center_lon - d],
            ],
            lower_altitude_m: 100.0,
            upper_altitude_m: 150.0,
            active: true,
            created_at: chrono::Utc::now(),
        };

        assert!(!geofence.intersects_segment(0.0, 0.0, 0.0, 0.0, 1.0, 200.0));
    }
}
