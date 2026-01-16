//! Core data models for the ATC system.

use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};

/// Telemetry data received from a drone.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Telemetry {
    pub drone_id: String,
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
        self.status = DroneStatus::Active;
    }
}

/// A flight plan submitted by an operator.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FlightPlan {
    pub flight_id: String,
    pub drone_id: String,
    pub waypoints: Vec<Waypoint>,
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
    /// If provided, specific waypoints are used. If not, a route is generated.
    pub waypoints: Option<Vec<Waypoint>>,
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

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum FlightStatus {
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

/// Request to create a new geofence.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CreateGeofenceRequest {
    pub name: String,
    pub geofence_type: GeofenceType,
    pub polygon: Vec<[f64; 2]>,
    pub lower_altitude_m: Option<f64>,
    pub upper_altitude_m: Option<f64>,
}

impl Geofence {
    /// Check if a point is inside this geofence's polygon.
    /// Uses ray casting algorithm.
    pub fn contains_point(&self, lat: f64, lon: f64, altitude_m: f64) -> bool {
        // Check altitude bounds first
        if altitude_m < self.lower_altitude_m || altitude_m > self.upper_altitude_m {
            return false;
        }
        
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
            
            if ((yi > lat) != (yj > lat)) 
                && (lon < (xj - xi) * (lat - yi) / (yj - yi) + xi) 
            {
                inside = !inside;
            }
            j = i;
        }
        
        inside
    }
}
