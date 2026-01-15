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

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum DroneStatus {
    /// Normal operation
    Active,
    /// Executing HOLD command
    Holding,
    /// Lost communication (timeout)
    Lost,
    /// Landed/inactive
    Inactive,
}

impl Default for DroneStatus {
    fn default() -> Self {
        Self::Active
    }
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

/// A mission (flight plan) for a drone.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Mission {
    pub mission_id: String,
    pub drone_id: String,
    pub waypoints: Vec<Waypoint>,
    pub status: MissionStatus,
    pub created_at: DateTime<Utc>,
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
pub enum MissionStatus {
    Pending,
    Active,
    Completed,
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
    /// Reroute to new waypoint
    Reroute { waypoint: Waypoint },
    /// Resume normal operation
    Resume,
}
