//! Safety rules and thresholds for the ATC system.

use serde::{Deserialize, Serialize};

/// Configuration for safety rules.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SafetyRules {
    /// Minimum horizontal separation in meters
    pub min_horizontal_separation_m: f64,
    /// Minimum vertical separation in meters
    pub min_vertical_separation_m: f64,
    /// Lookahead window for conflict prediction in seconds
    pub lookahead_seconds: f64,
    /// Multiplier for warning threshold (warning at separation * multiplier)
    pub warning_multiplier: f64,
    /// Timeout before drone is marked as lost (seconds)
    pub drone_timeout_secs: u64,
    /// Maximum allowed altitude in meters
    pub max_altitude_m: f64,
    /// Minimum allowed altitude in meters
    pub min_altitude_m: f64,
    /// Altitude bands for separation (each band is min_vertical_separation_m tall)
    pub altitude_bands: Vec<AltitudeBand>,
}

impl Default for SafetyRules {
    fn default() -> Self {
        Self {
            min_horizontal_separation_m: 50.0,
            min_vertical_separation_m: 30.0,
            lookahead_seconds: 20.0,
            warning_multiplier: 2.0,
            drone_timeout_secs: 10,
            max_altitude_m: 121.0, // FAA Part 107 limit (~400ft)
            min_altitude_m: 10.0,
            altitude_bands: vec![
                AltitudeBand { name: "Low".into(), min_m: 10.0, max_m: 40.0 },
                AltitudeBand { name: "Medium".into(), min_m: 40.0, max_m: 80.0 },
                AltitudeBand { name: "High".into(), min_m: 80.0, max_m: 121.0 },
            ],
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AltitudeBand {
    pub name: String,
    pub min_m: f64,
    pub max_m: f64,
}
