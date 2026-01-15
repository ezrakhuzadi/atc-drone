//! Pre-defined drone flight scenarios for testing.

use super::paths::LinearPath;
use std::sync::Arc;

use super::FlightPath;

/// A named scenario consisting of multiple drones with flight paths.
pub struct Scenario {
    pub name: String,
    pub drones: Vec<(String, Arc<dyn FlightPath>)>,
}

/// Create two drones on collision course (crossing at center).
///
/// - Drone 1: Flying West to East through center
/// - Drone 2: Flying South to North through center
pub fn create_crossing_scenario(center_lat: f64, center_lon: f64) -> Scenario {
    let offset = 0.003; // ~300m offset in degrees

    // Drone 1: West to East
    let drone1_path = Arc::new(LinearPath::new(
        center_lat,
        center_lon - offset,
        center_lat,
        center_lon + offset,
        50.0,  // altitude
        10.0,  // speed
    ));

    // Drone 2: South to North
    let drone2_path = Arc::new(LinearPath::new(
        center_lat - offset,
        center_lon,
        center_lat + offset,
        center_lon,
        50.0,
        10.0,
    ));

    Scenario {
        name: "crossing".to_string(),
        drones: vec![
            ("DRONE001".to_string(), drone1_path),
            ("DRONE002".to_string(), drone2_path),
        ],
    }
}

/// Create two drones flying parallel paths (no conflict).
pub fn create_parallel_scenario(center_lat: f64, center_lon: f64) -> Scenario {
    let offset = 0.003;
    let separation = 0.001; // ~100m separation

    let drone1_path = Arc::new(LinearPath::new(
        center_lat,
        center_lon - offset,
        center_lat,
        center_lon + offset,
        50.0,
        10.0,
    ));

    let drone2_path = Arc::new(LinearPath::new(
        center_lat + separation,
        center_lon - offset,
        center_lat + separation,
        center_lon + offset,
        50.0,
        10.0,
    ));

    Scenario {
        name: "parallel".to_string(),
        drones: vec![
            ("DRONE001".to_string(), drone1_path),
            ("DRONE002".to_string(), drone2_path),
        ],
    }
}

/// Create multiple drones converging on a central point.
pub fn create_converging_scenario(center_lat: f64, center_lon: f64) -> Scenario {
    let offset = 0.003;
    let angles: [f64; 4] = [0.0, 90.0, 180.0, 270.0]; // 4 drones from cardinal directions

    let drones: Vec<_> = angles
        .iter()
        .enumerate()
        .map(|(i, &angle)| {
            let angle_rad = angle.to_radians();
            let start_lat = center_lat + offset * angle_rad.cos();
            let start_lon = center_lon + offset * angle_rad.sin();

            let path = Arc::new(LinearPath::new(
                start_lat,
                start_lon,
                center_lat,
                center_lon,
                50.0,
                8.0,
            )) as Arc<dyn FlightPath>;

            (format!("DRONE{:03}", i + 1), path)
        })
        .collect();

    Scenario {
        name: "converging".to_string(),
        drones,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_crossing_scenario_creates_two_drones() {
        let scenario = create_crossing_scenario(33.0, -117.0);
        assert_eq!(scenario.drones.len(), 2);
        assert_eq!(scenario.name, "crossing");
    }

    #[test]
    fn test_converging_scenario_creates_four_drones() {
        let scenario = create_converging_scenario(33.0, -117.0);
        assert_eq!(scenario.drones.len(), 4);
        assert_eq!(scenario.name, "converging");
    }
}
