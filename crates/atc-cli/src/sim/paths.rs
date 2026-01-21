//! Flight path implementations.

use atc_core::spatial::{bearing, haversine_distance, offset_by_bearing};
use std::f64::consts::PI;

/// Trait for flight path implementations.
pub trait FlightPath: Send + Sync {
    /// Get (lat, lon, altitude_m) at time t seconds from start.
    fn get_position(&self, t: f64) -> (f64, f64, f64);

    /// Get approximate heading at time t (degrees, 0 = North).
    fn get_heading(&self, t: f64) -> f64 {
        // Default: estimate heading from position delta
        let dt = 0.1;
        let (lat1, lon1, _) = self.get_position(t);
        let (lat2, lon2, _) = self.get_position(t + dt);

        let dlat = lat2 - lat1;
        let dlon = lon2 - lon1;

        if dlat.abs() < 1e-10 && dlon.abs() < 1e-10 {
            return 0.0;
        }

        let heading_rad = dlon.atan2(dlat);
        let heading_deg = heading_rad.to_degrees();

        // Normalize to 0-360
        if heading_deg < 0.0 {
            heading_deg + 360.0
        } else {
            heading_deg
        }
    }

    /// Get speed in meters per second.
    fn get_speed_mps(&self) -> f64;
}

/// Circular flight path around a center point.
pub struct CircularPath {
    pub center_lat: f64,
    pub center_lon: f64,
    pub radius_m: f64,
    pub altitude_m: f64,
    pub speed_mps: f64,
    pub start_angle: f64,
    pub clockwise: bool,
    period: f64,
}

impl CircularPath {
    /// Create a new circular flight path.
    ///
    /// # Arguments
    /// * `center_lat` - Center latitude
    /// * `center_lon` - Center longitude
    /// * `radius_m` - Radius in meters
    /// * `altitude_m` - Flight altitude
    /// * `speed_mps` - Speed in m/s
    /// * `start_angle` - Starting angle in radians
    /// * `clockwise` - Direction of flight
    pub fn new(
        center_lat: f64,
        center_lon: f64,
        radius_m: f64,
        altitude_m: f64,
        speed_mps: f64,
        start_angle: f64,
        clockwise: bool,
    ) -> Self {
        let circumference = 2.0 * PI * radius_m;
        let period = circumference / speed_mps;

        Self {
            center_lat,
            center_lon,
            radius_m,
            altitude_m,
            speed_mps,
            start_angle,
            clockwise,
            period,
        }
    }
}

impl FlightPath for CircularPath {
    fn get_position(&self, t: f64) -> (f64, f64, f64) {
        let mut angle_rad = self.start_angle + (2.0 * PI * t / self.period);
        if self.clockwise {
            angle_rad = -angle_rad;
        }

        let (lat, lon) = offset_by_bearing(self.center_lat, self.center_lon, self.radius_m, angle_rad);

        (lat, lon, self.altitude_m)
    }

    fn get_speed_mps(&self) -> f64 {
        self.speed_mps
    }
}

/// Linear flight path between two points.
pub struct LinearPath {
    pub start_lat: f64,
    pub start_lon: f64,
    pub end_lat: f64,
    pub end_lon: f64,
    pub altitude_m: f64,
    pub speed_mps: f64,
    pub distance_m: f64,
    pub duration: f64,
    heading_deg: f64,
    heading_rad: f64,
}

impl LinearPath {
    /// Create a new linear flight path.
    pub fn new(
        start_lat: f64,
        start_lon: f64,
        end_lat: f64,
        end_lon: f64,
        altitude_m: f64,
        speed_mps: f64,
    ) -> Self {
        let distance_m = haversine_distance(start_lat, start_lon, end_lat, end_lon);
        let duration = if speed_mps > 0.0 {
            distance_m / speed_mps
        } else {
            0.0
        };

        // Calculate heading
        let heading_rad = bearing(start_lat, start_lon, end_lat, end_lon);
        let mut heading_deg = heading_rad.to_degrees();
        if heading_deg < 0.0 {
            heading_deg += 360.0;
        }

        Self {
            start_lat,
            start_lon,
            end_lat,
            end_lon,
            altitude_m,
            speed_mps,
            distance_m,
            duration,
            heading_deg,
            heading_rad,
        }
    }
}

impl FlightPath for LinearPath {
    fn get_position(&self, t: f64) -> (f64, f64, f64) {
        // Clamp progress to [0, 1]
        let progress = if self.duration > 0.0 {
            (t / self.duration).clamp(0.0, 1.0)
        } else {
            0.0
        };

        let distance = self.distance_m * progress;
        let (lat, lon) = offset_by_bearing(self.start_lat, self.start_lon, distance, self.heading_rad);

        (lat, lon, self.altitude_m)
    }

    fn get_heading(&self, _t: f64) -> f64 {
        self.heading_deg
    }

    fn get_speed_mps(&self) -> f64 {
        self.speed_mps
    }
}

// Removed local haversine_distance. Using atc_core::spatial::haversine_distance.

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_linear_path_start_position() {
        let path = LinearPath::new(33.0, -117.0, 34.0, -118.0, 50.0, 10.0);
        let (lat, lon, alt) = path.get_position(0.0);

        assert!((lat - 33.0).abs() < 0.0001);
        assert!((lon - (-117.0)).abs() < 0.0001);
        assert!((alt - 50.0).abs() < 0.01);
    }

    #[test]
    fn test_linear_path_end_position() {
        let path = LinearPath::new(33.0, -117.0, 34.0, -118.0, 50.0, 10.0);
        let (lat, lon, _) = path.get_position(path.duration + 100.0); // Past end

        assert!((lat - 34.0).abs() < 0.0001);
        assert!((lon - (-118.0)).abs() < 0.0001);
    }

    #[test]
    fn test_circular_path() {
        let path = CircularPath::new(33.0, -117.0, 200.0, 50.0, 10.0, 0.0, false);

        // Should return to approximately start after one period
        let (lat1, lon1, _) = path.get_position(0.0);
        let (lat2, lon2, _) = path.get_position(path.period);

        assert!((lat1 - lat2).abs() < 0.0001);
        assert!((lon1 - lon2).abs() < 0.0001);
    }
}
