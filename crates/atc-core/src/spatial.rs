//! Spatial math for conflict detection and distance calculations.

use crate::models::FlightPlan;

// Minimum separation distance in meters
const MIN_SEPARATION_M: f64 = 50.0;
// Minimum vertical separation in meters
const MIN_VERTICAL_SEP_M: f64 = 30.0;

/// Check if two flight plans conflict.
/// 
/// Uses line segment distance checks between consecutive waypoints
/// to detect potential conflicts along flight paths.
/// 
/// Returns true if conflict found.
pub fn check_plan_conflict(new_plan: &FlightPlan, existing_plan: &FlightPlan) -> bool {
    // 1. Check time overlap window
    let start1 = new_plan.departure_time;
    let end1 = new_plan.waypoints.last().map(|_| start1 + chrono::Duration::seconds(600)).unwrap_or(start1);
    
    let start2 = existing_plan.departure_time;
    let end2 = existing_plan.waypoints.last().map(|_| start2 + chrono::Duration::seconds(600)).unwrap_or(start2);
    
    // No time overlap = no conflict
    if end1 < start2 || start1 > end2 {
        return false;
    }
    
    // 2. Check path segments for proximity
    // Check each segment of new plan against each segment of existing plan
    let new_wps = &new_plan.waypoints;
    let existing_wps = &existing_plan.waypoints;
    
    // Check segment-to-segment proximity
    for i in 0..new_wps.len().saturating_sub(1) {
        for j in 0..existing_wps.len().saturating_sub(1) {
            // Get segments
            let (a1, a2) = (&new_wps[i], &new_wps[i + 1]);
            let (b1, b2) = (&existing_wps[j], &existing_wps[j + 1]);
            
            // Check if segments are close enough horizontally
            let min_dist = segment_to_segment_distance(
                a1.lat, a1.lon, a2.lat, a2.lon,
                b1.lat, b1.lon, b2.lat, b2.lon,
            );
            
            // Check altitude overlap
            let alt_overlap = altitude_ranges_overlap(
                a1.altitude_m, a2.altitude_m,
                b1.altitude_m, b2.altitude_m,
                MIN_VERTICAL_SEP_M,
            );
            
            if min_dist < MIN_SEPARATION_M && alt_overlap {
                return true;
            }
        }
    }
    
    // Also check waypoint proximity as fallback
    for wp1 in new_wps {
        for wp2 in existing_wps {
            let dist = haversine_distance(wp1.lat, wp1.lon, wp2.lat, wp2.lon);
            let alt_diff = (wp1.altitude_m - wp2.altitude_m).abs();
            
            if dist < MIN_SEPARATION_M && alt_diff < MIN_VERTICAL_SEP_M {
                return true;
            }
        }
    }
    
    false
}

/// Check if two altitude ranges overlap (with separation buffer).
fn altitude_ranges_overlap(a_min: f64, a_max: f64, b_min: f64, b_max: f64, sep: f64) -> bool {
    let a_lo = a_min.min(a_max) - sep;
    let a_hi = a_min.max(a_max) + sep;
    let b_lo = b_min.min(b_max);
    let b_hi = b_min.max(b_max);
    
    a_lo <= b_hi && b_lo <= a_hi
}

/// Approximate minimum distance between two line segments in meters.
/// Uses sampling approach for simplicity.
#[allow(clippy::too_many_arguments)]
fn segment_to_segment_distance(
    a1_lat: f64, a1_lon: f64, a2_lat: f64, a2_lon: f64,
    b1_lat: f64, b1_lon: f64, b2_lat: f64, b2_lon: f64,
) -> f64 {
    let mut min_dist = f64::MAX;
    
    // Sample 5 points along each segment
    for i in 0..=4 {
        let t = i as f64 / 4.0;
        let a_lat = a1_lat + t * (a2_lat - a1_lat);
        let a_lon = a1_lon + t * (a2_lon - a1_lon);
        
        for j in 0..=4 {
            let s = j as f64 / 4.0;
            let b_lat = b1_lat + s * (b2_lat - b1_lat);
            let b_lon = b1_lon + s * (b2_lon - b1_lon);
            
            let dist = haversine_distance(a_lat, a_lon, b_lat, b_lon);
            min_dist = min_dist.min(dist);
        }
    }
    
    min_dist
}

/// Calculate distance between two points in meters using Haversine formula.
/// 
/// This is the standard formula for calculating great-circle distance
/// between two points on a sphere given their latitudes and longitudes.
/// 
/// # Arguments
/// * `lat1`, `lon1` - First point coordinates in decimal degrees
/// * `lat2`, `lon2` - Second point coordinates in decimal degrees
/// 
/// # Returns
/// Distance in meters
pub fn haversine_distance(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    const R: f64 = 6_371_000.0; // Earth radius in meters
    let phi1 = lat1.to_radians();
    let phi2 = lat2.to_radians();
    let dphi = (lat2 - lat1).to_radians();
    let dlambda = (lon2 - lon1).to_radians();
    let a = (dphi / 2.0).sin().powi(2)
        + phi1.cos() * phi2.cos() * (dlambda / 2.0).sin().powi(2);
    2.0 * R * a.sqrt().atan2((1.0 - a).sqrt())
}

// ==== ENU (East-North-Up) Coordinate Conversion ====
// These functions convert between meters and degrees using proper latitude scaling.

/// Meters per degree of latitude (constant at all latitudes).
pub const METERS_PER_DEG_LAT: f64 = 111_320.0;

/// Meters per degree of longitude at a given latitude.
/// Longitude degrees shrink as you move toward the poles.
pub fn meters_per_deg_lon(lat_deg: f64) -> f64 {
    METERS_PER_DEG_LAT * lat_deg.to_radians().cos()
}

/// Convert a north/south offset in meters to degrees latitude.
pub fn meters_to_lat(meters: f64) -> f64 {
    meters / METERS_PER_DEG_LAT
}

/// Convert an east/west offset in meters to degrees longitude.
/// Requires the reference latitude for proper scaling.
pub fn meters_to_lon(meters: f64, ref_lat_deg: f64) -> f64 {
    meters / meters_per_deg_lon(ref_lat_deg)
}

/// Convert degrees latitude to meters.
pub fn lat_to_meters(deg: f64) -> f64 {
    deg * METERS_PER_DEG_LAT
}

/// Convert degrees longitude to meters at a given latitude.
pub fn lon_to_meters(deg: f64, ref_lat_deg: f64) -> f64 {
    deg * meters_per_deg_lon(ref_lat_deg)
}

/// Offset a position by meters in the north and east directions.
/// 
/// # Arguments
/// * `lat`, `lon` - Reference position in degrees
/// * `north_m` - Offset in meters (positive = north)
/// * `east_m` - Offset in meters (positive = east)
/// 
/// # Returns
/// (new_lat, new_lon) in degrees
pub fn offset_position(lat: f64, lon: f64, north_m: f64, east_m: f64) -> (f64, f64) {
    let new_lat = lat + meters_to_lat(north_m);
    let new_lon = lon + meters_to_lon(east_m, lat);
    (new_lat, new_lon)
}

/// Calculate bearing from point 1 to point 2 in radians.
/// Returns bearing in radians, 0 = north, π/2 = east.
pub fn bearing(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let phi1 = lat1.to_radians();
    let phi2 = lat2.to_radians();
    let delta_lambda = (lon2 - lon1).to_radians();
    
    let x = delta_lambda.sin() * phi2.cos();
    let y = phi1.cos() * phi2.sin() - phi1.sin() * phi2.cos() * delta_lambda.cos();
    
    x.atan2(y)
}

/// Offset a position by distance and bearing.
/// 
/// # Arguments
/// * `lat`, `lon` - Starting position in degrees
/// * `distance_m` - Distance in meters
/// * `bearing_rad` - Bearing in radians (0 = north, π/2 = east)
/// 
/// # Returns
/// (new_lat, new_lon) in degrees
pub fn offset_by_bearing(lat: f64, lon: f64, distance_m: f64, bearing_rad: f64) -> (f64, f64) {
    let north_m = distance_m * bearing_rad.cos();
    let east_m = distance_m * bearing_rad.sin();
    offset_position(lat, lon, north_m, east_m)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_haversine_known_distance() {
        // ~111km between these points (1 degree latitude)
        let dist = haversine_distance(0.0, 0.0, 1.0, 0.0);
        assert!((dist - 111_194.0).abs() < 100.0);
    }

    #[test]
    fn test_haversine_same_point() {
        let dist = haversine_distance(33.6846, -117.8265, 33.6846, -117.8265);
        assert!(dist < 0.001);
    }
}

