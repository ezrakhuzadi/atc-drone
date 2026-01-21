//! Spatial math for conflict detection and distance calculations.

use crate::models::{FlightPlan, TrajectoryPoint};
use std::cmp::Ordering;

// Minimum separation distance in meters
const MIN_SEPARATION_M: f64 = 50.0;
// Minimum vertical separation in meters
const MIN_VERTICAL_SEP_M: f64 = 30.0;
const TIME_SAMPLE_SECS_MIN: f64 = 0.5;
const TIME_SAMPLE_SECS_MAX: f64 = 2.0;

/// Check if two flight plans conflict.
/// 
/// Uses line segment distance checks between consecutive waypoints
/// to detect potential conflicts along flight paths.
/// 
/// Returns true if conflict found.
pub fn check_plan_conflict(new_plan: &FlightPlan, existing_plan: &FlightPlan) -> bool {
    if let (Some(path1), Some(path2)) = (build_timed_path(new_plan), build_timed_path(existing_plan)) {
        return check_timed_conflict(&path1, &path2);
    }

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

#[derive(Debug, Clone)]
struct TimedPoint {
    time_s: f64,
    lat: f64,
    lon: f64,
    altitude_m: f64,
}

fn build_timed_path(plan: &FlightPlan) -> Option<Vec<TimedPoint>> {
    let trajectory = plan.trajectory_log.as_ref()?;
    let base_time = plan.departure_time.timestamp_millis() as f64 / 1000.0;

    let mut points: Vec<TimedPoint> = trajectory
        .iter()
        .filter_map(|point| to_timed_point(point, base_time))
        .collect();

    if points.len() < 2 {
        return None;
    }

    points.sort_by(|a, b| a.time_s.partial_cmp(&b.time_s).unwrap_or(Ordering::Equal));
    Some(points)
}

fn to_timed_point(point: &TrajectoryPoint, base_time: f64) -> Option<TimedPoint> {
    let offset = point.time_offset_s?;
    if !offset.is_finite() || offset < 0.0 {
        return None;
    }
    Some(TimedPoint {
        time_s: base_time + offset,
        lat: point.lat,
        lon: point.lon,
        altitude_m: point.altitude_m,
    })
}

fn check_timed_conflict(path1: &[TimedPoint], path2: &[TimedPoint]) -> bool {
    let Some(first1) = path1.first() else {
        return false;
    };
    let Some(first2) = path2.first() else {
        return false;
    };
    let Some(last1) = path1.last() else {
        return false;
    };
    let Some(last2) = path2.last() else {
        return false;
    };

    let start = first1.time_s.max(first2.time_s);
    let end = last1.time_s.min(last2.time_s);
    if start > end {
        return false;
    }

    let step1 = derive_sample_step(path1);
    let step2 = derive_sample_step(path2);
    let step = step1.min(step2).clamp(TIME_SAMPLE_SECS_MIN, TIME_SAMPLE_SECS_MAX);

    let mut t = start;
    let mut idx1 = 0usize;
    let mut idx2 = 0usize;

    while t <= end {
        let pos1 = interpolate_position(path1, t, &mut idx1);
        let pos2 = interpolate_position(path2, t, &mut idx2);

        if let (Some(p1), Some(p2)) = (pos1, pos2) {
            let dist = haversine_distance(p1.lat, p1.lon, p2.lat, p2.lon);
            let alt_diff = (p1.altitude_m - p2.altitude_m).abs();
            if dist < MIN_SEPARATION_M && alt_diff < MIN_VERTICAL_SEP_M {
                return true;
            }
        }

        t += step;
    }

    false
}

fn derive_sample_step(path: &[TimedPoint]) -> f64 {
    if path.len() < 2 {
        return 1.0;
    }
    let delta = (path[1].time_s - path[0].time_s).abs();
    if delta.is_finite() && delta > 0.0 {
        delta
    } else {
        1.0
    }
}

fn interpolate_position(path: &[TimedPoint], t: f64, index: &mut usize) -> Option<TimedPoint> {
    if path.is_empty() {
        return None;
    }
    if t < path.first()?.time_s || t > path.last()?.time_s {
        return None;
    }

    while *index + 1 < path.len() && path[*index + 1].time_s < t {
        *index += 1;
    }

    let current = &path[*index];
    if (current.time_s - t).abs() < f64::EPSILON {
        return Some(current.clone());
    }

    let next = path.get(*index + 1)?;
    let span = next.time_s - current.time_s;
    if span <= 0.0 {
        return Some(current.clone());
    }

    let ratio = ((t - current.time_s) / span).clamp(0.0, 1.0);
    Some(TimedPoint {
        time_s: t,
        lat: current.lat + (next.lat - current.lat) * ratio,
        lon: current.lon + (next.lon - current.lon) * ratio,
        altitude_m: current.altitude_m + (next.altitude_m - current.altitude_m) * ratio,
    })
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
    // Check distance from A's endpoints to segment B
    let d1 = distance_to_segment_m(a1_lat, a1_lon, b1_lat, b1_lon, b2_lat, b2_lon);
    let d2 = distance_to_segment_m(a2_lat, a2_lon, b1_lat, b1_lon, b2_lat, b2_lon);
    
    // Check distance from B's endpoints to segment A
    let d3 = distance_to_segment_m(b1_lat, b1_lon, a1_lat, a1_lon, a2_lat, a2_lon);
    let d4 = distance_to_segment_m(b2_lat, b2_lon, a1_lat, a1_lon, a2_lat, a2_lon);
    
    d1.min(d2).min(d3).min(d4)
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
pub const EARTH_RADIUS_M: f64 = 6_371_000.0;

pub fn haversine_distance(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let phi1 = lat1.to_radians();
    let phi2 = lat2.to_radians();
    let dphi = (lat2 - lat1).to_radians();
    let dlambda = (lon2 - lon1).to_radians();
    let a = (dphi / 2.0).sin().powi(2)
        + phi1.cos() * phi2.cos() * (dlambda / 2.0).sin().powi(2);
    2.0 * EARTH_RADIUS_M * a.sqrt().atan2((1.0 - a).sqrt())
}

// ==== ENU (East-North-Up) Coordinate Conversion ====
// These functions convert between meters and degrees using latitude-aware scaling.

/// Meters per degree of latitude at a given latitude (WGS84 approximation).
pub fn meters_per_deg_lat(lat_deg: f64) -> f64 {
    let lat_rad = lat_deg.to_radians();
    111_132.954
        - 559.822 * (2.0 * lat_rad).cos()
        + 1.175 * (4.0 * lat_rad).cos()
        - 0.0023 * (6.0 * lat_rad).cos()
}

/// Meters per degree of longitude at a given latitude (WGS84 approximation).
pub fn meters_per_deg_lon(lat_deg: f64) -> f64 {
    let lat_rad = lat_deg.to_radians();
    111_412.84 * lat_rad.cos()
        - 93.5 * (3.0 * lat_rad).cos()
        + 0.118 * (5.0 * lat_rad).cos()
}

/// Convert a north/south offset in meters to degrees latitude.
pub fn meters_to_lat(meters: f64, ref_lat_deg: f64) -> f64 {
    let meters_per_deg = meters_per_deg_lat(ref_lat_deg).max(1e-9);
    meters / meters_per_deg
}

/// Convert an east/west offset in meters to degrees longitude.
/// Requires the reference latitude for proper scaling.
pub fn meters_to_lon(meters: f64, ref_lat_deg: f64) -> f64 {
    let meters_per_deg = meters_per_deg_lon(ref_lat_deg).max(1e-9);
    meters / meters_per_deg
}

/// Convert degrees latitude to meters using local scaling.
pub fn lat_to_meters(deg: f64, ref_lat_deg: f64) -> f64 {
    deg * meters_per_deg_lat(ref_lat_deg)
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
    let distance_m = (north_m * north_m + east_m * east_m).sqrt();
    if distance_m <= f64::EPSILON {
        return (lat, lon);
    }
    let bearing_rad = east_m.atan2(north_m);
    offset_by_bearing(lat, lon, distance_m, bearing_rad)
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
    if distance_m.abs() <= f64::EPSILON {
        return (lat, lon);
    }

    let lat1 = lat.to_radians();
    let lon1 = lon.to_radians();
    let angular_distance = distance_m / EARTH_RADIUS_M;

    let sin_lat1 = lat1.sin();
    let cos_lat1 = lat1.cos();
    let sin_ad = angular_distance.sin();
    let cos_ad = angular_distance.cos();

    let sin_lat2 = sin_lat1 * cos_ad + cos_lat1 * sin_ad * bearing_rad.cos();
    let lat2 = sin_lat2.clamp(-1.0, 1.0).asin();

    let y = bearing_rad.sin() * sin_ad * cos_lat1;
    let x = cos_ad - sin_lat1 * sin_lat2;
    let mut lon2 = lon1 + y.atan2(x);
    lon2 = (lon2 + std::f64::consts::PI).rem_euclid(2.0 * std::f64::consts::PI)
        - std::f64::consts::PI;

    (lat2.to_degrees(), lon2.to_degrees())
}

/// Calculate minimum distance from a point to a line segment (in meters).
/// 
/// This is the proper way to check if a stationary drone is "blocking" a path,
/// rather than using bounding-box checks.
/// 
/// # Arguments
/// * `point_lat`, `point_lon` - The point to measure from (e.g., priority drone)
/// * `seg_start_lat`, `seg_start_lon` - Segment start (e.g., give-way drone position)
/// * `seg_end_lat`, `seg_end_lon` - Segment end (e.g., give-way projected position)
/// 
/// # Returns
/// Distance in meters from the point to the closest point on the segment
pub fn distance_to_segment_m(
    point_lat: f64, point_lon: f64,
    seg_start_lat: f64, seg_start_lon: f64,
    seg_end_lat: f64, seg_end_lon: f64,
) -> f64 {
    // Convert to local ENU (using segment start as origin)
    let ref_lat = seg_start_lat;
    
    // Point in local coords
    let px = lon_to_meters(point_lon - seg_start_lon, ref_lat);
    let py = lat_to_meters(point_lat - seg_start_lat, ref_lat);
    
    // Segment end in local coords
    let sx = lon_to_meters(seg_end_lon - seg_start_lon, ref_lat);
    let sy = lat_to_meters(seg_end_lat - seg_start_lat, ref_lat);
    
    // Segment length squared
    let seg_len_sq = sx * sx + sy * sy;
    
    if seg_len_sq < 0.0001 {
        // Segment is essentially a point
        return (px * px + py * py).sqrt();
    }
    
    // Project point onto segment line: t = ((P-A) · (B-A)) / |B-A|²
    let t = ((px * sx + py * sy) / seg_len_sq).clamp(0.0, 1.0);
    
    // Closest point on segment
    let closest_x = t * sx;
    let closest_y = t * sy;
    
    // Distance from point to closest point on segment
    let dx = px - closest_x;
    let dy = py - closest_y;
    
    (dx * dx + dy * dy).sqrt()
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
