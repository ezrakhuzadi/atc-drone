//! Spatial math for conflict detection.

use crate::models::FlightPlan;

// Minimum separation distance in meters
const MIN_SEPARATION_M: f64 = 50.0;
// Minimum time separation in seconds (for traversing same point)
#[allow(dead_code)] // Reserved for future 4D conflict detection
const _MIN_TIME_SEP_S: f64 = 60.0; 

/// Check if two flight plans conflict.
/// 
/// Returns true if valid conflict found.
pub fn check_plan_conflict(new_plan: &FlightPlan, existing_plan: &FlightPlan) -> bool {
    // 1. Check time overlap window
    let start1 = new_plan.departure_time;
    let end1 = new_plan.waypoints.last().map(|_| start1 + chrono::Duration::seconds(600)).unwrap_or(start1);
    
    let start2 = existing_plan.departure_time;
    let end2 = existing_plan.waypoints.last().map(|_| start2 + chrono::Duration::seconds(600)).unwrap_or(start2);
    
    // Simple bounding box check on time
    if end1 < start2 || start1 > end2 {
        return false;
    }
    
    // 2. Check path segments
    // Simplification: Check waypoint proximity for now.
    // Proper implementation requires Line Segment distance in 3D + Time interpolation.
    
    for wp1 in &new_plan.waypoints {
        for wp2 in &existing_plan.waypoints {
            let dist = haversine_distance(wp1.lat, wp1.lon, wp2.lat, wp2.lon);
            let alt_diff = (wp1.altitude_m - wp2.altitude_m).abs();
            
            if dist < MIN_SEPARATION_M && alt_diff < MIN_SEPARATION_M {
                // Conflict!
                return true;
            }
        }
    }
    
    false
}

fn haversine_distance(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    const R: f64 = 6_371_000.0;
    let phi1 = lat1.to_radians();
    let phi2 = lat2.to_radians();
    let dphi = (lat2 - lat1).to_radians();
    let dlambda = (lon2 - lon1).to_radians();
    let a = (dphi / 2.0).sin().powi(2)
        + phi1.cos() * phi2.cos() * (dlambda / 2.0).sin().powi(2);
    2.0 * R * a.sqrt().atan2((1.0 - a).sqrt())
}
