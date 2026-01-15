//! Sync conflict volumes to Blender as geofences.
//!
//! When conflicts are detected, we push them as temporary
//! geofences so they appear as red zones in Spotlight.

use anyhow::Result;
use atc_core::{Conflict, ConflictSeverity};
use serde::Serialize;
use std::f64::consts::PI;

/// GeoJSON Feature for a conflict zone geofence.
#[derive(Debug, Serialize)]
pub struct ConflictGeofence {
    pub id: String,
    pub name: String,
    pub upper_limit: i32,
    pub lower_limit: i32,
    pub raw_geo_fence: GeoJsonFeature,
    pub geozone_type: String,
    pub status: String,
}

#[derive(Debug, Serialize)]
pub struct GeoJsonFeature {
    #[serde(rename = "type")]
    pub feature_type: String,
    pub geometry: GeoJsonGeometry,
    pub properties: GeoJsonProperties,
}

#[derive(Debug, Serialize)]
pub struct GeoJsonGeometry {
    #[serde(rename = "type")]
    pub geometry_type: String,
    pub coordinates: Vec<Vec<[f64; 2]>>,
}

#[derive(Debug, Serialize)]
pub struct GeoJsonProperties {
    pub name: String,
    pub severity: String,
    pub drone1_id: String,
    pub drone2_id: String,
    pub distance_m: f64,
    pub time_to_closest: f64,
}

/// Generate a circular polygon (approximated with 32 points) around a center point.
fn generate_circle_polygon(center_lat: f64, center_lon: f64, radius_m: f64) -> Vec<[f64; 2]> {
    const NUM_POINTS: usize = 32;
    let mut coords = Vec::with_capacity(NUM_POINTS + 1);
    
    for i in 0..=NUM_POINTS {
        let angle = 2.0 * PI * (i as f64) / (NUM_POINTS as f64);
        
        // Convert radius from meters to degrees (approximate)
        let lat_offset = (radius_m / 111_320.0) * angle.cos();
        let lon_offset = (radius_m / (111_320.0 * center_lat.to_radians().cos())) * angle.sin();
        
        // GeoJSON uses [lon, lat] order!
        coords.push([center_lon + lon_offset, center_lat + lat_offset]);
    }
    
    coords
}

/// Transform a Conflict into a geofence payload for Blender.
pub fn conflict_to_geofence(
    conflict: &Conflict,
    drone1_pos: Option<(f64, f64, f64)>, // (lat, lon, alt)
    drone2_pos: Option<(f64, f64, f64)>,
) -> ConflictGeofence {
    // Calculate midpoint between the two drones (if positions known)
    let (center_lat, center_lon, center_alt) = match (drone1_pos, drone2_pos) {
        (Some((lat1, lon1, alt1)), Some((lat2, lon2, alt2))) => {
            ((lat1 + lat2) / 2.0, (lon1 + lon2) / 2.0, (alt1 + alt2) / 2.0)
        }
        (Some(pos), None) | (None, Some(pos)) => pos,
        (None, None) => (33.6846, -117.8265, 50.0), // Default to Irvine if unknown
    };

    // Radius based on severity
    let radius_m = match conflict.severity {
        ConflictSeverity::Critical => 100.0,
        ConflictSeverity::Warning => 75.0,
        ConflictSeverity::Info => 50.0,
    };

    // Generate the polygon
    let polygon_coords = generate_circle_polygon(center_lat, center_lon, radius_m);

    // Create unique ID
    let geofence_id = format!(
        "CONFLICT_{}_{}",
        conflict.drone1_id.replace("DRONE", ""),
        conflict.drone2_id.replace("DRONE", "")
    );

    let severity_str = match conflict.severity {
        ConflictSeverity::Critical => "critical",
        ConflictSeverity::Warning => "warning",
        ConflictSeverity::Info => "info",
    };

    ConflictGeofence {
        id: geofence_id.clone(),
        name: format!("Conflict: {} vs {}", conflict.drone1_id, conflict.drone2_id),
        upper_limit: (center_alt + 50.0) as i32, // 50m above
        lower_limit: (center_alt - 50.0).max(0.0) as i32, // 50m below or ground
        raw_geo_fence: GeoJsonFeature {
            feature_type: "Feature".to_string(),
            geometry: GeoJsonGeometry {
                geometry_type: "Polygon".to_string(),
                coordinates: vec![polygon_coords],
            },
            properties: GeoJsonProperties {
                name: geofence_id,
                severity: severity_str.to_string(),
                drone1_id: conflict.drone1_id.clone(),
                drone2_id: conflict.drone2_id.clone(),
                distance_m: conflict.distance_m,
                time_to_closest: conflict.time_to_closest,
            },
        },
        geozone_type: "conflict".to_string(),
        status: "active".to_string(),
    }
}

use super::client::BlenderClient;

impl BlenderClient {
    /// Send conflict geofences to Blender.
    pub async fn send_conflict_geofences(&self, geofences: &[ConflictGeofence]) -> Result<u16> {
        if geofences.is_empty() {
            return Ok(200);
        }

        let url = format!("{}/geo_fence_ops/set_geo_fence", self.base_url);
        
        // Note: Blender expects each geofence separately or as a batch
        // We'll send them one by one for now (can optimize later)
        for gf in geofences {
            let response = self.send_geofence_request(&url, gf).await?;
            if response >= 400 {
                tracing::warn!("Failed to send conflict geofence {}: HTTP {}", gf.id, response);
            }
        }
        
        Ok(200)
    }

    async fn send_geofence_request(&self, url: &str, geofence: &ConflictGeofence) -> Result<u16> {
        let token = super::client::generate_dummy_jwt();
        
        let response = self
            .client
            .post(url)
            .header("Content-Type", "application/json")
            .header("Authorization", format!("Bearer {}", token))
            .json(geofence)
            .send()
            .await?;

        Ok(response.status().as_u16())
    }
}

