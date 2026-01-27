//! Sync conflict volumes to Blender as geofences.
//!
//! When conflicts are detected, we push them as temporary
//! geofences so they appear as red zones in Spotlight.

use anyhow::{Context, Result};
use atc_core::spatial::offset_by_bearing;
use atc_core::{Conflict, ConflictSeverity};
use chrono::{Duration as ChronoDuration, Utc};
use serde::Serialize;
use serde_json::Value;
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

#[derive(Debug, Serialize)]
struct BlenderGeofencePayload {
    #[serde(rename = "type")]
    payload_type: String,
    features: Vec<BlenderGeofenceFeature>,
}

#[derive(Debug, Serialize)]
struct BlenderGeofenceFeature {
    #[serde(rename = "type")]
    feature_type: String,
    properties: BlenderGeofenceProperties,
    geometry: BlenderGeofenceGeometry,
}

#[derive(Debug, Serialize)]
struct BlenderGeofenceProperties {
    name: String,
    upper_limit: i32,
    lower_limit: i32,
    start_time: String,
    end_time: String,
}

#[derive(Debug, Serialize)]
struct BlenderGeofenceGeometry {
    #[serde(rename = "type")]
    geometry_type: String,
    coordinates: Vec<Vec<[f64; 2]>>,
}

/// Generate a circular polygon (approximated with 32 points) around a center point.
fn generate_circle_polygon(center_lat: f64, center_lon: f64, radius_m: f64) -> Vec<[f64; 2]> {
    const NUM_POINTS: usize = 32;
    let mut coords = Vec::with_capacity(NUM_POINTS + 1);

    for i in 0..=NUM_POINTS {
        let angle = 2.0 * PI * (i as f64) / (NUM_POINTS as f64);

        let (lat, lon) = offset_by_bearing(center_lat, center_lon, radius_m, angle);

        // GeoJSON uses [lon, lat] order!
        coords.push([lon, lat]);
    }

    coords
}

/// Transform a Conflict into a geofence payload for Blender.
pub fn conflict_to_geofence(
    conflict: &Conflict,
    drone1_pos: Option<(f64, f64, f64)>, // (lat, lon, alt)
    drone2_pos: Option<(f64, f64, f64)>,
) -> ConflictGeofence {
    let (id_a, id_b) = if conflict.drone1_id <= conflict.drone2_id {
        (conflict.drone1_id.as_str(), conflict.drone2_id.as_str())
    } else {
        (conflict.drone2_id.as_str(), conflict.drone1_id.as_str())
    };

    // Calculate midpoint between the two drones (if positions known)
    let (center_lat, center_lon, center_alt) = match (drone1_pos, drone2_pos) {
        (Some((lat1, lon1, alt1)), Some((lat2, lon2, alt2))) => (
            (lat1 + lat2) / 2.0,
            (lon1 + lon2) / 2.0,
            (alt1 + alt2) / 2.0,
        ),
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
        id_a.replace("DRONE", ""),
        id_b.replace("DRONE", "")
    );

    let severity_str = match conflict.severity {
        ConflictSeverity::Critical => "critical",
        ConflictSeverity::Warning => "warning",
        ConflictSeverity::Info => "info",
    };

    ConflictGeofence {
        id: geofence_id.clone(),
        name: format!("Conflict: {} vs {}", id_a, id_b),
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
                drone1_id: id_a.to_string(),
                drone2_id: id_b.to_string(),
                distance_m: conflict.distance_m,
                time_to_closest: conflict.time_to_closest,
            },
        },
        geozone_type: "conflict".to_string(),
        status: "active".to_string(),
    }
}

pub fn conflict_payload(geofence: &ConflictGeofence) -> Result<Value> {
    let payload = build_blender_payload(geofence);
    serde_json::to_value(payload).context("Failed to serialize conflict geofence payload")
}

use super::client::BlenderClient;

fn build_blender_payload(geofence: &ConflictGeofence) -> BlenderGeofencePayload {
    let start_time = Utc::now().to_rfc3339();
    let end_time = (Utc::now() + ChronoDuration::hours(1)).to_rfc3339();

    BlenderGeofencePayload {
        payload_type: "FeatureCollection".to_string(),
        features: vec![BlenderGeofenceFeature {
            feature_type: "Feature".to_string(),
            properties: BlenderGeofenceProperties {
                name: geofence.name.clone(),
                upper_limit: geofence.upper_limit,
                lower_limit: geofence.lower_limit,
                start_time,
                end_time,
            },
            geometry: BlenderGeofenceGeometry {
                geometry_type: geofence.raw_geo_fence.geometry.geometry_type.clone(),
                coordinates: geofence.raw_geo_fence.geometry.coordinates.clone(),
            },
        }],
    }
}

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
                tracing::warn!(
                    "Failed to send conflict geofence {}: HTTP {}",
                    gf.id,
                    response
                );
            }
        }

        Ok(200)
    }

    async fn send_geofence_request(&self, url: &str, geofence: &ConflictGeofence) -> Result<u16> {
        let auth_header = self.auth_header();
        let payload = build_blender_payload(geofence);
        let response = self
            .client
            .put(url)
            .header("Content-Type", "application/json")
            .header("Authorization", auth_header)
            .json(&payload)
            .send()
            .await?;

        Ok(response.status().as_u16())
    }
}
