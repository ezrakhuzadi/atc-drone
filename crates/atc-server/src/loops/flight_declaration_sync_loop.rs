//! Flight declaration sync loop.
//!
//! Imports Flight Blender declarations into ATC flight plans for visibility.

use std::collections::HashSet;
use std::sync::Arc;
use std::time::Duration;

use atc_blender::BlenderClient;
use atc_core::models::{FlightPlan, FlightPlanMetadata, FlightStatus, Waypoint};
use chrono::{DateTime, Utc};
use serde_json::Value;
use tokio::sync::broadcast;
use tokio::time::interval;

use crate::config::Config;
use crate::state::AppState;

const LOOP_INTERVAL_SECS: u64 = 30;

pub async fn run_flight_declaration_sync_loop(
    state: Arc<AppState>,
    config: Config,
    mut shutdown: broadcast::Receiver<()>,
) {
    let blender = BlenderClient::new(
        &config.blender_url,
        &config.blender_session_id,
        &config.blender_auth_token,
    );

    let mut ticker = interval(Duration::from_secs(LOOP_INTERVAL_SECS));

    loop {
        tokio::select! {
            _ = shutdown.recv() => {
                tracing::info!("Flight declaration sync loop shutting down");
                break;
            }
            _ = ticker.tick() => {
                if let Err(err) = sync_flight_declarations(state.as_ref(), &blender).await {
                    tracing::warn!("Flight declaration sync failed: {}", err);
                }
            }
        }
    }
}

async fn sync_flight_declarations(state: &AppState, blender: &BlenderClient) -> anyhow::Result<()> {
    let declarations = blender.fetch_flight_declarations().await?;
    if declarations.is_empty() {
        return Ok(());
    }

    let existing_plans = state.get_flight_plans();
    let mut known_declarations = HashSet::new();
    let mut known_flight_ids = HashSet::new();
    for plan in existing_plans {
        if let Some(declaration_id) = plan
            .metadata
            .as_ref()
            .and_then(|meta| meta.blender_declaration_id.as_ref())
        {
            known_declarations.insert(declaration_id.clone());
        }
        known_flight_ids.insert(plan.flight_id.clone());
    }

    for declaration in declarations {
        let declaration_id = match extract_declaration_id(&declaration) {
            Some(id) => id,
            None => continue,
        };
        if known_declarations.contains(&declaration_id) {
            continue;
        }

        let Some(plan) = declaration_to_plan(&declaration, &declaration_id) else {
            continue;
        };

        if known_flight_ids.contains(&plan.flight_id) {
            continue;
        }

        known_declarations.insert(declaration_id);
        known_flight_ids.insert(plan.flight_id.clone());
        state.add_flight_plan(plan);
    }

    Ok(())
}

fn extract_declaration_id(declaration: &Value) -> Option<String> {
    first_string(&[
        declaration.get("id"),
        declaration.get("flight_declaration_id"),
        declaration.get("pk"),
    ])
}

fn declaration_to_plan(declaration: &Value, declaration_id: &str) -> Option<FlightPlan> {
    let geojson = extract_geojson(declaration)?;
    let feature = geojson
        .get("features")
        .and_then(|v| v.as_array())
        .and_then(|arr| arr.first())?;
    let geometry = feature.get("geometry")?;
    let properties = feature.get("properties").unwrap_or(&Value::Null);

    let coordinates = extract_coordinates(geometry);
    if coordinates.is_empty() {
        return None;
    }

    let min_alt = altitude_from_property(properties, "min_altitude");
    let max_alt = altitude_from_property(properties, "max_altitude");
    let altitude = max_alt.or(min_alt).unwrap_or(0.0);

    let waypoints: Vec<Waypoint> = coordinates
        .into_iter()
        .map(|(lon, lat)| Waypoint {
            lat,
            lon,
            altitude_m: altitude,
            speed_mps: None,
        })
        .collect();

    let compliance = properties.get("compliance");
    let flight_id = compliance
        .and_then(extract_atc_plan_id)
        .unwrap_or_else(|| format!("BLENDER-{}", declaration_id));

    let departure_time = parse_datetime(
        declaration.get("start_datetime")
            .or_else(|| properties.get("start_time"))
    )
    .unwrap_or_else(Utc::now);
    let arrival_time = parse_datetime(
        declaration.get("end_datetime")
            .or_else(|| properties.get("end_time"))
    );
    let created_at = parse_datetime(
        declaration.get("created_at")
            .or_else(|| declaration.get("submitted_at"))
            .or_else(|| declaration.get("updated_at"))
    )
    .unwrap_or_else(Utc::now);

    let status = map_declaration_status(declaration);
    let drone_id = first_string(&[
        declaration.get("aircraft_id"),
        declaration.get("aircraft"),
    ])
    .unwrap_or_else(|| format!("BLENDER-{}", declaration_id));

    let metadata = FlightPlanMetadata {
        blender_declaration_id: Some(declaration_id.to_string()),
        planned_altitude_m: Some(altitude),
        operation_type: declaration.get("type_of_operation").and_then(|v| v.as_u64()).map(|v| v as u8),
        compliance_report: compliance.cloned(),
        ..Default::default()
    };

    Some(FlightPlan {
        flight_id,
        drone_id,
        owner_id: None,
        waypoints,
        trajectory_log: None,
        metadata: Some(metadata),
        status,
        departure_time,
        arrival_time,
        created_at,
    })
}

fn extract_geojson(declaration: &Value) -> Option<Value> {
    let raw = declaration
        .get("flight_declaration_geojson")
        .or_else(|| declaration.get("flight_declaration_geo_json"))
        .or_else(|| declaration.get("flight_declaration_raw_geojson"))?;

    match raw {
        Value::String(content) => serde_json::from_str(content).ok(),
        Value::Object(_) | Value::Array(_) => Some(raw.clone()),
        _ => None,
    }
}

fn extract_coordinates(geometry: &Value) -> Vec<(f64, f64)> {
    let geom_type = geometry
        .get("type")
        .and_then(|v| v.as_str())
        .unwrap_or("");
    let coords = geometry.get("coordinates");

    match geom_type {
        "Point" => coords
            .and_then(parse_coord_pair)
            .into_iter()
            .collect(),
        "LineString" => coords
            .and_then(|v| v.as_array())
            .map(|arr| {
                arr.iter()
                    .filter_map(parse_coord_pair)
                    .collect()
            })
            .unwrap_or_default(),
        "MultiPoint" => coords
            .and_then(|v| v.as_array())
            .map(|arr| arr.iter().filter_map(parse_coord_pair).collect())
            .unwrap_or_default(),
        "MultiLineString" => coords
            .and_then(|v| v.as_array())
            .and_then(|arr| arr.first())
            .and_then(|v| v.as_array())
            .map(|arr| arr.iter().filter_map(parse_coord_pair).collect())
            .unwrap_or_default(),
        "Polygon" => coords
            .and_then(|v| v.as_array())
            .and_then(|arr| arr.first())
            .and_then(|v| v.as_array())
            .map(|arr| arr.iter().filter_map(parse_coord_pair).collect())
            .unwrap_or_default(),
        "MultiPolygon" => coords
            .and_then(|v| v.as_array())
            .and_then(|arr| arr.first())
            .and_then(|v| v.as_array())
            .and_then(|arr| arr.first())
            .and_then(|v| v.as_array())
            .map(|arr| arr.iter().filter_map(parse_coord_pair).collect())
            .unwrap_or_default(),
        _ => Vec::new(),
    }
}

fn parse_coord_pair(value: &Value) -> Option<(f64, f64)> {
    let arr = value.as_array()?;
    if arr.len() < 2 {
        return None;
    }
    let lon = arr[0].as_f64()?;
    let lat = arr[1].as_f64()?;
    if !lon.is_finite() || !lat.is_finite() {
        return None;
    }
    Some((lon, lat))
}

fn altitude_from_property(properties: &Value, key: &str) -> Option<f64> {
    let value = properties.get(key)?;
    if let Some(meters) = value.get("meters").and_then(|v| v.as_f64()) {
        return Some(meters);
    }
    value.as_f64()
}

fn extract_atc_plan_id(compliance: &Value) -> Option<String> {
    first_string(&[
        compliance.get("atc_plan_id"),
        compliance.get("atcPlanId"),
        compliance.get("atc_plan").and_then(|v| v.get("id")),
    ])
}

fn map_declaration_status(declaration: &Value) -> FlightStatus {
    let state_value = declaration.get("state").and_then(|v| v.as_i64());
    match state_value {
        Some(2 | 3 | 4) => FlightStatus::Active,
        Some(5 | 6 | 7 | 8) => FlightStatus::Completed,
        Some(1) => FlightStatus::Approved,
        Some(0) => FlightStatus::Pending,
        _ => FlightStatus::Pending,
    }
}

fn parse_datetime(value: Option<&Value>) -> Option<DateTime<Utc>> {
    let raw = value?.as_str()?.trim();
    if raw.is_empty() {
        return None;
    }
    DateTime::parse_from_rfc3339(raw)
        .map(|dt| dt.with_timezone(&Utc))
        .ok()
}

fn first_string(candidates: &[Option<&Value>]) -> Option<String> {
    for candidate in candidates {
        if let Some(value) = candidate {
            if let Some(text) = value.as_str() {
                let trimmed = text.trim();
                if !trimmed.is_empty() {
                    return Some(trimmed.to_string());
                }
            }
        }
    }
    None
}
