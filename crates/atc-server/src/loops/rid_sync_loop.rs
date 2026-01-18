//! Remote ID (RID) sync loop.
//!
//! Polls Flight Blender for RID data and feeds external traffic into the
//! conflict detector so multi-operator traffic is visible in ATC.

use std::collections::HashMap;
use std::sync::Arc;
use std::time::{Duration, Instant};

use chrono::{DateTime, TimeZone, Utc};
use serde_json::Value;
use tokio::time::interval;

use atc_blender::BlenderClient;
use crate::config::Config;
use crate::state::{AppState, ExternalTraffic};

const RID_POLL_SECS: u64 = 2;
const RID_SUBSCRIPTION_TTL_SECS: u64 = 20;
const RID_TRACK_TTL_SECS: i64 = 30;

/// Start RID polling loop.
pub async fn run_rid_loop(state: Arc<AppState>, config: Config) {
    let blender = BlenderClient::new(
        &config.blender_url,
        &config.blender_session_id,
        &config.blender_auth_token,
    );

    let mut ticker = interval(Duration::from_secs(RID_POLL_SECS));
    let mut subscription_id: Option<String> = None;
    let mut last_subscription = Instant::now()
        .checked_sub(Duration::from_secs(RID_SUBSCRIPTION_TTL_SECS + 1))
        .unwrap_or_else(Instant::now);
    let mut last_view = state.get_rid_view_bbox();

    loop {
        ticker.tick().await;

        let current_view = state.get_rid_view_bbox();
        if current_view != last_view {
            last_view = current_view.clone();
            subscription_id = None;
        }

        if current_view.trim().is_empty() {
            continue;
        }

        if subscription_id.is_none() || last_subscription.elapsed().as_secs() >= RID_SUBSCRIPTION_TTL_SECS {
            match blender.create_rid_subscription(&current_view).await {
                Ok(id) => {
                    subscription_id = Some(id);
                    last_subscription = Instant::now();
                    tracing::info!("RID subscription refreshed");
                }
                Err(err) => {
                    tracing::warn!("RID subscription failed: {}", err);
                    continue;
                }
            }
        }

        let Some(active_id) = subscription_id.as_deref() else {
            continue;
        };

        match blender.fetch_rid_data(active_id).await {
            Ok(payload) => {
                let tracks = normalize_rid_payload(payload);
                for track in tracks {
                    state.upsert_external_traffic(track);
                }
                state.purge_external_traffic(RID_TRACK_TTL_SECS);
            }
            Err(err) => {
                tracing::warn!("RID data fetch failed: {}", err);
            }
        }
    }
}

fn normalize_rid_payload(payload: Value) -> Vec<ExternalTraffic> {
    let observations: Vec<Value> = if let Some(array) = payload.as_array() {
        array.clone()
    } else if let Some(array) = payload.get("flights").and_then(|v| v.as_array()) {
        array.clone()
    } else {
        Vec::new()
    };

    let mut by_id: HashMap<String, ExternalTraffic> = HashMap::new();

    for observation in observations {
        let Some(track) = normalize_observation(&observation) else {
            continue;
        };

        let replace = match by_id.get(&track.traffic_id) {
            Some(existing) => track.last_update > existing.last_update,
            None => true,
        };

        if replace {
            by_id.insert(track.traffic_id.clone(), track);
        }
    }

    by_id.into_values().collect()
}

fn normalize_observation(observation: &Value) -> Option<ExternalTraffic> {
    let metadata = observation.get("metadata").unwrap_or(&Value::Null);
    let current_state = metadata
        .get("current_state")
        .or_else(|| metadata.get("currentState"))
        .unwrap_or(&Value::Null);
    let position = current_state.get("position").unwrap_or(&Value::Null);

    let lat = first_number(&[
        observation.get("latitude_dd"),
        observation.get("lat_dd"),
        position.get("lat"),
        position.get("latitude"),
    ])?;
    let lon = first_number(&[
        observation.get("longitude_dd"),
        observation.get("lon_dd"),
        position.get("lng"),
        position.get("lon"),
        position.get("longitude"),
    ])?;

    let altitude_raw = first_number(&[
        observation.get("altitude_mm"),
        position.get("alt"),
    ]);
    let altitude_m = altitude_raw
        .map(|value| if value > 5000.0 { value / 1000.0 } else { value })
        .unwrap_or(0.0);

    let speed_mps = first_number(&[
        current_state.get("speed"),
        metadata.get("speed_mps"),
        metadata.get("speed"),
    ]).unwrap_or(0.0);
    let heading_deg = first_number(&[
        current_state.get("track"),
        metadata.get("heading_deg"),
        metadata.get("heading"),
    ]).unwrap_or(0.0);

    let raw_id = first_string(&[
        observation.get("icao_address"),
        metadata.get("id"),
        observation.get("session_id"),
        observation.get("id"),
    ])?;
    let raw_id = raw_id.trim();
    if raw_id.is_empty() || raw_id.eq_ignore_ascii_case("unknown") {
        return None;
    }
    let traffic_id = if raw_id.starts_with("RID-") {
        raw_id.to_string()
    } else {
        format!("RID-{}", raw_id)
    };

    let timestamp_value = observation
        .get("updated_at")
        .or_else(|| observation.get("created_at"))
        .or_else(|| current_state.get("timestamp"));
    let last_update = parse_timestamp(timestamp_value).unwrap_or_else(Utc::now);

    Some(ExternalTraffic {
        traffic_id,
        source: "rid".to_string(),
        lat,
        lon,
        altitude_m,
        heading_deg,
        speed_mps,
        last_update,
    })
}

fn first_number(candidates: &[Option<&Value>]) -> Option<f64> {
    for value in candidates {
        if let Some(num) = to_f64(*value) {
            return Some(num);
        }
    }
    None
}

fn first_string(candidates: &[Option<&Value>]) -> Option<String> {
    for value in candidates {
        if let Some(text) = value.and_then(|v| v.as_str()) {
            return Some(text.to_string());
        }
    }
    None
}

fn to_f64(value: Option<&Value>) -> Option<f64> {
    let value = value?;
    if let Some(num) = value.as_f64() {
        return Some(num);
    }
    if let Some(text) = value.as_str() {
        return text.parse::<f64>().ok();
    }
    None
}

fn parse_timestamp(value: Option<&Value>) -> Option<DateTime<Utc>> {
    let value = value?;
    if let Some(num) = value.as_i64() {
        return Some(from_epoch(num));
    }
    if let Some(num) = value.as_u64() {
        return Some(from_epoch(num as i64));
    }
    if let Some(text) = value.as_str() {
        if let Ok(dt) = DateTime::parse_from_rfc3339(text) {
            return Some(dt.with_timezone(&Utc));
        }
        if let Ok(parsed) = text.parse::<i64>() {
            return Some(from_epoch(parsed));
        }
    }
    None
}

fn from_epoch(value: i64) -> DateTime<Utc> {
    if value > 1_000_000_000_000 {
        let seconds = value / 1000;
        let millis = value % 1000;
        Utc.timestamp_opt(seconds, (millis * 1_000_000) as u32)
            .single()
            .unwrap_or_else(Utc::now)
    } else {
        Utc.timestamp_opt(value, 0).single().unwrap_or_else(Utc::now)
    }
}
