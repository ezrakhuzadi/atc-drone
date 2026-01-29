use crate::compliance::RoutePoint;
use crate::state::store::AppState;
use crate::terrain::fetch_terrain_grid;
use reqwest::Client;
use serde_json::json;
use std::time::Duration;

pub async fn validate_route_altitudes(
    state: &AppState,
    points: &[RoutePoint],
    violations: &mut Vec<serde_json::Value>,
) {
    let rules = state.rules();
    let config = state.config();

    // Production semantics: SafetyRules altitude limits are treated as AGL (meters above ground).
    // This requires terrain; if terrain is required and fetching fails, fail closed by emitting a violation.
    if config.terrain_require {
        let client = Client::builder()
            .timeout(Duration::from_secs(15))
            .build()
            .unwrap_or_else(|_| Client::new());

        let terrain = match fetch_terrain_grid(
            &client,
            config,
            points,
            config.terrain_sample_spacing_m.max(5.0),
        )
        .await
        {
            Ok(terrain) => terrain,
            Err(err) => {
                violations.push(json!({
                    "type": "terrain",
                    "message": format!(
                        "Terrain fetch failed (required for AGL altitude checks): {}",
                        err
                    )
                }));
                return;
            }
        };

        let Some(grid) = terrain else {
            return;
        };

        for (idx, point) in points.iter().enumerate() {
            if !point.lat.is_finite()
                || !point.lon.is_finite()
                || !point.altitude_m.is_finite()
                || !(-90.0..=90.0).contains(&point.lat)
                || !(-180.0..=180.0).contains(&point.lon)
            {
                continue;
            }

            let ground_m = grid.sample(point.lat, point.lon);
            let agl_m = point.altitude_m - ground_m;

            if agl_m > rules.max_altitude_m {
                violations.push(json!({
                    "type": "altitude_agl",
                    "point_index": idx,
                    "altitude_amsl_m": point.altitude_m,
                    "ground_elevation_m": ground_m,
                    "agl_m": agl_m,
                    "max_agl_m": rules.max_altitude_m,
                    "message": format!(
                        "Altitude AGL {:.1}m exceeds max AGL {:.1}m",
                        agl_m,
                        rules.max_altitude_m
                    )
                }));
            }

            if agl_m < rules.min_altitude_m {
                violations.push(json!({
                    "type": "altitude_agl",
                    "point_index": idx,
                    "altitude_amsl_m": point.altitude_m,
                    "ground_elevation_m": ground_m,
                    "agl_m": agl_m,
                    "min_agl_m": rules.min_altitude_m,
                    "message": format!(
                        "Altitude AGL {:.1}m is below min AGL {:.1}m",
                        agl_m,
                        rules.min_altitude_m
                    )
                }));
            }
        }
        return;
    }

    // Dev semantics: without required terrain, keep the legacy checks against raw altitudes (AMSL after normalization).
    for (idx, point) in points.iter().enumerate() {
        if !point.lat.is_finite()
            || !point.lon.is_finite()
            || !point.altitude_m.is_finite()
            || !(-90.0..=90.0).contains(&point.lat)
            || !(-180.0..=180.0).contains(&point.lon)
        {
            continue;
        }

        if point.altitude_m > rules.max_altitude_m {
            violations.push(json!({
                "type": "altitude",
                "point_index": idx,
                "altitude_m": point.altitude_m,
                "max_m": rules.max_altitude_m,
                "message": format!(
                    "Altitude {:.1}m exceeds max altitude {:.1}m",
                    point.altitude_m,
                    rules.max_altitude_m
                )
            }));
        }

        if point.altitude_m < rules.min_altitude_m {
            violations.push(json!({
                "type": "altitude",
                "point_index": idx,
                "altitude_m": point.altitude_m,
                "min_m": rules.min_altitude_m,
                "message": format!(
                    "Altitude {:.1}m is below min altitude {:.1}m",
                    point.altitude_m,
                    rules.min_altitude_m
                )
            }));
        }
    }
}
