//! Server-side route planning using the backend A* engine.

use atc_core::models::{Geofence, GeofenceType, Waypoint};
use atc_core::route_engine::{
    RouteEngineConfig, RouteEngineResult, RouteEngineWaypoint, RouteGrid, RouteObstacle,
    apply_obstacles, build_lane_offsets, generate_grid_samples, optimize_airborne_path,
    optimize_flight_path, resolve_grid_spacing,
};
use atc_core::spatial::{bearing, haversine_distance, offset_by_bearing};
use reqwest::Client;
use serde::{Deserialize, Serialize};

use crate::compliance::{fetch_obstacles, ObstacleFootprint, ObstacleHazard, RoutePoint};
use crate::config::Config;
use crate::state::AppState;
use crate::terrain::{fetch_terrain_grid, TerrainGrid};
use chrono::Utc;

const DEFAULT_LANE_RADIUS_M: f64 = 90.0;
const DEFAULT_LANE_SPACING_M: f64 = 15.0;
const DEFAULT_SAMPLE_SPACING_M: f64 = 5.0;
const DEFAULT_MAX_LANE_RADIUS_M: f64 = 500.0;
const DEFAULT_LANE_EXPANSION_STEP_M: f64 = 50.0;
const MAX_ROUTE_GRID_POINTS: usize = 500_000;
const DEFAULT_SEGMENT_LENGTH_M: f64 = 8_000.0;
const MIN_SEGMENT_LENGTH_M: f64 = 2_000.0;
const MAX_SAMPLE_SPACING_M: f64 = 75.0;

#[derive(Debug)]
enum SegmentError {
    Truncated,
    Obstacle(String),
    Terrain(String),
    GridTooLarge(usize),
    Path(Vec<String>),
}

#[derive(Debug, Clone)]
struct SegmentPlan {
    waypoints: Vec<RouteEngineWaypoint>,
    stats: Option<atc_core::route_engine::RouteEngineStats>,
    nodes_visited: usize,
    optimized_points: usize,
    sample_points: usize,
    hazards: Vec<ObstacleHazard>,
    terrain: Option<TerrainGrid>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct RoutePlanRequest {
    pub waypoints: Vec<Waypoint>,
    pub lane_radius_m: Option<f64>,
    pub lane_spacing_m: Option<f64>,
    pub sample_spacing_m: Option<f64>,
    pub safety_buffer_m: Option<f64>,
    pub max_lane_radius_m: Option<f64>,
    pub lane_expansion_step_m: Option<f64>,
}

#[derive(Debug, Clone, Serialize)]
pub struct RoutePlanResponse {
    pub ok: bool,
    pub waypoints: Vec<RouteEngineWaypoint>,
    pub stats: Option<atc_core::route_engine::RouteEngineStats>,
    pub nodes_visited: usize,
    pub optimized_points: usize,
    pub sample_points: usize,
    pub hazards: Vec<ObstacleHazard>,
    pub errors: Vec<String>,
}

pub async fn plan_route(
    state: &AppState,
    config: &Config,
    request: RoutePlanRequest,
) -> RoutePlanResponse {
    if request.waypoints.len() < 2 {
        return RoutePlanResponse {
            ok: false,
            waypoints: Vec::new(),
            stats: None,
            nodes_visited: 0,
            optimized_points: 0,
            sample_points: 0,
            hazards: Vec::new(),
            errors: vec!["need at least 2 waypoints".to_string()],
        };
    }
    let route_distance_total = route_distance_m(&request.waypoints);
    let use_segments = route_distance_total > DEFAULT_SEGMENT_LENGTH_M;

    if !use_segments {
        let response = plan_route_single(state, config, &request).await;
        if response.ok {
            return response;
        }
        let retry = response.errors.iter().any(|err| {
            err.contains("truncated") || err.contains("route grid too large")
        });
        if !retry {
            return response;
        }
    }

    plan_route_segmented(state, config, request).await
}

async fn plan_route_single(
    state: &AppState,
    config: &Config,
    request: &RoutePlanRequest,
) -> RoutePlanResponse {
    let lane_radius = request.lane_radius_m.unwrap_or(DEFAULT_LANE_RADIUS_M);
    let lane_spacing = request.lane_spacing_m.unwrap_or(DEFAULT_LANE_SPACING_M);
    let default_spacing = request.sample_spacing_m.unwrap_or(DEFAULT_SAMPLE_SPACING_M);
    let route_distance_total = route_distance_m(&request.waypoints);
    let base_spacing = resolve_grid_spacing(&request.waypoints, default_spacing);
    let max_lane_radius = resolve_max_lane_radius(
        route_distance_total,
        lane_radius,
        request.max_lane_radius_m,
    );
    let expansion_step = request
        .lane_expansion_step_m
        .unwrap_or(DEFAULT_LANE_EXPANSION_STEP_M)
        .max(5.0);

    let points: Vec<RoutePoint> = request
        .waypoints
        .iter()
        .map(|wp| RoutePoint {
            lat: wp.lat,
            lon: wp.lon,
            altitude_m: wp.altitude_m,
        })
        .collect();

    let clearance_m = request
        .safety_buffer_m
        .unwrap_or(config.compliance_default_clearance_m);
    let client = Client::new();
    let analysis = match fetch_obstacles(
        &client,
        config,
        &points,
        clearance_m,
        Some(max_lane_radius + clearance_m),
    )
    .await
    {
        Ok(result) => result,
        Err(err) => {
            return RoutePlanResponse {
                ok: false,
                waypoints: Vec::new(),
                stats: None,
                nodes_visited: 0,
                optimized_points: 0,
                sample_points: 0,
                hazards: Vec::new(),
                errors: vec![format!("obstacle fetch failed: {}", err)],
            };
        }
    };

    if analysis.truncated {
        return RoutePlanResponse {
            ok: false,
            waypoints: Vec::new(),
            stats: None,
            nodes_visited: 0,
            optimized_points: 0,
            sample_points: 0,
            hazards: Vec::new(),
            errors: vec![
                "obstacle dataset truncated; increase ATC_COMPLIANCE_MAX_OVERPASS_ELEMENTS"
                    .to_string(),
            ],
        };
    }

    let hazards = analysis.hazards.clone();
    let obstacles: Vec<RouteObstacle> = analysis
        .candidates
        .iter()
        .map(|candidate| RouteObstacle {
            lat: candidate.lat,
            lon: candidate.lon,
            radius_m: candidate.radius_m,
            height_m: Some(candidate.height_m),
        })
        .collect();

    let terrain = match fetch_terrain_grid(&client, config, &points, base_spacing).await {
        Ok(grid) => grid,
        Err(err) => {
            if config.terrain_require {
                return RoutePlanResponse {
                    ok: false,
                    waypoints: Vec::new(),
                    stats: None,
                    nodes_visited: 0,
                    optimized_points: 0,
                    sample_points: 0,
                    hazards: Vec::new(),
                    errors: vec![format!("terrain fetch failed: {}", err)],
                };
            }
            tracing::warn!("Terrain fetch failed, continuing without terrain: {}", err);
            None
        }
    };

    let mut geofences = state.get_geofences();
    geofences.extend(build_obstacle_geofences(&analysis.footprints, terrain.as_ref()));
    let geofences: Vec<Geofence> = geofences
        .into_iter()
        .filter(|fence| fence.active && fence.geofence_type != GeofenceType::Advisory)
        .collect();

    let mut lane_radius = lane_radius;
    let mut last_errors = Vec::new();
    let mut last_sample_points = 0usize;
    let mut last_result = None;

    while lane_radius <= max_lane_radius + f64::EPSILON {
        let lane_offsets = build_lane_offsets(lane_radius, lane_spacing);
        let spacing = resolve_spacing_for_grid(route_distance_total, base_spacing, lane_offsets.len());
        let Some(mut grid) = generate_grid_samples(&request.waypoints, spacing, &lane_offsets) else {
            last_errors = vec!["failed to generate grid".to_string()];
            break;
        };
        last_sample_points = grid
            .lanes
            .first()
            .map(|lane| lane.len() * grid.lanes.len())
            .unwrap_or(0);
        if last_sample_points > MAX_ROUTE_GRID_POINTS {
            last_errors = vec![format!(
                "route grid too large ({} points)",
                last_sample_points
            )];
            break;
        }

        let terrain_ref = terrain.as_ref();
        apply_obstacles(&mut grid, &obstacles, |lat, lon| {
            terrain_ref
                .map(|grid| grid.sample(lat, lon))
                .unwrap_or(0.0)
        });

        let mut engine_config = RouteEngineConfig::default();
        engine_config.safety_buffer_m = clearance_m;
        engine_config.geofence_sample_step_m = spacing.clamp(5.0, 25.0);

        let result = optimize_flight_path(&request.waypoints, &grid, &geofences, &engine_config);
        if result.success {
            return build_response(result, hazards, last_sample_points);
        }

        last_errors = result.errors.clone();
        last_result = Some(result);
        lane_radius += expansion_step;
    }

    let mut errors = last_errors;
    if errors.is_empty() {
        errors.push("A* failed to find a path".to_string());
    }
    errors.push(format!(
        "no path within lane radius {:.1}m",
        max_lane_radius
    ));

    let (stats, nodes_visited, optimized_points) = match last_result.as_ref() {
        Some(result) => (result.stats.clone(), result.nodes_visited, result.optimized_points),
        None => (None, 0, 0),
    };

    RoutePlanResponse {
        ok: false,
        waypoints: Vec::new(),
        stats,
        nodes_visited,
        optimized_points,
        sample_points: last_sample_points,
        hazards,
        errors,
    }
}

async fn plan_route_segmented(
    state: &AppState,
    config: &Config,
    request: RoutePlanRequest,
) -> RoutePlanResponse {
    let lane_radius = request.lane_radius_m.unwrap_or(DEFAULT_LANE_RADIUS_M);
    let lane_spacing = request.lane_spacing_m.unwrap_or(DEFAULT_LANE_SPACING_M);
    let default_spacing = request.sample_spacing_m.unwrap_or(DEFAULT_SAMPLE_SPACING_M);
    let clearance_m = request
        .safety_buffer_m
        .unwrap_or(config.compliance_default_clearance_m);
    let route_distance_total = route_distance_m(&request.waypoints);
    let expansion_step = request
        .lane_expansion_step_m
        .unwrap_or(DEFAULT_LANE_EXPANSION_STEP_M)
        .max(5.0);

    let mut segment_length = DEFAULT_SEGMENT_LENGTH_M.min(route_distance_total);
    let mut last_error: Option<Vec<String>> = None;

    for _attempt in 0..3 {
        let segments = build_segments(&request.waypoints, segment_length);
        let segment_count = segments.len();
        if segment_count == 0 {
            return RoutePlanResponse {
                ok: false,
                waypoints: Vec::new(),
                stats: None,
                nodes_visited: 0,
                optimized_points: 0,
                sample_points: 0,
                hazards: Vec::new(),
                errors: vec!["failed to segment route".to_string()],
            };
        }

        let mut combined_waypoints: Vec<RouteEngineWaypoint> = Vec::new();
        let mut hazards: Vec<ObstacleHazard> = Vec::new();
        let mut hazards_seen = std::collections::HashSet::new();
        let mut nodes_visited = 0usize;
        let mut optimized_points = 0usize;
        let mut sample_points = 0usize;
        let mut stats = None;
        let mut start_terrain: Option<TerrainGrid> = None;
        let mut end_terrain: Option<TerrainGrid> = None;
        let mut truncated = false;

        let mut previous_altitude: Option<f64> = None;
        for (idx, segment) in segments.iter().enumerate() {
            let mut segment = segment.clone();
            if let Some(prev_alt) = previous_altitude {
                if let Some(first) = segment.first_mut() {
                    first.altitude_m = prev_alt;
                }
                if idx + 1 < segment_count {
                    if let Some(last) = segment.last_mut() {
                        last.altitude_m = prev_alt;
                    }
                }
            }

            let base_spacing = resolve_grid_spacing(&segment, default_spacing);
            let plan = match plan_airborne_segment(
                state,
                config,
                &segment,
                lane_radius,
                lane_spacing,
                base_spacing,
                resolve_max_lane_radius(
                    route_distance_m(&segment),
                    lane_radius,
                    request.max_lane_radius_m,
                ),
                expansion_step,
                clearance_m,
            )
            .await
            {
                Ok(plan) => plan,
                Err(SegmentError::Truncated) => {
                    truncated = true;
                    break;
                }
                Err(SegmentError::Obstacle(err)) => {
                    return RoutePlanResponse {
                        ok: false,
                        waypoints: Vec::new(),
                        stats: None,
                        nodes_visited: 0,
                        optimized_points: 0,
                        sample_points: 0,
                        hazards: Vec::new(),
                        errors: vec![format!("obstacle fetch failed: {}", err)],
                    };
                }
                Err(SegmentError::Terrain(err)) => {
                    return RoutePlanResponse {
                        ok: false,
                        waypoints: Vec::new(),
                        stats: None,
                        nodes_visited: 0,
                        optimized_points: 0,
                        sample_points: 0,
                        hazards: Vec::new(),
                        errors: vec![format!("terrain fetch failed: {}", err)],
                    };
                }
                Err(SegmentError::GridTooLarge(count)) => {
                    return RoutePlanResponse {
                        ok: false,
                        waypoints: Vec::new(),
                        stats: None,
                        nodes_visited: 0,
                        optimized_points: 0,
                        sample_points: count,
                        hazards: Vec::new(),
                        errors: vec![format!("route grid too large ({} points)", count)],
                    };
                }
                Err(SegmentError::Path(errors)) => {
                    return RoutePlanResponse {
                        ok: false,
                        waypoints: Vec::new(),
                        stats: None,
                        nodes_visited: 0,
                        optimized_points: 0,
                        sample_points: 0,
                        hazards: Vec::new(),
                        errors,
                    };
                }
            };

            if idx == 0 {
                start_terrain = plan.terrain.clone();
            }
            if idx + 1 == segment_count {
                end_terrain = plan.terrain.clone();
            }

            for hazard in &plan.hazards {
                if hazards_seen.insert(hazard.id.clone()) {
                    hazards.push(hazard.clone());
                }
            }

            nodes_visited += plan.nodes_visited;
            optimized_points += plan.optimized_points;
            sample_points += plan.sample_points;
            stats = merge_stats(stats, plan.stats.clone());

            append_segment_waypoints(&mut combined_waypoints, plan.waypoints);
            previous_altitude = combined_waypoints.last().map(|wp| wp.altitude_m);
        }

        if truncated {
            segment_length = (segment_length * 0.5).max(MIN_SEGMENT_LENGTH_M);
            last_error = Some(vec![
                "obstacle dataset truncated; segment length reduced".to_string(),
            ]);
            continue;
        }

        if combined_waypoints.is_empty() {
            return RoutePlanResponse {
                ok: false,
                waypoints: Vec::new(),
                stats: None,
                nodes_visited,
                optimized_points,
                sample_points,
                hazards,
                errors: vec!["segment planning produced no waypoints".to_string()],
            };
        }

        let full_points: Vec<RoutePoint> = request
            .waypoints
            .iter()
            .map(|wp| RoutePoint {
                lat: wp.lat,
                lon: wp.lon,
                altitude_m: wp.altitude_m,
            })
            .collect();
        let client = Client::new();
        let full_terrain = fetch_terrain_grid(&client, config, &full_points, default_spacing)
            .await
            .ok()
            .flatten();

        let final_waypoints = wrap_with_takeoff_landing(
            &combined_waypoints,
            full_terrain.as_ref(),
            start_terrain.as_ref(),
            end_terrain.as_ref(),
        );
        let stats = stats.or_else(|| compute_stats_with_terrain(&final_waypoints, full_terrain.as_ref()));

        return RoutePlanResponse {
            ok: true,
            waypoints: final_waypoints,
            stats,
            nodes_visited,
            optimized_points,
            sample_points,
            hazards,
            errors: Vec::new(),
        };
    }

    RoutePlanResponse {
        ok: false,
        waypoints: Vec::new(),
        stats: None,
        nodes_visited: 0,
        optimized_points: 0,
        sample_points: 0,
        hazards: Vec::new(),
        errors: last_error.unwrap_or_else(|| vec!["route segmentation failed".to_string()]),
    }
}

async fn plan_airborne_segment(
    state: &AppState,
    config: &Config,
    waypoints: &[Waypoint],
    lane_radius: f64,
    lane_spacing: f64,
    base_spacing: f64,
    max_lane_radius: f64,
    expansion_step: f64,
    safety_buffer_m: f64,
) -> Result<SegmentPlan, SegmentError> {
    let points: Vec<RoutePoint> = waypoints
        .iter()
        .map(|wp| RoutePoint {
            lat: wp.lat,
            lon: wp.lon,
            altitude_m: wp.altitude_m,
        })
        .collect();

    let client = Client::new();
    let analysis = fetch_obstacles(
        &client,
        config,
        &points,
        safety_buffer_m,
        Some(max_lane_radius + safety_buffer_m),
    )
    .await
    .map_err(SegmentError::Obstacle)?;
    if analysis.truncated {
        return Err(SegmentError::Truncated);
    }

    let hazards = analysis.hazards.clone();
    let obstacles: Vec<RouteObstacle> = analysis
        .candidates
        .iter()
        .map(|candidate| RouteObstacle {
            lat: candidate.lat,
            lon: candidate.lon,
            radius_m: candidate.radius_m,
            height_m: Some(candidate.height_m),
        })
        .collect();

    let terrain = match fetch_terrain_grid(&client, config, &points, base_spacing).await {
        Ok(grid) => grid,
        Err(err) => {
            if config.terrain_require {
                return Err(SegmentError::Terrain(err));
            }
            tracing::warn!("Terrain fetch failed, continuing without terrain: {}", err);
            None
        }
    };

    let mut geofences = state.get_geofences();
    geofences.extend(build_obstacle_geofences(&analysis.footprints, terrain.as_ref()));
    let geofences: Vec<Geofence> = geofences
        .into_iter()
        .filter(|fence| fence.active && fence.geofence_type != GeofenceType::Advisory)
        .collect();

    let route_distance_m = route_distance_m(waypoints);
    let mut lane_radius = lane_radius;
    let mut last_errors = Vec::new();
    let mut last_sample_points = 0usize;

    while lane_radius <= max_lane_radius + f64::EPSILON {
        let lane_offsets = build_lane_offsets(lane_radius, lane_spacing);
        let spacing = resolve_spacing_for_grid(route_distance_m, base_spacing, lane_offsets.len());
        let Some(mut grid) = generate_grid_samples(waypoints, spacing, &lane_offsets) else {
            last_errors = vec!["failed to generate grid".to_string()];
            break;
        };
        last_sample_points = grid
            .lanes
            .first()
            .map(|lane| lane.len() * grid.lanes.len())
            .unwrap_or(0);
        if last_sample_points > MAX_ROUTE_GRID_POINTS {
            return Err(SegmentError::GridTooLarge(last_sample_points));
        }

        let terrain_ref = terrain.as_ref();
        apply_obstacles(&mut grid, &obstacles, |lat, lon| {
            terrain_ref
                .map(|grid| grid.sample(lat, lon))
                .unwrap_or(0.0)
        });

        let mut engine_config = RouteEngineConfig::default();
        engine_config.safety_buffer_m = safety_buffer_m;
        engine_config.geofence_sample_step_m = spacing.clamp(5.0, 25.0);

        let result = optimize_airborne_path(waypoints, &grid, &geofences, &engine_config);
        if result.success && !result.waypoints.is_empty() {
            return Ok(SegmentPlan {
                waypoints: result.waypoints,
                stats: result.stats,
                nodes_visited: result.nodes_visited,
                optimized_points: result.optimized_points,
                sample_points: last_sample_points,
                hazards,
                terrain,
            });
        }

        last_errors = result.errors.clone();
        lane_radius += expansion_step;
    }

    let errors = if last_errors.is_empty() {
        vec!["A* failed to find a path".to_string()]
    } else {
        last_errors
    };
    Err(SegmentError::Path(errors))
}

pub fn plan_reroute(
    grid: &RouteGrid,
    waypoints: &[Waypoint],
    geofences: &[Geofence],
    safety_buffer_m: f64,
) -> RouteEngineResult {
    let mut engine_config = RouteEngineConfig::default();
    engine_config.safety_buffer_m = safety_buffer_m;
    optimize_flight_path(waypoints, grid, geofences, &engine_config)
}

fn build_obstacle_geofences(
    footprints: &[ObstacleFootprint],
    terrain: Option<&TerrainGrid>,
) -> Vec<Geofence> {
    let mut geofences = Vec::new();
    for footprint in footprints {
        if footprint.polygon.len() < 3 {
            continue;
        }
        if footprint.height_m <= 0.0 {
            continue;
        }
        let (centroid_lat, centroid_lon) = polygon_centroid(&footprint.polygon);
        let base_alt = terrain
            .map(|grid| grid.sample(centroid_lat, centroid_lon))
            .unwrap_or(0.0)
            .max(0.0);
        let upper = (base_alt + footprint.height_m).max(base_alt + 1.0);
        geofences.push(Geofence {
            id: format!("osm-{}", footprint.id),
            name: format!("OSM {}", footprint.name),
            geofence_type: GeofenceType::NoFlyZone,
            polygon: footprint.polygon.clone(),
            lower_altitude_m: base_alt,
            upper_altitude_m: upper,
            active: true,
            created_at: Utc::now(),
        });
    }
    geofences
}

pub async fn plan_airborne_route(
    state: &AppState,
    config: &Config,
    waypoints: &[Waypoint],
    safety_buffer_m: f64,
    extra_geofences: &[Geofence],
) -> Option<Vec<Waypoint>> {
    if waypoints.len() < 2 {
        return None;
    }

    let lane_radius = DEFAULT_LANE_RADIUS_M;
    let lane_spacing = DEFAULT_LANE_SPACING_M;
    let base_spacing = resolve_grid_spacing(waypoints, DEFAULT_SAMPLE_SPACING_M);
    let route_distance_m = route_distance_m(waypoints);

    let points: Vec<RoutePoint> = waypoints
        .iter()
        .map(|wp| RoutePoint {
            lat: wp.lat,
            lon: wp.lon,
            altitude_m: wp.altitude_m,
        })
        .collect();

    let max_lane_radius = resolve_max_lane_radius(route_distance_m, lane_radius, None);

    let client = Client::new();
    let analysis = fetch_obstacles(
        &client,
        config,
        &points,
        safety_buffer_m,
        Some(max_lane_radius + safety_buffer_m),
    )
    .await
    .ok()?;
    if analysis.truncated {
        return None;
    }

    let obstacles: Vec<RouteObstacle> = analysis
        .candidates
        .iter()
        .map(|candidate| RouteObstacle {
            lat: candidate.lat,
            lon: candidate.lon,
            radius_m: candidate.radius_m,
            height_m: Some(candidate.height_m),
        })
        .collect();

    let terrain = fetch_terrain_grid(&client, config, &points, base_spacing).await.ok().flatten();

    let mut geofences = state.get_geofences();
    geofences.extend(build_obstacle_geofences(&analysis.footprints, terrain.as_ref()));
    geofences.extend(extra_geofences.iter().cloned());
    let geofences: Vec<Geofence> = geofences
        .into_iter()
        .filter(|fence| fence.active && fence.geofence_type != GeofenceType::Advisory)
        .collect();

    let mut lane_radius = lane_radius;
    let mut best_result: Option<RouteEngineResult> = None;
    while lane_radius <= max_lane_radius + f64::EPSILON {
        let lane_offsets = build_lane_offsets(lane_radius, lane_spacing);
        let spacing = resolve_spacing_for_grid(route_distance_m, base_spacing, lane_offsets.len());
        let Some(mut grid) = generate_grid_samples(waypoints, spacing, &lane_offsets) else {
            break;
        };
        let sample_points = grid
            .lanes
            .first()
            .map(|lane| lane.len() * grid.lanes.len())
            .unwrap_or(0);
        if sample_points > MAX_ROUTE_GRID_POINTS {
            break;
        }

        let terrain_ref = terrain.as_ref();
        apply_obstacles(&mut grid, &obstacles, |lat, lon| {
            terrain_ref
                .map(|grid| grid.sample(lat, lon))
                .unwrap_or(0.0)
        });

        let mut engine_config = RouteEngineConfig::default();
        engine_config.safety_buffer_m = safety_buffer_m;
        engine_config.faa_limit_agl = 500.0;
        engine_config.geofence_sample_step_m = spacing.clamp(5.0, 25.0);

        let result = optimize_airborne_path(waypoints, &grid, &geofences, &engine_config);
        if result.success && !result.waypoints.is_empty() {
            best_result = Some(result);
            break;
        }

        lane_radius += DEFAULT_LANE_EXPANSION_STEP_M;
    }

    let result = best_result?;
    let speed_mps = waypoints.first().and_then(|wp| wp.speed_mps);
    Some(
        result
            .waypoints
            .into_iter()
            .map(|wp| Waypoint {
                lat: wp.lat,
                lon: wp.lon,
                altitude_m: wp.altitude_m,
                speed_mps,
            })
            .collect(),
    )
}

fn build_response(
    result: RouteEngineResult,
    hazards: Vec<ObstacleHazard>,
    sample_points: usize,
) -> RoutePlanResponse {
    RoutePlanResponse {
        ok: result.success,
        waypoints: result.waypoints,
        stats: result.stats,
        nodes_visited: result.nodes_visited,
        optimized_points: result.optimized_points,
        sample_points,
        hazards,
        errors: result.errors,
    }
}

fn route_distance_m(waypoints: &[Waypoint]) -> f64 {
    let mut total = 0.0;
    for i in 1..waypoints.len() {
        total += haversine_distance(
            waypoints[i - 1].lat,
            waypoints[i - 1].lon,
            waypoints[i].lat,
            waypoints[i].lon,
        );
    }
    total
}

fn resolve_max_lane_radius(
    route_distance_m: f64,
    base_radius: f64,
    override_radius: Option<f64>,
) -> f64 {
    let mut radius = override_radius.unwrap_or_else(|| {
        if route_distance_m > 12_000.0 {
            1500.0
        } else if route_distance_m > 8_000.0 {
            1000.0
        } else if route_distance_m > 4_000.0 {
            700.0
        } else {
            DEFAULT_MAX_LANE_RADIUS_M
        }
    });
    if radius < base_radius {
        radius = base_radius;
    }
    radius
}

fn polygon_centroid(polygon: &[[f64; 2]]) -> (f64, f64) {
    let mut sum_lat = 0.0;
    let mut sum_lon = 0.0;
    let mut count = 0.0;
    for point in polygon {
        sum_lat += point[0];
        sum_lon += point[1];
        count += 1.0;
    }
    if count <= 0.0 {
        (0.0, 0.0)
    } else {
        (sum_lat / count, sum_lon / count)
    }
}

fn build_segments(waypoints: &[Waypoint], segment_length_m: f64) -> Vec<Vec<Waypoint>> {
    if waypoints.len() < 2 {
        return Vec::new();
    }
    let mut segments = Vec::new();
    let length = segment_length_m.max(500.0);

    for pair in waypoints.windows(2) {
        let start = &pair[0];
        let end = &pair[1];
        let distance = haversine_distance(start.lat, start.lon, end.lat, end.lon);
        if !distance.is_finite() || distance <= 0.0 {
            continue;
        }
        if distance <= length {
            segments.push(vec![start.clone(), end.clone()]);
            continue;
        }

        let heading = bearing(start.lat, start.lon, end.lat, end.lon);
        let segment_count = (distance / length).ceil().max(1.0) as usize;
        let speed_mps = start.speed_mps.or(end.speed_mps);

        for idx in 0..segment_count {
            let start_dist = idx as f64 * length;
            let end_dist = ((idx + 1) as f64 * length).min(distance);

            let start_frac = start_dist / distance;
            let end_frac = end_dist / distance;

            let (seg_start_lat, seg_start_lon) =
                offset_by_bearing(start.lat, start.lon, start_dist, heading);
            let (seg_end_lat, seg_end_lon) =
                offset_by_bearing(start.lat, start.lon, end_dist, heading);

            let seg_start_alt = start.altitude_m + start_frac * (end.altitude_m - start.altitude_m);
            let seg_end_alt = start.altitude_m + end_frac * (end.altitude_m - start.altitude_m);

            segments.push(vec![
                Waypoint {
                    lat: seg_start_lat,
                    lon: seg_start_lon,
                    altitude_m: seg_start_alt,
                    speed_mps,
                },
                Waypoint {
                    lat: seg_end_lat,
                    lon: seg_end_lon,
                    altitude_m: seg_end_alt,
                    speed_mps,
                },
            ]);
        }
    }

    segments
}

fn resolve_spacing_for_grid(distance_m: f64, base_spacing: f64, lane_count: usize) -> f64 {
    let mut spacing = base_spacing.max(1.0);
    loop {
        let points = estimate_grid_points(distance_m, spacing, lane_count);
        if points <= MAX_ROUTE_GRID_POINTS || spacing >= MAX_SAMPLE_SPACING_M {
            return spacing;
        }
        spacing = (spacing * 1.5).min(MAX_SAMPLE_SPACING_M);
    }
}

fn estimate_grid_points(distance_m: f64, spacing_m: f64, lane_count: usize) -> usize {
    let steps = (distance_m / spacing_m.max(1.0)).ceil().max(1.0) as usize + 1;
    steps.saturating_mul(lane_count.max(1))
}

fn append_segment_waypoints(
    combined: &mut Vec<RouteEngineWaypoint>,
    segment: Vec<RouteEngineWaypoint>,
) {
    for waypoint in segment {
        if let Some(last) = combined.last() {
            if (last.lat - waypoint.lat).abs() < 1e-6
                && (last.lon - waypoint.lon).abs() < 1e-6
                && (last.altitude_m - waypoint.altitude_m).abs() < 0.1
            {
                continue;
            }
        }
        combined.push(waypoint);
    }
}

fn wrap_with_takeoff_landing(
    cruise_path: &[RouteEngineWaypoint],
    terrain: Option<&TerrainGrid>,
    start_fallback: Option<&TerrainGrid>,
    end_fallback: Option<&TerrainGrid>,
) -> Vec<RouteEngineWaypoint> {
    if cruise_path.is_empty() {
        return Vec::new();
    }

    let first = &cruise_path[0];
    let last = cruise_path.last().unwrap();
    let start_ground = sample_ground(
        terrain,
        start_fallback,
        first.lat,
        first.lon,
    );
    let end_ground = sample_ground(
        terrain,
        end_fallback,
        last.lat,
        last.lon,
    );

    let mut output = Vec::new();
    output.push(RouteEngineWaypoint {
        lat: first.lat,
        lon: first.lon,
        altitude_m: start_ground,
        phase: Some("GROUND_START".to_string()),
    });
    output.push(RouteEngineWaypoint {
        lat: first.lat,
        lon: first.lon,
        altitude_m: first.altitude_m,
        phase: Some("VERTICAL_ASCENT".to_string()),
    });
    output.extend_from_slice(cruise_path);
    output.push(RouteEngineWaypoint {
        lat: last.lat,
        lon: last.lon,
        altitude_m: last.altitude_m,
        phase: Some("VERTICAL_DESCENT".to_string()),
    });
    output.push(RouteEngineWaypoint {
        lat: last.lat,
        lon: last.lon,
        altitude_m: end_ground,
        phase: Some("GROUND_END".to_string()),
    });
    output
}

fn sample_ground(
    primary: Option<&TerrainGrid>,
    fallback: Option<&TerrainGrid>,
    lat: f64,
    lon: f64,
) -> f64 {
    let primary_val = primary.map(|grid| grid.sample(lat, lon));
    let fallback_val = fallback.map(|grid| grid.sample(lat, lon));
    primary_val
        .or(fallback_val)
        .unwrap_or(0.0)
        .max(0.0)
}

fn merge_stats(
    current: Option<atc_core::route_engine::RouteEngineStats>,
    next: Option<atc_core::route_engine::RouteEngineStats>,
) -> Option<atc_core::route_engine::RouteEngineStats> {
    match (current, next) {
        (None, None) => None,
        (Some(stats), None) | (None, Some(stats)) => Some(stats),
        (Some(a), Some(b)) => Some(atc_core::route_engine::RouteEngineStats {
            avg_agl: (a.avg_agl + b.avg_agl) / 2.0,
            max_agl: a.max_agl.max(b.max_agl),
            max_altitude: a.max_altitude.max(b.max_altitude),
        }),
    }
}

fn compute_stats_with_terrain(
    waypoints: &[RouteEngineWaypoint],
    terrain: Option<&TerrainGrid>,
) -> Option<atc_core::route_engine::RouteEngineStats> {
    if waypoints.is_empty() {
        return None;
    }
    let mut max_alt = f64::NEG_INFINITY;
    let mut max_agl = f64::NEG_INFINITY;
    let mut sum_agl = 0.0;
    let mut count = 0.0;
    for wp in waypoints {
        let ground = terrain.map(|grid| grid.sample(wp.lat, wp.lon)).unwrap_or(0.0);
        let agl = (wp.altitude_m - ground).max(0.0);
        sum_agl += agl;
        count += 1.0;
        if wp.altitude_m > max_alt {
            max_alt = wp.altitude_m;
        }
        if agl > max_agl {
            max_agl = agl;
        }
    }
    if count <= 0.0 {
        return None;
    }
    Some(atc_core::route_engine::RouteEngineStats {
        avg_agl: sum_agl / count,
        max_agl,
        max_altitude: max_alt,
    })
}
