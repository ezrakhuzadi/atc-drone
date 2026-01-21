//! Server-side route planning using the backend A* engine.

use atc_core::models::{Geofence, GeofenceType, Waypoint};
use atc_core::route_engine::{
    RouteEngineConfig, RouteEngineResult, RouteEngineWaypoint, RouteObstacle,
    apply_obstacles, build_lane_offsets, generate_grid_samples, optimize_airborne_path,
    optimize_flight_path, resolve_grid_spacing,
};
use atc_core::spatial::{bearing, haversine_distance, offset_by_bearing};
use reqwest::Client;
use serde::{Deserialize, Serialize};
use std::sync::Arc;
use std::time::Instant;
use tokio::task;
use tokio::sync::Semaphore;

use crate::altitude::altitude_to_amsl;
use crate::compliance::{fetch_obstacles, ObstacleAnalysis, ObstacleHazard, ObstacleQueryMode, RoutePoint};
use crate::config::Config;
use crate::state::AppState;
use crate::terrain::{fetch_terrain_grid, TerrainGrid};

const DEFAULT_LANE_RADIUS_M: f64 = 90.0;
const DEFAULT_LANE_SPACING_M: f64 = 15.0;
const MIN_LANE_SPACING_M: f64 = 5.0;
const DEFAULT_SAMPLE_SPACING_M: f64 = 5.0;
const DEFAULT_MAX_LANE_RADIUS_M: f64 = 800.0;
const DEFAULT_LANE_EXPANSION_STEP_M: f64 = 50.0;
const MAX_ROUTE_GRID_POINTS: usize = 500_000;
const DEFAULT_SEGMENT_LENGTH_M: f64 = 8_000.0;
const MIN_SEGMENT_LENGTH_M: f64 = 1_000.0;
const MAX_SAMPLE_SPACING_M: f64 = 75.0;
const SEGMENT_PREFETCH_CONCURRENCY: usize = 4;
const HARD_MAX_LANE_RADIUS_M: f64 = 5_000.0;
const HARD_MAX_SAFETY_BUFFER_M: f64 = 500.0;

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
    terrain: Option<Arc<TerrainGrid>>,
}

#[derive(Debug)]
struct SegmentInputs {
    hazards: Vec<ObstacleHazard>,
    obstacles: Vec<RouteObstacle>,
    terrain: Option<Arc<TerrainGrid>>,
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
    mut request: RoutePlanRequest,
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

    let mut validation_errors = Vec::new();
    for (idx, wp) in request.waypoints.iter().enumerate() {
        if !wp.lat.is_finite() || !wp.lon.is_finite() {
            validation_errors.push(format!("waypoint[{idx}] lat/lon must be finite"));
            continue;
        }
        if !( -90.0..=90.0).contains(&wp.lat) || !( -180.0..=180.0).contains(&wp.lon) {
            validation_errors.push(format!("waypoint[{idx}] lat/lon out of range"));
        }
        if !wp.altitude_m.is_finite() {
            validation_errors.push(format!("waypoint[{idx}] altitude_m must be finite"));
        }
        if let Some(speed) = wp.speed_mps {
            if !speed.is_finite() || speed < 0.0 {
                validation_errors.push(format!("waypoint[{idx}] speed_mps must be a non-negative finite number"));
            }
        }
    }

    let clamp_opt = |value: Option<f64>, min: f64, max: f64| -> Option<f64> {
        value.map(|v| v.clamp(min, max))
    };

    if let Some(radius) = request.lane_radius_m {
        if !radius.is_finite() || radius <= 0.0 {
            validation_errors.push("lane_radius_m must be a positive finite number".to_string());
        }
    }
    if let Some(radius) = request.max_lane_radius_m {
        if !radius.is_finite() || radius <= 0.0 {
            validation_errors.push("max_lane_radius_m must be a positive finite number".to_string());
        }
    }
    if let Some(spacing) = request.lane_spacing_m {
        if !spacing.is_finite() || spacing <= 0.0 {
            validation_errors.push("lane_spacing_m must be a positive finite number".to_string());
        }
    }
    if let Some(spacing) = request.sample_spacing_m {
        if !spacing.is_finite() || spacing <= 0.0 {
            validation_errors.push("sample_spacing_m must be a positive finite number".to_string());
        }
    }
    if let Some(buffer) = request.safety_buffer_m {
        if !buffer.is_finite() || buffer < 0.0 {
            validation_errors.push("safety_buffer_m must be a non-negative finite number".to_string());
        }
    }
    if let Some(step) = request.lane_expansion_step_m {
        if !step.is_finite() || step <= 0.0 {
            validation_errors.push("lane_expansion_step_m must be a positive finite number".to_string());
        }
    }

    if !validation_errors.is_empty() {
        return RoutePlanResponse {
            ok: false,
            waypoints: Vec::new(),
            stats: None,
            nodes_visited: 0,
            optimized_points: 0,
            sample_points: 0,
            hazards: Vec::new(),
            errors: validation_errors,
        };
    }

    request.lane_radius_m = clamp_opt(request.lane_radius_m, 1.0, HARD_MAX_LANE_RADIUS_M);
    request.max_lane_radius_m = clamp_opt(request.max_lane_radius_m, DEFAULT_MAX_LANE_RADIUS_M, HARD_MAX_LANE_RADIUS_M);
    request.lane_spacing_m = clamp_opt(request.lane_spacing_m, MIN_LANE_SPACING_M, 250.0);
    request.sample_spacing_m = clamp_opt(request.sample_spacing_m, 1.0, MAX_SAMPLE_SPACING_M);
    request.safety_buffer_m = clamp_opt(request.safety_buffer_m, 0.0, HARD_MAX_SAFETY_BUFFER_M);
    request.lane_expansion_step_m = clamp_opt(request.lane_expansion_step_m, 5.0, 2_000.0);
    let route_distance_total = route_distance_m(&request.waypoints);
    if route_distance_total <= f64::EPSILON {
        return RoutePlanResponse {
            ok: false,
            waypoints: Vec::new(),
            stats: None,
            nodes_visited: 0,
            optimized_points: 0,
            sample_points: 0,
            hazards: Vec::new(),
            errors: vec!["route distance is zero (start and end waypoints must differ)".to_string()],
        };
    }
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
    let started_at = Instant::now();
    let lane_radius = request.lane_radius_m.unwrap_or(DEFAULT_LANE_RADIUS_M);
    let lane_spacing = request.lane_spacing_m.unwrap_or(DEFAULT_LANE_SPACING_M);
    let default_spacing = request.sample_spacing_m.unwrap_or(DEFAULT_SAMPLE_SPACING_M);
    let waypoints = normalize_waypoints(&request.waypoints, config);
    let route_distance_total = route_distance_m(&waypoints);
    let base_spacing = resolve_grid_spacing(&waypoints, default_spacing);
    let max_lane_radius = resolve_max_lane_radius(
        route_distance_total,
        lane_radius,
        request.max_lane_radius_m,
    );
    let expansion_step = request
        .lane_expansion_step_m
        .unwrap_or(DEFAULT_LANE_EXPANSION_STEP_M)
        .max(5.0);

    let points: Vec<RoutePoint> = waypoints
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
    let obstacles_started_at = Instant::now();
    let analysis = match fetch_obstacles(
        &client,
        config,
        &points,
        clearance_m,
        Some(max_lane_radius + clearance_m),
        ObstacleQueryMode::RoutePlanner,
    )
    .await
    {
        Ok(result) => result,
        Err(err) => {
            if config.route_planner_require_obstacles {
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
            tracing::warn!("Obstacle fetch failed, continuing without obstacles: {}", err);
            empty_obstacle_analysis()
        }
    };
    tracing::info!(
        "RoutePlan obstacles: {} candidate(s), {} footprint(s), truncated={}, {} ms",
        analysis.obstacle_count,
        analysis.footprints.len(),
        analysis.truncated,
        obstacles_started_at.elapsed().as_millis()
    );

    if analysis.truncated {
        if config.route_planner_allow_truncated_obstacles {
            tracing::warn!("Obstacle dataset truncated; continuing with partial data");
        } else {
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

    let terrain_started_at = Instant::now();
    let terrain = match fetch_terrain_grid(&client, config, &points, base_spacing).await {
        Ok(grid) => grid.map(Arc::new),
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
    tracing::info!(
        "RoutePlan terrain: enabled={}, {} ms",
        terrain.is_some(),
        terrain_started_at.elapsed().as_millis()
    );

    let geofences: Arc<Vec<Geofence>> = Arc::new(
        state
            .get_geofences()
            .into_iter()
            .filter(|fence| fence.active && fence.geofence_type != GeofenceType::Advisory)
            .collect(),
    );

    let waypoints: Arc<Vec<Waypoint>> = Arc::new(waypoints);
    let obstacles: Arc<Vec<RouteObstacle>> = Arc::new(obstacles);

    let mut last_errors = Vec::new();
    let mut last_sample_points = 0usize;
    let mut last_result = None;
    let spacing_candidates = lane_spacing_candidates(lane_spacing);
    let wind_mps = config.route_planner_wind_mps.max(0.0);
    let radius_candidates = lane_radius_candidates(lane_radius, max_lane_radius, expansion_step);
    let phase_all = grid_phase_candidates();

    for (radius_idx, lane_radius) in radius_candidates.iter().copied().enumerate() {
        let is_last_radius = radius_idx + 1 == radius_candidates.len();
        let primary_spacing = spacing_candidates.first().copied().unwrap_or(lane_spacing);
        let spacing_single = [primary_spacing];
        let phases_single = [0.0_f64];
        let phases = if is_last_radius { &phase_all[..] } else { &phases_single[..] };
        let spacings = if is_last_radius {
            &spacing_candidates[..]
        } else {
            &spacing_single[..]
        };

        let mut attempted = false;
        for lane_spacing in spacings.iter().copied() {
            let lane_offsets = build_lane_offsets(lane_radius, lane_spacing);
            let spacing = resolve_spacing_for_grid(route_distance_total, base_spacing, lane_offsets.len());
            let estimated_points = estimate_grid_points(route_distance_total, spacing, lane_offsets.len());
            if estimated_points > MAX_ROUTE_GRID_POINTS {
                last_errors = vec![format!("route grid too large (estimated {} points)", estimated_points)];
                continue;
            }

            for phase in phases.iter().copied() {
                attempted = true;
                let waypoints = waypoints.clone();
                let obstacles = obstacles.clone();
                let geofences = geofences.clone();
                let terrain = terrain.clone();
                let lane_offsets = lane_offsets.clone();
                let attempt_started_at = Instant::now();
                let engine_config = {
                    let mut engine_config = RouteEngineConfig::default();
                    engine_config.safety_buffer_m = clearance_m;
                    engine_config.wind_mps = wind_mps;
                    engine_config.geofence_sample_step_m = spacing.clamp(5.0, 25.0);
                    engine_config
                };

                let handle = task::spawn_blocking(move || {
                    let Some(mut grid) = generate_grid_samples(&waypoints, spacing, &lane_offsets, phase) else {
                        return Err(vec!["failed to generate grid".to_string()]);
                    };
                    let sample_points = grid
                        .lanes
                        .first()
                        .map(|lane| lane.len() * grid.lanes.len())
                        .unwrap_or(0);
                    if sample_points > MAX_ROUTE_GRID_POINTS {
                        return Err(vec![format!("route grid too large ({} points)", sample_points)]);
                    }
                    apply_obstacles(&mut grid, &obstacles, |lat, lon| {
                        terrain
                            .as_ref()
                            .map(|grid| grid.sample(lat, lon))
                            .unwrap_or(0.0)
                    });
                    let result = optimize_flight_path(&waypoints, &grid, &geofences, &engine_config);
                    Ok((result, sample_points))
                });

                let attempt = match handle.await {
                    Ok(Ok(value)) => value,
                    Ok(Err(errors)) => {
                        last_errors = errors;
                        continue;
                    }
                    Err(err) => {
                        last_errors = vec![format!("planner task failed: {}", err)];
                        continue;
                    }
                };

                let (result, sample_points) = attempt;
                last_sample_points = sample_points;
                let attempt_ms = attempt_started_at.elapsed().as_millis();
                if attempt_ms >= 250 {
                    tracing::info!(
                        lane_radius_m = lane_radius,
                        lane_spacing_m = lane_spacing,
                        grid_spacing_m = spacing,
                        grid_phase = phase,
                        success = result.success,
                        nodes_visited = result.nodes_visited,
                        optimized_points = result.optimized_points,
                        elapsed_ms = attempt_ms as u64,
                        errors = ?result.errors,
                        "RoutePlan A* attempt"
                    );
                }
                if result.success {
                    let response = build_response(result, hazards, last_sample_points);
                    tracing::info!(
                        ok = response.ok,
                        nodes_visited = response.nodes_visited,
                        optimized_points = response.optimized_points,
                        sample_points = response.sample_points,
                        elapsed_ms = started_at.elapsed().as_millis() as u64,
                        "RoutePlan completed"
                    );
                    return response;
                }

                last_errors = result.errors.clone();
                last_result = Some(result);
            }
        }

        if !attempted && last_errors.is_empty() {
            last_errors = vec!["failed to generate grid".to_string()];
        }
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

    let response = RoutePlanResponse {
        ok: false,
        waypoints: Vec::new(),
        stats,
        nodes_visited,
        optimized_points,
        sample_points: last_sample_points,
        hazards,
        errors,
    };
    tracing::info!(
        ok = response.ok,
        nodes_visited = response.nodes_visited,
        optimized_points = response.optimized_points,
        sample_points = response.sample_points,
        elapsed_ms = started_at.elapsed().as_millis() as u64,
        errors = ?response.errors,
        "RoutePlan completed"
    );
    response
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
    let normalized_waypoints = normalize_waypoints(&request.waypoints, config);
    let route_distance_total = route_distance_m(&normalized_waypoints);
    let expansion_step = request
        .lane_expansion_step_m
        .unwrap_or(DEFAULT_LANE_EXPANSION_STEP_M)
        .max(5.0);

    let mut segment_length = DEFAULT_SEGMENT_LENGTH_M.min(route_distance_total);
    let mut last_error: Option<Vec<String>> = None;
    let wind_mps = config.route_planner_wind_mps.max(0.0);
    let geofences: Arc<Vec<Geofence>> = Arc::new(
        state
            .get_geofences()
            .into_iter()
            .filter(|fence| fence.active && fence.geofence_type != GeofenceType::Advisory)
            .collect(),
    );
    let client = Client::new();

    for _attempt in 0..4 {
        let segments = build_segments(&normalized_waypoints, segment_length);
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
        let mut start_terrain: Option<Arc<TerrainGrid>> = None;
        let mut end_terrain: Option<Arc<TerrainGrid>> = None;
        let mut truncated = false;

        let semaphore = Arc::new(Semaphore::new(SEGMENT_PREFETCH_CONCURRENCY.max(1)));
        let mut join_set = task::JoinSet::new();
        let mut segment_params: Vec<(f64, f64)> = Vec::with_capacity(segment_count);

        for (idx, segment) in segments.iter().enumerate() {
            let base_spacing = resolve_grid_spacing(segment, default_spacing);
            let max_lane_radius = resolve_max_lane_radius(
                route_distance_m(segment),
                lane_radius,
                request.max_lane_radius_m,
            );
            segment_params.push((base_spacing, max_lane_radius));

            let permit = match semaphore.clone().acquire_owned().await {
                Ok(permit) => permit,
                Err(_) => {
                    return RoutePlanResponse {
                        ok: false,
                        waypoints: Vec::new(),
                        stats: None,
                        nodes_visited: 0,
                        optimized_points: 0,
                        sample_points: 0,
                        hazards: Vec::new(),
                        errors: vec!["failed to acquire segment prefetch permit".to_string()],
                    };
                }
            };

            let client = client.clone();
            let config = config.clone();
            let segment = segment.clone();
            let base_spacing = base_spacing;
            let max_lane_radius = max_lane_radius;
            join_set.spawn(async move {
                let _permit = permit;
                let inputs = fetch_segment_inputs(
                    &client,
                    &config,
                    &segment,
                    clearance_m,
                    base_spacing,
                    max_lane_radius,
                )
                .await;
                (idx, inputs)
            });
        }

        let mut prefetched: Vec<Option<Result<SegmentInputs, SegmentError>>> =
            Vec::with_capacity(segment_count);
        prefetched.resize_with(segment_count, || None);
        while let Some(res) = join_set.join_next().await {
            match res {
                Ok((idx, inputs)) => {
                    if idx < prefetched.len() {
                        prefetched[idx] = Some(inputs);
                    }
                }
                Err(err) => {
                    return RoutePlanResponse {
                        ok: false,
                        waypoints: Vec::new(),
                        stats: None,
                        nodes_visited: 0,
                        optimized_points: 0,
                        sample_points: 0,
                        hazards: Vec::new(),
                        errors: vec![format!("segment prefetch task failed: {}", err)],
                    };
                }
            }
        }

        let mut previous_altitude: Option<f64> = None;
        for idx in 0..segment_count {
            let mut segment = segments[idx].clone();
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

            let (base_spacing, max_lane_radius) = segment_params[idx];
            let inputs = match prefetched
                .get_mut(idx)
                .and_then(|entry| entry.take())
                .unwrap_or_else(|| Err(SegmentError::Obstacle("missing segment data".to_string())))
            {
                Ok(inputs) => inputs,
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
                Err(err) => {
                    return RoutePlanResponse {
                        ok: false,
                        waypoints: Vec::new(),
                        stats: None,
                        nodes_visited: 0,
                        optimized_points: 0,
                        sample_points: 0,
                        hazards: Vec::new(),
                        errors: vec![format!("segment prefetch failed: {:?}", err)],
                    };
                }
            };

            let plan = match solve_airborne_segment(
                &segment,
                lane_radius,
                lane_spacing,
                base_spacing,
                max_lane_radius,
                expansion_step,
                clearance_m,
                wind_mps,
                inputs,
                geofences.clone(),
            )
            .await
            {
                Ok(plan) => plan,
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
                Err(err) => {
                    return RoutePlanResponse {
                        ok: false,
                        waypoints: Vec::new(),
                        stats: None,
                        nodes_visited: 0,
                        optimized_points: 0,
                        sample_points: 0,
                        hazards: Vec::new(),
                errors: vec![format!("segment planning failed: {:?}", err)],
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

        let full_points: Vec<RoutePoint> = normalized_waypoints
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
            .flatten()
            .map(Arc::new);

        let final_waypoints = wrap_with_takeoff_landing(
            &combined_waypoints,
            full_terrain.as_deref(),
            start_terrain.as_deref(),
            end_terrain.as_deref(),
        );
        let stats = stats.or_else(|| compute_stats_with_terrain(&final_waypoints, full_terrain.as_deref()));

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

async fn fetch_segment_inputs(
    client: &Client,
    config: &Config,
    waypoints: &[Waypoint],
    safety_buffer_m: f64,
    base_spacing: f64,
    max_lane_radius: f64,
) -> Result<SegmentInputs, SegmentError> {
    let points: Vec<RoutePoint> = waypoints
        .iter()
        .map(|wp| RoutePoint {
            lat: wp.lat,
            lon: wp.lon,
            altitude_m: wp.altitude_m,
        })
        .collect();

    let obstacles_started_at = Instant::now();
    let analysis = match fetch_obstacles(
        client,
        config,
        &points,
        safety_buffer_m,
        Some(max_lane_radius + safety_buffer_m),
        ObstacleQueryMode::RoutePlanner,
    )
    .await
    {
        Ok(result) => result,
        Err(err) => {
            if config.route_planner_require_obstacles {
                return Err(SegmentError::Obstacle(err));
            }
            tracing::warn!("Obstacle fetch failed, continuing without obstacles: {}", err);
            empty_obstacle_analysis()
        }
    };
    tracing::info!(
        "RoutePlan segment obstacles: {} candidate(s), {} footprint(s), truncated={}, {} ms",
        analysis.obstacle_count,
        analysis.footprints.len(),
        analysis.truncated,
        obstacles_started_at.elapsed().as_millis()
    );
    if analysis.truncated {
        if config.route_planner_allow_truncated_obstacles {
            tracing::warn!("Obstacle dataset truncated; continuing with partial data");
        } else {
            return Err(SegmentError::Truncated);
        }
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

    let terrain_started_at = Instant::now();
    let terrain = match fetch_terrain_grid(client, config, &points, base_spacing).await {
        Ok(grid) => grid.map(Arc::new),
        Err(err) => {
            if config.terrain_require {
                return Err(SegmentError::Terrain(err));
            }
            tracing::warn!("Terrain fetch failed, continuing without terrain: {}", err);
            None
        }
    };
    tracing::info!(
        "RoutePlan segment terrain: enabled={}, {} ms",
        terrain.is_some(),
        terrain_started_at.elapsed().as_millis()
    );

    Ok(SegmentInputs {
        hazards,
        obstacles,
        terrain,
    })
}

async fn solve_airborne_segment(
    waypoints: &[Waypoint],
    lane_radius: f64,
    lane_spacing: f64,
    base_spacing: f64,
    max_lane_radius: f64,
    expansion_step: f64,
    safety_buffer_m: f64,
    wind_mps: f64,
    inputs: SegmentInputs,
    geofences: Arc<Vec<Geofence>>,
) -> Result<SegmentPlan, SegmentError> {
    let route_distance_m = route_distance_m(waypoints);
    let waypoints: Arc<Vec<Waypoint>> = Arc::new(waypoints.to_vec());
    let hazards = inputs.hazards;
    let obstacles: Arc<Vec<RouteObstacle>> = Arc::new(inputs.obstacles);
    let terrain = inputs.terrain;

    let mut last_errors = Vec::new();
    let spacing_candidates = lane_spacing_candidates(lane_spacing);
    let radius_candidates = lane_radius_candidates(lane_radius, max_lane_radius, expansion_step);
    let phase_all = grid_phase_candidates();

    for (radius_idx, lane_radius) in radius_candidates.iter().copied().enumerate() {
        let is_last_radius = radius_idx + 1 == radius_candidates.len();
        let primary_spacing = spacing_candidates.first().copied().unwrap_or(lane_spacing);
        let spacing_single = [primary_spacing];
        let phases_single = [0.0_f64];
        let phases = if is_last_radius { &phase_all[..] } else { &phases_single[..] };
        let spacings = if is_last_radius {
            &spacing_candidates[..]
        } else {
            &spacing_single[..]
        };

        let mut attempted = false;
        for lane_spacing in spacings.iter().copied() {
            let lane_offsets = build_lane_offsets(lane_radius, lane_spacing);
            let spacing = resolve_spacing_for_grid(route_distance_m, base_spacing, lane_offsets.len());
            let estimated_points = estimate_grid_points(route_distance_m, spacing, lane_offsets.len());
            if estimated_points > MAX_ROUTE_GRID_POINTS {
                last_errors = vec![format!("route grid too large (estimated {} points)", estimated_points)];
                continue;
            }

            for phase in phases.iter().copied() {
                attempted = true;
                let waypoints = waypoints.clone();
                let obstacles = obstacles.clone();
                let geofences = geofences.clone();
                let terrain_for_task = terrain.clone();
                let lane_offsets = lane_offsets.clone();
                let engine_config = {
                    let mut engine_config = RouteEngineConfig::default();
                    engine_config.safety_buffer_m = safety_buffer_m;
                    engine_config.wind_mps = wind_mps;
                    engine_config.geofence_sample_step_m = spacing.clamp(5.0, 25.0);
                    engine_config
                };

                let attempt_started_at = Instant::now();
                let handle = task::spawn_blocking(move || {
                    let Some(mut grid) = generate_grid_samples(&waypoints, spacing, &lane_offsets, phase) else {
                        return Err(SegmentError::Path(vec!["failed to generate grid".to_string()]));
                    };
                    let sample_points = grid
                        .lanes
                        .first()
                        .map(|lane| lane.len() * grid.lanes.len())
                        .unwrap_or(0);
                    if sample_points > MAX_ROUTE_GRID_POINTS {
                        return Err(SegmentError::GridTooLarge(sample_points));
                    }
                    apply_obstacles(&mut grid, &obstacles, |lat, lon| {
                        terrain_for_task
                            .as_ref()
                            .map(|grid| grid.sample(lat, lon))
                            .unwrap_or(0.0)
                    });
                    let result = optimize_airborne_path(&waypoints, &grid, &geofences, &engine_config);
                    Ok((result, sample_points))
                });

                let attempt = match handle.await {
                    Ok(Ok(value)) => value,
                    Ok(Err(SegmentError::GridTooLarge(count))) => {
                        return Err(SegmentError::GridTooLarge(count));
                    }
                    Ok(Err(SegmentError::Path(errors))) => {
                        last_errors = errors;
                        continue;
                    }
                    Ok(Err(err)) => {
                        last_errors = vec![format!("segment planner failed: {:?}", err)];
                        continue;
                    }
                    Err(err) => {
                        last_errors = vec![format!("planner task failed: {}", err)];
                        continue;
                    }
                };

                let (result, sample_points) = attempt;
                let attempt_ms = attempt_started_at.elapsed().as_millis();
                if attempt_ms >= 250 {
                    tracing::info!(
                        lane_radius_m = lane_radius,
                        lane_spacing_m = lane_spacing,
                        grid_spacing_m = spacing,
                        grid_phase = phase,
                        success = result.success,
                        nodes_visited = result.nodes_visited,
                        optimized_points = result.optimized_points,
                        elapsed_ms = attempt_ms as u64,
                        errors = ?result.errors,
                        "RoutePlan segment A* attempt"
                    );
                }
                if result.success && !result.waypoints.is_empty() {
                    return Ok(SegmentPlan {
                        waypoints: result.waypoints,
                        stats: result.stats,
                        nodes_visited: result.nodes_visited,
                        optimized_points: result.optimized_points,
                        sample_points,
                        hazards,
                        terrain,
                    });
                }

                last_errors = result.errors.clone();
            }
        }
        if !attempted && last_errors.is_empty() {
            last_errors = vec!["failed to generate grid".to_string()];
        }
    }

    let errors = if last_errors.is_empty() {
        vec!["A* failed to find a path".to_string()]
    } else {
        last_errors
    };
    Err(SegmentError::Path(errors))
}


fn empty_obstacle_analysis() -> ObstacleAnalysis {
    ObstacleAnalysis {
        candidates: Vec::new(),
        hazards: Vec::new(),
        footprints: Vec::new(),
        obstacle_count: 0,
        truncated: false,
        building_count: 0,
        estimated_population: 0.0,
        density: 0.0,
        area_km2: 0.0,
    }
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
    let analysis = match fetch_obstacles(
        &client,
        config,
        &points,
        safety_buffer_m,
        Some(max_lane_radius + safety_buffer_m),
        ObstacleQueryMode::RoutePlanner,
    )
    .await
    {
        Ok(result) => result,
        Err(err) => {
            if config.route_planner_require_obstacles {
                return None;
            }
            tracing::warn!("Obstacle fetch failed, continuing without obstacles: {}", err);
            empty_obstacle_analysis()
        }
    };
    if analysis.truncated {
        if config.route_planner_allow_truncated_obstacles {
            tracing::warn!("Obstacle dataset truncated; continuing with partial data");
        } else {
            return None;
        }
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
    geofences.extend(extra_geofences.iter().cloned());
    let geofences: Vec<Geofence> = geofences
        .into_iter()
        .filter(|fence| fence.active && fence.geofence_type != GeofenceType::Advisory)
        .collect();

    let mut lane_radius = lane_radius;
    let mut best_result: Option<RouteEngineResult> = None;
    let spacing_candidates = lane_spacing_candidates(lane_spacing);
    let phase_candidates = grid_phase_candidates();
    while lane_radius <= max_lane_radius + f64::EPSILON {
        for lane_spacing in spacing_candidates.iter().copied() {
            let lane_offsets = build_lane_offsets(lane_radius, lane_spacing);
            let spacing = resolve_spacing_for_grid(route_distance_m, base_spacing, lane_offsets.len());
            let estimated_points = estimate_grid_points(route_distance_m, spacing, lane_offsets.len());
            if estimated_points > MAX_ROUTE_GRID_POINTS {
                continue;
            }

            for phase in phase_candidates {
                let Some(mut grid) = generate_grid_samples(waypoints, spacing, &lane_offsets, phase) else {
                    continue;
                };
                let sample_points = grid
                    .lanes
                    .first()
                    .map(|lane| lane.len() * grid.lanes.len())
                    .unwrap_or(0);
                if sample_points > MAX_ROUTE_GRID_POINTS {
                    continue;
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
                engine_config.wind_mps = config.route_planner_wind_mps.max(0.0);
                engine_config.geofence_sample_step_m = spacing.clamp(5.0, 25.0);

                let result = optimize_airborne_path(waypoints, &grid, &geofences, &engine_config);
                if result.success && !result.waypoints.is_empty() {
                    best_result = Some(result);
                    break;
                }
            }
            if best_result.is_some() {
                break;
            }
        }
        if best_result.is_some() {
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
            1800.0
        } else if route_distance_m > 8_000.0 {
            1400.0
        } else if route_distance_m > 4_000.0 {
            1000.0
        } else {
            DEFAULT_MAX_LANE_RADIUS_M
        }
    });
    if radius < base_radius {
        radius = base_radius;
    }
    if radius < DEFAULT_MAX_LANE_RADIUS_M {
        radius = DEFAULT_MAX_LANE_RADIUS_M;
    }
    radius
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

fn lane_radius_candidates(start: f64, max: f64, expansion_step: f64) -> Vec<f64> {
    let mut candidates = Vec::new();
    if !start.is_finite() || !max.is_finite() {
        return vec![start.max(1.0)];
    }
    let start = start.max(1.0);
    let max = max.max(start);
    let step = expansion_step.max(1.0);

    let mut radius = start;
    let mut guard = 0u32;
    while radius + f64::EPSILON < max && guard < 32 {
        candidates.push(radius);
        let next = (radius * 1.8).max(radius + step);
        if !next.is_finite() || next <= radius + f64::EPSILON {
            break;
        }
        radius = next.min(max);
        guard += 1;
    }
    if candidates.last().copied().unwrap_or(0.0) + f64::EPSILON < max {
        candidates.push(max);
    } else if candidates.is_empty() {
        candidates.push(max);
    }
    candidates
}

fn lane_spacing_candidates(base_spacing: f64) -> Vec<f64> {
    let base = base_spacing.max(1.0);
    let mut candidates = vec![base];
    let refined = (base / 2.0).max(MIN_LANE_SPACING_M);
    if refined + f64::EPSILON < base {
        candidates.push(refined);
    }
    candidates
}

fn grid_phase_candidates() -> [f64; 2] {
    [0.0, 0.5]
}

fn normalize_waypoints(waypoints: &[Waypoint], config: &Config) -> Vec<Waypoint> {
    waypoints
        .iter()
        .map(|wp| Waypoint {
            altitude_m: altitude_to_amsl(wp.altitude_m, config.altitude_reference, config.geoid_offset_m),
            ..wp.clone()
        })
        .collect()
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
    // SAFETY: cruise_path is non-empty (checked above on line 999)
    let last = &cruise_path[cruise_path.len() - 1];
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
