//! Backend route engine ported from the frontend A* planner.
//!
//! This module keeps routing logic server-side so ATC is the source of truth.

use crate::models::{Geofence, GeofenceType, Waypoint};
use crate::spatial::{
    bearing, haversine_distance, meters_per_deg_lat, meters_per_deg_lon, offset_by_bearing,
};
use serde::{Deserialize, Serialize};
use std::cmp::{Ordering, Reverse};
use std::collections::{BinaryHeap, HashMap, HashSet};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RouteEngineConfig {
    pub faa_limit_agl: f64,
    pub safety_buffer_m: f64,
    pub climb_speed_mps: f64,
    pub cruise_speed_mps: f64,
    pub descent_speed_mps: f64,
    pub wind_mps: f64,
    pub cost_time_weight: f64,
    pub cost_climb_penalty: f64,
    pub cost_lane_change: f64,
    pub cost_proximity_penalty: f64,
    pub geofence_sample_step_m: f64,
}

impl Default for RouteEngineConfig {
    fn default() -> Self {
        Self {
            faa_limit_agl: 121.0,
            safety_buffer_m: 20.0,
            climb_speed_mps: 2.0,
            cruise_speed_mps: 15.0,
            descent_speed_mps: 3.0,
            wind_mps: 0.0,
            cost_time_weight: 1.0,
            cost_climb_penalty: 15.0,
            cost_lane_change: 50.0,
            cost_proximity_penalty: 100.0,
            geofence_sample_step_m: 25.0,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RouteObstacle {
    pub lat: f64,
    pub lon: f64,
    pub radius_m: f64,
    pub height_m: Option<f64>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RouteGridPoint {
    pub lat: f64,
    pub lon: f64,
    pub altitude_m: f64,
    pub terrain_height_m: f64,
    pub obstacle_height_m: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RouteGrid {
    pub lanes: Vec<Vec<RouteGridPoint>>,
    pub waypoint_indices: Vec<usize>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RouteEngineWaypoint {
    pub lat: f64,
    pub lon: f64,
    pub altitude_m: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub phase: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RouteEngineStats {
    pub avg_agl: f64,
    pub max_agl: f64,
    pub max_altitude: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RouteEngineResult {
    pub success: bool,
    pub waypoints: Vec<RouteEngineWaypoint>,
    pub optimized_points: usize,
    pub nodes_visited: usize,
    pub stats: Option<RouteEngineStats>,
    pub errors: Vec<String>,
}

#[derive(Debug, Clone)]
struct Node {
    step: usize,
    lane: usize,
    g_score: f64,
    alt: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
struct NodeKey {
    step: usize,
    lane: usize,
}

#[derive(Debug, Clone, Copy)]
struct FloatOrd(f64);

impl PartialEq for FloatOrd {
    fn eq(&self, other: &Self) -> bool {
        self.0.to_bits() == other.0.to_bits()
    }
}

impl Eq for FloatOrd {}

impl PartialOrd for FloatOrd {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for FloatOrd {
    fn cmp(&self, other: &Self) -> Ordering {
        self.0.total_cmp(&other.0)
    }
}

#[derive(Debug, Clone, Copy)]
struct OpenNode {
    step: usize,
    lane: usize,
    g_score: FloatOrd,
    f_score: FloatOrd,
    alt: f64,
}

impl OpenNode {
    fn key(&self) -> NodeKey {
        NodeKey {
            step: self.step,
            lane: self.lane,
        }
    }
}

impl PartialEq for OpenNode {
    fn eq(&self, other: &Self) -> bool {
        self.step == other.step
            && self.lane == other.lane
            && self.g_score == other.g_score
            && self.f_score == other.f_score
            && self.alt.to_bits() == other.alt.to_bits()
    }
}

impl Eq for OpenNode {}

impl PartialOrd for OpenNode {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for OpenNode {
    fn cmp(&self, other: &Self) -> Ordering {
        self.f_score
            .cmp(&other.f_score)
            .then_with(|| self.g_score.cmp(&other.g_score))
            .then_with(|| self.step.cmp(&other.step))
            .then_with(|| self.lane.cmp(&other.lane))
            .then_with(|| self.alt.total_cmp(&other.alt))
    }
}

struct PathResult {
    smoothed_path: Vec<Node>,
    nodes_visited: usize,
    max_cruise_alt: f64,
}

impl Node {
    fn key(&self) -> NodeKey {
        NodeKey {
            step: self.step,
            lane: self.lane,
        }
    }
}

pub fn build_lane_offsets(radius_m: f64, spacing_m: f64) -> Vec<f64> {
    let spacing = spacing_m.max(1.0);
    let steps = (radius_m / spacing).floor() as i32;
    let mut offsets = Vec::new();
    for i in -steps..=steps {
        offsets.push(i as f64 * spacing);
    }
    if !offsets.iter().any(|value| value.abs() < f64::EPSILON) {
        offsets.push(0.0);
    }
    offsets.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    offsets
}

pub fn resolve_grid_spacing(waypoints: &[Waypoint], default_spacing: f64) -> f64 {
    if waypoints.len() < 2 {
        return default_spacing;
    }
    let mut distance_m = 0.0;
    for i in 1..waypoints.len() {
        distance_m += haversine_distance(
            waypoints[i - 1].lat,
            waypoints[i - 1].lon,
            waypoints[i].lat,
            waypoints[i].lon,
        );
    }
    if distance_m > 8000.0 {
        10.0
    } else if distance_m > 4000.0 {
        7.5
    } else if distance_m > 2000.0 {
        6.0
    } else {
        default_spacing
    }
}

pub fn generate_grid_samples(
    waypoints: &[Waypoint],
    spacing_m: f64,
    lane_offsets: &[f64],
    sample_phase: f64,
) -> Option<RouteGrid> {
    if waypoints.len() < 2 {
        return None;
    }
    let spacing = spacing_m.max(1.0);
    let mut lanes: Vec<Vec<RouteGridPoint>> = lane_offsets.iter().map(|_| Vec::new()).collect();
    let mut waypoint_indices = vec![0];

    let phase = sample_phase.clamp(0.0, 0.999);

    for i in 0..waypoints.len() - 1 {
        let start = &waypoints[i];
        let end = &waypoints[i + 1];
        let distance_m = haversine_distance(start.lat, start.lon, end.lat, end.lon);
        let heading = bearing(start.lat, start.lon, end.lat, end.lon);
        let num_steps = (distance_m / spacing).ceil().max(1.0) as usize;

        for step_idx in 0..=num_steps {
            if step_idx == num_steps && i < waypoints.len() - 2 {
                continue;
            }
            let fraction = if step_idx == 0 {
                0.0
            } else if step_idx == num_steps {
                1.0
            } else {
                (step_idx as f64 + phase) / num_steps as f64
            };
            let (center_lat, center_lon) =
                offset_by_bearing(start.lat, start.lon, distance_m * fraction, heading);
            let altitude_m = start.altitude_m + fraction * (end.altitude_m - start.altitude_m);

            for (lane_idx, offset) in lane_offsets.iter().enumerate() {
                let (lat, lon) = if offset.abs() < f64::EPSILON {
                    (center_lat, center_lon)
                } else {
                    let lateral_bearing = heading
                        + if *offset >= 0.0 {
                            std::f64::consts::FRAC_PI_2
                        } else {
                            -std::f64::consts::FRAC_PI_2
                        };
                    offset_by_bearing(center_lat, center_lon, offset.abs(), lateral_bearing)
                };
                lanes[lane_idx].push(RouteGridPoint {
                    lat,
                    lon,
                    altitude_m,
                    terrain_height_m: 0.0,
                    obstacle_height_m: 0.0,
                });
            }
        }

        let total_steps = lanes.first().map(|lane| lane.len()).unwrap_or(0);
        if i < waypoints.len() - 2 && total_steps > 0 {
            waypoint_indices.push(total_steps - 1);
        }
    }

    if let Some(last_lane) = lanes.first() {
        if !last_lane.is_empty() {
            waypoint_indices.push(last_lane.len() - 1);
        }
    }

    Some(RouteGrid {
        lanes,
        waypoint_indices,
    })
}

pub fn apply_obstacles<F>(grid: &mut RouteGrid, obstacles: &[RouteObstacle], terrain_height: F)
where
    F: Fn(f64, f64) -> f64,
{
    if obstacles.is_empty() {
        for lane in &mut grid.lanes {
            for point in lane {
                let terrain = terrain_height(point.lat, point.lon).max(0.0);
                point.terrain_height_m = terrain;
                point.obstacle_height_m = terrain;
            }
        }
        return;
    }

    #[derive(Clone, Copy)]
    struct IndexedObstacle {
        x_m: f64,
        y_m: f64,
        radius2_m: f64,
        height_m: f64,
    }

    const CELL_SIZE_M: f64 = 100.0;
    let inv_cell = 1.0 / CELL_SIZE_M;

    let center_lane_idx = grid.lanes.len() / 2;
    let mean_lat = grid
        .lanes
        .get(center_lane_idx)
        .and_then(|lane| {
            let first = lane.first()?;
            let last = lane.last()?;
            Some((first.lat + last.lat) / 2.0)
        })
        .unwrap_or(0.0);
    let meters_lat = meters_per_deg_lat(mean_lat);
    let meters_lon = meters_per_deg_lon(mean_lat).max(1.0);

    let mut obstacle_index: HashMap<(i32, i32), Vec<IndexedObstacle>> = HashMap::new();
    for obstacle in obstacles {
        let radius = obstacle.radius_m.max(0.0);
        if radius <= 0.0 {
            continue;
        }
        let height_m = obstacle.height_m.unwrap_or(0.0).max(0.0);
        let x_m = obstacle.lon * meters_lon;
        let y_m = obstacle.lat * meters_lat;
        let cell_x = (x_m * inv_cell).floor() as i32;
        let cell_y = (y_m * inv_cell).floor() as i32;
        let mut cell_radius = (radius * inv_cell).ceil() as i32;
        cell_radius = cell_radius.clamp(0, 16);
        let entry = IndexedObstacle {
            x_m,
            y_m,
            radius2_m: radius * radius,
            height_m,
        };
        for dx in -cell_radius..=cell_radius {
            for dy in -cell_radius..=cell_radius {
                obstacle_index
                    .entry((cell_x + dx, cell_y + dy))
                    .or_default()
                    .push(entry);
            }
        }
    }

    for lane in &mut grid.lanes {
        for point in lane {
            let terrain = terrain_height(point.lat, point.lon).max(0.0);
            point.terrain_height_m = terrain;
            point.obstacle_height_m = terrain;

            let x_m = point.lon * meters_lon;
            let y_m = point.lat * meters_lat;
            let cell_x = (x_m * inv_cell).floor() as i32;
            let cell_y = (y_m * inv_cell).floor() as i32;
            if let Some(obstacles) = obstacle_index.get(&(cell_x, cell_y)) {
                for obstacle in obstacles {
                    let dx = obstacle.x_m - x_m;
                    let dy = obstacle.y_m - y_m;
                    if dx * dx + dy * dy > obstacle.radius2_m {
                        continue;
                    }
                    let obstacle_alt = terrain + obstacle.height_m;
                    if obstacle_alt > point.obstacle_height_m {
                        point.obstacle_height_m = obstacle_alt;
                    }
                }
            }
        }
    }
}

pub fn optimize_flight_path(
    waypoints: &[Waypoint],
    grid: &RouteGrid,
    geofences: &[Geofence],
    config: &RouteEngineConfig,
) -> RouteEngineResult {
    let result = match compute_path_nodes(waypoints, grid, geofences, config, None) {
        Ok(result) => result,
        Err(errors) => {
            return RouteEngineResult {
                success: false,
                waypoints: Vec::new(),
                optimized_points: 0,
                nodes_visited: 0,
                stats: None,
                errors,
            };
        }
    };

    let num_steps = grid.lanes[0].len();
    let center_lane_idx = grid.lanes.len() / 2;
    let waypoint_indices = if grid.waypoint_indices.is_empty() {
        vec![0, num_steps - 1]
    } else {
        grid.waypoint_indices.clone()
    };

    let mut final_waypoints = Vec::new();
    for (idx, step_idx) in waypoint_indices.iter().enumerate() {
        let point = &grid.lanes[center_lane_idx][*step_idx];
        let is_first = idx == 0;
        let is_last = idx + 1 == waypoint_indices.len();

        final_waypoints.push(RouteEngineWaypoint {
            lat: point.lat,
            lon: point.lon,
            altitude_m: point.terrain_height_m,
            phase: Some(
                if is_first {
                    "GROUND_START"
                } else if is_last {
                    "GROUND_END"
                } else {
                    "GROUND_WAYPOINT"
                }
                .to_string(),
            ),
        });

        if !is_last {
            let next_step_idx = waypoint_indices[idx + 1];
            let mut segment_cruise_alt: f64 = 0.0;
            for node in &result.smoothed_path {
                if node.step > *step_idx && node.step < next_step_idx {
                    segment_cruise_alt = segment_cruise_alt.max(node.alt);
                }
            }

            let mut segment_planned_alt: f64 = 0.0;
            let mut max_obstacle: f64 = 0.0;
            for step in *step_idx..=next_step_idx {
                let pt = &grid.lanes[center_lane_idx][step];
                segment_planned_alt = segment_planned_alt.max(pt.altitude_m);
                max_obstacle = max_obstacle
                    .max(pt.obstacle_height_m)
                    .max(pt.terrain_height_m);
            }

            let min_safe_alt = max_obstacle + config.safety_buffer_m;
            segment_cruise_alt = segment_cruise_alt
                .max(min_safe_alt)
                .max(segment_planned_alt);

            final_waypoints.push(RouteEngineWaypoint {
                lat: point.lat,
                lon: point.lon,
                altitude_m: segment_cruise_alt,
                phase: Some("VERTICAL_ASCENT".to_string()),
            });

            let mut last_output_lane = center_lane_idx;
            let mut last_output_node: Option<Node> = None;
            let mut last_node_before_lane_change: Option<Node> = None;
            const MAX_SEGMENT_DISTANCE_M: f64 = 15.0;

            for node in &result.smoothed_path {
                if node.step <= *step_idx || node.step >= next_step_idx {
                    continue;
                }
                let node_point = &grid.lanes[node.lane][node.step];

                if node.lane != last_output_lane {
                    if let Some(prev) = &last_node_before_lane_change {
                        let prev_point = &grid.lanes[prev.lane][prev.step];
                        final_waypoints.push(RouteEngineWaypoint {
                            lat: prev_point.lat,
                            lon: prev_point.lon,
                            altitude_m: segment_cruise_alt,
                            phase: Some("CRUISE_CORNER".to_string()),
                        });
                    }
                    final_waypoints.push(RouteEngineWaypoint {
                        lat: node_point.lat,
                        lon: node_point.lon,
                        altitude_m: segment_cruise_alt,
                        phase: Some("CRUISE".to_string()),
                    });
                    last_output_lane = node.lane;
                    last_output_node = Some(node.clone());
                } else if let Some(last_node) = &last_output_node {
                    let last_point = &grid.lanes[last_node.lane][last_node.step];
                    let dist = haversine_distance(
                        last_point.lat,
                        last_point.lon,
                        node_point.lat,
                        node_point.lon,
                    );
                    if dist > MAX_SEGMENT_DISTANCE_M {
                        final_waypoints.push(RouteEngineWaypoint {
                            lat: node_point.lat,
                            lon: node_point.lon,
                            altitude_m: segment_cruise_alt,
                            phase: Some("CRUISE_INTERMEDIATE".to_string()),
                        });
                        last_output_node = Some(node.clone());
                    }
                }

                last_node_before_lane_change = Some(node.clone());
            }

            let next_point = &grid.lanes[center_lane_idx][next_step_idx];
            final_waypoints.push(RouteEngineWaypoint {
                lat: next_point.lat,
                lon: next_point.lon,
                altitude_m: segment_cruise_alt,
                phase: Some("VERTICAL_DESCENT".to_string()),
            });
        }
    }

    let ground = grid.lanes[center_lane_idx][0].terrain_height_m;
    let stats = RouteEngineStats {
        avg_agl: result.max_cruise_alt - ground,
        max_agl: result.max_cruise_alt - ground,
        max_altitude: result.max_cruise_alt,
    };

    RouteEngineResult {
        success: true,
        waypoints: final_waypoints.clone(),
        optimized_points: final_waypoints.len(),
        nodes_visited: result.nodes_visited,
        stats: Some(stats),
        errors: Vec::new(),
    }
}

pub fn optimize_airborne_path(
    waypoints: &[Waypoint],
    grid: &RouteGrid,
    geofences: &[Geofence],
    config: &RouteEngineConfig,
) -> RouteEngineResult {
    // Treat input waypoint altitude as a "desired" altitude, but don't allow starting below terrain.
    // `compute_path_nodes` will clamp start altitude to at least the terrain height when override is None.
    let result = match compute_path_nodes(waypoints, grid, geofences, config, None) {
        Ok(result) => result,
        Err(errors) => {
            return RouteEngineResult {
                success: false,
                waypoints: Vec::new(),
                optimized_points: 0,
                nodes_visited: 0,
                stats: None,
                errors,
            };
        }
    };

    let mut waypoints_out = Vec::new();
    for node in &result.smoothed_path {
        let point = &grid.lanes[node.lane][node.step];
        waypoints_out.push(RouteEngineWaypoint {
            lat: point.lat,
            lon: point.lon,
            altitude_m: node.alt,
            phase: Some("CRUISE".to_string()),
        });
    }

    let stats = RouteEngineStats {
        avg_agl: result.max_cruise_alt,
        max_agl: result.max_cruise_alt,
        max_altitude: result.max_cruise_alt,
    };

    RouteEngineResult {
        success: true,
        waypoints: waypoints_out.clone(),
        optimized_points: waypoints_out.len(),
        nodes_visited: result.nodes_visited,
        stats: Some(stats),
        errors: Vec::new(),
    }
}

fn compute_path_nodes(
    waypoints: &[Waypoint],
    grid: &RouteGrid,
    geofences: &[Geofence],
    config: &RouteEngineConfig,
    start_altitude_override: Option<f64>,
) -> Result<PathResult, Vec<String>> {
    if waypoints.len() < 2 {
        return Err(vec!["need at least 2 waypoints".to_string()]);
    }
    if grid.lanes.is_empty() || grid.lanes[0].is_empty() {
        return Err(vec!["grid is empty".to_string()]);
    }

    let active_geofences: Vec<&Geofence> = geofences
        .iter()
        .filter(|fence| fence.active && fence.geofence_type != GeofenceType::Advisory)
        .collect();

    let num_lanes = grid.lanes.len();
    let num_steps = grid.lanes[0].len();
    let center_lane_idx = num_lanes / 2;

    let start_point = &grid.lanes[center_lane_idx][0];
    let start_alt =
        start_altitude_override.unwrap_or(start_point.altitude_m.max(start_point.terrain_height_m));
    let start_node = Node {
        step: 0,
        lane: center_lane_idx,
        g_score: 0.0,
        alt: start_alt,
    };

    let effective_speed = (config.cruise_speed_mps - config.wind_mps).max(1.0);
    let end_point = &grid.lanes[center_lane_idx][num_steps - 1];
    let start_h = haversine_distance(
        start_point.lat,
        start_point.lon,
        end_point.lat,
        end_point.lon,
    ) / effective_speed;

    let mut open_set: BinaryHeap<Reverse<OpenNode>> = BinaryHeap::new();
    open_set.push(Reverse(OpenNode {
        step: start_node.step,
        lane: start_node.lane,
        g_score: FloatOrd(start_node.g_score),
        f_score: FloatOrd(start_node.g_score + start_h),
        alt: start_node.alt,
    }));
    let mut closed_set: HashSet<NodeKey> = HashSet::new();
    let mut g_score: HashMap<NodeKey, f64> = HashMap::new();
    let mut came_from: HashMap<NodeKey, Node> = HashMap::new();
    g_score.insert(start_node.key(), 0.0);

    let mut final_node: Option<Node> = None;
    let mut nodes_visited = 0usize;

    while let Some(Reverse(current)) = open_set.pop() {
        let current_key = current.key();
        if closed_set.contains(&current_key) {
            continue;
        }
        let best_g = g_score.get(&current_key).copied().unwrap_or(f64::INFINITY);
        if current.g_score.0 > best_g + 1e-9 {
            continue;
        }

        nodes_visited += 1;

        if current.step == num_steps - 1 && current.lane == center_lane_idx {
            final_node = Some(Node {
                step: current.step,
                lane: current.lane,
                g_score: best_g,
                alt: current.alt,
            });
            break;
        }

        closed_set.insert(current_key);
        let next_step = current.step + 1;
        if next_step >= num_steps {
            continue;
        }

        let curr_point = &grid.lanes[current.lane][current.step];
        let candidate_lanes = [current.lane.wrapping_sub(1), current.lane, current.lane + 1];

        for next_lane in candidate_lanes.iter().copied() {
            if next_lane >= num_lanes {
                continue;
            }
            let next_key = NodeKey {
                step: next_step,
                lane: next_lane,
            };
            if closed_set.contains(&next_key) {
                continue;
            }

            let next_point = &grid.lanes[next_lane][next_step];
            let feature_height = next_point
                .obstacle_height_m
                .max(next_point.terrain_height_m);
            let target_alt = (feature_height + config.safety_buffer_m).max(next_point.altitude_m);
            let current_alt = current.alt;

            // FAA 400ft AGL constraint: ensure the altitude at each grid point is within the local ceiling.
            //
            // Note: we allow descending between points even if current_alt is above next ceiling, because the
            // model assumes a smooth transition between points and `target_alt` will be within the next ceiling.
            let faa_ceiling_curr = curr_point.terrain_height_m + config.faa_limit_agl;
            let faa_ceiling_next = next_point.terrain_height_m + config.faa_limit_agl;
            if current_alt > faa_ceiling_curr || target_alt > faa_ceiling_next {
                continue;
            }

            let dist = haversine_distance(
                curr_point.lat,
                curr_point.lon,
                next_point.lat,
                next_point.lon,
            );
            let time_to_travel = dist / effective_speed;

            let mut alt_cost = 0.0;
            if current_alt < target_alt {
                let alt_change = target_alt - current_alt;
                alt_cost = alt_change * config.cost_climb_penalty;
            }

            let cruise_alt = current_alt.max(target_alt);
            if !active_geofences.is_empty()
                && geofence_blocks_segment(
                    &active_geofences,
                    curr_point,
                    next_point,
                    current_alt,
                    target_alt,
                    config.geofence_sample_step_m,
                )
            {
                continue;
            }

            let lane_change_cost =
                (next_lane as i32 - current.lane as i32).abs() as f64 * config.cost_lane_change;

            let mut proximity_cost = 0.0;
            if next_lane > 0 {
                let left = &grid.lanes[next_lane - 1][next_step];
                let left_min_safe =
                    left.obstacle_height_m.max(left.terrain_height_m) + config.safety_buffer_m;
                if left_min_safe > cruise_alt {
                    proximity_cost += config.cost_proximity_penalty;
                }
            }
            if next_lane + 1 < num_lanes {
                let right = &grid.lanes[next_lane + 1][next_step];
                let right_min_safe =
                    right.obstacle_height_m.max(right.terrain_height_m) + config.safety_buffer_m;
                if right_min_safe > cruise_alt {
                    proximity_cost += config.cost_proximity_penalty;
                }
            }

            let step_cost = time_to_travel + alt_cost + lane_change_cost + proximity_cost;
            let tentative_g = best_g + step_cost;
            if tentative_g < g_score.get(&next_key).copied().unwrap_or(f64::INFINITY) {
                came_from.insert(
                    next_key,
                    Node {
                        step: current.step,
                        lane: current.lane,
                        g_score: best_g,
                        alt: current.alt,
                    },
                );
                g_score.insert(next_key, tentative_g);

                let dist_to_end = haversine_distance(
                    next_point.lat,
                    next_point.lon,
                    end_point.lat,
                    end_point.lon,
                );
                let h_score = dist_to_end / effective_speed;

                // Allow altitude to decrease when it is safe to do so (e.g. when terrain drops).
                // This keeps paths feasible under an AGL ceiling without requiring "segment reset" tricks.
                let new_alt = target_alt;
                open_set.push(Reverse(OpenNode {
                    step: next_step,
                    lane: next_lane,
                    g_score: FloatOrd(tentative_g),
                    f_score: FloatOrd(tentative_g + h_score),
                    alt: new_alt,
                }));
            }
        }
    }

    let Some(final_node) = final_node else {
        return Err(vec!["A* failed to find a path".to_string()]);
    };

    let mut path_nodes = Vec::new();
    let mut current = Some(final_node.clone());
    while let Some(node) = current {
        path_nodes.push(node.clone());
        current = came_from.get(&node.key()).cloned();
    }
    path_nodes.reverse();

    let smoothed_path = smooth_path(&path_nodes, grid, &active_geofences, config);
    let mut max_cruise_alt: f64 = 0.0;
    for node in &path_nodes {
        max_cruise_alt = max_cruise_alt.max(node.alt);
    }

    Ok(PathResult {
        smoothed_path,
        nodes_visited,
        max_cruise_alt,
    })
}

fn smooth_path(
    path_nodes: &[Node],
    grid: &RouteGrid,
    geofences: &[&Geofence],
    config: &RouteEngineConfig,
) -> Vec<Node> {
    if path_nodes.len() <= 2 {
        return path_nodes.to_vec();
    }

    let mut smoothed = vec![path_nodes[0].clone()];
    let mut current_idx = 0usize;

    while current_idx < path_nodes.len() - 1 {
        let current = &path_nodes[current_idx];
        let mut furthest_valid = current_idx + 1;

        for target_idx in (current_idx + 2)..path_nodes.len() {
            let target = &path_nodes[target_idx];
            if is_line_of_sight_clear(
                current,
                target,
                path_nodes,
                current_idx,
                target_idx,
                grid,
                geofences,
                config,
            ) {
                furthest_valid = target_idx;
            }
        }

        smoothed.push(path_nodes[furthest_valid].clone());
        current_idx = furthest_valid;
    }

    smoothed
}

#[allow(clippy::too_many_arguments)]
fn is_line_of_sight_clear(
    start: &Node,
    end: &Node,
    all_nodes: &[Node],
    start_idx: usize,
    end_idx: usize,
    grid: &RouteGrid,
    geofences: &[&Geofence],
    config: &RouteEngineConfig,
) -> bool {
    let mut max_alt = start.alt.max(end.alt);
    for node in &all_nodes[start_idx..=end_idx] {
        max_alt = max_alt.max(node.alt);
    }

    let num_lanes = grid.lanes.len();
    let num_steps = grid.lanes[0].len();
    let num_samples = (end_idx as i32 - start_idx as i32).max(1) as usize * 2;
    let num_samples = num_samples.max(5);

    let step_delta = end.step as i32 - start.step as i32;
    let lane_delta = end.lane as i32 - start.lane as i32;

    for i in 1..num_samples {
        let t = i as f64 / num_samples as f64;
        let mid_step = ((start.step as f64) + t * (step_delta as f64)).round() as i32;
        let mid_lane = ((start.lane as f64) + t * (lane_delta as f64)).round() as i32;
        if mid_lane < 0 || mid_lane as usize >= num_lanes {
            return false;
        }
        if mid_step < 0 || mid_step as usize >= num_steps {
            return false;
        }
        let grid_point = &grid.lanes[mid_lane as usize][mid_step as usize];
        let obstacle_height = grid_point
            .obstacle_height_m
            .max(grid_point.terrain_height_m);
        let min_safe_alt = obstacle_height + config.safety_buffer_m;
        if !geofences.is_empty() {
            let sample_alt = start.alt + t * (end.alt - start.alt);
            if geofence_blocks_point(geofences, grid_point.lat, grid_point.lon, sample_alt) {
                return false;
            }
        }
        if min_safe_alt > max_alt {
            return false;
        }
        if mid_lane > 0 {
            let left = &grid.lanes[mid_lane as usize - 1][mid_step as usize];
            let left_height = left.obstacle_height_m.max(left.terrain_height_m);
            if left_height + config.safety_buffer_m > max_alt {
                return false;
            }
        }
        if (mid_lane as usize) + 1 < num_lanes {
            let right = &grid.lanes[mid_lane as usize + 1][mid_step as usize];
            let right_height = right.obstacle_height_m.max(right.terrain_height_m);
            if right_height + config.safety_buffer_m > max_alt {
                return false;
            }
        }
    }

    true
}

fn geofence_blocks_point(geofences: &[&Geofence], lat: f64, lon: f64, altitude_m: f64) -> bool {
    for geofence in geofences {
        if geofence.contains_point(lat, lon, altitude_m) {
            return true;
        }
    }
    false
}

fn geofence_blocks_segment(
    geofences: &[&Geofence],
    start: &RouteGridPoint,
    end: &RouteGridPoint,
    start_alt: f64,
    end_alt: f64,
    step_m: f64,
) -> bool {
    if geofences.is_empty() {
        return false;
    }
    let distance_m = haversine_distance(start.lat, start.lon, end.lat, end.lon);
    let step = step_m.max(1.0);
    let steps = ((distance_m / step).ceil() as usize).clamp(1, 1000);
    for i in 0..=steps {
        let t = i as f64 / steps as f64;
        let lat = start.lat + t * (end.lat - start.lat);
        let lon = start.lon + t * (end.lon - start.lon);
        let alt = start_alt + t * (end_alt - start_alt);
        if geofence_blocks_point(geofences, lat, lon, alt) {
            return true;
        }
    }
    false
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn airborne_path_allows_descent_under_agl_ceiling() {
        // Synthetic 2-point path where terrain drops sharply. The planner should be allowed
        // to descend so long as each point's altitude is within its local AGL ceiling.
        let waypoints = vec![
            Waypoint {
                lat: 33.0,
                lon: -117.0,
                altitude_m: 0.0,
                speed_mps: None,
            },
            Waypoint {
                lat: 33.0,
                lon: -117.0001,
                altitude_m: 0.0,
                speed_mps: None,
            },
        ];

        let grid = RouteGrid {
            lanes: vec![vec![
                RouteGridPoint {
                    lat: 33.0,
                    lon: -117.0,
                    altitude_m: 0.0,
                    terrain_height_m: 200.0,
                    obstacle_height_m: 200.0,
                },
                RouteGridPoint {
                    lat: 33.0,
                    lon: -117.0001,
                    altitude_m: 0.0,
                    terrain_height_m: 0.0,
                    obstacle_height_m: 0.0,
                },
            ]],
            waypoint_indices: vec![0, 1],
        };

        let result = optimize_airborne_path(&waypoints, &grid, &[], &RouteEngineConfig::default());
        assert!(
            result.success,
            "expected path to be feasible: {:?}",
            result.errors
        );
        assert_eq!(result.waypoints.len(), 2);
        assert!(result.waypoints[0].altitude_m > result.waypoints[1].altitude_m);
    }
}
