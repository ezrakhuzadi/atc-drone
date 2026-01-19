use atc_core::models::Waypoint;
use atc_core::rules::SafetyRules;
use atc_core::spatial::haversine_distance;
use atc_server::config::Config;
use atc_server::route_planner::{plan_route, RoutePlanRequest};
use atc_server::state::AppState;

#[derive(Clone)]
struct StressRoute {
    name: &'static str,
    waypoints: Vec<Waypoint>,
    max_lane_radius_m: Option<f64>,
}

#[tokio::main]
async fn main() {
    let config = Config::from_env();
    let state = AppState::with_rules_and_config(SafetyRules::default(), config.clone());

    let routes = vec![
        StressRoute {
            name: "US Bank Tower -> Wilshire Grand",
            waypoints: vec![
                waypoint(34.0522, -118.2551, 60.0),
                waypoint(34.0506, -118.2569, 60.0),
            ],
            max_lane_radius_m: Some(600.0),
        },
        StressRoute {
            name: "Irvine -> Anaheim",
            waypoints: vec![
                waypoint(33.6846, -117.8255, 60.0),
                waypoint(33.8353, -117.9145, 60.0),
            ],
            max_lane_radius_m: Some(1000.0),
        },
        StressRoute {
            name: "Irvine -> Santa Ana",
            waypoints: vec![
                waypoint(33.6846, -117.8255, 60.0),
                waypoint(33.7456, -117.8678, 60.0),
            ],
            max_lane_radius_m: Some(700.0),
        },
        StressRoute {
            name: "Anaheim -> Long Beach",
            waypoints: vec![
                waypoint(33.8353, -117.9145, 60.0),
                waypoint(33.7701, -118.1937, 60.0),
            ],
            max_lane_radius_m: Some(1200.0),
        },
    ];

    for route in routes {
        println!("\n=== {} ===", route.name);
        let request = RoutePlanRequest {
            waypoints: route.waypoints.clone(),
            lane_radius_m: None,
            lane_spacing_m: None,
            sample_spacing_m: None,
            safety_buffer_m: None,
            max_lane_radius_m: route.max_lane_radius_m,
            lane_expansion_step_m: None,
        };

        let result = plan_route(&state, &config, request).await;
        if !result.ok {
            println!("Result: FAIL");
            println!("Errors: {:?}", result.errors);
            continue;
        }

        let stats = result.stats.as_ref();
        println!(
            "Result: OK | nodes={} optimized={} samples={}",
            result.nodes_visited, result.optimized_points, result.sample_points
        );
        if let Some(stats) = stats {
            println!(
                "Stats: avg_agl={:.1}m max_agl={:.1}m max_altitude={:.1}m",
                stats.avg_agl, stats.max_agl, stats.max_altitude
            );
        }

        let violations = find_hazard_violations(
            &result.waypoints,
            &result.hazards,
            config.compliance_default_clearance_m,
        );
        if violations.is_empty() {
            println!("Hazard check: PASS");
        } else {
            println!("Hazard check: FAIL ({})", violations.len());
            for violation in violations {
                println!(" - {}", violation);
            }
        }
    }
}

fn waypoint(lat: f64, lon: f64, altitude_m: f64) -> Waypoint {
    Waypoint {
        lat,
        lon,
        altitude_m,
        speed_mps: Some(15.0),
    }
}

fn find_hazard_violations(
    waypoints: &[atc_core::route_engine::RouteEngineWaypoint],
    hazards: &[atc_server::compliance::ObstacleHazard],
    clearance_m: f64,
) -> Vec<String> {
    if waypoints.len() < 2 || hazards.is_empty() {
        return Vec::new();
    }

    let mut violations = Vec::new();
    for hazard in hazards {
        let radius = hazard.radius_m;
        let height = hazard.height_m.unwrap_or(0.0) + clearance_m;
        if !radius.is_finite() || !height.is_finite() {
            continue;
        }
        let mut hit = false;
        for segment in waypoints.windows(2) {
            if segment_hits_hazard(segment[0].clone(), segment[1].clone(), hazard, radius, height) {
                hit = true;
                break;
            }
        }
        if hit {
            violations.push(format!(
                "{} within {:.0}m @ <= {:.0}m",
                hazard.name, radius, height
            ));
        }
    }
    violations
}

fn segment_hits_hazard(
    start: atc_core::route_engine::RouteEngineWaypoint,
    end: atc_core::route_engine::RouteEngineWaypoint,
    hazard: &atc_server::compliance::ObstacleHazard,
    radius: f64,
    max_alt: f64,
) -> bool {
    let samples = 25usize;
    for i in 0..=samples {
        let t = i as f64 / samples as f64;
        let lat = start.lat + t * (end.lat - start.lat);
        let lon = start.lon + t * (end.lon - start.lon);
        let alt = start.altitude_m + t * (end.altitude_m - start.altitude_m);
        if alt > max_alt {
            continue;
        }
        let distance = haversine_distance(lat, lon, hazard.lat, hazard.lon);
        if distance <= radius {
            return true;
        }
    }
    false
}
