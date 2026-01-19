//! Server-side compliance evaluation for flight plans.

use crate::config::Config;
use atc_core::models::FlightPlanRequest;
use chrono::Utc;
use reqwest::Client;
use serde::{Deserialize, Serialize};
use std::collections::{HashMap, HashSet};
use std::time::Duration;

#[derive(Debug, Clone, Copy)]
pub struct RoutePoint {
    pub lat: f64,
    pub lon: f64,
    pub altitude_m: f64,
}

#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum ComplianceStatus {
    Pass,
    Warn,
    Fail,
    Pending,
}

#[derive(Debug, Clone, Serialize)]
pub struct ComplianceReport {
    pub generated_at: String,
    pub overall_status: ComplianceStatus,
    pub route: RouteMetrics,
    pub checks: ComplianceChecks,
}

#[derive(Debug, Clone, Serialize)]
pub struct RouteMetrics {
    pub distance_m: f64,
    pub estimated_minutes: f64,
    pub has_route: bool,
}

#[derive(Debug, Clone, Serialize)]
pub struct ComplianceChecks {
    pub weather: WeatherCheck,
    pub battery: BatteryCheck,
    pub population: PopulationCheck,
    pub obstacles: ObstaclesCheck,
}

#[derive(Debug, Clone, Serialize)]
pub struct WeatherCheck {
    pub status: ComplianceStatus,
    pub message: String,
    pub wind_mps: Option<f64>,
    pub gust_mps: Option<f64>,
    pub precip_mm: Option<f64>,
    pub max_wind_mps: f64,
    pub max_gust_mps: f64,
    pub max_precip_mm: f64,
    pub source: String,
}

#[derive(Debug, Clone, Serialize)]
pub struct BatteryCheck {
    pub status: ComplianceStatus,
    pub message: String,
    pub estimated_minutes: Option<f64>,
    pub capacity_min: Option<f64>,
    pub reserve_min: Option<f64>,
    pub remaining_min: Option<f64>,
    pub cruise_speed_mps: Option<f64>,
}

#[derive(Debug, Clone, Serialize)]
pub struct PopulationCheck {
    pub status: ComplianceStatus,
    pub message: String,
    pub density: Option<f64>,
    pub classification: Option<String>,
    pub building_count: Option<usize>,
    pub estimated_population: Option<f64>,
    pub area_km2: Option<f64>,
    pub source: Option<String>,
}

#[derive(Debug, Clone, Serialize)]
pub struct ObstaclesCheck {
    pub status: ComplianceStatus,
    pub message: String,
    pub clearance_m: f64,
    pub conflicts: Vec<ObstacleConflict>,
    pub hazards: Vec<ObstacleHazard>,
    pub obstacle_count: usize,
    pub truncated: bool,
}

#[derive(Debug, Clone, Serialize)]
pub struct ObstacleHazard {
    pub id: String,
    pub name: String,
    pub lat: f64,
    pub lon: f64,
    pub radius_m: f64,
    pub height_m: Option<f64>,
    pub hazard_type: String,
    pub source: String,
    pub distance_m: Option<f64>,
}

#[derive(Debug, Clone, Serialize)]
pub struct ObstacleConflict {
    pub id: String,
    pub name: String,
    pub distance_m: f64,
    pub severity: String,
}

#[derive(Debug, Clone)]
pub struct ComplianceEvaluation {
    pub report: ComplianceReport,
    pub blocking: Vec<String>,
    pub ok: bool,
}

#[derive(Debug)]
struct Bounds {
    min_lat: f64,
    max_lat: f64,
    min_lon: f64,
    max_lon: f64,
}

#[derive(Debug)]
struct ObstacleAnalysis {
    hazards: Vec<ObstacleHazard>,
    obstacle_count: usize,
    truncated: bool,
    building_count: usize,
    estimated_population: f64,
    density: f64,
    area_km2: f64,
}

#[derive(Debug, Deserialize)]
struct OverpassResponse {
    elements: Vec<OverpassElement>,
}

#[derive(Debug, Deserialize)]
struct OverpassElement {
    id: i64,
    lat: Option<f64>,
    lon: Option<f64>,
    center: Option<OverpassCenter>,
    tags: Option<HashMap<String, String>>,
}

#[derive(Debug, Deserialize)]
struct OverpassCenter {
    lat: f64,
    lon: f64,
}

#[derive(Debug, Deserialize)]
struct WeatherResponse {
    current: Option<WeatherCurrent>,
    current_weather: Option<WeatherCurrentAlt>,
}

#[derive(Debug, Deserialize)]
struct WeatherCurrent {
    wind_speed_10m: Option<f64>,
    wind_gusts_10m: Option<f64>,
    precipitation: Option<f64>,
}

#[derive(Debug, Deserialize)]
struct WeatherCurrentAlt {
    windspeed: Option<f64>,
    windgusts: Option<f64>,
    precipitation: Option<f64>,
}

pub async fn evaluate_compliance(
    config: &Config,
    request: &FlightPlanRequest,
    points: &[RoutePoint],
) -> ComplianceEvaluation {
    let metadata = request.metadata.as_ref();
    let cruise_speed_mps = metadata.and_then(|m| m.drone_speed_mps);
    let battery_capacity_min = metadata.and_then(|m| m.battery_capacity_min);
    let battery_reserve_min = metadata.and_then(|m| m.battery_reserve_min);
    let clearance_m = metadata
        .and_then(|m| m.clearance_m)
        .unwrap_or(config.compliance_default_clearance_m);
    let operation_type = metadata.and_then(|m| m.operation_type).unwrap_or(1);
    let override_enabled = metadata
        .and_then(|m| m.compliance_override_enabled)
        .unwrap_or(false);
    let override_notes = metadata
        .and_then(|m| m.compliance_override_notes.clone())
        .unwrap_or_default();

    let route = build_route_metrics(points, cruise_speed_mps);
    let client = Client::builder()
        .timeout(Duration::from_secs(15))
        .build()
        .unwrap_or_else(|_| Client::new());

    let (weather_result, obstacle_result) = tokio::join!(
        fetch_weather(&client, config, points),
        fetch_obstacles(&client, config, points, clearance_m)
    );

    let weather_check = match weather_result {
        Ok(weather) => evaluate_weather(config, &weather),
        Err(err) => WeatherCheck {
            status: ComplianceStatus::Pending,
            message: format!("Weather fetch failed: {}", err),
            wind_mps: None,
            gust_mps: None,
            precip_mm: None,
            max_wind_mps: config.compliance_max_wind_mps,
            max_gust_mps: config.compliance_max_gust_mps,
            max_precip_mm: config.compliance_max_precip_mm,
            source: "Open-Meteo".to_string(),
        },
    };

    let (population_check, obstacles_check) = match obstacle_result {
        Ok(analysis) => {
            let population = evaluate_population(config, operation_type, &analysis);
            let obstacles = evaluate_obstacles(points, clearance_m, analysis);
            (population, obstacles)
        }
        Err(err) => (
            PopulationCheck {
                status: ComplianceStatus::Pending,
                message: format!("Population analysis failed: {}", err),
                density: None,
                classification: None,
                building_count: None,
                estimated_population: None,
                area_km2: None,
                source: None,
            },
            ObstaclesCheck {
                status: ComplianceStatus::Pending,
                message: format!("Obstacle analysis failed: {}", err),
                clearance_m,
                conflicts: Vec::new(),
                hazards: default_hazards(),
                obstacle_count: 0,
                truncated: false,
            },
        ),
    };

    let battery_check = evaluate_battery(config, &route, cruise_speed_mps, battery_capacity_min, battery_reserve_min);

    let checks = ComplianceChecks {
        weather: weather_check.clone(),
        battery: battery_check.clone(),
        population: population_check.clone(),
        obstacles: obstacles_check.clone(),
    };

    let overall_status = summarize_status(&checks);
    let mut blocking: Vec<String> = Vec::new();
    for (key, status) in [
        ("weather", &checks.weather.status),
        ("battery", &checks.battery.status),
        ("population", &checks.population.status),
        ("obstacles", &checks.obstacles.status),
    ] {
        if matches!(status, ComplianceStatus::Fail | ComplianceStatus::Pending) {
            blocking.push(key.to_string());
        }
    }

    let mut ok = blocking.is_empty();
    if override_enabled {
        if override_notes.trim().len() < 8 {
            ok = false;
            blocking.push("override".to_string());
        } else {
            ok = true;
        }
    }

    ComplianceEvaluation {
        report: ComplianceReport {
            generated_at: Utc::now().to_rfc3339(),
            overall_status,
            route,
            checks,
        },
        blocking,
        ok,
    }
}

fn build_route_metrics(points: &[RoutePoint], cruise_speed_mps: Option<f64>) -> RouteMetrics {
    if points.len() < 2 {
        return RouteMetrics {
            distance_m: 0.0,
            estimated_minutes: 0.0,
            has_route: false,
        };
    }
    let mut distance_m = 0.0;
    for idx in 1..points.len() {
        distance_m += haversine_distance_m(points[idx - 1], points[idx]);
    }
    let speed = cruise_speed_mps.unwrap_or(0.0);
    let estimated_minutes = if speed > 0.0 {
        distance_m / speed / 60.0
    } else {
        0.0
    };
    RouteMetrics {
        distance_m,
        estimated_minutes,
        has_route: true,
    }
}

fn evaluate_weather(config: &Config, weather: &WeatherCurrent) -> WeatherCheck {
    let wind = weather.wind_speed_10m;
    let gust = weather.wind_gusts_10m;
    let precip = weather.precipitation;
    let max_wind = config.compliance_max_wind_mps;
    let max_gust = config.compliance_max_gust_mps;
    let max_precip = config.compliance_max_precip_mm;

    if wind.is_none() || gust.is_none() || precip.is_none() {
        return WeatherCheck {
            status: ComplianceStatus::Pending,
            message: "Weather values missing".to_string(),
            wind_mps: wind,
            gust_mps: gust,
            precip_mm: precip,
            max_wind_mps: max_wind,
            max_gust_mps: max_gust,
            max_precip_mm: max_precip,
            source: "Open-Meteo".to_string(),
        };
    }

    let wind_value = wind.unwrap_or(0.0);
    let gust_value = gust.unwrap_or(0.0);
    let precip_value = precip.unwrap_or(0.0);

    let mut status = ComplianceStatus::Pass;
    if wind_value > max_wind || gust_value > max_gust || precip_value > max_precip {
        status = ComplianceStatus::Fail;
    } else if wind_value > max_wind * config.compliance_wind_warn_ratio
        || gust_value > max_gust * config.compliance_wind_warn_ratio
        || precip_value > max_precip * config.compliance_wind_warn_ratio
    {
        status = ComplianceStatus::Warn;
    }

    WeatherCheck {
        status,
        message: format!(
            "Wind {:.1} m/s, Gust {:.1} m/s, Precip {:.1} mm (Source: Open-Meteo)",
            wind_value, gust_value, precip_value
        ),
        wind_mps: Some(wind_value),
        gust_mps: Some(gust_value),
        precip_mm: Some(precip_value),
        max_wind_mps: max_wind,
        max_gust_mps: max_gust,
        max_precip_mm: max_precip,
        source: "Open-Meteo".to_string(),
    }
}

fn evaluate_battery(
    config: &Config,
    route: &RouteMetrics,
    cruise_speed_mps: Option<f64>,
    capacity_min: Option<f64>,
    reserve_min: Option<f64>,
) -> BatteryCheck {
    if !route.has_route
        || capacity_min.is_none()
        || reserve_min.is_none()
        || cruise_speed_mps.unwrap_or(0.0) <= 0.0
    {
        return BatteryCheck {
            status: ComplianceStatus::Pending,
            message: "Battery inputs missing".to_string(),
            estimated_minutes: None,
            capacity_min,
            reserve_min,
            remaining_min: None,
            cruise_speed_mps,
        };
    }

    let estimated_minutes = route.estimated_minutes;
    let capacity = capacity_min.unwrap_or(0.0);
    let reserve = reserve_min.unwrap_or(0.0);
    let remaining = capacity - estimated_minutes;
    let mut status = ComplianceStatus::Pass;

    if remaining < reserve {
        status = ComplianceStatus::Fail;
    } else if remaining < reserve + config.compliance_battery_warn_margin_min {
        status = ComplianceStatus::Warn;
    }

    BatteryCheck {
        status,
        message: format!(
            "Est {:.1} min | Remaining {:.1} min",
            estimated_minutes, remaining
        ),
        estimated_minutes: Some(estimated_minutes),
        capacity_min: Some(capacity),
        reserve_min: Some(reserve),
        remaining_min: Some(remaining),
        cruise_speed_mps,
    }
}

fn evaluate_population(config: &Config, operation_type: u8, analysis: &ObstacleAnalysis) -> PopulationCheck {
    let density = analysis.density;
    let classification = classify_density(density);
    let is_bvlos = operation_type == 2;
    let mut status = ComplianceStatus::Pass;

    if density >= config.compliance_population_absolute_max {
        status = ComplianceStatus::Fail;
    } else if is_bvlos && density > config.compliance_population_bvlos_max {
        status = ComplianceStatus::Fail;
    } else if density >= config.compliance_population_warn {
        status = ComplianceStatus::Warn;
    }

    PopulationCheck {
        status,
        message: format!(
            "Density {:.0} people/km^2 ({})",
            density, classification
        ),
        density: Some(density),
        classification: Some(classification),
        building_count: Some(analysis.building_count),
        estimated_population: Some(analysis.estimated_population),
        area_km2: Some(analysis.area_km2),
        source: Some("OpenStreetMap".to_string()),
    }
}

fn evaluate_obstacles(
    points: &[RoutePoint],
    clearance_m: f64,
    mut analysis: ObstacleAnalysis,
) -> ObstaclesCheck {
    let hazard_list = if analysis.hazards.is_empty() {
        default_hazards()
    } else {
        let mut hazards = default_hazards();
        hazards.extend(analysis.hazards.drain(..));
        hazards
    };

    if points.is_empty() {
        return ObstaclesCheck {
            status: ComplianceStatus::Pending,
            message: "Route missing".to_string(),
            clearance_m,
            conflicts: Vec::new(),
            hazards: hazard_list,
            obstacle_count: analysis.obstacle_count,
            truncated: analysis.truncated,
        };
    }

    let mut conflicts = Vec::new();
    let mut warnings = Vec::new();
    let warn_buffer = clearance_m * 1.5;

    for hazard in &hazard_list {
        let distance = distance_to_route_meters(hazard.lat, hazard.lon, points);
        let conflict_threshold = hazard.radius_m + clearance_m;
        let warn_threshold = hazard.radius_m + warn_buffer;
        if distance <= conflict_threshold {
            conflicts.push(ObstacleConflict {
                id: hazard.id.clone(),
                name: hazard.name.clone(),
                distance_m: distance,
                severity: "conflict".to_string(),
            });
        } else if distance <= warn_threshold {
            warnings.push(ObstacleConflict {
                id: hazard.id.clone(),
                name: hazard.name.clone(),
                distance_m: distance,
                severity: "warning".to_string(),
            });
        }
    }

    let status = if !conflicts.is_empty() {
        ComplianceStatus::Fail
    } else if !warnings.is_empty() {
        ComplianceStatus::Warn
    } else {
        ComplianceStatus::Pass
    };

    let mut conflicts_all = conflicts;
    conflicts_all.extend(warnings);

    ObstaclesCheck {
        status,
        message: format!("{} conflicts", conflicts_all.len()),
        clearance_m,
        conflicts: conflicts_all,
        hazards: hazard_list,
        obstacle_count: analysis.obstacle_count,
        truncated: analysis.truncated,
    }
}

fn summarize_status(checks: &ComplianceChecks) -> ComplianceStatus {
    let mut has_warn = false;
    let mut has_pending = false;
    let mut has_fail = false;

    for status in [
        &checks.weather.status,
        &checks.battery.status,
        &checks.population.status,
        &checks.obstacles.status,
    ] {
        match status {
            ComplianceStatus::Fail => has_fail = true,
            ComplianceStatus::Pending => has_pending = true,
            ComplianceStatus::Warn => has_warn = true,
            ComplianceStatus::Pass => {}
        }
    }

    if has_fail {
        ComplianceStatus::Fail
    } else if has_pending {
        ComplianceStatus::Pending
    } else if has_warn {
        ComplianceStatus::Warn
    } else {
        ComplianceStatus::Pass
    }
}

async fn fetch_weather(
    client: &Client,
    config: &Config,
    points: &[RoutePoint],
) -> Result<WeatherCurrent, String> {
    let (lat, lon) = route_center(points).ok_or("missing route center")?;
    let response = client
        .get(&config.compliance_weather_url)
        .query(&[
            ("latitude", lat.to_string()),
            ("longitude", lon.to_string()),
            ("current", "temperature_2m,wind_speed_10m,wind_gusts_10m,precipitation,weather_code".to_string()),
            ("windspeed_unit", "ms".to_string()),
            ("timezone", "UTC".to_string()),
        ])
        .send()
        .await
        .map_err(|err| err.to_string())?;

    if !response.status().is_success() {
        return Err(format!("weather provider HTTP {}", response.status()));
    }

    let payload: WeatherResponse = response.json().await.map_err(|err| err.to_string())?;
    if let Some(current) = payload.current {
        Ok(current)
    } else if let Some(alt) = payload.current_weather {
        Ok(WeatherCurrent {
            wind_speed_10m: alt.windspeed,
            wind_gusts_10m: alt.windgusts,
            precipitation: alt.precipitation,
        })
    } else {
        Err("weather response missing current data".to_string())
    }
}

async fn fetch_obstacles(
    client: &Client,
    config: &Config,
    points: &[RoutePoint],
    clearance_m: f64,
) -> Result<ObstacleAnalysis, String> {
    let base_bounds = compute_bounds(points).ok_or("invalid route bounds")?;
    let bounds = expand_bounds(&base_bounds);
    let area_km2 = bounds_area_km2(&bounds);
    let bbox = format!("{},{},{},{}", bounds.min_lat, bounds.min_lon, bounds.max_lat, bounds.max_lon);
    let query = format!(
        "[out:json][timeout:25];\n(\n  node[\"man_made\"~\"tower|mast|chimney\"]({bbox});\n  node[\"power\"=\"tower\"]({bbox});\n  node[\"aeroway\"~\"helipad|heliport\"]({bbox});\n  way[\"man_made\"~\"tower|mast|chimney\"]({bbox});\n  way[\"power\"=\"tower\"]({bbox});\n  way[\"aeroway\"~\"helipad|heliport\"]({bbox});\n  way[\"building\"]({bbox});\n);\nout center tags;"
    );

    let response = client
        .post(&config.compliance_overpass_url)
        .header("Content-Type", "text/plain")
        .body(query)
        .send()
        .await
        .map_err(|err| err.to_string())?;

    if !response.status().is_success() {
        return Err(format!("OSM provider HTTP {}", response.status()));
    }

    let payload: OverpassResponse = response.json().await.map_err(|err| err.to_string())?;
    let elements = payload.elements;
    let truncated = elements.len() > config.compliance_max_overpass_elements;
    let sample: Vec<OverpassElement> = if truncated {
        elements.into_iter().take(config.compliance_max_overpass_elements).collect()
    } else {
        elements
    };

    let mut building_count = 0usize;
    let mut hazards = Vec::new();
    let mut seen = HashSet::new();
    let max_distance = 400.0_f64.max(clearance_m * 4.0);

    for element in sample {
        let tags = element.tags.clone().unwrap_or_default();
        let center = element_center(&element).or_else(|| element.center.as_ref().map(|c| (c.lat, c.lon)));
        let Some((lat, lon)) = center else { continue };

        let is_building = tags.get("building").is_some();
        if is_building {
            building_count += 1;
        }

        let man_made = tags.get("man_made").map(String::as_str);
        let aeroway = tags.get("aeroway").map(String::as_str);
        let power = tags.get("power").map(String::as_str);
        let levels = tags.get("building:levels").and_then(|v| v.parse::<f64>().ok());
        let height_m = parse_height(tags.get("height"))
            .or_else(|| parse_height(tags.get("height:roof")))
            .or_else(|| levels.map(|lvl| lvl * 3.0));

        let is_tower = matches!(man_made, Some("tower") | Some("mast") | Some("chimney"));
        let is_power_tower = matches!(power, Some("tower"));
        let is_helipad = matches!(aeroway, Some("helipad") | Some("heliport"));
        let is_tall_building = is_building && height_m.unwrap_or(0.0) >= clearance_m.max(20.0);

        let obstacle_type = if is_tower {
            man_made.unwrap_or("tower")
        } else if is_power_tower {
            "power_tower"
        } else if is_helipad {
            aeroway.unwrap_or("helipad")
        } else if is_tall_building {
            "tall_building"
        } else {
            ""
        };

        if obstacle_type.is_empty() {
            continue;
        }

        let distance_m = distance_to_route_meters(lat, lon, points);
        if distance_m.is_finite() && distance_m > max_distance {
            continue;
        }

        let base_radius = clearance_m.max(50.0);
        let radius_m = height_m
            .map(|h| base_radius.max(200.0_f64.min(h * 1.2)))
            .unwrap_or(base_radius);

        let key = format!("{obstacle_type}:{:.5}:{:.5}", lat, lon);
        if !seen.insert(key) {
            continue;
        }

        hazards.push(ObstacleHazard {
            id: format!("{obstacle_type}-{}", element.id),
            name: tags
                .get("name")
                .cloned()
                .unwrap_or_else(|| obstacle_type.replace('_', " ")),
            lat,
            lon,
            radius_m,
            height_m,
            hazard_type: obstacle_type.to_string(),
            source: "OpenStreetMap".to_string(),
            distance_m: distance_m.is_finite().then_some(distance_m),
        });
    }

    hazards.sort_by(|a, b| a.distance_m.partial_cmp(&b.distance_m).unwrap_or(std::cmp::Ordering::Equal));
    let obstacle_count = hazards.len();
    let trimmed: Vec<ObstacleHazard> = hazards
        .into_iter()
        .take(config.compliance_max_obstacles_response)
        .collect();

    let estimated_population = building_count as f64 * config.compliance_population_per_building;
    let density = if area_km2 > 0.0 {
        estimated_population / area_km2
    } else {
        0.0
    };

    Ok(ObstacleAnalysis {
        hazards: trimmed,
        obstacle_count,
        truncated,
        building_count,
        estimated_population,
        density,
        area_km2,
    })
}

fn route_center(points: &[RoutePoint]) -> Option<(f64, f64)> {
    if points.is_empty() {
        return None;
    }
    let mut sum_lat = 0.0;
    let mut sum_lon = 0.0;
    for point in points {
        sum_lat += point.lat;
        sum_lon += point.lon;
    }
    Some((sum_lat / points.len() as f64, sum_lon / points.len() as f64))
}

fn compute_bounds(points: &[RoutePoint]) -> Option<Bounds> {
    if points.is_empty() {
        return None;
    }
    let mut min_lat = f64::INFINITY;
    let mut max_lat = f64::NEG_INFINITY;
    let mut min_lon = f64::INFINITY;
    let mut max_lon = f64::NEG_INFINITY;
    for point in points {
        if !point.lat.is_finite() || !point.lon.is_finite() {
            continue;
        }
        min_lat = min_lat.min(point.lat);
        max_lat = max_lat.max(point.lat);
        min_lon = min_lon.min(point.lon);
        max_lon = max_lon.max(point.lon);
    }
    if !min_lat.is_finite() || !min_lon.is_finite() {
        return None;
    }
    Some(Bounds {
        min_lat,
        max_lat,
        min_lon,
        max_lon,
    })
}

fn expand_bounds(bounds: &Bounds) -> Bounds {
    let lat_span = bounds.max_lat - bounds.min_lat;
    let lon_span = bounds.max_lon - bounds.min_lon;
    let pad_lat = (lat_span * 0.3).max(0.002);
    let pad_lon = (lon_span * 0.3).max(0.002);
    Bounds {
        min_lat: bounds.min_lat - pad_lat,
        max_lat: bounds.max_lat + pad_lat,
        min_lon: bounds.min_lon - pad_lon,
        max_lon: bounds.max_lon + pad_lon,
    }
}

fn bounds_area_km2(bounds: &Bounds) -> f64 {
    let mean_lat = ((bounds.min_lat + bounds.max_lat) / 2.0).to_radians();
    let meters_per_deg_lat = 111_320.0;
    let meters_per_deg_lon = 111_320.0 * mean_lat.cos();
    let width_m = (bounds.max_lon - bounds.min_lon) * meters_per_deg_lon;
    let height_m = (bounds.max_lat - bounds.min_lat) * meters_per_deg_lat;
    let area_km2 = (width_m.max(0.0) * height_m.max(0.0)) / 1_000_000.0;
    area_km2.max(0.15)
}

fn parse_height(value: Option<&String>) -> Option<f64> {
    let value = value?;
    let mut digits = String::new();
    for ch in value.chars() {
        if ch.is_ascii_digit() || ch == '.' {
            digits.push(ch);
        } else if !digits.is_empty() {
            break;
        }
    }
    if digits.is_empty() {
        return None;
    }
    digits.parse::<f64>().ok()
}

fn element_center(element: &OverpassElement) -> Option<(f64, f64)> {
    match (element.lat, element.lon) {
        (Some(lat), Some(lon)) => Some((lat, lon)),
        _ => None,
    }
}

fn classify_density(density: f64) -> String {
    if !density.is_finite() {
        return "unknown".to_string();
    }
    if density < 200.0 {
        "rural".to_string()
    } else if density < 1000.0 {
        "suburban".to_string()
    } else if density < 2500.0 {
        "urban".to_string()
    } else {
        "dense".to_string()
    }
}

fn distance_to_route_meters(hazard_lat: f64, hazard_lon: f64, points: &[RoutePoint]) -> f64 {
    if points.is_empty() {
        return f64::INFINITY;
    }

    let ref_lat = hazard_lat.to_radians();
    let meters_per_deg_lat = 111_320.0;
    let meters_per_deg_lon = 111_320.0 * ref_lat.cos();

    let to_xy = |point: &RoutePoint| -> (f64, f64) {
        (
            (point.lon - hazard_lon) * meters_per_deg_lon,
            (point.lat - hazard_lat) * meters_per_deg_lat,
        )
    };

    let mut min = f64::INFINITY;
    for point in points {
        let (x, y) = to_xy(point);
        let dist = (x * x + y * y).sqrt();
        if dist < min {
            min = dist;
        }
    }

    for idx in 1..points.len() {
        let (ax, ay) = to_xy(&points[idx - 1]);
        let (bx, by) = to_xy(&points[idx]);
        let dx = bx - ax;
        let dy = by - ay;
        let len_sq = dx * dx + dy * dy;
        if len_sq == 0.0 {
            continue;
        }
        let mut t = -(ax * dx + ay * dy) / len_sq;
        if t < 0.0 {
            t = 0.0;
        } else if t > 1.0 {
            t = 1.0;
        }
        let cx = ax + t * dx;
        let cy = ay + t * dy;
        let dist = (cx * cx + cy * cy).sqrt();
        if dist < min {
            min = dist;
        }
    }

    min
}

fn haversine_distance_m(a: RoutePoint, b: RoutePoint) -> f64 {
    let lat1 = a.lat.to_radians();
    let lat2 = b.lat.to_radians();
    let dlat = lat2 - lat1;
    let dlon = (b.lon - a.lon).to_radians();
    let h = (dlat / 2.0).sin().powi(2)
        + lat1.cos() * lat2.cos() * (dlon / 2.0).sin().powi(2);
    let c = 2.0 * h.sqrt().asin();
    6_371_000.0 * c
}

fn default_hazards() -> Vec<ObstacleHazard> {
    vec![
        ObstacleHazard {
            id: "tower-1".to_string(),
            name: "Campus Tower".to_string(),
            lat: 33.6459,
            lon: -117.8422,
            radius_m: 80.0,
            height_m: None,
            hazard_type: "tower".to_string(),
            source: "static".to_string(),
            distance_m: None,
        },
        ObstacleHazard {
            id: "power-1".to_string(),
            name: "Power Corridor".to_string(),
            lat: 33.6835,
            lon: -117.8302,
            radius_m: 120.0,
            height_m: None,
            hazard_type: "power".to_string(),
            source: "static".to_string(),
            distance_m: None,
        },
        ObstacleHazard {
            id: "hospital-1".to_string(),
            name: "Helipad Zone".to_string(),
            lat: 33.6431,
            lon: -117.8455,
            radius_m: 150.0,
            height_m: None,
            hazard_type: "helipad".to_string(),
            source: "static".to_string(),
            distance_m: None,
        },
        ObstacleHazard {
            id: "stadium-1".to_string(),
            name: "Stadium Complex".to_string(),
            lat: 33.6505,
            lon: -117.8372,
            radius_m: 180.0,
            height_m: None,
            hazard_type: "stadium".to_string(),
            source: "static".to_string(),
            distance_m: None,
        },
    ]
}
