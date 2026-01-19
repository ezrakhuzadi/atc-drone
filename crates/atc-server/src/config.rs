//! Server configuration from environment.

use std::env;
use atc_core::rules::{AltitudeBand, SafetyRules};

#[derive(Debug, Clone)]
pub struct Config {
    pub server_port: u16,
    pub blender_url: String,
    pub blender_session_id: String,
    pub rid_view_bbox: String,
    pub geofence_sync_state_path: String,
    pub blender_auth_token: String,
    /// Comma-separated list of allowed CORS origins
    pub allowed_origins: Vec<String>,
    /// Admin token for protected endpoints (generate random if not set)
    pub admin_token: String,
    /// Optional token required for WebSocket stream access.
    pub ws_token: Option<String>,
    /// Require WebSocket token for /v1/ws.
    pub require_ws_token: bool,
    /// Optional shared token required for drone registration.
    pub registration_token: Option<String>,
    /// Require registration token for /v1/drones/register.
    pub require_registration_token: bool,
    /// Enable rate limiting (default: true in prod)
    pub rate_limit_enabled: bool,
    /// Max requests per second per IP for telemetry
    pub rate_limit_rps: u32,
    /// Max requests per second per IP for drone registration
    pub registration_rate_limit_rps: u32,
    /// Path to SQLite database file
    pub database_path: String,
    /// Max connections for database pool
    pub database_max_connections: u32,
    pub compliance_weather_url: String,
    pub compliance_overpass_url: String,
    pub compliance_population_per_building: f64,
    pub compliance_max_overpass_elements: usize,
    pub compliance_max_obstacles_response: usize,
    pub compliance_max_wind_mps: f64,
    pub compliance_max_gust_mps: f64,
    pub compliance_max_precip_mm: f64,
    pub compliance_wind_warn_ratio: f64,
    pub compliance_battery_warn_margin_min: f64,
    pub compliance_population_bvlos_max: f64,
    pub compliance_population_warn: f64,
    pub compliance_population_absolute_max: f64,
    pub compliance_default_clearance_m: f64,
    pub compliance_default_building_height_m: f64,
    pub compliance_overpass_timeout_s: u64,
    pub compliance_overpass_retries: u32,
    pub compliance_overpass_retry_backoff_ms: u64,
    pub obstacle_cache_ttl_s: u64,
    pub terrain_provider_url: String,
    pub terrain_sample_spacing_m: f64,
    pub terrain_max_points_per_request: usize,
    pub terrain_max_grid_points: usize,
    pub terrain_request_timeout_s: u64,
    pub terrain_request_retries: u32,
    pub terrain_request_backoff_ms: u64,
    pub terrain_request_min_interval_ms: u64,
    pub terrain_require: bool,
    pub terrain_cache_ttl_s: u64,
    pub terrain_max_requests: usize,
    pub telemetry_min_alt_m: f64,
    pub telemetry_max_alt_m: f64,
    pub telemetry_max_speed_mps: f64,
    pub telemetry_max_future_s: i64,
    pub telemetry_max_age_s: i64,
    pub pull_blender_geofences: bool,
    pub allow_admin_reset: bool,
    pub require_blender_declaration: bool,
    pub tls_cert_path: Option<String>,
    pub tls_key_path: Option<String>,
    pub require_tls: bool,
    pub rules_min_horizontal_separation_m: f64,
    pub rules_min_vertical_separation_m: f64,
    pub rules_lookahead_seconds: f64,
    pub rules_warning_multiplier: f64,
    pub rules_drone_timeout_secs: u64,
    pub rules_max_altitude_m: f64,
    pub rules_min_altitude_m: f64,
}


impl Config {
    pub fn from_env() -> Self {
        let is_dev = env::var("ATC_ENV").unwrap_or_else(|_| "development".to_string()) == "development";
        
        // Generate admin token if not provided
        let admin_token = env::var("ATC_ADMIN_TOKEN").unwrap_or_else(|_| {
            let token = uuid::Uuid::new_v4().to_string();
            tracing::warn!("No ATC_ADMIN_TOKEN set, generated: {}", token);
            token
        });

        let ws_token = env::var("ATC_WS_TOKEN")
            .ok()
            .map(|v| v.trim().to_string())
            .filter(|v| !v.is_empty());
        
        let default_rules = SafetyRules::default();

        Self {
            server_port: env::var("ATC_PORT")
                .ok()
                .and_then(|s| s.parse().ok())
                .unwrap_or(3000),
            blender_url: env::var("BLENDER_URL")
                .unwrap_or_else(|_| "http://localhost:8000".to_string()),
            blender_session_id: env::var("BLENDER_SESSION_ID")
                .unwrap_or_else(|_| "00000000-0000-0000-0000-000000000001".to_string()),
            rid_view_bbox: env::var("RID_VIEW_BBOX")
                .unwrap_or_else(|_| "33.654600,-117.856500,33.714600,-117.796500".to_string()),
            geofence_sync_state_path: env::var("GEOFENCE_SYNC_STATE_PATH")
                .unwrap_or_else(|_| "data/geofence_sync.json".to_string()),
            blender_auth_token: env::var("BLENDER_AUTH_TOKEN").unwrap_or_default(),
            allowed_origins: env::var("ATC_ALLOWED_ORIGINS")
                .unwrap_or_else(|_| if is_dev {
                    "http://localhost:5000,http://localhost:3000,http://localhost:5050,http://127.0.0.1:5000,http://127.0.0.1:5050".to_string()
                } else {
                    "".to_string() // Must be explicitly set in production
                })
                .split(',')
                .filter(|s| !s.is_empty())
                .map(String::from)
                .collect(),
            admin_token,
            ws_token: ws_token.clone(),
            require_ws_token: env::var("ATC_REQUIRE_WS_TOKEN")
                .map(|v| v != "0" && v.to_lowercase() != "false")
                .unwrap_or(!is_dev && ws_token.is_some()),
            registration_token: env::var("ATC_REGISTRATION_TOKEN")
                .ok()
                .map(|v| v.trim().to_string())
                .filter(|v| !v.is_empty()),
            require_registration_token: env::var("ATC_REQUIRE_REGISTRATION_TOKEN")
                .map(|v| v != "0" && v.to_lowercase() != "false")
                .unwrap_or(true),
            rate_limit_enabled: env::var("ATC_RATE_LIMIT")
                .map(|v| v != "0" && v.to_lowercase() != "false")
                .unwrap_or(!is_dev), // Enabled by default in prod
            rate_limit_rps: env::var("ATC_RATE_LIMIT_RPS")
                .ok()
                .and_then(|s| s.parse().ok())
                .unwrap_or(100),
            registration_rate_limit_rps: env::var("ATC_REGISTER_RATE_LIMIT_RPS")
                .ok()
                .and_then(|s| s.parse().ok())
                .unwrap_or(10),
            database_path: env::var("ATC_DATABASE_PATH")
                .unwrap_or_else(|_| "data/atc.db".to_string()),
            database_max_connections: env::var("ATC_DB_MAX_CONNECTIONS")
                .ok()
                .and_then(|s| s.parse().ok())
                .unwrap_or(10),
            compliance_weather_url: env::var("ATC_COMPLIANCE_WEATHER_URL")
                .unwrap_or_else(|_| "https://api.open-meteo.com/v1/forecast".to_string()),
            compliance_overpass_url: env::var("ATC_COMPLIANCE_OVERPASS_URL")
                .unwrap_or_else(|_| "https://overpass-api.de/api/interpreter".to_string()),
            compliance_population_per_building: env::var("ATC_COMPLIANCE_POP_PER_BUILDING")
                .ok()
                .and_then(|s| s.parse().ok())
                .unwrap_or(2.4),
            compliance_max_overpass_elements: env::var("ATC_COMPLIANCE_MAX_OVERPASS_ELEMENTS")
                .ok()
                .and_then(|s| s.parse().ok())
                .unwrap_or(3000),
            compliance_max_obstacles_response: env::var("ATC_COMPLIANCE_MAX_OBSTACLES_RESPONSE")
                .ok()
                .and_then(|s| s.parse().ok())
                .unwrap_or(200),
            compliance_max_wind_mps: env::var("ATC_COMPLIANCE_MAX_WIND_MPS")
                .ok()
                .and_then(|s| s.parse().ok())
                .unwrap_or(12.0),
            compliance_max_gust_mps: env::var("ATC_COMPLIANCE_MAX_GUST_MPS")
                .ok()
                .and_then(|s| s.parse().ok())
                .unwrap_or(15.0),
            compliance_max_precip_mm: env::var("ATC_COMPLIANCE_MAX_PRECIP_MM")
                .ok()
                .and_then(|s| s.parse().ok())
                .unwrap_or(2.0),
            compliance_wind_warn_ratio: env::var("ATC_COMPLIANCE_WIND_WARN_RATIO")
                .ok()
                .and_then(|s| s.parse().ok())
                .unwrap_or(0.8),
            compliance_battery_warn_margin_min: env::var("ATC_COMPLIANCE_BATTERY_WARN_MARGIN_MIN")
                .ok()
                .and_then(|s| s.parse().ok())
                .unwrap_or(5.0),
            compliance_population_bvlos_max: env::var("ATC_COMPLIANCE_POP_BVLOS_MAX")
                .ok()
                .and_then(|s| s.parse().ok())
                .unwrap_or(1500.0),
            compliance_population_warn: env::var("ATC_COMPLIANCE_POP_WARN")
                .ok()
                .and_then(|s| s.parse().ok())
                .unwrap_or(2000.0),
            compliance_population_absolute_max: env::var("ATC_COMPLIANCE_POP_ABS_MAX")
                .ok()
                .and_then(|s| s.parse().ok())
                .unwrap_or(4000.0),
            compliance_default_clearance_m: env::var("ATC_COMPLIANCE_DEFAULT_CLEARANCE_M")
                .ok()
                .and_then(|s| s.parse().ok())
                .unwrap_or(60.0),
            compliance_default_building_height_m: env::var("ATC_COMPLIANCE_DEFAULT_BUILDING_HEIGHT_M")
                .ok()
                .and_then(|s| s.parse().ok())
                .unwrap_or(30.0),
            obstacle_cache_ttl_s: env::var("ATC_OBSTACLE_CACHE_TTL_S")
                .ok()
                .and_then(|s| s.parse().ok())
                .unwrap_or(900),
            terrain_provider_url: env::var("ATC_TERRAIN_PROVIDER_URL")
                .unwrap_or_else(|_| "https://api.open-meteo.com/v1/elevation".to_string()),
            terrain_sample_spacing_m: env::var("ATC_TERRAIN_SAMPLE_SPACING_M")
                .ok()
                .and_then(|s| s.parse().ok())
                .unwrap_or(30.0),
            terrain_max_points_per_request: env::var("ATC_TERRAIN_MAX_POINTS_PER_REQUEST")
                .ok()
                .and_then(|s| s.parse().ok())
                .unwrap_or(100),
            terrain_max_grid_points: env::var("ATC_TERRAIN_MAX_GRID_POINTS")
                .ok()
                .and_then(|s| s.parse().ok())
                .unwrap_or(50000),
            terrain_request_timeout_s: env::var("ATC_TERRAIN_REQUEST_TIMEOUT_S")
                .ok()
                .and_then(|s| s.parse().ok())
                .unwrap_or(15),
            terrain_require: env::var("ATC_TERRAIN_REQUIRE")
                .map(|v| v != "0" && v.to_lowercase() != "false")
                .unwrap_or(!is_dev),
            terrain_cache_ttl_s: env::var("ATC_TERRAIN_CACHE_TTL_S")
                .ok()
                .and_then(|s| s.parse().ok())
                .unwrap_or(900),
            telemetry_min_alt_m: env::var("ATC_TELEMETRY_MIN_ALT_M")
                .ok()
                .and_then(|s| s.parse().ok())
                .unwrap_or(-100.0),
            telemetry_max_alt_m: env::var("ATC_TELEMETRY_MAX_ALT_M")
                .ok()
                .and_then(|s| s.parse().ok())
                .unwrap_or(20000.0),
            telemetry_max_speed_mps: env::var("ATC_TELEMETRY_MAX_SPEED_MPS")
                .ok()
                .and_then(|s| s.parse().ok())
                .unwrap_or(150.0),
            telemetry_max_future_s: env::var("ATC_TELEMETRY_MAX_FUTURE_S")
                .ok()
                .and_then(|s| s.parse().ok())
                .unwrap_or(30),
            telemetry_max_age_s: env::var("ATC_TELEMETRY_MAX_AGE_S")
                .ok()
                .and_then(|s| s.parse().ok())
                .unwrap_or(300),
            pull_blender_geofences: env::var("ATC_PULL_BLENDER_GEOFENCES")
                .map(|v| v != "0" && v.to_lowercase() != "false")
                .unwrap_or(true),
            allow_admin_reset: env::var("ATC_ALLOW_ADMIN_RESET")
                .map(|v| v != "0" && v.to_lowercase() != "false")
                .unwrap_or(is_dev),
            require_blender_declaration: env::var("ATC_REQUIRE_BLENDER_DECLARATION")
                .map(|v| v != "0" && v.to_lowercase() != "false")
                .unwrap_or(true),
            tls_cert_path: env::var("ATC_TLS_CERT_PATH")
                .ok()
                .and_then(|v| {
                    let trimmed = v.trim().to_string();
                    if trimmed.is_empty() { None } else { Some(trimmed) }
                }),
            tls_key_path: env::var("ATC_TLS_KEY_PATH")
                .ok()
                .and_then(|v| {
                    let trimmed = v.trim().to_string();
                    if trimmed.is_empty() { None } else { Some(trimmed) }
                }),
            require_tls: env::var("ATC_REQUIRE_TLS")
                .map(|v| v != "0" && v.to_lowercase() != "false")
                .unwrap_or(!is_dev),
            rules_min_horizontal_separation_m: env::var("ATC_RULES_MIN_HORIZONTAL_SEPARATION_M")
                .ok()
                .and_then(|s| s.parse().ok())
                .unwrap_or(default_rules.min_horizontal_separation_m),
            rules_min_vertical_separation_m: env::var("ATC_RULES_MIN_VERTICAL_SEPARATION_M")
                .ok()
                .and_then(|s| s.parse().ok())
                .unwrap_or(default_rules.min_vertical_separation_m),
            rules_lookahead_seconds: env::var("ATC_RULES_LOOKAHEAD_SECONDS")
                .ok()
                .and_then(|s| s.parse().ok())
                .unwrap_or(default_rules.lookahead_seconds),
            rules_warning_multiplier: env::var("ATC_RULES_WARNING_MULTIPLIER")
                .ok()
                .and_then(|s| s.parse().ok())
                .unwrap_or(default_rules.warning_multiplier),
            rules_drone_timeout_secs: env::var("ATC_RULES_DRONE_TIMEOUT_SECS")
                .ok()
                .and_then(|s| s.parse().ok())
                .unwrap_or(default_rules.drone_timeout_secs),
            rules_max_altitude_m: env::var("ATC_RULES_MAX_ALTITUDE_M")
                .ok()
                .and_then(|s| s.parse().ok())
                .unwrap_or(default_rules.max_altitude_m),
            rules_min_altitude_m: env::var("ATC_RULES_MIN_ALTITUDE_M")
                .ok()
                .and_then(|s| s.parse().ok())
                .unwrap_or(default_rules.min_altitude_m),
        }
    }

    pub fn safety_rules(&self) -> SafetyRules {
        let mut rules = SafetyRules::default();
        rules.min_horizontal_separation_m = self.rules_min_horizontal_separation_m;
        rules.min_vertical_separation_m = self.rules_min_vertical_separation_m;
        rules.lookahead_seconds = self.rules_lookahead_seconds;
        rules.warning_multiplier = self.rules_warning_multiplier;
        rules.drone_timeout_secs = self.rules_drone_timeout_secs;
        rules.max_altitude_m = self.rules_max_altitude_m;
        rules.min_altitude_m = self.rules_min_altitude_m;
        if rules.max_altitude_m > rules.min_altitude_m {
            let band_size = (rules.max_altitude_m - rules.min_altitude_m) / 3.0;
            rules.altitude_bands = vec![
                AltitudeBand {
                    name: "Low".to_string(),
                    min_m: rules.min_altitude_m,
                    max_m: rules.min_altitude_m + band_size,
                },
                AltitudeBand {
                    name: "Medium".to_string(),
                    min_m: rules.min_altitude_m + band_size,
                    max_m: rules.min_altitude_m + (band_size * 2.0),
                },
                AltitudeBand {
                    name: "High".to_string(),
                    min_m: rules.min_altitude_m + (band_size * 2.0),
                    max_m: rules.max_altitude_m,
                },
            ];
        }
        rules
    }
}
