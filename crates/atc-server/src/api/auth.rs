//! Authentication middleware for protected endpoints.

use axum::{
    extract::{ConnectInfo, Request, State},
    http::{header, StatusCode},
    middleware::Next,
    response::{IntoResponse, Response},
    Json,
};
use axum::http::HeaderMap;
use std::net::SocketAddr;
use std::sync::{Arc, Mutex};

use crate::state::AppState;

/// Extractor for admin token from config.
#[derive(Clone)]
pub struct AdminToken(pub Arc<String>);

/// Middleware that requires a valid admin token in the Authorization header.
/// 
/// Expected header format: `Authorization: Bearer <admin_token>`
pub async fn require_admin(
    State(admin_token): State<AdminToken>,
    request: Request,
    next: Next,
) -> Response {
    // Extract Authorization header
    let auth_header = request
        .headers()
        .get(header::AUTHORIZATION)
        .and_then(|value| value.to_str().ok());

    match auth_header {
        Some(auth) if auth.starts_with("Bearer ") => {
            let token = auth.trim_start_matches("Bearer ");
            if token == admin_token.0.as_str() {
                next.run(request).await
            } else {
                (
                    StatusCode::FORBIDDEN,
                    Json(serde_json::json!({
                        "error": "Invalid admin token",
                        "hint": "Check ATC_ADMIN_TOKEN environment variable"
                    })),
                )
                    .into_response()
            }
        }
        Some(_) => (
            StatusCode::BAD_REQUEST,
            Json(serde_json::json!({
                "error": "Invalid Authorization header format",
                "expected": "Bearer <token>"
            })),
        )
            .into_response(),
        None => (
            StatusCode::UNAUTHORIZED,
            Json(serde_json::json!({
                "error": "Authorization required",
                "hint": "Add header: Authorization: Bearer <admin_token>"
            })),
        )
            .into_response(),
    }
}

/// Simple rate limiter state (per-IP tracking).
/// For production, consider tower-governor or similar.
use dashmap::DashMap;
use std::time::Instant;

#[derive(Clone)]
pub struct RateLimiter {
    requests: Arc<DashMap<String, Vec<Instant>>>,
    last_cleanup: Arc<Mutex<Instant>>,
    cleanup_interval: std::time::Duration,
    max_rps: u32,
    enabled: bool,
    trust_proxy: bool,
}

impl RateLimiter {
    pub fn new(max_rps: u32, enabled: bool, trust_proxy: bool) -> Self {
        Self {
            requests: Arc::new(DashMap::new()),
            last_cleanup: Arc::new(Mutex::new(Instant::now())),
            cleanup_interval: std::time::Duration::from_secs(60),
            max_rps,
            enabled,
            trust_proxy,
        }
    }

    /// Check if request should be allowed. Returns true if allowed.
    pub fn check(&self, ip: &str) -> bool {
        if !self.enabled {
            return true;
        }
        
        let now = Instant::now();
        let window = std::time::Duration::from_secs(1);
        let do_cleanup = {
            let mut last_cleanup = self
                .last_cleanup
                .lock()
                .expect("Rate limiter cleanup lock poisoned");
            if now.duration_since(*last_cleanup) >= self.cleanup_interval {
                *last_cleanup = now;
                true
            } else {
                false
            }
        };
        if do_cleanup {
            self.purge_stale_entries(now, window);
        }
        
        let mut entry = self.requests.entry(ip.to_string()).or_default();
        let timestamps = entry.value_mut();
        
        // Remove old timestamps
        timestamps.retain(|t| now.duration_since(*t) < window);
        
        if timestamps.len() < self.max_rps as usize {
            timestamps.push(now);
            true
        } else {
            false
        }
    }

    fn purge_stale_entries(&self, now: Instant, window: std::time::Duration) {
        let stale: Vec<String> = self
            .requests
            .iter()
            .filter(|entry| entry.value().iter().all(|t| now.duration_since(*t) >= window))
            .map(|entry| entry.key().clone())
            .collect();
        for ip in stale {
            self.requests.remove(&ip);
        }
    }
}

/// Rate limiting middleware for telemetry endpoint.
pub async fn rate_limit(
    State(limiter): State<RateLimiter>,
    request: Request,
    next: Next,
) -> Response {
    // Extract IP from request (socket addr, optionally X-Forwarded-For if trusted)
    let ip = if limiter.trust_proxy {
        request
            .headers()
            .get("X-Forwarded-For")
            .and_then(|v| v.to_str().ok())
            .map(|s| s.split(',').next().unwrap_or("unknown").trim().to_string())
    } else {
        None
    }
    .or_else(|| {
        request
            .extensions()
            .get::<ConnectInfo<SocketAddr>>()
            .map(|info| info.0.ip().to_string())
    })
    .unwrap_or_else(|| "unknown".to_string());

    if limiter.check(&ip) {
        next.run(request).await
    } else {
        (
            StatusCode::TOO_MANY_REQUESTS,
            Json(serde_json::json!({
                "error": "Rate limit exceeded",
                "retry_after": "1 second"
            })),
        )
            .into_response()
    }
}

/// Extract drone session token from headers.
/// Accepts `Authorization: Bearer <token>` or `X-Drone-Token: <token>`.
pub fn extract_drone_token(headers: &HeaderMap) -> Option<String> {
    if let Some(value) = headers.get(header::AUTHORIZATION) {
        if let Ok(text) = value.to_str() {
            if let Some(token) = text.strip_prefix("Bearer ") {
                let token = token.trim();
                if !token.is_empty() {
                    return Some(token.to_string());
                }
            }
        }
    }

    if let Some(value) = headers.get("X-Drone-Token") {
        if let Ok(token) = value.to_str() {
            let token = token.trim();
            if !token.is_empty() {
                return Some(token.to_string());
            }
        }
    }

    None
}

/// Extract shared registration token from headers.
/// Accepts `X-Registration-Token: <token>` or `Authorization: Bearer <token>`.
pub fn extract_registration_token(headers: &HeaderMap) -> Option<String> {
    if let Some(value) = headers.get("X-Registration-Token") {
        if let Ok(token) = value.to_str() {
            let token = token.trim();
            if !token.is_empty() {
                return Some(token.to_string());
            }
        }
    }

    if let Some(value) = headers.get(header::AUTHORIZATION) {
        if let Ok(text) = value.to_str() {
            if let Some(token) = text.strip_prefix("Bearer ") {
                let token = token.trim();
                if !token.is_empty() {
                    return Some(token.to_string());
                }
            }
        }
    }

    None
}

/// Require a valid token for a specific drone ID.
pub fn authorize_drone_for(
    state: &AppState,
    drone_id: &str,
    headers: &HeaderMap,
) -> Result<(), StatusCode> {
    let token = extract_drone_token(headers).ok_or(StatusCode::UNAUTHORIZED)?;
    if state.validate_drone_token(drone_id, &token) {
        Ok(())
    } else {
        Err(StatusCode::FORBIDDEN)
    }
}

/// Require a valid token and return the associated drone ID.
pub fn authorize_drone_from_headers(
    state: &AppState,
    headers: &HeaderMap,
) -> Result<String, StatusCode> {
    let token = extract_drone_token(headers).ok_or(StatusCode::UNAUTHORIZED)?;
    state.drone_id_for_token(&token).ok_or(StatusCode::FORBIDDEN)
}
