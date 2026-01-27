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
use std::sync::{
    atomic::{AtomicU64, Ordering},
    Arc,
};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use crate::state::AppState;

/// Extractor for admin token from config.
#[derive(Clone)]
pub struct AdminToken(pub Arc<String>);

fn constant_time_eq(a: &[u8], b: &[u8]) -> bool {
    if a.len() != b.len() {
        return false;
    }
    let mut diff: u8 = 0;
    for (&left, &right) in a.iter().zip(b.iter()) {
        diff |= left ^ right;
    }
    diff == 0
}

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
            if constant_time_eq(token.as_bytes(), admin_token.0.as_bytes()) {
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

#[derive(Debug, Clone, Copy)]
struct RateLimitEntry {
    window_start_epoch_s: u64,
    window_count: u32,
    last_seen_epoch_s: u64,
}

#[derive(Clone)]
pub struct RateLimiter {
    requests: Arc<DashMap<String, RateLimitEntry>>,
    last_cleanup_epoch_s: Arc<AtomicU64>,
    cleanup_interval: Duration,
    entry_ttl: Duration,
    max_tracked_ips: usize,
    max_rps: u32,
    enabled: bool,
    trust_proxy: bool,
}

impl RateLimiter {
    pub fn new(
        max_rps: u32,
        enabled: bool,
        trust_proxy: bool,
        max_tracked_ips: usize,
        entry_ttl: Duration,
    ) -> Self {
        Self {
            requests: Arc::new(DashMap::new()),
            last_cleanup_epoch_s: Arc::new(AtomicU64::new(0)),
            cleanup_interval: Duration::from_secs(60),
            entry_ttl,
            max_tracked_ips,
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

        let now_epoch_s = match SystemTime::now().duration_since(UNIX_EPOCH) {
            Ok(value) => value.as_secs(),
            Err(_) => 0,
        };

        self.maybe_cleanup(now_epoch_s);

        if !self.ensure_capacity(ip, now_epoch_s) {
            return false;
        }

        let mut entry = self
            .requests
            .entry(ip.to_string())
            .or_insert(RateLimitEntry {
                window_start_epoch_s: now_epoch_s,
                window_count: 0,
                last_seen_epoch_s: now_epoch_s,
            });

        let state = entry.value_mut();
        if state.window_start_epoch_s != now_epoch_s {
            state.window_start_epoch_s = now_epoch_s;
            state.window_count = 0;
        }
        state.window_count = state.window_count.saturating_add(1);
        state.last_seen_epoch_s = now_epoch_s;

        state.window_count <= self.max_rps
    }

    fn maybe_cleanup(&self, now_epoch_s: u64) {
        if now_epoch_s == 0 {
            return;
        }
        let last_cleanup = self.last_cleanup_epoch_s.load(Ordering::Relaxed);
        if last_cleanup != 0
            && now_epoch_s.saturating_sub(last_cleanup) < self.cleanup_interval.as_secs()
        {
            return;
        }

        if self
            .last_cleanup_epoch_s
            .compare_exchange(last_cleanup, now_epoch_s, Ordering::Relaxed, Ordering::Relaxed)
            .is_err()
        {
            return;
        }

        self.purge_stale_entries(now_epoch_s);
    }

    fn ensure_capacity(&self, ip: &str, now_epoch_s: u64) -> bool {
        if self.max_tracked_ips == 0 {
            return true;
        }
        if self.requests.contains_key(ip) {
            return true;
        }
        if self.requests.len() < self.max_tracked_ips {
            return true;
        }

        self.purge_stale_entries(now_epoch_s);

        if self.requests.contains_key(ip) {
            return true;
        }

        self.requests.len() < self.max_tracked_ips
    }

    fn purge_stale_entries(&self, now_epoch_s: u64) {
        if now_epoch_s == 0 {
            return;
        }
        let ttl = self.entry_ttl.as_secs();
        if ttl == 0 {
            return;
        }

        let stale: Vec<String> = self
            .requests
            .iter()
            .filter(|entry| now_epoch_s.saturating_sub(entry.value().last_seen_epoch_s) >= ttl)
            .map(|entry| entry.key().clone())
            .collect();

        for key in stale {
            self.requests.remove(&key);
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
