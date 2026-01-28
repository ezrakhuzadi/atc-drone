//! ATC Server - Always-on backend for drone traffic management

mod altitude;
mod api;
mod backoff;
mod blender_auth;
mod cache;
mod compliance;
mod config;
mod loops;
mod persistence;
mod route_planner;
mod state;
mod terrain;

use anyhow::{bail, Result};
use axum::http::StatusCode;
use axum::http::{HeaderValue, Method};
use axum::response::IntoResponse;
use axum::{extract::State, routing::get, Json};
use axum_server::tls_rustls::RustlsConfig;
use serde::Serialize;
use std::future::Future;
use std::net::SocketAddr;
use std::time::Duration;
use tokio::sync::broadcast;
use tower_http::cors::{Any, CorsLayer};

use crate::config::Config;
use crate::state::AppState;
use std::sync::Arc;
use std::time::{Instant, SystemTime, UNIX_EPOCH};

#[derive(Debug, Serialize)]
struct LoopStatus {
    name: &'static str,
    ok: bool,
    age_secs: u64,
    max_age_secs: u64,
    last_tick_secs: Option<u64>,
}

#[derive(Debug, Serialize)]
struct ReadyResponse {
    ok: bool,
    db_ok: bool,
    loops_ok: bool,
    db_latency_ms: Option<u128>,
    loops: Vec<LoopStatus>,
    error: Option<String>,
}

async fn ready_handler(State(state): State<Arc<AppState>>) -> impl IntoResponse {
    let now_secs = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0);

    let loop_limits: [(&'static str, u64); 9] = [
        ("conflict", 5),
        ("blender-sync", 5),
        ("telemetry-persist", 10),
        ("rid", 10),
        ("mission", 10),
        ("oi-expiry", 20),
        ("conformance", 45),
        ("geofence-sync", 60),
        ("flight-declaration-sync", 120),
    ];

    let mut loops = Vec::with_capacity(loop_limits.len());
    let mut loops_ok = true;
    for (name, max_age_secs) in loop_limits {
        let last_tick_secs = state.loop_last_tick_secs(name);
        let (ok, age_secs) = match last_tick_secs {
            Some(last) => {
                let age = now_secs.saturating_sub(last);
                (age <= max_age_secs, age)
            }
            None => (false, u64::MAX),
        };
        if !ok {
            loops_ok = false;
        }
        loops.push(LoopStatus {
            name,
            ok,
            age_secs,
            max_age_secs,
            last_tick_secs,
        });
    }

    let (db_ok, db_latency_ms, db_error) = match state.database() {
        Some(db) => {
            let started_at = Instant::now();
            let result = tokio::time::timeout(
                Duration::from_millis(750),
                sqlx::query("SELECT 1").execute(db.pool()),
            )
            .await;
            match result {
                Ok(Ok(_)) => (true, Some(started_at.elapsed().as_millis()), None),
                Ok(Err(err)) => (
                    false,
                    Some(started_at.elapsed().as_millis()),
                    Some(err.to_string()),
                ),
                Err(_) => (
                    false,
                    Some(started_at.elapsed().as_millis()),
                    Some("database ping timed out".to_string()),
                ),
            }
        }
        None => (true, None, None),
    };

    let ok = db_ok && loops_ok;
    let status = if ok {
        StatusCode::OK
    } else {
        StatusCode::SERVICE_UNAVAILABLE
    };

    let error = if let Some(err) = db_error {
        Some(err)
    } else if !loops_ok {
        let stale = loops
            .iter()
            .filter(|entry| !entry.ok)
            .map(|entry| entry.name)
            .collect::<Vec<_>>()
            .join(",");
        Some(format!("stale loops: {}", stale))
    } else {
        None
    };

    (
        status,
        Json(ReadyResponse {
            ok,
            db_ok,
            loops_ok,
            db_latency_ms,
            loops,
            error,
        }),
    )
}

#[tokio::main]
async fn main() -> Result<()> {
    // Initialize tracing
    let env_filter = tracing_subscriber::EnvFilter::from_default_env()
        .add_directive("atc_server=debug".parse()?);
    let log_format = std::env::var("ATC_LOG_FORMAT").unwrap_or_default();
    if log_format.trim().eq_ignore_ascii_case("json") {
        tracing_subscriber::fmt()
            .json()
            .with_env_filter(env_filter)
            .init();
    } else {
        tracing_subscriber::fmt().with_env_filter(env_filter).init();
    }

    tracing::info!("Starting ATC Server...");

    let config = Config::from_env();
    let port = config.server_port;

    if config.require_registration_token && config.registration_token.is_none() {
        bail!("ATC_REGISTRATION_TOKEN is required when ATC_REQUIRE_REGISTRATION_TOKEN is enabled");
    }
    if !config.allow_dummy_blender_auth
        && config.blender_auth_token.trim().is_empty()
        && config.blender_oauth_config().is_none()
    {
        bail!("BLENDER_AUTH_TOKEN or BLENDER_OAUTH_* is required when ATC_ENV is not development");
    }
    if !config.allow_dummy_blender_auth && config.admin_token.trim().is_empty() {
        bail!("ATC_ADMIN_TOKEN must be set when ATC_ENV is not development");
    }
    if !config.allow_dummy_blender_auth && config.admin_token == "change-me-admin" {
        bail!("ATC_ADMIN_TOKEN is still set to the insecure default 'change-me-admin'");
    }
    if !config.allow_dummy_blender_auth {
        if let Some(token) = config.registration_token.as_deref() {
            if token == "change-me-registration-token" {
                bail!("ATC_REGISTRATION_TOKEN is still set to the insecure default 'change-me-registration-token'");
            }
        }

        if config.require_ws_token && config.ws_token.is_none() {
            bail!("ATC_WS_TOKEN must be set when ATC_REQUIRE_WS_TOKEN is enabled");
        }
        if let Some(token) = config.ws_token.as_deref() {
            if token == "change-me-ws-token" {
                bail!("ATC_WS_TOKEN is still set to the insecure default 'change-me-ws-token'");
            }
        }
    }

    // Initialize database
    tracing::info!("Initializing database: {}", config.database_path);
    let db =
        persistence::init_database(&config.database_path, config.database_max_connections).await?;
    tracing::info!("Database initialized successfully");

    // Create application state with database
    let state = Arc::new(AppState::with_database(db, config.clone()));
    state.set_rid_view_bbox(config.rid_view_bbox.clone());
    state.load_from_database().await?;

    // Log security config
    tracing::info!("Admin token loaded");
    tracing::info!(
        "Registration token required: {}",
        config.require_registration_token
    );
    tracing::info!(
        "Rate limiting: {} ({} rps)",
        config.rate_limit_enabled,
        config.rate_limit_rps
    );
    tracing::info!("CORS origins: {:?}", config.allowed_origins);

    let (shutdown_tx, _) = broadcast::channel(1);

    if let (Some(db), Some(telemetry_rx)) =
        (state.database().cloned(), state.take_telemetry_receiver())
    {
        let telemetry_state = state.clone();
        let telemetry_shutdown = shutdown_tx.subscribe();
        tokio::spawn(async move {
            loops::telemetry_persist_loop::run_telemetry_persist_loop(
                db,
                telemetry_state,
                telemetry_rx,
                telemetry_shutdown,
            )
            .await;
        });
    }

    // Start background loops with supervision
    {
        let state = state.clone();
        let config = config.clone();
        spawn_supervised_loop("conflict", shutdown_tx.clone(), move |shutdown| {
            loops::conflict_loop::run_conflict_loop(state.clone(), config.clone(), shutdown)
        });
    }
    {
        let state = state.clone();
        let config = config.clone();
        spawn_supervised_loop("conformance", shutdown_tx.clone(), move |shutdown| {
            loops::conformance_loop::run_conformance_loop(state.clone(), config.clone(), shutdown)
        });
    }
    {
        let state = state.clone();
        spawn_supervised_loop("mission", shutdown_tx.clone(), move |shutdown| {
            loops::mission_loop::run_mission_loop(state.clone(), shutdown)
        });
    }
    {
        let state = state.clone();
        spawn_supervised_loop("oi-expiry", shutdown_tx.clone(), move |shutdown| {
            loops::operational_intent_expiry_loop::run_operational_intent_expiry_loop(
                state.clone(),
                shutdown,
            )
        });
    }
    {
        let state = state.clone();
        let config = config.clone();
        spawn_supervised_loop("rid", shutdown_tx.clone(), move |shutdown| {
            loops::rid_sync_loop::run_rid_loop(state.clone(), config.clone(), shutdown)
        });
    }
    {
        let state = state.clone();
        let config = config.clone();
        spawn_supervised_loop("geofence-sync", shutdown_tx.clone(), move |shutdown| {
            loops::geofence_sync_loop::run_geofence_sync_loop(
                state.clone(),
                config.clone(),
                shutdown,
            )
        });
    }
    {
        let state = state.clone();
        let config = config.clone();
        spawn_supervised_loop(
            "flight-declaration-sync",
            shutdown_tx.clone(),
            move |shutdown| {
                loops::flight_declaration_sync_loop::run_flight_declaration_sync_loop(
                    state.clone(),
                    config.clone(),
                    shutdown,
                )
            },
        );
    }
    {
        let state = state.clone();
        let config = config.clone();
        spawn_supervised_loop("blender-sync", shutdown_tx.clone(), move |shutdown| {
            loops::blender_sync_loop::run_blender_loop(state.clone(), config.clone(), shutdown)
        });
    }

    // Build CORS layer
    // Build the app
    let app = api::routes(&config)
        .route("/health", get(|| async { "OK" }))
        .route("/ready", get(ready_handler))
        .with_state(state);

    let app = if config.allowed_origins.is_empty() {
        tracing::warn!("No CORS origins configured - CORS disabled (same-origin only)");
        app
    } else {
        let origins: Vec<HeaderValue> = config
            .allowed_origins
            .iter()
            .filter_map(|o| o.parse().ok())
            .collect();
        app.layer(
            CorsLayer::new()
                .allow_origin(origins)
                .allow_methods([Method::GET, Method::POST, Method::PUT, Method::DELETE])
                .allow_headers(Any),
        )
    };

    // Run server
    let addr = SocketAddr::from(([0, 0, 0, 0], port));
    tracing::info!("Listening on {}", addr);

    if let (Some(cert_path), Some(key_path)) = (&config.tls_cert_path, &config.tls_key_path) {
        let tls_config = RustlsConfig::from_pem_file(cert_path, key_path).await?;
        let handle = axum_server::Handle::new();
        let server = axum_server::bind_rustls(addr, tls_config)
            .handle(handle.clone())
            .serve(app.into_make_service_with_connect_info::<SocketAddr>());
        let shutdown = shutdown_signal(shutdown_tx.clone());

        tokio::select! {
            result = server => result?,
            _ = shutdown => {
                handle.graceful_shutdown(Some(Duration::from_secs(10)));
            }
        }
    } else if config.require_tls {
        bail!("TLS required but ATC_TLS_CERT_PATH/ATC_TLS_KEY_PATH not set");
    } else {
        let listener = tokio::net::TcpListener::bind(addr).await?;
        axum::serve(
            listener,
            app.into_make_service_with_connect_info::<SocketAddr>(),
        )
        .with_graceful_shutdown(shutdown_signal(shutdown_tx.clone()))
        .await?;
    }

    Ok(())
}

#[cfg(unix)]
async fn shutdown_signal(shutdown_tx: broadcast::Sender<()>) {
    match tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate()) {
        Ok(mut sigterm) => {
            tokio::select! {
                _ = tokio::signal::ctrl_c() => {},
                _ = sigterm.recv() => {},
            }
        }
        Err(err) => {
            tracing::warn!("Failed to install SIGTERM handler (CTRL-C only): {}", err);
            let _ = tokio::signal::ctrl_c().await;
        }
    }
    tracing::info!("Shutdown signal received");
    let _ = shutdown_tx.send(());
}

fn spawn_supervised_loop<F, Fut>(
    name: &'static str,
    shutdown_tx: broadcast::Sender<()>,
    make_future: F,
) where
    F: Fn(broadcast::Receiver<()>) -> Fut + Send + Sync + 'static,
    Fut: Future<Output = ()> + Send + 'static,
{
    tokio::spawn(async move {
        let mut shutdown_rx = shutdown_tx.subscribe();
        loop {
            let task_shutdown = shutdown_tx.subscribe();
            let handle = tokio::spawn(make_future(task_shutdown));
            tokio::pin!(handle);
            tokio::select! {
                biased;
                _ = shutdown_rx.recv() => {
                    handle.as_mut().abort();
                    break;
                }
                result = &mut handle => {
                    match result {
                        Ok(_) => tracing::warn!("{} loop exited unexpectedly", name),
                        Err(err) => tracing::error!("{} loop crashed: {}", name, err),
                    }
                }
            }

            if shutdown_rx.try_recv().is_ok() {
                break;
            }

            tracing::warn!("Restarting {} loop in 1s", name);
            tokio::time::sleep(Duration::from_secs(1)).await;
        }
    });
}

#[cfg(not(unix))]
async fn shutdown_signal(shutdown_tx: broadcast::Sender<()>) {
    let _ = tokio::signal::ctrl_c().await;
    tracing::info!("Shutdown signal received");
    let _ = shutdown_tx.send(());
}
