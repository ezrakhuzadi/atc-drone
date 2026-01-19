//! ATC Server - Always-on backend for drone traffic management

mod api;
mod compliance;
mod state;
mod loops;
mod config;
mod persistence;
mod route_planner;
mod terrain;

use anyhow::{Result, bail};
use axum::routing::get;
use std::net::SocketAddr;
use std::future::Future;
use std::time::Duration;
use tower_http::cors::{CorsLayer, Any};
use axum::http::{HeaderValue, Method};
use tracing_subscriber::{layer::SubscriberExt, util::SubscriberInitExt};
use axum_server::tls_rustls::RustlsConfig;
use tokio::sync::broadcast;

use crate::state::AppState;
use crate::config::Config;
use std::sync::Arc;

#[tokio::main]
async fn main() -> Result<()> {
    // Initialize tracing
    tracing_subscriber::registry()
        .with(tracing_subscriber::fmt::layer())
        .with(tracing_subscriber::EnvFilter::from_default_env()
            .add_directive("atc_server=debug".parse()?))
        .init();

    tracing::info!("Starting ATC Server...");

    let config = Config::from_env();
    let port = config.server_port;

    if config.require_registration_token && config.registration_token.is_none() {
        bail!("ATC_REGISTRATION_TOKEN is required when ATC_REQUIRE_REGISTRATION_TOKEN is enabled");
    }
    
    // Initialize database
    tracing::info!("Initializing database: {}", config.database_path);
    let db = persistence::init_database(
        &config.database_path,
        config.database_max_connections,
    ).await?;
    tracing::info!("Database initialized successfully");
    
    // Create application state with database
    let state = Arc::new(AppState::with_database(db, config.clone()));
    state.set_rid_view_bbox(config.rid_view_bbox.clone());
    state.load_from_database().await?;
    
    // Log security config
    tracing::info!("Admin token loaded");
    tracing::info!("Registration token required: {}", config.require_registration_token);
    tracing::info!("Rate limiting: {} ({} rps)", config.rate_limit_enabled, config.rate_limit_rps);
    tracing::info!("CORS origins: {:?}", config.allowed_origins);
    
    let (shutdown_tx, _) = broadcast::channel(1);

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
        let config = config.clone();
        spawn_supervised_loop("rid", shutdown_tx.clone(), move |shutdown| {
            loops::rid_sync_loop::run_rid_loop(state.clone(), config.clone(), shutdown)
        });
    }
    {
        let state = state.clone();
        let config = config.clone();
        spawn_supervised_loop("geofence-sync", shutdown_tx.clone(), move |shutdown| {
            loops::geofence_sync_loop::run_geofence_sync_loop(state.clone(), config.clone(), shutdown)
        });
    }
    {
        let state = state.clone();
        let config = config.clone();
        spawn_supervised_loop("flight-declaration-sync", shutdown_tx.clone(), move |shutdown| {
            loops::flight_declaration_sync_loop::run_flight_declaration_sync_loop(
                state.clone(),
                config.clone(),
                shutdown,
            )
        });
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
        .with_state(state);

    let app = if config.allowed_origins.is_empty() {
        tracing::warn!("No CORS origins configured - CORS disabled (same-origin only)");
        app
    } else {
        let origins: Vec<HeaderValue> = config.allowed_origins.iter()
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
            .serve(app.into_make_service());
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
        axum::serve(listener, app)
            .with_graceful_shutdown(shutdown_signal(shutdown_tx.clone()))
            .await?;
    }

    Ok(())
}

#[cfg(unix)]
async fn shutdown_signal(shutdown_tx: broadcast::Sender<()>) {
    let mut sigterm = tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate())
        .expect("failed to install SIGTERM handler");
    tokio::select! {
        _ = tokio::signal::ctrl_c() => {},
        _ = sigterm.recv() => {},
    }
    tracing::info!("Shutdown signal received");
    let _ = shutdown_tx.send(());
}

fn spawn_supervised_loop<F, Fut>(
    name: &'static str,
    shutdown_tx: broadcast::Sender<()>,
    make_future: F,
)
where
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
