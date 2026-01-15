//! ATC Server - Always-on backend for drone traffic management

mod api;
mod state;
mod loops;
mod config;

use anyhow::Result;
use axum::routing::get;
use std::net::SocketAddr;
use tower_http::cors::CorsLayer;
use tracing_subscriber::{layer::SubscriberExt, util::SubscriberInitExt};

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
    let state = Arc::new(AppState::new());
    
    // Start background loops
    tokio::spawn(loops::conflict_loop::run_conflict_loop(state.clone(), config.clone()));
    tokio::spawn(loops::blender_sync_loop::run_blender_loop(state.clone(), config));

    // Build the app
    let app = api::routes()
        .route("/health", get(|| async { "OK" }))
        .route("/v1/stream", get(api::ws::ws_handler))
        .with_state(state) // Inject state into all routes
        .layer(CorsLayer::permissive());

    // Run server
    let addr = SocketAddr::from(([0, 0, 0, 0], port));
    tracing::info!("Listening on {}", addr);

    let listener = tokio::net::TcpListener::bind(addr).await?;
    axum::serve(listener, app).await?;

    Ok(())
}
