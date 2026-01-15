//! ATC Server - Always-on backend for drone traffic management

mod api;
mod state;
mod loops;
mod config;

use anyhow::Result;
use axum::{routing::get, Router};
use std::net::SocketAddr;
use tower_http::cors::CorsLayer;
use tracing_subscriber::{layer::SubscriberExt, util::SubscriberInitExt};

#[tokio::main]
async fn main() -> Result<()> {
    // Initialize tracing
    tracing_subscriber::registry()
        .with(tracing_subscriber::fmt::layer())
        .with(tracing_subscriber::EnvFilter::from_default_env()
            .add_directive("atc_server=debug".parse()?))
        .init();

    tracing::info!("Starting ATC Server...");

    // Build the app
    let app = Router::new()
        .route("/health", get(|| async { "OK" }))
        .merge(api::routes())
        .layer(CorsLayer::permissive());

    // Run server
    let addr = SocketAddr::from(([0, 0, 0, 0], 3000));
    tracing::info!("Listening on {}", addr);

    let listener = tokio::net::TcpListener::bind(addr).await?;
    axum::serve(listener, app).await?;

    Ok(())
}
