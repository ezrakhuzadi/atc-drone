//! API routes for the ATC server.

pub mod auth;
pub mod commands;
pub mod daa;
pub mod flights;
pub mod geofences;
mod routes;
pub mod ws;

use axum::Router;
use crate::config::Config;



pub fn routes(config: &Config) -> Router<std::sync::Arc<crate::state::AppState>> {
    routes::create_router(config)
}

#[cfg(test)]
mod tests;
