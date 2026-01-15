//! API routes for the ATC server.

pub mod commands;
pub mod flights;
mod routes;
pub mod ws;

use axum::Router;

pub fn routes() -> Router<std::sync::Arc<crate::state::AppState>> {
    routes::create_router()
}
