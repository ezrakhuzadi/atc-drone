//! API routes for the ATC server.

mod routes;
pub mod ws;

use axum::Router;

pub fn routes() -> Router<std::sync::Arc<crate::state::AppState>> {
    routes::create_router()
}
