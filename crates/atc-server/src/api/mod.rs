//! API routes for the ATC server.

mod routes;
mod ws;

use axum::Router;

pub fn routes() -> Router {
    routes::create_router()
}
