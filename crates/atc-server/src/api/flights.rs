use crate::state::store::AppState;
use atc_core::models::{FlightPlan, FlightPlanRequest, FlightStatus};
use atc_core::routing::generate_random_route;
use axum::{extract::State, http::StatusCode, response::IntoResponse, Json};
use chrono::Utc;
use std::sync::Arc;
use uuid::Uuid;

pub async fn create_flight_plan(
    State(state): State<Arc<AppState>>,
    Json(payload): Json<FlightPlanRequest>,
) -> impl IntoResponse {
    let flight_id = Uuid::new_v4().to_string();
    
    // Generate route if not provided
    let waypoints = if let Some(wp) = payload.waypoints {
        wp
    } else {
        // TODO: Use origin/dest if provided to generate route options
        // For now, simple random route for simulation
        generate_random_route()
    };
    
    let plan = FlightPlan {
        flight_id: flight_id.clone(),
        drone_id: payload.drone_id,
        waypoints,
        status: FlightStatus::Pending, // Starts as pending
        departure_time: payload.departure_time.unwrap_or_else(Utc::now),
        arrival_time: None, // Could estimate based on speed/dist
        created_at: Utc::now(),
    };
    
    state.add_flight_plan(plan.clone());
    
    // In a real system, we might trigger strategic deconfliction here
    
    (StatusCode::CREATED, Json(plan))
}

pub async fn get_flight_plans(State(state): State<Arc<AppState>>) -> impl IntoResponse {
    let plans = state.get_flight_plans();
    Json(plans)
}
