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
    
    // Gather existing active plans for deconfliction
    let existing_plans = state.get_flight_plans();
    let active_plans: Vec<FlightPlan> = existing_plans
        .into_iter()
        .filter(|p| p.status == FlightStatus::Approved || p.status == FlightStatus::Active)
        .collect();

    // Determine candidate routes
    let candidates = if let Some(wp) = payload.waypoints {
         // User provided specific route - wrap as single option
         vec![atc_core::routing::RouteOption {
             option_id: "user".to_string(),
             name: "User Defined".to_string(),
             description: "Custom route".to_string(),
             waypoints: wp,
             estimated_duration_secs: 0,
             conflict_risk: atc_core::routing::ConflictRisk::High,
         }]
    } else if let (Some(origin), Some(dest)) = (payload.origin, payload.destination) {
        atc_core::routing::generate_route_options(origin, dest, 50.0)
    } else {
        // Fallback to random
        vec![atc_core::routing::RouteOption {
            option_id: "random".to_string(),
            name: "Random".to_string(),
            description: "Randomly generated".to_string(),
            waypoints: generate_random_route(),
            estimated_duration_secs: 0,
            conflict_risk: atc_core::routing::ConflictRisk::High,
        }]
    };

    // Select the first safe route
    let mut selected_waypoints = None;
    let mut flight_status = FlightStatus::Rejected;

    for option in &candidates {
        // Create a temp plan to test against
        let test_plan = FlightPlan {
            flight_id: flight_id.clone(),
            drone_id: payload.drone_id.clone(),
            waypoints: option.waypoints.clone(),
            status: FlightStatus::Pending,
            departure_time: payload.departure_time.unwrap_or_else(Utc::now),
            arrival_time: None,
            created_at: Utc::now(),
        };

        let has_conflict = active_plans.iter().any(|existing| {
            atc_core::spatial::check_plan_conflict(&test_plan, existing)
        });

        if !has_conflict {
            selected_waypoints = Some(option.waypoints.clone());
            flight_status = FlightStatus::Approved;
            break; // Found a good one!
        }
    }
    
    // If approved, use selected. If rejected, use first option (but mark rejected) or empty?
    // We should probably return the rejected plan so user sees why (or which path failed).
    let final_waypoints = selected_waypoints.unwrap_or_else(|| {
        // If rejected, just take the first candidate to show what failed
        candidates.first().map(|c| c.waypoints.clone()).unwrap_or_default()
    });

    let plan = FlightPlan {
        flight_id: flight_id.clone(),
        drone_id: payload.drone_id,
        waypoints: final_waypoints,
        status: flight_status,
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
