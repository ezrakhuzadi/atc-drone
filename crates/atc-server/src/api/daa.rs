//! DAA (Detect and Avoid) advisory endpoints.

use axum::{
    extract::{Query, State},
    Json,
};
use serde::Deserialize;
use std::sync::Arc;

use atc_core::models::DaaAdvisory;

use crate::state::AppState;

#[derive(Debug, Deserialize)]
pub struct DaaQuery {
    /// Filter advisories by owner ID
    pub owner_id: Option<String>,
    /// Only return unresolved advisories
    pub active_only: Option<bool>,
}

pub async fn list_daa(
    State(state): State<Arc<AppState>>,
    Query(query): Query<DaaQuery>,
) -> Json<Vec<DaaAdvisory>> {
    let mut advisories = state.get_daa_advisories();

    if let Some(owner_id) = query.owner_id {
        advisories.retain(|advisory| advisory.owner_id.as_ref() == Some(&owner_id));
    }

    if query.active_only.unwrap_or(false) {
        advisories.retain(|advisory| !advisory.resolved);
    }

    advisories.sort_by(|a, b| b.updated_at.cmp(&a.updated_at));
    Json(advisories)
}
