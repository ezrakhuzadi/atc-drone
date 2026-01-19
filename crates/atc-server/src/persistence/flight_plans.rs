//! Flight plan persistence operations.

use anyhow::Result;
use sqlx::SqlitePool;
use atc_core::models::{FlightPlan, FlightStatus, Waypoint, FlightPlanMetadata, TrajectoryPoint};
use chrono::{DateTime, Utc};

/// Upsert a flight plan into the database.
pub async fn upsert_flight_plan(pool: &SqlitePool, plan: &FlightPlan) -> Result<()> {
    let waypoints_json = serde_json::to_string(&plan.waypoints)?;
    let trajectory_log_json = match &plan.trajectory_log {
        Some(log) => Some(serde_json::to_string(log)?),
        None => None,
    };
    let metadata_json = match &plan.metadata {
        Some(metadata) => Some(serde_json::to_string(metadata)?),
        None => None,
    };
    let status = format!("{:?}", plan.status);
    
    let (origin_lat, origin_lon) = plan.waypoints.first()
        .map(|w| (Some(w.lat), Some(w.lon)))
        .unwrap_or((None, None));
    
    let (dest_lat, dest_lon) = plan.waypoints.last()
        .map(|w| (Some(w.lat), Some(w.lon)))
        .unwrap_or((None, None));
    
    sqlx::query(
        r#"
        INSERT INTO flight_plans (
            flight_id, drone_id, owner_id,
            origin_lat, origin_lon, dest_lat, dest_lon,
            waypoints, trajectory_log, metadata,
            status, start_time, end_time, created_at
        )
        VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7, ?8, ?9, ?10, ?11, ?12, ?13, ?14)
        ON CONFLICT(flight_id) DO UPDATE SET
            drone_id = ?2, owner_id = ?3,
            origin_lat = ?4, origin_lon = ?5,
            dest_lat = ?6, dest_lon = ?7,
            waypoints = ?8, trajectory_log = ?9, metadata = ?10,
            status = ?11, start_time = ?12, end_time = ?13
        "#,
    )
    .bind(&plan.flight_id)
    .bind(&plan.drone_id)
    .bind(&plan.owner_id)
    .bind(origin_lat)
    .bind(origin_lon)
    .bind(dest_lat)
    .bind(dest_lon)
    .bind(&waypoints_json)
    .bind(&trajectory_log_json)
    .bind(&metadata_json)
    .bind(&status)
    .bind(plan.departure_time.to_rfc3339())
    .bind(plan.arrival_time.map(|t| t.to_rfc3339()))
    .bind(plan.created_at.to_rfc3339())
    .execute(pool)
    .await?;
    
    Ok(())
}

/// Load all flight plans from the database.
pub async fn load_all_flight_plans(pool: &SqlitePool) -> Result<Vec<FlightPlan>> {
    let rows = sqlx::query_as::<_, FlightPlanRow>(
        "SELECT flight_id, drone_id, owner_id, waypoints, trajectory_log, metadata, status, start_time, end_time, created_at FROM flight_plans"
    )
    .fetch_all(pool)
    .await?;
    
    rows.into_iter().map(|r| r.try_into()).collect()
}

/// Load flight plans by owner.
pub async fn load_flight_plans_by_owner(pool: &SqlitePool, owner_id: &str) -> Result<Vec<FlightPlan>> {
    let rows = sqlx::query_as::<_, FlightPlanRow>(
        "SELECT flight_id, drone_id, owner_id, waypoints, trajectory_log, metadata, status, start_time, end_time, created_at FROM flight_plans WHERE owner_id = ?1"
    )
    .bind(owner_id)
    .fetch_all(pool)
    .await?;
    
    rows.into_iter().map(|r| r.try_into()).collect()
}

/// Load a single flight plan by ID.
pub async fn load_flight_plan(pool: &SqlitePool, flight_id: &str) -> Result<Option<FlightPlan>> {
    let row = sqlx::query_as::<_, FlightPlanRow>(
        "SELECT flight_id, drone_id, owner_id, waypoints, trajectory_log, metadata, status, start_time, end_time, created_at FROM flight_plans WHERE flight_id = ?1"
    )
    .bind(flight_id)
    .fetch_optional(pool)
    .await?;
    
    match row {
        Some(r) => Ok(Some(r.try_into()?)),
        None => Ok(None),
    }
}

/// Delete a flight plan by ID.
pub async fn delete_flight_plan(pool: &SqlitePool, flight_id: &str) -> Result<bool> {
    let result = sqlx::query("DELETE FROM flight_plans WHERE flight_id = ?1")
        .bind(flight_id)
        .execute(pool)
        .await?;
    
    Ok(result.rows_affected() > 0)
}

// Internal row type for SQLx
#[derive(sqlx::FromRow)]
struct FlightPlanRow {
    flight_id: String,
    drone_id: String,
    owner_id: Option<String>,
    waypoints: String,
    trajectory_log: Option<String>,
    metadata: Option<String>,
    status: String,
    start_time: String,
    end_time: Option<String>,
    created_at: String,
}

impl TryFrom<FlightPlanRow> for FlightPlan {
    type Error = anyhow::Error;
    
    fn try_from(row: FlightPlanRow) -> Result<Self> {
        let status = match row.status.as_str() {
            "Pending" => FlightStatus::Pending,
            "Approved" => FlightStatus::Approved,
            "Active" => FlightStatus::Active,
            "Completed" => FlightStatus::Completed,
            "Rejected" => FlightStatus::Rejected,
            "Cancelled" => FlightStatus::Cancelled,
            _ => FlightStatus::Pending,
        };
        
        let waypoints: Vec<Waypoint> = serde_json::from_str(&row.waypoints)?;
        let trajectory_log: Option<Vec<TrajectoryPoint>> = match row.trajectory_log {
            Some(raw) => Some(serde_json::from_str(&raw)?),
            None => None,
        };
        let metadata: Option<FlightPlanMetadata> = match row.metadata {
            Some(raw) => Some(serde_json::from_str(&raw)?),
            None => None,
        };
        
        let departure_time = DateTime::parse_from_rfc3339(&row.start_time)
            .map(|dt| dt.with_timezone(&Utc))
            .unwrap_or_else(|_| Utc::now());
        
        let arrival_time = row.end_time.as_ref()
            .and_then(|s| DateTime::parse_from_rfc3339(s).ok())
            .map(|dt| dt.with_timezone(&Utc));
        
        let created_at = DateTime::parse_from_rfc3339(&row.created_at)
            .map(|dt| dt.with_timezone(&Utc))
            .unwrap_or_else(|_| Utc::now());
        
        Ok(FlightPlan {
            flight_id: row.flight_id,
            drone_id: row.drone_id,
            owner_id: row.owner_id,
            waypoints,
            trajectory_log,
            metadata,
            status,
            departure_time,
            arrival_time,
            created_at,
        })
    }
}
