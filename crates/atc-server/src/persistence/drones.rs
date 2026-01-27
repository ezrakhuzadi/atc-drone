//! Drone persistence operations.

use anyhow::Result;
use atc_core::models::{DroneState, DroneStatus};
use chrono::{DateTime, Utc};
use sqlx::{Sqlite, SqlitePool};

/// Upsert a drone state into the database.
pub async fn upsert_drone(pool: &SqlitePool, drone: &DroneState) -> Result<()> {
    sqlx::query(
        r#"
        INSERT INTO drones (drone_id, owner_id, lat, lon, altitude_m, heading_deg, speed_mps, velocity_x, velocity_y, velocity_z, status, last_update)
        VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7, ?8, ?9, ?10, ?11, ?12)
        ON CONFLICT(drone_id) DO UPDATE SET
            owner_id = COALESCE(?2, owner_id),
            lat = ?3, lon = ?4, altitude_m = ?5,
            heading_deg = ?6, speed_mps = ?7,
            velocity_x = ?8, velocity_y = ?9, velocity_z = ?10,
            status = ?11, last_update = ?12
        "#,
    )
    .bind(&drone.drone_id)
    .bind(&drone.owner_id)
    .bind(drone.lat)
    .bind(drone.lon)
    .bind(drone.altitude_m)
    .bind(drone.heading_deg)
    .bind(drone.speed_mps)
    .bind(drone.velocity_x)
    .bind(drone.velocity_y)
    .bind(drone.velocity_z)
    .bind(format!("{:?}", drone.status))
    .bind(drone.last_update.to_rfc3339())
    .execute(pool)
    .await?;

    Ok(())
}

/// Upsert a drone state into the database within an existing transaction.
pub async fn upsert_drone_tx(
    tx: &mut sqlx::Transaction<'_, Sqlite>,
    drone: &DroneState,
) -> Result<()> {
    sqlx::query(
        r#"
        INSERT INTO drones (drone_id, owner_id, lat, lon, altitude_m, heading_deg, speed_mps, velocity_x, velocity_y, velocity_z, status, last_update)
        VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7, ?8, ?9, ?10, ?11, ?12)
        ON CONFLICT(drone_id) DO UPDATE SET
            owner_id = COALESCE(?2, owner_id),
            lat = ?3, lon = ?4, altitude_m = ?5,
            heading_deg = ?6, speed_mps = ?7,
            velocity_x = ?8, velocity_y = ?9, velocity_z = ?10,
            status = ?11, last_update = ?12
        "#,
    )
    .bind(&drone.drone_id)
    .bind(&drone.owner_id)
    .bind(drone.lat)
    .bind(drone.lon)
    .bind(drone.altitude_m)
    .bind(drone.heading_deg)
    .bind(drone.speed_mps)
    .bind(drone.velocity_x)
    .bind(drone.velocity_y)
    .bind(drone.velocity_z)
    .bind(format!("{:?}", drone.status))
    .bind(drone.last_update.to_rfc3339())
    .execute(&mut **tx)
    .await?;

    Ok(())
}

/// Load all drones from the database.
pub async fn load_all_drones(pool: &SqlitePool) -> Result<Vec<DroneState>> {
    let rows = sqlx::query_as::<_, DroneRow>(
        "SELECT drone_id, owner_id, lat, lon, altitude_m, heading_deg, speed_mps, velocity_x, velocity_y, velocity_z, status, last_update FROM drones"
    )
    .fetch_all(pool)
    .await?;

    Ok(rows.into_iter().map(|r| r.into()).collect())
}

// Internal row type for SQLx
#[derive(sqlx::FromRow)]
struct DroneRow {
    drone_id: String,
    owner_id: Option<String>,
    lat: f64,
    lon: f64,
    altitude_m: f64,
    heading_deg: f64,
    speed_mps: f64,
    velocity_x: f64,
    velocity_y: f64,
    velocity_z: f64,
    status: String,
    last_update: String,
}

impl From<DroneRow> for DroneState {
    fn from(row: DroneRow) -> Self {
        let status = match row.status.as_str() {
            "Active" => DroneStatus::Active,
            "Holding" => DroneStatus::Holding,
            "Lost" => DroneStatus::Lost,
            _ => DroneStatus::Inactive,
        };

        let last_update = DateTime::parse_from_rfc3339(&row.last_update)
            .map(|dt| dt.with_timezone(&Utc))
            .unwrap_or_else(|_| Utc::now());

        DroneState {
            drone_id: row.drone_id,
            owner_id: row.owner_id,
            lat: row.lat,
            lon: row.lon,
            altitude_m: row.altitude_m,
            heading_deg: row.heading_deg,
            speed_mps: row.speed_mps,
            velocity_x: row.velocity_x,
            velocity_y: row.velocity_y,
            velocity_z: row.velocity_z,
            status,
            last_update,
        }
    }
}
