//! Geofence persistence operations.

use anyhow::Result;
use sqlx::SqlitePool;
use atc_core::models::{Geofence, GeofenceType};
use chrono::{DateTime, Utc};

/// Upsert a geofence into the database.
pub async fn upsert_geofence(pool: &SqlitePool, geofence: &Geofence) -> Result<()> {
    let polygon_json = serde_json::to_string(&geofence.polygon)?;
    let geofence_type = format!("{:?}", geofence.geofence_type);
    
    sqlx::query(
        r#"
        INSERT INTO geofences (id, name, geofence_type, vertices, lower_altitude_m, upper_altitude_m, active, updated_at)
        VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7, CURRENT_TIMESTAMP)
        ON CONFLICT(id) DO UPDATE SET
            name = ?2, geofence_type = ?3, vertices = ?4,
            lower_altitude_m = ?5, upper_altitude_m = ?6, active = ?7,
            updated_at = CURRENT_TIMESTAMP
        "#,
    )
    .bind(&geofence.id)
    .bind(&geofence.name)
    .bind(&geofence_type)
    .bind(&polygon_json)
    .bind(geofence.lower_altitude_m)
    .bind(geofence.upper_altitude_m)
    .bind(geofence.active)
    .execute(pool)
    .await?;
    
    Ok(())
}

/// Load all geofences from the database.
pub async fn load_all_geofences(pool: &SqlitePool) -> Result<Vec<Geofence>> {
    let rows = sqlx::query_as::<_, GeofenceRow>(
        "SELECT id, name, geofence_type, vertices, lower_altitude_m, upper_altitude_m, active, created_at FROM geofences"
    )
    .fetch_all(pool)
    .await?;
    
    rows.into_iter().map(|r| r.try_into()).collect()
}



/// Delete a geofence by ID.
pub async fn delete_geofence(pool: &SqlitePool, id: &str) -> Result<bool> {
    let result = sqlx::query("DELETE FROM geofences WHERE id = ?1")
        .bind(id)
        .execute(pool)
        .await?;
    
    Ok(result.rows_affected() > 0)
}

// Internal row type for SQLx
#[derive(sqlx::FromRow)]
struct GeofenceRow {
    id: String,
    name: String,
    geofence_type: String,
    vertices: String,
    lower_altitude_m: f64,
    upper_altitude_m: f64,
    active: bool,
    created_at: String,
}

impl TryFrom<GeofenceRow> for Geofence {
    type Error = anyhow::Error;
    
    fn try_from(row: GeofenceRow) -> Result<Self> {
        let geofence_type = match row.geofence_type.as_str() {
            "NoFlyZone" => GeofenceType::NoFlyZone,
            "RestrictedArea" => GeofenceType::RestrictedArea,
            "TemporaryRestriction" => GeofenceType::TemporaryRestriction,
            "Advisory" => GeofenceType::Advisory,
            _ => GeofenceType::NoFlyZone,
        };
        
        let polygon: Vec<[f64; 2]> = serde_json::from_str(&row.vertices)?;
        
        let created_at = DateTime::parse_from_rfc3339(&row.created_at)
            .map(|dt| dt.with_timezone(&Utc))
            .unwrap_or_else(|_| Utc::now());
        
        Ok(Geofence {
            id: row.id,
            name: row.name,
            geofence_type,
            polygon,
            lower_altitude_m: row.lower_altitude_m,
            upper_altitude_m: row.upper_altitude_m,
            active: row.active,
            created_at,
        })
    }
}

