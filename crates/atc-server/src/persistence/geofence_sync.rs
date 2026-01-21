//! Persistence for Blender geofence sync state.

use anyhow::{Context, Result};
use sqlx::SqlitePool;

#[derive(Debug, Clone)]
pub struct GeofenceSyncState {
    pub local_id: String,
    pub blender_id: String,
    pub fingerprint: u64,
    pub expires_at: i64,
}

#[derive(sqlx::FromRow)]
struct GeofenceSyncRow {
    local_id: String,
    blender_id: String,
    fingerprint: String,
    expires_at: i64,
}

pub async fn load_geofence_sync_state(pool: &SqlitePool) -> Result<Vec<GeofenceSyncState>> {
    let rows = sqlx::query_as::<_, GeofenceSyncRow>(
        "SELECT local_id, blender_id, fingerprint, expires_at FROM geofence_sync_state",
    )
    .fetch_all(pool)
    .await?;

    let mut entries = Vec::with_capacity(rows.len());
    for row in rows {
        let fingerprint = row
            .fingerprint
            .parse::<u64>()
            .with_context(|| format!("Invalid geofence sync fingerprint for {}", row.local_id))?;
        entries.push(GeofenceSyncState {
            local_id: row.local_id,
            blender_id: row.blender_id,
            fingerprint,
            expires_at: row.expires_at,
        });
    }
    Ok(entries)
}

pub async fn replace_geofence_sync_state(
    pool: &SqlitePool,
    entries: &[GeofenceSyncState],
) -> Result<()> {
    let mut tx = pool.begin().await?;
    sqlx::query("DELETE FROM geofence_sync_state")
        .execute(&mut *tx)
        .await?;

    for entry in entries {
        sqlx::query(
            r#"
            INSERT INTO geofence_sync_state (local_id, blender_id, fingerprint, expires_at)
            VALUES (?1, ?2, ?3, ?4)
            "#,
        )
        .bind(&entry.local_id)
        .bind(&entry.blender_id)
        .bind(entry.fingerprint.to_string())
        .bind(entry.expires_at)
        .execute(&mut *tx)
        .await?;
    }

    tx.commit().await?;
    Ok(())
}
