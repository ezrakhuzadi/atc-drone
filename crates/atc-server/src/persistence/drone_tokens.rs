//! Drone session token persistence.

use anyhow::Result;
use sqlx::SqlitePool;

#[derive(sqlx::FromRow)]
pub struct DroneTokenRow {
    pub drone_id: String,
    pub session_token: String,
}

/// Upsert a drone session token.
#[allow(dead_code)]
pub async fn upsert_drone_token(pool: &SqlitePool, drone_id: &str, token: &str) -> Result<()> {
    sqlx::query(
        r#"
        INSERT INTO drone_tokens (drone_id, session_token, updated_at)
        VALUES (?1, ?2, CURRENT_TIMESTAMP)
        ON CONFLICT(drone_id) DO UPDATE SET
            session_token = ?2,
            updated_at = CURRENT_TIMESTAMP
        "#,
    )
    .bind(drone_id)
    .bind(token)
    .execute(pool)
    .await?;

    Ok(())
}

/// Load all persisted drone tokens.
pub async fn load_all_drone_tokens(pool: &SqlitePool) -> Result<Vec<DroneTokenRow>> {
    let rows =
        sqlx::query_as::<_, DroneTokenRow>("SELECT drone_id, session_token FROM drone_tokens")
            .fetch_all(pool)
            .await?;

    Ok(rows)
}
