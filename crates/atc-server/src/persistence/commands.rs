//! Command persistence operations.

use anyhow::Result;
use sqlx::SqlitePool;
use atc_core::models::{Command, CommandType};
use chrono::{DateTime, Utc};

/// Insert a command into the database.
pub async fn insert_command(pool: &SqlitePool, cmd: &Command) -> Result<()> {
    let command_type_json = serde_json::to_string(&cmd.command_type)?;
    
    sqlx::query(
        r#"
        INSERT INTO commands (command_id, drone_id, command_type, issued_at, expires_at, acknowledged)
        VALUES (?1, ?2, ?3, ?4, ?5, ?6)
        ON CONFLICT(command_id) DO UPDATE SET
            acknowledged = ?6, acked_at = CASE WHEN ?6 = 1 THEN CURRENT_TIMESTAMP ELSE acked_at END
        "#,
    )
    .bind(&cmd.command_id)
    .bind(&cmd.drone_id)
    .bind(&command_type_json)
    .bind(cmd.issued_at.to_rfc3339())
    .bind(cmd.expires_at.map(|t| t.to_rfc3339()))
    .bind(cmd.acknowledged)
    .execute(pool)
    .await?;
    
    Ok(())
}

/// Mark a command as acknowledged.
pub async fn ack_command(pool: &SqlitePool, command_id: &str) -> Result<bool> {
    let result = sqlx::query(
        "UPDATE commands SET acknowledged = 1, acked_at = CURRENT_TIMESTAMP WHERE command_id = ?1"
    )
    .bind(command_id)
    .execute(pool)
    .await?;
    
    Ok(result.rows_affected() > 0)
}



/// Load all pending commands (for all drones).
pub async fn load_all_pending_commands(pool: &SqlitePool) -> Result<Vec<Command>> {
    let rows = sqlx::query_as::<_, CommandRow>(
        r#"
        SELECT command_id, drone_id, command_type, issued_at, expires_at, acknowledged
        FROM commands
        WHERE acknowledged = 0
        AND (expires_at IS NULL OR expires_at > datetime('now'))
        ORDER BY drone_id, issued_at ASC
        "#
    )
    .fetch_all(pool)
    .await?;
    
    rows.into_iter().map(|r| r.try_into()).collect()
}

/// Delete expired commands.
pub async fn delete_expired_commands(pool: &SqlitePool) -> Result<u64> {
    let result = sqlx::query(
        "DELETE FROM commands WHERE expires_at IS NOT NULL AND expires_at < datetime('now')"
    )
    .execute(pool)
    .await?;
    
    Ok(result.rows_affected())
}

/// Delete commands that never received an acknowledgement within the timeout.
pub async fn delete_stale_commands(pool: &SqlitePool, ack_timeout_secs: i64) -> Result<u64> {
    if ack_timeout_secs <= 0 {
        return Ok(0);
    }

    let modifier = format!("-{} seconds", ack_timeout_secs);
    let result = sqlx::query(
        "DELETE FROM commands WHERE acknowledged = 0 AND datetime(issued_at) < datetime('now', ?1)"
    )
    .bind(modifier)
    .execute(pool)
    .await?;

    Ok(result.rows_affected())
}



// Internal row type for SQLx
#[derive(sqlx::FromRow)]
struct CommandRow {
    command_id: String,
    drone_id: String,
    command_type: String,
    issued_at: String,
    expires_at: Option<String>,
    acknowledged: bool,
}

impl TryFrom<CommandRow> for Command {
    type Error = anyhow::Error;
    
    fn try_from(row: CommandRow) -> Result<Self> {
        let command_type: CommandType = serde_json::from_str(&row.command_type)?;
        
        let issued_at = DateTime::parse_from_rfc3339(&row.issued_at)
            .map(|dt| dt.with_timezone(&Utc))
            .unwrap_or_else(|_| Utc::now());
        
        let expires_at = row.expires_at.as_ref()
            .and_then(|s| DateTime::parse_from_rfc3339(s).ok())
            .map(|dt| dt.with_timezone(&Utc));
        
        Ok(Command {
            command_id: row.command_id,
            drone_id: row.drone_id,
            command_type,
            issued_at,
            expires_at,
            acknowledged: row.acknowledged,
        })
    }
}
