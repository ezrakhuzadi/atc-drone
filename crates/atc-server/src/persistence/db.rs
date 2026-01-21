//! Database connection and initialization.

use anyhow::Result;
use sqlx::{sqlite::SqlitePoolOptions, Row, SqlitePool};
use std::path::Path;
use tracing::{info, warn};

/// Database connection wrapper.
#[derive(Clone)]
pub struct Database {
    pool: SqlitePool,
}

impl Database {
    /// Get the underlying connection pool.
    pub fn pool(&self) -> &SqlitePool {
        &self.pool
    }
}

/// Clear all persisted state (drones, geofences, flight plans, commands).
pub async fn clear_all(pool: &SqlitePool) -> Result<()> {
    let mut tx = pool.begin().await?;
    sqlx::query("DELETE FROM commands").execute(&mut *tx).await?;
    sqlx::query("DELETE FROM flight_plans").execute(&mut *tx).await?;
    sqlx::query("DELETE FROM geofences").execute(&mut *tx).await?;
    sqlx::query("DELETE FROM geofence_sync_state").execute(&mut *tx).await?;
    sqlx::query("DELETE FROM drone_tokens").execute(&mut *tx).await?;
    sqlx::query("DELETE FROM drones").execute(&mut *tx).await?;
    tx.commit().await?;
    Ok(())
}

/// Initialize the SQLite database.
/// 
/// Creates the database file if it doesn't exist, runs migrations,
/// and returns a connection pool.
pub async fn init_database(db_path: &str, max_connections: u32) -> Result<Database> {
    // Ensure parent directory exists
    if let Some(parent) = Path::new(db_path).parent() {
        std::fs::create_dir_all(parent)?;
    }
    
    // Create database URL
    let db_url = format!("sqlite:{}?mode=rwc", db_path);
    
    info!("Connecting to database: {}", db_path);
    
    // Create connection pool
    let pool = SqlitePoolOptions::new()
        .max_connections(max_connections)
        .connect(&db_url)
        .await?;
    
    // Run migrations
    run_migrations(&pool).await?;
    
    Ok(Database { pool })
}

/// Run database migrations.
async fn run_migrations(pool: &SqlitePool) -> Result<()> {
    // Read and execute the init migration
    let migration_sql = include_str!("../../migrations/001_init.sql");
    
    info!("Running database migrations...");
    
    // Split by semicolons and execute each statement
    for statement in migration_sql.split(';') {
        // Remove comment lines and trim whitespace
        let statement: String = statement
            .lines()
            .filter(|line| !line.trim().starts_with("--"))
            .collect::<Vec<_>>()
            .join("\n");
        let statement = statement.trim();
        if statement.is_empty() {
            continue;
        }
        
        if let Err(e) = sqlx::query(statement).execute(pool).await {
            let err_str = e.to_string();
            // "already exists" is expected on re-runs
            if err_str.contains("already exists") {
                continue;
            }
            // CREATE TABLE and CREATE INDEX should fail fast
            if statement.to_uppercase().starts_with("CREATE") {
                anyhow::bail!("Migration failed on CREATE statement: {}", e);
            }
            warn!("Migration statement failed: {}", e);
        }
    }

    ensure_flight_plan_columns(pool).await?;
    
    info!("Database migrations complete");
    Ok(())
}

async fn ensure_flight_plan_columns(pool: &SqlitePool) -> Result<()> {
    let rows = sqlx::query("PRAGMA table_info(flight_plans)")
        .fetch_all(pool)
        .await?;
    if rows.is_empty() {
        return Ok(());
    }

    let mut columns = std::collections::HashSet::new();
    for row in rows {
        let name: String = row.try_get("name")?;
        columns.insert(name);
    }

    let mut missing: Vec<(&str, &str)> = Vec::new();
    if !columns.contains("trajectory_log") {
        missing.push(("trajectory_log", "TEXT"));
    }
    if !columns.contains("metadata") {
        missing.push(("metadata", "TEXT"));
    }

    for (name, data_type) in missing {
        let statement = format!("ALTER TABLE flight_plans ADD COLUMN {} {}", name, data_type);
        if let Err(err) = sqlx::query(&statement).execute(pool).await {
            let err_str = err.to_string();
            if err_str.contains("duplicate column") {
                continue;
            }
            return Err(err.into());
        }
    }

    Ok(())
}


#[cfg(test)]
mod tests {
    use super::*;
    
    #[tokio::test]
    async fn test_init_database() {
        let db = init_database(":memory:", 1).await.unwrap();
        
        // Verify tables exist
        let result: (i32,) = sqlx::query_as("SELECT COUNT(*) FROM sqlite_master WHERE type='table' AND name='drones'")
            .fetch_one(db.pool())
            .await
            .unwrap();
        
        assert_eq!(result.0, 1);
    }
}
