//! Persistence layer for ATC Server.
//!
//! Provides SQLite-backed storage for drones, geofences, flight plans, and commands.
//! Uses write-through caching with DashMap for hot data access.

pub mod db;
pub mod drones;
pub mod geofences;
pub mod flight_plans;
pub mod commands;

pub use db::{Database, init_database};
