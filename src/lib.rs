//! ATC Drone System - Local traffic management for cooperative UAVs
//!
//! This crate provides conflict detection, telemetry simulation, and
//! JWT token generation for interacting with Flight Blender/OpenUTM.

pub mod auth;
pub mod conflict;
pub mod sim;

pub use auth::generate_dummy_token;
pub use conflict::{Conflict, ConflictDetector, ConflictSeverity, DronePosition};
pub use sim::{BlenderClient, CircularPath, FlightPath, LinearPath};
