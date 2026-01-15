//! ATC Core - Pure logic for conflict detection and routing
//!
//! This crate contains the domain models and business logic
//! with NO networking dependencies.

pub mod conflict;
pub mod models;
pub mod routing;
pub mod rules;
pub mod spatial;

pub use conflict::{Conflict, ConflictDetector, ConflictSeverity, DronePosition};
pub use models::{Command, CommandType, DroneState, FlightPlan, FlightPlanRequest, FlightStatus, Telemetry};
