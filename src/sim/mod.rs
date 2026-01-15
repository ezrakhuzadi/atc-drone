//! Simulation module for drone telemetry.
//!
//! Provides flight paths, scenarios, and HTTP client for sending
//! telemetry to Flight Blender.

mod client;
mod paths;
mod scenarios;

pub use client::BlenderClient;
pub use paths::{CircularPath, FlightPath, LinearPath};
pub use scenarios::{
    create_converging_scenario, create_crossing_scenario, create_parallel_scenario, Scenario,
};
