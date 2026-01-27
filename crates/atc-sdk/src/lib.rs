//! ATC SDK - Drone integration library
//!
//! Provides a simple API for drones to connect to the ATC system.

pub mod client;
pub mod commands;
pub mod telemetry;

pub use atc_core::models::Telemetry;
pub use client::AtcClient;
