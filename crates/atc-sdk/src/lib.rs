//! ATC SDK - Drone integration library
//!
//! Provides a simple API for drones to connect to the ATC system.

pub mod client;
pub mod telemetry;
pub mod commands;

pub use client::AtcClient;
