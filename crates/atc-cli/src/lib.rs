//! ATC CLI - Command line tools for the ATC drone system.
//!
//! This crate provides the original CLI binaries:
//! - generate_token: JWT token generator
//! - send_one_track: Single drone simulator
//! - send_multi_track: Multi-drone scenario simulator

pub mod auth;
pub mod sim;

pub use auth::generate_dummy_token;
