//! ATC Blender - Flight Blender API client
//!
//! Handles all communication with the Flight Blender UTM backend.

pub mod client;
pub mod sync_geofences;

pub use client::BlenderClient;
pub use sync_geofences::{ConflictGeofence, conflict_payload, conflict_to_geofence};
