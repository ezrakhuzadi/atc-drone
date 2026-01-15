//! Sync conflict volumes to Blender as geofences.
//!
//! When conflicts are detected, we can push them as temporary
//! geofences so they appear in Spotlight.

use anyhow::Result;
use atc_core::Conflict;

/// Placeholder for geofence sync.
/// TODO: Implement Blender geofence API.
pub async fn sync_conflict_geofences(_conflicts: &[Conflict]) -> Result<()> {
    // Will push conflict volumes as temporary geofences
    Ok(())
}
