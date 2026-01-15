//! In-memory state store using DashMap.

use atc_core::models::{DroneState, Telemetry};
use atc_core::{Conflict, ConflictDetector};
use dashmap::DashMap;
use std::sync::atomic::{AtomicU32, Ordering};

/// Application state - thread-safe store for drones and conflicts.
pub struct AppState {
    drones: DashMap<String, DroneState>,
    detector: std::sync::Mutex<ConflictDetector>,
    conflicts: DashMap<String, Conflict>,
    drone_counter: AtomicU32,
}

impl AppState {
    pub fn new() -> Self {
        Self {
            drones: DashMap::new(),
            detector: std::sync::Mutex::new(ConflictDetector::default()),
            conflicts: DashMap::new(),
            drone_counter: AtomicU32::new(1),
        }
    }

    /// Get next drone ID number.
    pub fn next_drone_id(&self) -> u32 {
        self.drone_counter.fetch_add(1, Ordering::SeqCst)
    }

    /// Register a new drone.
    pub fn register_drone(&self, drone_id: &str) {
        // Pre-register with no position
    }

    /// Update drone state from telemetry.
    pub fn update_telemetry(&self, telemetry: Telemetry) {
        let drone_id = telemetry.drone_id.clone();

        // Update or create drone state
        self.drones
            .entry(drone_id.clone())
            .and_modify(|state| state.update(&telemetry))
            .or_insert_with(|| DroneState::from_telemetry(&telemetry));

        // Update conflict detector
        if let Ok(mut detector) = self.detector.lock() {
            detector.update_position(atc_core::DronePosition::new(
                &drone_id,
                telemetry.lat,
                telemetry.lon,
                telemetry.altitude_m,
            ).with_velocity(telemetry.heading_deg, telemetry.speed_mps));

            // Run conflict detection
            let new_conflicts = detector.detect_conflicts();
            
            // Update conflicts map
            self.conflicts.clear();
            for conflict in new_conflicts {
                let key = format!("{}-{}", conflict.drone1_id, conflict.drone2_id);
                self.conflicts.insert(key, conflict);
            }
        }
    }

    /// Get all drone states.
    pub fn get_all_drones(&self) -> Vec<DroneState> {
        self.drones.iter().map(|r| r.value().clone()).collect()
    }

    /// Get current conflicts.
    pub fn get_conflicts(&self) -> Vec<Conflict> {
        self.conflicts.iter().map(|r| r.value().clone()).collect()
    }
}
