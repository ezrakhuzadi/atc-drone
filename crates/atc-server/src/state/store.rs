//! In-memory state store using DashMap.

use atc_core::models::{DroneState, FlightPlan, Telemetry};
use atc_core::{Conflict, ConflictDetector};
use dashmap::DashMap;
use std::sync::atomic::{AtomicU32, Ordering};

use tokio::sync::broadcast;

/// Application state - thread-safe store for drones and conflicts.
pub struct AppState {
    drones: DashMap<String, DroneState>,
    pub flight_plans: DashMap<String, FlightPlan>,
    detector: std::sync::Mutex<ConflictDetector>,
    conflicts: DashMap<String, Conflict>,
    drone_counter: AtomicU32,
    pub tx: broadcast::Sender<DroneState>, // For WS broadcasting
}

impl AppState {
    pub fn new() -> Self {
        let (tx, _) = broadcast::channel(100);
        Self {
            drones: DashMap::new(),
            flight_plans: DashMap::new(),
            detector: std::sync::Mutex::new(ConflictDetector::default()),
            conflicts: DashMap::new(),
            drone_counter: AtomicU32::new(1),
            tx,
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
        let mut updated_state = None;

        // Update or create drone state
        self.drones
            .entry(drone_id.clone())
            .and_modify(|state| {
                state.update(&telemetry);
                updated_state = Some(state.clone());
            })
            .or_insert_with(|| {
                let state = DroneState::from_telemetry(&telemetry);
                updated_state = Some(state.clone());
                state
            });

        // Broadcast update via WebSocket
        if let Some(state) = updated_state {
            let _ = self.tx.send(state);
        }

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
    
    /// Get all flight plans
    pub fn get_flight_plans(&self) -> Vec<FlightPlan> {
        self.flight_plans.iter().map(|r| r.value().clone()).collect()
    }
    
    /// Add or update a flight plan
    pub fn add_flight_plan(&self, plan: FlightPlan) {
        self.flight_plans.insert(plan.flight_id.clone(), plan);
    }
}
