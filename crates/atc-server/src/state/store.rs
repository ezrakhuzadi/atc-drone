//! In-memory state store using DashMap.

use atc_core::models::{Command, DroneState, FlightPlan, Telemetry, Geofence};
use atc_core::rules::SafetyRules;
use atc_core::{Conflict, ConflictDetector};
use dashmap::DashMap;
use std::collections::VecDeque;
use std::sync::atomic::{AtomicU32, Ordering};

use tokio::sync::broadcast;

/// Application state - thread-safe store for drones and conflicts.
pub struct AppState {
    drones: DashMap<String, DroneState>,
    pub flight_plans: DashMap<String, FlightPlan>,
    detector: std::sync::Mutex<ConflictDetector>,
    conflicts: DashMap<String, Conflict>,
    /// Command queues per drone (FIFO)
    commands: DashMap<String, VecDeque<Command>>,
    /// Track recently issued commands to prevent spam
    command_cooldowns: DashMap<String, std::time::Instant>,
    drone_counter: AtomicU32,
    pub tx: broadcast::Sender<DroneState>, // For WS broadcasting
    /// Safety rules (for timeout checks, etc.)
    rules: SafetyRules,
    /// Geofences/No-fly zones
    geofences: DashMap<String, Geofence>,
}

impl AppState {
    /// Create new AppState with default safety rules.
    pub fn new() -> Self {
        Self::with_rules(SafetyRules::default())
    }

    /// Create new AppState with custom safety rules.
    pub fn with_rules(rules: SafetyRules) -> Self {
        let (tx, _) = broadcast::channel(100);
        
        // Create detector with configurable thresholds from rules
        let detector = ConflictDetector::new(
            rules.lookahead_seconds,
            rules.min_horizontal_separation_m,
            rules.min_vertical_separation_m,
            rules.warning_multiplier,
        );
        
        Self {
            drones: DashMap::new(),
            flight_plans: DashMap::new(),
            detector: std::sync::Mutex::new(detector),
            conflicts: DashMap::new(),
            commands: DashMap::new(),
            command_cooldowns: DashMap::new(),
            drone_counter: AtomicU32::new(1),
            tx,
            rules,
            geofences: DashMap::new(),
        }
    }

    /// Get the configured safety rules.
    #[allow(dead_code)] // Available for API endpoints
    pub fn rules(&self) -> &SafetyRules {
        &self.rules
    }

    /// Get next drone ID number.
    pub fn next_drone_id(&self) -> u32 {
        self.drone_counter.fetch_add(1, Ordering::SeqCst)
    }

    /// Register a new drone.
    pub fn register_drone(&self, _drone_id: &str) {
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

    /// Check for drones that haven't reported in and mark them as Lost.
    /// Returns the IDs of newly-lost drones.
    pub fn check_timeouts(&self) -> Vec<String> {
        use atc_core::models::DroneStatus;
        use chrono::Utc;
        
        let timeout_secs = self.rules.drone_timeout_secs as i64;
        let now = Utc::now();
        let mut lost_drones = Vec::new();
        
        for mut entry in self.drones.iter_mut() {
            let drone = entry.value_mut();
            
            // Skip if already marked lost or inactive
            if matches!(drone.status, DroneStatus::Lost | DroneStatus::Inactive) {
                continue;
            }
            
            // Check if last update is older than timeout
            let elapsed = (now - drone.last_update).num_seconds();
            if elapsed > timeout_secs {
                drone.status = DroneStatus::Lost;
                lost_drones.push(drone.drone_id.clone());
                
                // Remove from conflict detector (stale data)
                if let Ok(mut detector) = self.detector.lock() {
                    detector.remove_drone(&drone.drone_id);
                }
            }
        }
        
        lost_drones
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

    // ========== COMMAND MANAGEMENT ==========

    /// Enqueue a command for a drone.
    pub fn enqueue_command(&self, command: Command) {
        let drone_id = command.drone_id.clone();
        self.commands
            .entry(drone_id)
            .or_default()
            .push_back(command);
    }

    /// Check if a command was recently issued (cooldown check).
    pub fn can_issue_command(&self, drone_id: &str, cooldown_secs: u64) -> bool {
        if let Some(last_time) = self.command_cooldowns.get(drone_id) {
            last_time.elapsed().as_secs() >= cooldown_secs
        } else {
            true
        }
    }

    /// Mark that a command was just issued (start cooldown).
    pub fn mark_command_issued(&self, drone_id: &str) {
        self.command_cooldowns.insert(drone_id.to_string(), std::time::Instant::now());
    }

    /// Get the next pending command for a drone (does not remove it).
    pub fn peek_command(&self, drone_id: &str) -> Option<Command> {
        self.commands
            .get(drone_id)
            .and_then(|queue| queue.front().cloned())
    }

    /// Get and remove the next pending command for a drone.
    #[allow(dead_code)] // Will be used by SDK clients polling for commands
    pub fn pop_command(&self, drone_id: &str) -> Option<Command> {
        self.commands
            .get_mut(drone_id)
            .and_then(|mut queue| queue.pop_front())
    }

    /// Acknowledge a command by ID (removes it from queue).
    pub fn ack_command(&self, command_id: &str) -> bool {
        for mut entry in self.commands.iter_mut() {
            let queue = entry.value_mut();
            if let Some(pos) = queue.iter().position(|c| c.command_id == command_id) {
                queue.remove(pos);
                return true;
            }
        }
        false
    }

    /// Get all pending commands (for debugging/UI).
    pub fn get_all_pending_commands(&self) -> Vec<Command> {
        self.commands
            .iter()
            .flat_map(|entry| entry.value().iter().cloned().collect::<Vec<_>>())
            .collect()
    }

    // ========== GEOFENCE METHODS ==========

    /// Add a geofence.
    pub fn add_geofence(&self, geofence: Geofence) {
        self.geofences.insert(geofence.id.clone(), geofence);
    }

    /// Get all geofences.
    pub fn get_geofences(&self) -> Vec<Geofence> {
        self.geofences.iter().map(|e| e.value().clone()).collect()
    }

    /// Get a specific geofence by ID.
    pub fn get_geofence(&self, id: &str) -> Option<Geofence> {
        self.geofences.get(id).map(|e| e.value().clone())
    }

    /// Remove a geofence by ID.
    pub fn remove_geofence(&self, id: &str) -> bool {
        self.geofences.remove(id).is_some()
    }

    /// Check if a point is inside any active geofence.\n    #[allow(dead_code)] // Will be used for routing avoidance
    pub fn check_point_in_geofences(&self, lat: f64, lon: f64, altitude_m: f64) -> Vec<String> {
        self.geofences
            .iter()
            .filter(|e| e.value().active && e.value().contains_point(lat, lon, altitude_m))
            .map(|e| e.key().clone())
            .collect()
    }
}
