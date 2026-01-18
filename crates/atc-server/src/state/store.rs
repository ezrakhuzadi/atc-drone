//! In-memory state store using DashMap.

use atc_core::models::{Command, DroneState, FlightPlan, Telemetry, Geofence, ConformanceStatus, DaaAdvisory, DroneStatus};
use atc_core::rules::SafetyRules;
use atc_core::{Conflict, ConflictDetector};
use dashmap::DashMap;
use std::collections::VecDeque;
use std::sync::{atomic::{AtomicU32, Ordering}, RwLock};
use chrono::Utc;
use serde::{Deserialize, Serialize};

use tokio::sync::broadcast;

/// Application state - thread-safe store for drones and conflicts.
pub struct AppState {
    drones: DashMap<String, DroneState>,
    drone_owners: DashMap<String, String>,
    external_traffic: DashMap<String, ExternalTraffic>,
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
    /// Latest conformance status per drone
    conformance: DashMap<String, ConformanceStatus>,
    /// Latest DAA advisories per drone
    daa_advisories: DashMap<String, DaaAdvisory>,
    /// Current RID viewport (min_lat,min_lon,max_lat,max_lon)
    rid_view_bbox: RwLock<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExternalTraffic {
    pub traffic_id: String,
    pub source: String,
    pub lat: f64,
    pub lon: f64,
    pub altitude_m: f64,
    pub heading_deg: f64,
    pub speed_mps: f64,
    pub last_update: chrono::DateTime<chrono::Utc>,
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
            drone_owners: DashMap::new(),
            external_traffic: DashMap::new(),
            flight_plans: DashMap::new(),
            detector: std::sync::Mutex::new(detector),
            conflicts: DashMap::new(),
            commands: DashMap::new(),
            command_cooldowns: DashMap::new(),
            drone_counter: AtomicU32::new(1),
            tx,
            rules,
            geofences: DashMap::new(),
            conformance: DashMap::new(),
            daa_advisories: DashMap::new(),
            rid_view_bbox: RwLock::new(String::new()),
        }
    }

    /// Get the configured safety rules.
    #[allow(dead_code)] // Available for API endpoints
    pub fn rules(&self) -> &SafetyRules {
        &self.rules
    }

    /// Update the RID viewport used for DSS subscriptions.
    pub fn set_rid_view_bbox(&self, view: String) {
        if let Ok(mut guard) = self.rid_view_bbox.write() {
            *guard = view;
        }
    }

    /// Get the current RID viewport string.
    pub fn get_rid_view_bbox(&self) -> String {
        self.rid_view_bbox
            .read()
            .map(|guard| guard.clone())
            .unwrap_or_default()
    }

    /// Get next drone ID number.
    pub fn next_drone_id(&self) -> u32 {
        self.drone_counter.fetch_add(1, Ordering::SeqCst)
    }

    /// Register a new drone.
    pub fn register_drone(&self, drone_id: &str, owner_id: Option<String>) {
        let owner_for_state = owner_id
            .clone()
            .or_else(|| self.drone_owners.get(drone_id).map(|entry| entry.value().clone()));
        let now = Utc::now();

        self.drones
            .entry(drone_id.to_string())
            .or_insert_with(|| DroneState {
                drone_id: drone_id.to_string(),
                owner_id: owner_for_state.clone(),
                lat: 0.0,
                lon: 0.0,
                altitude_m: 0.0,
                heading_deg: 0.0,
                speed_mps: 0.0,
                velocity_x: 0.0,
                velocity_y: 0.0,
                velocity_z: 0.0,
                last_update: now,
                status: DroneStatus::Inactive,
            });

        if let Some(owner_id) = owner_id {
            self.drone_owners.insert(drone_id.to_string(), owner_id.clone());
            if let Some(mut entry) = self.drones.get_mut(drone_id) {
                if entry.owner_id.is_none() {
                    entry.owner_id = Some(owner_id);
                }
            }
        }
    }

    /// Update drone state from telemetry.
    pub fn update_telemetry(&self, telemetry: Telemetry) {
        let drone_id = telemetry.drone_id.clone();
        let mut telemetry = telemetry;
        let hold_active = self.has_active_hold_command(&drone_id);

        if telemetry.owner_id.is_none() {
            if let Some(owner_id) = self.drone_owners.get(&drone_id) {
                telemetry.owner_id = Some(owner_id.value().clone());
            }
        }

        if let Some(owner_id) = telemetry.owner_id.clone() {
            self.drone_owners.insert(drone_id.clone(), owner_id);
        }
        let mut updated_state = None;

        self.external_traffic.remove(&drone_id);

        // Update or create drone state
        self.drones
            .entry(drone_id.clone())
            .and_modify(|state| {
                state.update(&telemetry);
                if hold_active {
                    state.status = DroneStatus::Holding;
                }
                updated_state = Some(state.clone());
            })
            .or_insert_with(|| {
                let mut state = DroneState::from_telemetry(&telemetry);
                if hold_active {
                    state.status = DroneStatus::Holding;
                }
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

            self.update_conflicts_from_detector(&mut detector);
        }
    }

    /// Get all drone states.
    pub fn get_all_drones(&self) -> Vec<DroneState> {
        self.drones.iter().map(|r| r.value().clone()).collect()
    }

    /// Upsert external traffic and feed it into conflict detection.
    pub fn upsert_external_traffic(&self, traffic: ExternalTraffic) {
        let traffic_id = traffic.traffic_id.clone();
        self.external_traffic.insert(traffic_id.clone(), traffic.clone());

        if let Ok(mut detector) = self.detector.lock() {
            detector.update_position(
                atc_core::DronePosition::new(
                    &traffic_id,
                    traffic.lat,
                    traffic.lon,
                    traffic.altitude_m,
                ).with_velocity(traffic.heading_deg, traffic.speed_mps)
            );
            self.update_conflicts_from_detector(&mut detector);
        }
    }

    /// Get all external traffic tracks.
    pub fn get_external_traffic(&self) -> Vec<ExternalTraffic> {
        self.external_traffic.iter().map(|r| r.value().clone()).collect()
    }

    /// Remove stale external tracks and purge them from the conflict detector.
    pub fn purge_external_traffic(&self, max_age_secs: i64) -> Vec<String> {
        let now = Utc::now();
        let stale_ids: Vec<String> = self.external_traffic.iter()
            .filter_map(|entry| {
                let age = (now - entry.last_update).num_seconds();
                if age > max_age_secs {
                    Some(entry.key().clone())
                } else {
                    None
                }
            })
            .collect();

        for id in &stale_ids {
            self.external_traffic.remove(id);
        }

        if !stale_ids.is_empty() {
            if let Ok(mut detector) = self.detector.lock() {
                for id in &stale_ids {
                    detector.remove_drone(id);
                }
                self.update_conflicts_from_detector(&mut detector);
            }
        }

        stale_ids
    }

    /// Check if a drone ID belongs to external traffic.
    pub fn is_external_traffic(&self, drone_id: &str) -> bool {
        self.external_traffic.contains_key(drone_id)
    }

    fn update_conflicts_from_detector(&self, detector: &mut ConflictDetector) {
        let new_conflicts = detector.detect_conflicts();

        self.conflicts.clear();
        for conflict in new_conflicts {
            let key = format!("{}-{}", conflict.drone1_id, conflict.drone2_id);
            self.conflicts.insert(key, conflict);
        }
    }

    /// Get a single drone state by ID.
    pub fn get_drone(&self, drone_id: &str) -> Option<DroneState> {
        self.drones.get(drone_id).map(|entry| entry.value().clone())
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

    // ========== CONFORMANCE METHODS ==========

    /// Store the latest conformance status for a drone.
    pub fn set_conformance_status(&self, status: ConformanceStatus) {
        self.conformance.insert(status.drone_id.clone(), status);
    }

    /// Get conformance status for all drones.
    pub fn get_conformance_statuses(&self) -> Vec<ConformanceStatus> {
        self.conformance.iter().map(|r| r.value().clone()).collect()
    }

    /// Get conformance status for a single drone.
    pub fn get_conformance_status(&self, drone_id: &str) -> Option<ConformanceStatus> {
        self.conformance.get(drone_id).map(|r| r.value().clone())
    }

    // ========== DAA METHODS ==========

    /// Store or update a DAA advisory.
    pub fn set_daa_advisory(&self, advisory: DaaAdvisory) {
        self.daa_advisories
            .insert(advisory.advisory_id.clone(), advisory);
    }

    /// Mark a DAA advisory resolved.
    pub fn resolve_daa_advisory(&self, advisory_id: &str) {
        if let Some(mut entry) = self.daa_advisories.get_mut(advisory_id) {
            entry.resolved = true;
            entry.updated_at = chrono::Utc::now();
        }
    }

    /// Get all DAA advisories.
    pub fn get_daa_advisories(&self) -> Vec<DaaAdvisory> {
        self.daa_advisories.iter().map(|r| r.value().clone()).collect()
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

    /// Check if drone has an active HOLD or REROUTE command (is in controlled state).
    /// Only considers non-expired, non-acknowledged commands.
    pub fn has_active_command(&self, drone_id: &str) -> bool {
        let now = chrono::Utc::now();
        if let Some(queue) = self.commands.get(drone_id) {
            queue.iter().any(|cmd| {
                // Must be a control command (HOLD or REROUTE)
                let is_control = matches!(cmd.command_type, 
                    atc_core::models::CommandType::Hold { .. } | 
                    atc_core::models::CommandType::Reroute { .. }
                );
                // Must not be acknowledged
                let not_acked = !cmd.acknowledged;
                // Must not be expired
                let not_expired = cmd.expires_at
                    .map(|exp| exp > now)
                    .unwrap_or(true); // No expiry = never expires
                
                is_control && not_acked && not_expired
            })
        } else {
            false
        }
    }

    /// Check if drone has an active HOLD command (non-expired, non-acknowledged).
    pub fn has_active_hold_command(&self, drone_id: &str) -> bool {
        let now = chrono::Utc::now();
        if let Some(queue) = self.commands.get(drone_id) {
            queue.iter().any(|cmd| {
                let is_hold = matches!(cmd.command_type, atc_core::models::CommandType::Hold { .. });
                let not_acked = !cmd.acknowledged;
                let not_expired = cmd.expires_at
                    .map(|exp| exp > now)
                    .unwrap_or(true);
                is_hold && not_acked && not_expired
            })
        } else {
            false
        }
    }
    
    /// Purge expired commands from all queues.
    /// Should be called periodically (e.g., from conflict loop).
    pub fn purge_expired_commands(&self) -> usize {
        let now = chrono::Utc::now();
        let mut purged_count = 0;
        
        for mut entry in self.commands.iter_mut() {
            let queue = entry.value_mut();
            let before_len = queue.len();
            queue.retain(|cmd| {
                // Keep if no expiry or not yet expired
                cmd.expires_at
                    .map(|exp| exp > now)
                    .unwrap_or(true)
            });
            purged_count += before_len - queue.len();
        }
        
        if purged_count > 0 {
            tracing::debug!("Purged {} expired commands", purged_count);
        }
        purged_count
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

    /// Check if a point is inside any active geofence.
    #[allow(dead_code)] // Will be used for routing avoidance
    pub fn check_point_in_geofences(&self, lat: f64, lon: f64, altitude_m: f64) -> Vec<String> {
        self.geofences
            .iter()
            .filter(|e| e.value().active && e.value().contains_point(lat, lon, altitude_m))
            .map(|e| e.key().clone())
            .collect()
    }

    // ========== ADMIN METHODS ==========

    /// Clear all state for demo reset.
    /// This removes all drones, conflicts, commands, flight plans, and geofences.
    pub fn clear_all(&self) {
        self.drones.clear();
        self.drone_owners.clear();
        self.external_traffic.clear();
        self.conflicts.clear();
        self.commands.clear();
        self.command_cooldowns.clear();
        self.flight_plans.clear();
        self.geofences.clear();
        self.conformance.clear();
        self.daa_advisories.clear();
        
        // Reset the conflict detector
        if let Ok(mut detector) = self.detector.lock() {
            *detector = ConflictDetector::new(
                self.rules.lookahead_seconds,
                self.rules.min_horizontal_separation_m,
                self.rules.min_vertical_separation_m,
                self.rules.warning_multiplier,
            );
        }
        
        // Reset drone counter
        self.drone_counter.store(1, Ordering::SeqCst);
        
        tracing::info!("All state cleared for demo reset");
    }
}
