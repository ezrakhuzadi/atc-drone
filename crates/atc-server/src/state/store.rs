//! In-memory state store using DashMap.

use anyhow::Result;
use atc_core::models::{
    Command, ConformanceStatus, DaaAdvisory, DroneState, DroneStatus, FlightPlan, Geofence,
    Telemetry,
};
use atc_core::rules::SafetyRules;
use atc_core::{Conflict, ConflictDetector, DronePosition};
use chrono::{DateTime, Duration as ChronoDuration, Utc};
use dashmap::DashMap;
use serde::{Deserialize, Serialize};
use std::collections::{HashMap, HashSet, VecDeque};
use std::sync::{
    atomic::{AtomicU32, AtomicU64, Ordering},
    Arc,
    RwLock,
};

use crate::altitude::altitude_to_amsl;
use crate::config::Config;
use crate::persistence::db as db_persistence;
use crate::persistence::{
    commands as commands_db, drone_tokens as drone_tokens_db, drones as drones_db,
    flight_plans as flight_plans_db, geofences as geofences_db, Database,
};
use tokio::sync::{broadcast, mpsc, Mutex};

const TELEMETRY_QUEUE_DEPTH: usize = 4096;
const DETECTOR_QUEUE_DEPTH: usize = 4096;
const DETECTOR_QUEUE_WARN_INTERVAL_SECS: u64 = 5;
const STATE_CAP_WARN_INTERVAL_SECS: u64 = 10;

/// Application state - thread-safe store for drones and conflicts.
pub struct AppState {
    drones: DashMap<String, DroneState>,
    drone_owners: DashMap<String, String>,
    drone_tokens: DashMap<String, String>,
    external_traffic: DashMap<String, ExternalTraffic>,
    external_traffic_cap_warn_last: AtomicU64,
    pub flight_plans: DashMap<String, FlightPlan>,
    flight_plan_booking_lock: Mutex<()>,
    detector: std::sync::Mutex<ConflictDetector>,
    detector_tx: mpsc::Sender<DetectorUpdate>,
    detector_rx: Mutex<mpsc::Receiver<DetectorUpdate>>,
    detector_overflow: Mutex<HashMap<String, DetectorUpdate>>,
    detector_queue_warn_last: AtomicU64,
    detector_overflow_warn_last: AtomicU64,
    conflicts: DashMap<String, Conflict>,
    /// Command queues per drone (FIFO)
    commands: DashMap<String, VecDeque<Command>>,
    /// Track recently issued commands to prevent spam
    command_cooldowns: DashMap<String, std::time::Instant>,
    /// Track active HOLD commands after acknowledgment
    active_holds: DashMap<String, DateTime<Utc>>,
    drone_counter: AtomicU32,
    pub tx: broadcast::Sender<DroneState>, // For WS broadcasting
    command_tx: broadcast::Sender<Command>,
    /// Telemetry persistence queue (coalesced in background).
    telemetry_tx: mpsc::Sender<DroneState>,
    telemetry_rx: std::sync::Mutex<Option<mpsc::Receiver<DroneState>>>,
    telemetry_overflow: std::sync::Mutex<HashMap<String, DroneState>>,
    telemetry_overflow_warn_last: AtomicU64,
    /// Safety rules (for timeout checks, etc.)
    rules: SafetyRules,
    /// Geofences/No-fly zones
    geofences: DashMap<String, Geofence>,
    /// External geofences pulled from Blender/DSS (not persisted)
    external_geofences: DashMap<String, Geofence>,
    /// Conflict geofence IDs pushed to Blender (avoid re-ingest)
    conflict_geofences: DashMap<String, i64>,
    /// Latest conformance status per drone
    conformance: DashMap<String, ConformanceStatus>,
    /// Latest DAA advisories per drone
    daa_advisories: DashMap<String, DaaAdvisory>,
    /// Current RID viewport (min_lat,min_lon,max_lat,max_lon)
    rid_view_bbox: RwLock<String>,
    /// SQLite database for persistence (optional for backwards compat)
    database: Option<Database>,
    /// Server configuration (for compliance lookups, etc.)
    config: Config,
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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RegisterDroneOutcome {
    Registered,
    AlreadyRegistered,
}

#[derive(Debug)]
enum DetectorUpdate {
    Upsert(DronePosition),
    Remove(String),
}

impl DetectorUpdate {
    fn key(&self) -> &str {
        match self {
            DetectorUpdate::Upsert(position) => &position.drone_id,
            DetectorUpdate::Remove(drone_id) => drone_id,
        }
    }
}

impl AppState {
    /// Create new AppState with database for persistence.
    pub fn with_database(db: Database, config: Config) -> Self {
        let rules = config.safety_rules();
        let mut state = Self::with_rules_and_config(rules, config);
        state.database = Some(db);
        state
    }

    /// Create new AppState with custom safety rules.

    /// Create new AppState with custom rules and config.
    pub fn with_rules_and_config(rules: SafetyRules, config: Config) -> Self {
        let (tx, _) = broadcast::channel(100);
        let (command_tx, _) = broadcast::channel(100);
        let (telemetry_tx, telemetry_rx) = mpsc::channel(TELEMETRY_QUEUE_DEPTH);
        let (detector_tx, detector_rx) = mpsc::channel(DETECTOR_QUEUE_DEPTH);

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
            drone_tokens: DashMap::new(),
            external_traffic: DashMap::new(),
            external_traffic_cap_warn_last: AtomicU64::new(0),
            flight_plans: DashMap::new(),
            flight_plan_booking_lock: Mutex::new(()),
            detector: std::sync::Mutex::new(detector),
            detector_tx,
            detector_rx: Mutex::new(detector_rx),
            detector_overflow: Mutex::new(HashMap::new()),
            detector_queue_warn_last: AtomicU64::new(0),
            detector_overflow_warn_last: AtomicU64::new(0),
            conflicts: DashMap::new(),
            commands: DashMap::new(),
            command_cooldowns: DashMap::new(),
            active_holds: DashMap::new(),
            drone_counter: AtomicU32::new(1),
            tx,
            command_tx,
            telemetry_tx,
            telemetry_rx: std::sync::Mutex::new(Some(telemetry_rx)),
            telemetry_overflow: std::sync::Mutex::new(HashMap::new()),
            telemetry_overflow_warn_last: AtomicU64::new(0),
            rules,
            geofences: DashMap::new(),
            external_geofences: DashMap::new(),
            conflict_geofences: DashMap::new(),
            conformance: DashMap::new(),
            daa_advisories: DashMap::new(),
            rid_view_bbox: RwLock::new(String::new()),
            database: None,
            config,
        }
    }

    /// Get database reference (if available).
    #[allow(dead_code)]
    pub fn database(&self) -> Option<&Database> {
        self.database.as_ref()
    }

    /// Take the telemetry receiver for the persistence loop.
    pub fn take_telemetry_receiver(&self) -> Option<mpsc::Receiver<DroneState>> {
        if let Ok(mut guard) = self.telemetry_rx.lock() {
            guard.take()
        } else {
            None
        }
    }

    /// Drain any telemetry snapshots that couldn't fit in the queue.
    pub fn take_telemetry_overflow(&self) -> HashMap<String, DroneState> {
        if let Ok(mut guard) = self.telemetry_overflow.lock() {
            std::mem::take(&mut *guard)
        } else {
            HashMap::new()
        }
    }

    fn store_telemetry_overflow(&self, state: DroneState) {
        if self.config.max_overflow_entries == 0 {
            return;
        }
        if let Ok(mut guard) = self.telemetry_overflow.lock() {
            if guard.len() >= self.config.max_overflow_entries
                && !guard.contains_key(&state.drone_id)
            {
                self.warn_state_cap(
                    &self.telemetry_overflow_warn_last,
                    "Telemetry overflow cap reached; dropping snapshots",
                );
                return;
            }
            guard.insert(state.drone_id.clone(), state);
        }
    }

    /// Load persisted state from the database into memory.
    pub async fn load_from_database(&self) -> Result<()> {
        let Some(db) = self.database.clone() else {
            return Ok(());
        };

        let pool = db.pool().clone();

        self.drones.clear();
        self.drone_owners.clear();
        self.drone_tokens.clear();
        self.flight_plans.clear();
        self.geofences.clear();
        self.commands.clear();
        self.active_holds.clear();

        let drones = drones_db::load_all_drones(&pool).await?;
        for drone in drones {
            if let Some(owner_id) = drone.owner_id.clone() {
                self.drone_owners.insert(drone.drone_id.clone(), owner_id);
            }
            self.drones.insert(drone.drone_id.clone(), drone);
        }

        self.seed_drone_counter();

        if let Ok(mut detector) = self.detector.lock() {
            for drone in self.drones.iter() {
                let drone = drone.value();
                detector.update_position(
                    atc_core::DronePosition::new(
                        &drone.drone_id,
                        drone.lat,
                        drone.lon,
                        drone.altitude_m,
                    )
                    .with_velocity(
                        drone.heading_deg,
                        drone.speed_mps,
                        drone.velocity_z,
                    ),
                );
            }
            self.update_conflicts_from_detector(&mut detector);
        }

        let geofences = geofences_db::load_all_geofences(&pool).await?;
        for geofence in geofences {
            self.geofences.insert(geofence.id.clone(), geofence);
        }

        let plans = flight_plans_db::load_all_flight_plans(&pool).await?;
        for plan in plans {
            self.flight_plans.insert(plan.flight_id.clone(), plan);
        }

        let pending_commands = commands_db::load_all_pending_commands(&pool).await?;
        for command in pending_commands {
            self.commands
                .entry(command.drone_id.clone())
                .or_default()
                .push_back(command);
        }

        let tokens = drone_tokens_db::load_all_drone_tokens(&pool).await?;
        for token in tokens {
            self.drone_tokens
                .insert(token.drone_id, token.session_token);
        }

        Ok(())
    }

    fn queue_telemetry_persist(&self, state: DroneState) {
        match self.telemetry_tx.try_send(state) {
            Ok(_) => {}
            Err(mpsc::error::TrySendError::Full(state)) => {
                tracing::warn!("Telemetry persistence queue full; stashing latest snapshot");
                self.store_telemetry_overflow(state);
            }
            Err(mpsc::error::TrySendError::Closed(state)) => {
                tracing::warn!("Telemetry persistence queue closed; stashing latest snapshot");
                self.store_telemetry_overflow(state);
            }
        }
    }

    async fn queue_detector_update(&self, update: DetectorUpdate) {
        let key = update.key().to_string();
        match self.detector_tx.try_send(update) {
            Ok(_) => {
                self.detector_overflow.lock().await.remove(&key);
            }
            Err(mpsc::error::TrySendError::Full(update)) => {
                self.warn_detector_backpressure("full");
                self.store_detector_overflow(key, update).await;
            }
            Err(mpsc::error::TrySendError::Closed(update)) => {
                self.warn_detector_backpressure("closed");
                self.store_detector_overflow(key, update).await;
            }
        }
    }

    fn warn_detector_backpressure(&self, reason: &'static str) {
        let Ok(now) = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH) else {
            return;
        };
        let now_secs = now.as_secs();

        let last = self.detector_queue_warn_last.load(Ordering::Relaxed);
        if now_secs.saturating_sub(last) < DETECTOR_QUEUE_WARN_INTERVAL_SECS {
            return;
        }

        if self
            .detector_queue_warn_last
            .compare_exchange(last, now_secs, Ordering::Relaxed, Ordering::Relaxed)
            .is_ok()
        {
            tracing::warn!(
                "Conflict detector queue {} (backpressure); stashing latest updates",
                reason
            );
        }
    }

    async fn store_detector_overflow(&self, key: String, update: DetectorUpdate) {
        if self.config.max_overflow_entries == 0 {
            return;
        }
        let mut guard = self.detector_overflow.lock().await;
        if guard.len() >= self.config.max_overflow_entries && !guard.contains_key(&key) {
            self.warn_state_cap(
                &self.detector_overflow_warn_last,
                "Conflict detector overflow cap reached; dropping updates",
            );
            return;
        }
        guard.insert(key, update);
    }

    fn warn_state_cap(&self, last: &AtomicU64, message: &'static str) {
        let Ok(now) = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH) else {
            return;
        };
        let now_secs = now.as_secs();
        let prev = last.load(Ordering::Relaxed);
        if now_secs.saturating_sub(prev) < STATE_CAP_WARN_INTERVAL_SECS {
            return;
        }
        if last
            .compare_exchange(prev, now_secs, Ordering::Relaxed, Ordering::Relaxed)
            .is_ok()
        {
            tracing::warn!("{}", message);
        }
    }

    /// Get the configured safety rules.
    #[allow(dead_code)] // Available for API endpoints
    pub fn rules(&self) -> &SafetyRules {
        &self.rules
    }

    /// Serialize flight plan creation to avoid double-booking.
    pub fn flight_plan_booking_lock(&self) -> &Mutex<()> {
        &self.flight_plan_booking_lock
    }

    /// Subscribe to command broadcasts (for WS streaming).
    pub fn subscribe_commands(&self) -> broadcast::Receiver<Command> {
        self.command_tx.subscribe()
    }

    /// Get config reference (for compliance evaluation, etc.).
    pub fn config(&self) -> &Config {
        &self.config
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

    fn seed_drone_counter(&self) {
        let mut max_id = 0u32;
        for entry in self.drones.iter() {
            if let Some(id) = Self::parse_drone_numeric_suffix(entry.key()) {
                max_id = max_id.max(id);
            }
        }
        if max_id > 0 {
            self.drone_counter
                .store(max_id.saturating_add(1), Ordering::SeqCst);
        }
    }

    fn parse_drone_numeric_suffix(drone_id: &str) -> Option<u32> {
        let upper = drone_id.trim().to_ascii_uppercase();
        let rest = upper.strip_prefix("DRONE")?;
        let rest = rest.trim_start_matches(|c: char| c == '-' || c == '_');
        if rest.is_empty() || !rest.chars().all(|c| c.is_ascii_digit()) {
            return None;
        }
        rest.parse().ok()
    }

    /// Get next drone ID number.
    pub fn next_drone_id(&self) -> u32 {
        self.drone_counter.fetch_add(1, Ordering::SeqCst)
    }

    pub fn drone_count(&self) -> usize {
        self.drones.len()
    }

    /// Register a new drone, persisting before updating in-memory state.
    pub async fn register_drone(&self, drone_id: &str, owner_id: Option<String>) -> Result<()> {
        self.register_drone_internal(drone_id, owner_id, None)
            .await
            .map(|_| ())
    }

    /// Register a new drone and persist its session token in a single transaction.
    pub async fn register_drone_with_token(
        &self,
        drone_id: &str,
        owner_id: Option<String>,
        token: String,
    ) -> Result<RegisterDroneOutcome> {
        self.register_drone_internal(drone_id, owner_id, Some(token))
            .await
    }

    async fn register_drone_internal(
        &self,
        drone_id: &str,
        owner_id: Option<String>,
        token: Option<String>,
    ) -> Result<RegisterDroneOutcome> {
        let owner_for_state = owner_id.clone().or_else(|| {
            self.drone_owners
                .get(drone_id)
                .map(|entry| entry.value().clone())
        });
        let now = Utc::now();
        let mut state_for_db = self
            .drones
            .get(drone_id)
            .map(|entry| entry.value().clone())
            .unwrap_or_else(|| DroneState {
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

        if state_for_db.owner_id.is_none() {
            state_for_db.owner_id = owner_for_state.clone();
        }

        if let Some(db) = self.database.clone() {
            if let Some(token_value) = token.as_deref() {
                let mut tx = db.pool().begin().await?;
                sqlx::query(
                    r#"
                    INSERT INTO drones (drone_id, owner_id, lat, lon, altitude_m, heading_deg, speed_mps, velocity_x, velocity_y, velocity_z, status, last_update)
                    VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7, ?8, ?9, ?10, ?11, ?12)
                    ON CONFLICT(drone_id) DO NOTHING
                    "#,
                )
                .bind(&state_for_db.drone_id)
                .bind(&state_for_db.owner_id)
                .bind(state_for_db.lat)
                .bind(state_for_db.lon)
                .bind(state_for_db.altitude_m)
                .bind(state_for_db.heading_deg)
                .bind(state_for_db.speed_mps)
                .bind(state_for_db.velocity_x)
                .bind(state_for_db.velocity_y)
                .bind(state_for_db.velocity_z)
                .bind(format!("{:?}", state_for_db.status))
                .bind(state_for_db.last_update.to_rfc3339())
                .execute(&mut *tx)
                .await?;

                let token_result = sqlx::query(
                    r#"
                    INSERT INTO drone_tokens (drone_id, session_token, updated_at)
                    VALUES (?1, ?2, CURRENT_TIMESTAMP)
                    ON CONFLICT(drone_id) DO NOTHING
                    "#,
                )
                .bind(&state_for_db.drone_id)
                .bind(token_value)
                .execute(&mut *tx)
                .await?;

                if token_result.rows_affected() == 0 {
                    tx.rollback().await?;
                    return Ok(RegisterDroneOutcome::AlreadyRegistered);
                }

                if let Some(owner) = state_for_db.owner_id.as_deref() {
                    sqlx::query(
                        r#"
                        UPDATE drones
                        SET owner_id = COALESCE(owner_id, ?2)
                        WHERE drone_id = ?1
                        "#,
                    )
                    .bind(&state_for_db.drone_id)
                    .bind(owner)
                    .execute(&mut *tx)
                    .await?;
                }

                tx.commit().await?;
            } else {
                drones_db::upsert_drone(db.pool(), &state_for_db).await?;
            }
        }

        if let Some(owner_id) = owner_id {
            self.drone_owners.insert(drone_id.to_string(), owner_id);
        }

        self.drones
            .entry(drone_id.to_string())
            .and_modify(|state| {
                if state.owner_id.is_none() {
                    state.owner_id = owner_for_state.clone();
                }
            })
            .or_insert(state_for_db);

        if let Some(token) = token {
            self.drone_tokens.insert(drone_id.to_string(), token);
        }

        Ok(RegisterDroneOutcome::Registered)
    }

    /// Store or update the session token for a drone.
    pub async fn set_drone_token(&self, drone_id: &str, token: String) -> Result<()> {
        if let Some(db) = self.database.clone() {
            drone_tokens_db::upsert_drone_token(db.pool(), drone_id, &token).await?;
        }
        self.drone_tokens.insert(drone_id.to_string(), token);
        Ok(())
    }

    /// Get the session token for a drone, if present.
    pub fn drone_token(&self, drone_id: &str) -> Option<String> {
        self.drone_tokens
            .get(drone_id)
            .map(|entry| entry.value().clone())
    }

    /// Check if a token matches the registered token for a drone.
    pub fn validate_drone_token(&self, drone_id: &str, token: &str) -> bool {
        self.drone_tokens
            .get(drone_id)
            .map(|entry| entry.value() == token)
            .unwrap_or(false)
    }

    /// Resolve a drone ID from a session token (if any).
    pub fn drone_id_for_token(&self, token: &str) -> Option<String> {
        for entry in self.drone_tokens.iter() {
            if entry.value() == token {
                return Some(entry.key().clone());
            }
        }
        None
    }

    /// Update drone state from telemetry.
    pub async fn update_telemetry(&self, telemetry: Telemetry) {
        let drone_id = telemetry.drone_id.clone();
        let mut telemetry = telemetry;
        let hold_active = self.has_active_hold_command(&drone_id);
        telemetry.altitude_m = altitude_to_amsl(
            telemetry.altitude_m,
            self.config.altitude_reference,
            self.config.geoid_offset_m,
        );

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
            let _ = self.tx.send(state.clone());
            self.queue_telemetry_persist(state);
        }

        self.queue_detector_update(DetectorUpdate::Upsert(
            DronePosition::new(
                &drone_id,
                telemetry.lat,
                telemetry.lon,
                telemetry.altitude_m,
            )
            .with_velocity(
                telemetry.heading_deg,
                telemetry.speed_mps,
                telemetry.velocity_z,
            ),
        ))
        .await;
    }

    /// Get all drone states.
    pub fn get_all_drones(&self) -> Vec<DroneState> {
        self.drones.iter().map(|r| r.value().clone()).collect()
    }

    /// Upsert external traffic and feed it into conflict detection.
    pub async fn upsert_external_traffic(&self, traffic: ExternalTraffic) {
        let mut traffic = traffic;
        let traffic_id = traffic.traffic_id.clone();
        let is_new = !self.external_traffic.contains_key(&traffic_id);
        if is_new
            && self.config.max_external_traffic_tracks > 0
            && self.external_traffic.len() >= self.config.max_external_traffic_tracks
        {
            self.warn_state_cap(
                &self.external_traffic_cap_warn_last,
                "External traffic cap reached; dropping new RID tracks",
            );
            return;
        }
        traffic.altitude_m = altitude_to_amsl(
            traffic.altitude_m,
            self.config.altitude_reference,
            self.config.geoid_offset_m,
        );
        self.external_traffic
            .insert(traffic_id.clone(), traffic.clone());

        self.queue_detector_update(DetectorUpdate::Upsert(
            DronePosition::new(&traffic_id, traffic.lat, traffic.lon, traffic.altitude_m)
                .with_velocity(traffic.heading_deg, traffic.speed_mps, 0.0),
        ))
        .await;
    }

    /// Get all external traffic tracks.
    pub fn get_external_traffic(&self) -> Vec<ExternalTraffic> {
        self.external_traffic
            .iter()
            .map(|r| r.value().clone())
            .collect()
    }

    /// Remove stale external tracks and purge them from the conflict detector.
    pub async fn purge_external_traffic(&self, max_age_secs: i64) -> Vec<String> {
        let now = Utc::now();
        let stale_ids: Vec<String> = self
            .external_traffic
            .iter()
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
            self.queue_detector_update(DetectorUpdate::Remove(id.clone())).await;
        }

        stale_ids
    }

    fn update_conflicts_from_detector(&self, detector: &mut ConflictDetector) {
        let new_conflicts = detector.detect_conflicts();

        self.conflicts.clear();
        for conflict in new_conflicts {
            let key = format!("{}-{}", conflict.drone1_id, conflict.drone2_id);
            self.conflicts.insert(key, conflict);
        }
    }

    /// Recompute conflicts from the latest detector state.
    pub async fn refresh_conflicts(self: &Arc<Self>) {
        let mut updates = Vec::new();
        {
            let mut rx = self.detector_rx.lock().await;
            while let Ok(update) = rx.try_recv() {
                updates.push(update);
            }
        }

        let overflow_updates = {
            let mut guard = self.detector_overflow.lock().await;
            std::mem::take(&mut *guard)
        };

        if updates.is_empty() && overflow_updates.is_empty() {
            return;
        }

        // Apply overflow updates after queued updates so they overwrite older channel entries.
        updates.extend(overflow_updates.into_values());

        let state = self.clone();
        let refresh_task = tokio::task::spawn_blocking(move || {
            let mut detector = match state.detector.lock() {
                Ok(guard) => guard,
                Err(poisoned) => {
                    tracing::error!("Conflict detector mutex poisoned; continuing with inner state");
                    poisoned.into_inner()
                }
            };
            for update in updates {
                match update {
                    DetectorUpdate::Upsert(position) => detector.update_position(position),
                    DetectorUpdate::Remove(drone_id) => detector.remove_drone(&drone_id),
                }
            }
            state.update_conflicts_from_detector(&mut detector);
        });

        if let Err(err) = refresh_task.await {
            tracing::warn!("Conflict detector refresh task failed: {}", err);
        }
    }

    /// Get a single drone state by ID.
    pub fn get_drone(&self, drone_id: &str) -> Option<DroneState> {
        self.drones.get(drone_id).map(|entry| entry.value().clone())
    }

    /// Check for drones that haven't reported in and mark them as Lost.
    /// Returns the IDs of newly-lost drones.
    pub async fn check_timeouts(&self) -> Vec<String> {
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
            }
        }

        for drone_id in &lost_drones {
            self.queue_detector_update(DetectorUpdate::Remove(drone_id.clone()))
                .await;
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
        self.daa_advisories
            .iter()
            .map(|r| r.value().clone())
            .collect()
    }

    /// Get all flight plans
    pub fn get_flight_plans(&self) -> Vec<FlightPlan> {
        self.flight_plans
            .iter()
            .map(|r| r.value().clone())
            .collect()
    }

    /// Add or update a flight plan, persisting before updating in-memory state.
    pub async fn add_flight_plan(&self, plan: FlightPlan) -> Result<()> {
        if let Some(db) = self.database.clone() {
            flight_plans_db::upsert_flight_plan(db.pool(), &plan).await?;
        }
        self.flight_plans.insert(plan.flight_id.clone(), plan);
        Ok(())
    }

    // ========== COMMAND MANAGEMENT ==========

    /// Enqueue a command for a drone.
    pub async fn enqueue_command(&self, command: Command) -> Result<()> {
        if let Some(db) = self.database.clone() {
            commands_db::insert_command(db.pool(), &command).await?;
        }
        let drone_id = command.drone_id.clone();
        let command_for_broadcast = command.clone();
        self.commands
            .entry(drone_id)
            .or_default()
            .push_back(command);
        let _ = self.command_tx.send(command_for_broadcast);
        Ok(())
    }

    /// Check if a command was recently issued (cooldown check).
    pub fn can_issue_command(&self, drone_id: &str, cooldown_secs: u64) -> bool {
        if self.has_pending_command(drone_id) {
            return false;
        }
        if let Some(last_time) = self.command_cooldowns.get(drone_id) {
            last_time.elapsed().as_secs() >= cooldown_secs
        } else {
            true
        }
    }

    /// Mark that a command was just issued (start cooldown).
    pub fn mark_command_issued(&self, drone_id: &str) {
        self.command_cooldowns
            .insert(drone_id.to_string(), std::time::Instant::now());
    }

    /// Get the next pending command for a drone (does not remove it).
    pub fn peek_command(&self, drone_id: &str) -> Option<Command> {
        self.commands
            .get(drone_id)
            .and_then(|queue| queue.front().cloned())
    }

    /// Get all pending commands for a specific drone.
    pub fn get_pending_commands(&self, drone_id: &str) -> Vec<Command> {
        self.commands
            .get(drone_id)
            .map(|queue| queue.iter().cloned().collect())
            .unwrap_or_default()
    }

    /// Find the drone ID for a command ID (if queued).
    pub fn command_drone_id(&self, command_id: &str) -> Option<String> {
        for entry in self.commands.iter() {
            if entry.value().iter().any(|cmd| cmd.command_id == command_id) {
                return Some(entry.key().clone());
            }
        }
        None
    }

    /// Check if drone has an active HOLD (acknowledged) or pending HOLD/REROUTE command.
    pub fn has_active_command(&self, drone_id: &str) -> bool {
        if self.has_active_hold_command(drone_id) {
            return true;
        }
        let now = chrono::Utc::now();
        if let Some(queue) = self.commands.get(drone_id) {
            queue.iter().any(|cmd| {
                // Must be a control command (HOLD or REROUTE)
                let is_control = matches!(
                    cmd.command_type,
                    atc_core::models::CommandType::Hold { .. }
                        | atc_core::models::CommandType::Reroute { .. }
                );
                let awaiting_ack = self.command_waiting_for_ack(cmd, now);
                // Must not be expired
                let not_expired = cmd.expires_at.map(|exp| exp > now).unwrap_or(true); // No expiry = never expires

                is_control && awaiting_ack && not_expired
            })
        } else {
            false
        }
    }

    /// Check if drone has an acknowledged HOLD command still in effect.
    pub fn has_active_hold_command(&self, drone_id: &str) -> bool {
        let now = Utc::now();
        let expires_at = self
            .active_holds
            .get(drone_id)
            .map(|entry| entry.value().clone());
        let Some(expires_at) = expires_at else {
            return false;
        };
        if expires_at > now {
            true
        } else {
            self.active_holds.remove(drone_id);
            false
        }
    }

    /// Check if a drone has any pending (non-expired) command awaiting acknowledgement.
    pub fn has_pending_command(&self, drone_id: &str) -> bool {
        let now = Utc::now();
        self.commands
            .get(drone_id)
            .map(|queue| {
                queue.iter().any(|cmd| {
                    let awaiting_ack = self.command_waiting_for_ack(cmd, now);
                    let not_expired = cmd.expires_at.map(|exp| exp > now).unwrap_or(true);
                    awaiting_ack && not_expired
                })
            })
            .unwrap_or(false)
    }

    /// Remove expired HOLD states.
    pub fn purge_expired_active_holds(&self) -> usize {
        let now = Utc::now();
        let expired: Vec<String> = self
            .active_holds
            .iter()
            .filter(|entry| *entry.value() <= now)
            .map(|entry| entry.key().clone())
            .collect();
        for drone_id in &expired {
            self.active_holds.remove(drone_id);
        }
        expired.len()
    }

    /// Purge expired commands from all queues.
    /// Should be called periodically (e.g., from conflict loop).
    pub async fn purge_expired_commands(&self) -> Result<usize> {
        let now = chrono::Utc::now();
        let mut purged_count = 0;
        let mut stale_count = 0;
        let mut stale_drone_ids: HashSet<String> = HashSet::new();
        let ack_timeout = self.config.command_ack_timeout_secs;
        let waiting_for_ack = |cmd: &Command| {
            if cmd.acknowledged {
                return false;
            }
            if ack_timeout <= 0 {
                return true;
            }
            cmd.issued_at + ChronoDuration::seconds(ack_timeout) > now
        };

        for mut entry in self.commands.iter_mut() {
            let queue = entry.value_mut();
            let before_len = queue.len();
            queue.retain(|cmd| {
                // Keep if no expiry or not yet expired
                let not_expired = cmd.expires_at.map(|exp| exp > now).unwrap_or(true);
                let awaiting_ack = waiting_for_ack(cmd);
                if not_expired && !awaiting_ack && !cmd.acknowledged {
                    stale_count += 1;
                    stale_drone_ids.insert(cmd.drone_id.clone());
                }
                not_expired && awaiting_ack
            });
            purged_count += before_len - queue.len();
        }

        for drone_id in stale_drone_ids {
            self.command_cooldowns.remove(&drone_id);
        }

        if purged_count > 0 {
            tracing::debug!("Purged {} expired commands", purged_count);
            if let Some(db) = self.database.clone() {
                commands_db::delete_expired_commands(db.pool())
                    .await
                    .map(|_| ())?;
            }
        }
        if stale_count > 0 {
            if let Some(db) = self.database.clone() {
                commands_db::delete_stale_commands(db.pool(), ack_timeout)
                    .await
                    .map(|_| ())?;
            }
        }
        Ok(purged_count)
    }

    fn command_waiting_for_ack(&self, cmd: &Command, now: DateTime<Utc>) -> bool {
        if cmd.acknowledged {
            return false;
        }
        let ack_timeout = self.config.command_ack_timeout_secs;
        if ack_timeout <= 0 {
            return true;
        }
        cmd.issued_at + ChronoDuration::seconds(ack_timeout) > now
    }

    /// Acknowledge a command by ID (removes it from queue).
    pub async fn ack_command(&self, command_id: &str) -> Result<bool> {
        let command = match self.find_command(command_id) {
            Some(command) => command,
            None => return Ok(false),
        };

        if let Some(db) = self.database.clone() {
            commands_db::ack_command(db.pool(), command_id).await?;
        }

        let removed = self.remove_command_by_id(command_id);
        let command_to_apply = removed.as_ref().unwrap_or(&command);
        self.apply_command_ack_effects(command_to_apply);

        Ok(true)
    }

    fn find_command(&self, command_id: &str) -> Option<Command> {
        for entry in self.commands.iter() {
            if let Some(cmd) = entry
                .value()
                .iter()
                .find(|cmd| cmd.command_id == command_id)
            {
                return Some(cmd.clone());
            }
        }
        None
    }

    fn remove_command_by_id(&self, command_id: &str) -> Option<Command> {
        for mut entry in self.commands.iter_mut() {
            let queue = entry.value_mut();
            if let Some(pos) = queue.iter().position(|c| c.command_id == command_id) {
                return queue.remove(pos);
            }
        }
        None
    }

    fn apply_command_ack_effects(&self, command: &Command) {
        match command.command_type {
            atc_core::models::CommandType::Hold { duration_secs } => {
                let now = Utc::now();
                let mut hold_until = now + ChronoDuration::seconds(duration_secs as i64);
                if let Some(expires_at) = command.expires_at {
                    if expires_at < hold_until {
                        hold_until = expires_at;
                    }
                }
                if hold_until > now {
                    self.active_holds
                        .insert(command.drone_id.clone(), hold_until);
                }
            }
            atc_core::models::CommandType::Resume => {
                self.active_holds.remove(&command.drone_id);
            }
            _ => {}
        }
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
    pub async fn add_geofence(&self, geofence: Geofence) -> Result<()> {
        if let Some(db) = self.database.clone() {
            geofences_db::upsert_geofence(db.pool(), &geofence).await?;
        }
        self.geofences.insert(geofence.id.clone(), geofence);
        Ok(())
    }

    /// Replace external geofences from Blender/DSS.
    pub fn set_external_geofences(&self, geofences: Vec<Geofence>) {
        self.external_geofences.clear();
        for geofence in geofences {
            self.external_geofences
                .insert(geofence.id.clone(), geofence);
        }
    }

    /// Track a Blender conflict geofence ID to avoid re-ingest.
    pub fn mark_conflict_geofence(&self, blender_id: String, expires_at: i64) {
        self.conflict_geofences.insert(blender_id, expires_at);
    }

    /// Remove a Blender conflict geofence from tracking.
    pub fn clear_conflict_geofence(&self, blender_id: &str) {
        self.conflict_geofences.remove(blender_id);
    }

    /// Purge expired conflict geofence IDs.
    pub fn purge_conflict_geofences(&self, now_ts: i64) {
        let expired: Vec<String> = self
            .conflict_geofences
            .iter()
            .filter(|entry| *entry.value() <= now_ts)
            .map(|entry| entry.key().clone())
            .collect();
        for blender_id in expired {
            self.conflict_geofences.remove(&blender_id);
        }
    }

    /// Get conflict Blender IDs currently tracked.
    pub fn get_conflict_geofence_ids(&self) -> HashSet<String> {
        self.conflict_geofences
            .iter()
            .map(|entry| entry.key().clone())
            .collect()
    }

    /// Get all local geofences (ATC-managed only).
    pub fn get_local_geofences(&self) -> Vec<Geofence> {
        self.geofences.iter().map(|e| e.value().clone()).collect()
    }

    /// Get all geofences.
    pub fn get_geofences(&self) -> Vec<Geofence> {
        let mut all = Vec::new();
        all.extend(self.geofences.iter().map(|e| e.value().clone()));
        all.extend(self.external_geofences.iter().map(|e| e.value().clone()));
        all
    }

    /// Get a specific geofence by ID.
    pub fn get_geofence(&self, id: &str) -> Option<Geofence> {
        if let Some(entry) = self.geofences.get(id) {
            return Some(entry.value().clone());
        }
        self.external_geofences.get(id).map(|e| e.value().clone())
    }

    /// Check if a geofence ID belongs to an external source.
    pub fn is_external_geofence(&self, id: &str) -> bool {
        self.external_geofences.contains_key(id)
    }

    /// Remove a geofence by ID.
    pub async fn remove_geofence(&self, id: &str) -> Result<bool> {
        if !self.geofences.contains_key(id) {
            return Ok(false);
        }
        if let Some(db) = self.database.clone() {
            geofences_db::delete_geofence(db.pool(), id)
                .await
                .map(|_| ())?;
        }
        Ok(self.geofences.remove(id).is_some())
    }

    /// Check if a point is inside any active geofence.
    pub fn check_point_in_geofences(&self, lat: f64, lon: f64, altitude_m: f64) -> Vec<String> {
        self.geofences
            .iter()
            .chain(self.external_geofences.iter())
            .filter(|e| e.value().active && e.value().contains_point(lat, lon, altitude_m))
            .map(|e| e.key().clone())
            .collect()
    }

    // ========== ADMIN METHODS ==========

    /// Clear all state for demo reset.
    /// This removes all drones, conflicts, commands, flight plans, and geofences.
    pub async fn clear_all(&self) -> Result<()> {
        if let Some(db) = self.database.clone() {
            db_persistence::clear_all(db.pool()).await?;
        }

        self.drones.clear();
        self.drone_owners.clear();
        self.drone_tokens.clear();
        self.external_traffic.clear();
        self.conflicts.clear();
        self.commands.clear();
        self.command_cooldowns.clear();
        self.active_holds.clear();
        self.flight_plans.clear();
        self.geofences.clear();
        self.external_geofences.clear();
        self.conflict_geofences.clear();
        self.conformance.clear();
        self.daa_advisories.clear();
        if let Ok(mut guard) = self.telemetry_overflow.lock() {
            guard.clear();
        }
        self.detector_overflow.lock().await.clear();

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
        Ok(())
    }
}
