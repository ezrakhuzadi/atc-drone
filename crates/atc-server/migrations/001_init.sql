-- ATC-Drone SQLite Schema
-- Initial migration: core tables for persistence

-- Drones table: stores registered drones and their latest state
CREATE TABLE IF NOT EXISTS drones (
    drone_id TEXT PRIMARY KEY,
    owner_id TEXT,
    lat REAL NOT NULL DEFAULT 0.0,
    lon REAL NOT NULL DEFAULT 0.0,
    altitude_m REAL NOT NULL DEFAULT 0.0,
    heading_deg REAL NOT NULL DEFAULT 0.0,
    speed_mps REAL NOT NULL DEFAULT 0.0,
    velocity_x REAL NOT NULL DEFAULT 0.0,
    velocity_y REAL NOT NULL DEFAULT 0.0,
    velocity_z REAL NOT NULL DEFAULT 0.0,
    status TEXT NOT NULL DEFAULT 'Inactive',
    last_update TEXT NOT NULL,
    created_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP
);

-- Geofences table: no-fly zones and restricted areas
CREATE TABLE IF NOT EXISTS geofences (
    id TEXT PRIMARY KEY,
    name TEXT NOT NULL,
    geofence_type TEXT NOT NULL DEFAULT 'NoFly',
    vertices TEXT NOT NULL, -- JSON array of {lat, lon} points
    lower_altitude_m REAL NOT NULL DEFAULT 0.0,
    upper_altitude_m REAL NOT NULL DEFAULT 400.0,
    active INTEGER NOT NULL DEFAULT 1,
    created_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP,
    updated_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP
);

-- Flight plans table: submitted flight plans
CREATE TABLE IF NOT EXISTS flight_plans (
    flight_id TEXT PRIMARY KEY,
    drone_id TEXT NOT NULL,
    owner_id TEXT,
    origin_lat REAL,
    origin_lon REAL,
    dest_lat REAL,
    dest_lon REAL,
    waypoints TEXT NOT NULL, -- JSON array of waypoints
    trajectory_log TEXT, -- JSON array of trajectory points
    metadata TEXT, -- JSON object of FlightPlanMetadata
    status TEXT NOT NULL DEFAULT 'Pending',
    start_time TEXT,
    end_time TEXT,
    created_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP,
    FOREIGN KEY (drone_id) REFERENCES drones(drone_id)
);

-- Commands table: pending and historical commands
CREATE TABLE IF NOT EXISTS commands (
    command_id TEXT PRIMARY KEY,
    drone_id TEXT NOT NULL,
    command_type TEXT NOT NULL, -- JSON: {type: "Hold", params: {...}}
    issued_at TEXT NOT NULL,
    expires_at TEXT,
    acknowledged INTEGER NOT NULL DEFAULT 0,
    acked_at TEXT,
    FOREIGN KEY (drone_id) REFERENCES drones(drone_id)
);

-- Drone session tokens table: persists registration tokens across restarts
CREATE TABLE IF NOT EXISTS drone_tokens (
    drone_id TEXT PRIMARY KEY,
    session_token TEXT NOT NULL,
    updated_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP,
    FOREIGN KEY (drone_id) REFERENCES drones(drone_id)
);

-- Geofence sync table: track Blender IDs for ATC-managed geofences
CREATE TABLE IF NOT EXISTS geofence_sync_state (
    local_id TEXT PRIMARY KEY,
    blender_id TEXT NOT NULL,
    fingerprint TEXT NOT NULL,
    expires_at INTEGER NOT NULL,
    updated_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP
);

-- Scheduler lock: single-row table used to serialize strategic scheduling transactions across processes.
CREATE TABLE IF NOT EXISTS scheduler_lock (
    id INTEGER PRIMARY KEY CHECK (id = 1),
    updated_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP
);
INSERT OR IGNORE INTO scheduler_lock (id) VALUES (1);

-- Indexes for common queries
CREATE INDEX IF NOT EXISTS idx_drones_owner ON drones(owner_id);
CREATE INDEX IF NOT EXISTS idx_drones_status ON drones(status);
CREATE INDEX IF NOT EXISTS idx_flight_plans_drone ON flight_plans(drone_id);
CREATE INDEX IF NOT EXISTS idx_flight_plans_owner ON flight_plans(owner_id);
CREATE INDEX IF NOT EXISTS idx_commands_drone ON commands(drone_id);
CREATE INDEX IF NOT EXISTS idx_commands_expires ON commands(expires_at);
CREATE INDEX IF NOT EXISTS idx_drone_tokens_updated ON drone_tokens(updated_at);
CREATE INDEX IF NOT EXISTS idx_geofence_sync_expires ON geofence_sync_state(expires_at);
