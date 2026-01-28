# ATC-Drone

**Autonomous Traffic Control for Unmanned Aircraft Systems**

A Rust-based strategic conflict detection and resolution system for drone traffic management. This project serves as the "brain" of a micro-UTM (Unmanned Traffic Management) system, providing real-time conflict detection, automatic avoidance routing, and command dispatch to drones.

## About

`atc-drone` is the core ATC backend for the stack. It exposes HTTP and WebSocket APIs for registration, telemetry, conflict detection, geofences, and command dispatch, and ships with a Rust SDK plus CLI simulators for demos and testing. It can run standalone for development or as part of the unified `atc-stack` Docker Compose environment.

## Architecture

```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│   Drone/Sim     │────▶│   ATC Server    │────▶│ Flight Blender  │
│   (atc-sdk)     │◀────│   (atc-server)  │     │   (optional)    │
└─────────────────┘     └─────────────────┘     └─────────────────┘
                               │
                               ▼
                        ┌─────────────────┐
                        │ Flight Spotlight│
                        │   (Web UI)      │
                        └─────────────────┘
```

## Crates

| Crate | Description |
|-------|-------------|
| **atc-core** | Pure logic layer - conflict detection, routing algorithms, spatial math (ENU coordinates, haversine distance). No networking dependencies. |
| **atc-server** | Axum-based HTTP/WebSocket server. Runs conflict detection loop, command dispatch, telemetry ingestion, and geofence management. |
| **atc-sdk** | Client library for drones. Handles registration, telemetry reporting, command polling, and acknowledgement. |
| **atc-blender** | Integration client for Flight Blender (OpenUTM). Syncs telemetry and geofences to external UTM systems. |
| **atc-cli** | CLI tools and simulators for testing. Includes the `demo_scenario` binary for showcasing the full conflict resolution workflow. |

## Features

### Conflict Detection
- **3D Euclidean distance** with configurable lookahead (default 20s)
- **Closest Point of Approach (CPA)** prediction using velocity extrapolation
- **Severity classification**: Info → Warning → Critical based on separation
- **ENU coordinate system** with proper cos(lat) scaling for accurate distance calculations

### Automatic Resolution
- **Avoidance routing**: Vertical (climb), Lateral (offset), or Combined strategies
- **Meter-based waypoint generation** (100m lateral offset, 30m vertical)
- **Priority-based deconfliction**: Lower-priority drone yields
- **Hold-aware logic**: Prevents cascading reroutes when priority drone is already maneuvering

### Command System
- **Command types**: Reroute, Hold, Resume, AltitudeChange
- **Expiration handling**: Commands auto-expire after configurable duration
- **Lifecycle tracking**: Prevents duplicate commands via cooldown periods
- **Distance-based blocking check**: Uses segment-to-segment distance (not bounding box)

### Geofencing
- **Polygon geofences** with altitude bounds (floor/ceiling)
- **Validation**: Auto-closes polygons, enforces lower < upper altitude
- **Route conflict checking**: API endpoint to verify flight plans against active geofences
- **Types**: Advisory, NoFly, Restricted

### Simulation
- **Realistic drone lifecycle**: Preflight → Takeoff → Cruise → Landing → Landed
- **Dynamic rerouting**: Drones follow avoidance waypoints when commanded
- **Distance-based phase transitions**: No teleportation bugs

## Quick Start

### Prerequisites
- Rust 1.75+ (2021 edition)
- [Flight Spotlight](https://github.com/openskies-sh/flight-spotlight) (optional, for visualization)
- [Flight Blender](https://github.com/openskies-sh/flight-blender) (optional, for UTM integration)

### Build
```bash
cargo build --workspace
```

### Run the Demo

**Terminal 1: Start the ATC Server**
```bash
cargo run -p atc-server
```

**Terminal 2: Run the Demo Scenario**
```bash
cargo run -p atc-cli --bin demo_scenario
```

This launches two simulated drones on a collision course. The system will:
1. Detect the conflict (~10s before intersection)
2. Issue a REROUTE command to the lower-priority drone
3. The drone climbs to 80m, flies over the conflict zone, and descends
4. Both drones land at their destinations

### API Endpoints

| Method | Endpoint | Description |
|--------|----------|-------------|
| POST | `/v1/telemetry` | Submit drone telemetry |
| GET | `/v1/drones` | List all registered drones |
| GET | `/v1/conflicts` | Get active conflicts |
| POST | `/v1/geofences` | Create a geofence |
| GET | `/v1/geofences` | List all geofences |
| POST | `/v1/geofences/check-route` | Check if a route conflicts with geofences |
| POST | `/v1/commands` | Issue a command to a drone |
| GET | `/v1/commands/next?drone_id=X` | Poll for pending commands |
| POST | `/v1/commands/ack` | Acknowledge command receipt |
| GET | `/v1/commands/ws` | WebSocket command stream (auth required) |
| POST | `/v1/admin/reset` | Reset all server state (requires confirm payload) |
| GET | `/v1/ws` | WebSocket for real-time updates (supports `token`, `owner_id`, `drone_id` query params) |

Note: `/v1/drones/register` requires `X-Registration-Token` when `ATC_REQUIRE_REGISTRATION_TOKEN` is enabled.
Drone-facing endpoints (telemetry + command polling/ack) require `Authorization: Bearer <session_token>` from `/v1/drones/register`.

### API Versioning
- Current stable version: `/v1`
- Breaking changes will land in a new versioned prefix (e.g., `/v2`).
- Deprecated endpoints will remain available for at least one release cycle and should emit `Deprecation` + `Sunset` headers when scheduled.

### OpenAPI
Machine-readable spec: `openapi.yaml`

### Load Testing
Simple telemetry load script:
```
python3 scripts/load_test.py --base-url http://localhost:3000 --drones 20 --duration 60 --interval-ms 500
```

### Failure/Chaos Smoke Tests
Basic failure-mode checks:
```
python3 scripts/failure_test.py --base-url http://localhost:3000 --registration-token change-me --admin-token <ATC_ADMIN_TOKEN>
```

## Configuration

Environment variables:
- `ATC_PORT` - Server port (default: `3000`)
- `BLENDER_URL` - Flight Blender URL (optional)
- `BLENDER_AUTH_TOKEN` - Flight Blender auth token (optional)
- `ATC_REGISTRATION_TOKEN` - Shared token for drone registration (required when enabled)
- `ATC_REQUIRE_REGISTRATION_TOKEN` - Enforce token for `/v1/drones/register` (default: `true`)
- `ATC_REGISTER_RATE_LIMIT_RPS` - Max registration requests per second per IP (default: `10`)
- `ATC_DB_MAX_CONNECTIONS` - Max SQLite pool connections (default: `10`)
- `ATC_WS_TOKEN` - Shared token required for `/v1/ws` when enabled (default: unset)
- `ATC_REQUIRE_WS_TOKEN` - Enforce token for `/v1/ws` (default: `true` in prod when token set)
- `ATC_TELEMETRY_MIN_ALT_M` - Minimum accepted telemetry altitude (default: `-100`)
- `ATC_TELEMETRY_MAX_ALT_M` - Maximum accepted telemetry altitude (default: `20000`)
- `ATC_TELEMETRY_MAX_SPEED_MPS` - Maximum accepted telemetry speed (default: `150`)
- `ATC_TELEMETRY_MAX_FUTURE_S` - Max seconds allowed in the future for telemetry timestamps (default: `30`)
- `ATC_TELEMETRY_MAX_AGE_S` - Max age in seconds for telemetry timestamps (default: `300`)
- `ATC_PULL_BLENDER_GEOFENCES` - Pull Blender/DSS geofences into ATC (default: `true`)
- `ATC_ALLOW_ADMIN_RESET` - Enable `/v1/admin/reset` (default: `true` in dev, `false` in prod)
- `ATC_RULES_MIN_HORIZONTAL_SEPARATION_M` - Minimum horizontal separation (default: `50`)
- `ATC_RULES_MIN_VERTICAL_SEPARATION_M` - Minimum vertical separation (default: `30`)
- `ATC_RULES_LOOKAHEAD_SECONDS` - Conflict lookahead window (default: `20`)
- `ATC_RULES_WARNING_MULTIPLIER` - Warning threshold multiplier (default: `2.0`)
- `ATC_RULES_DRONE_TIMEOUT_SECS` - Seconds before drone marked lost (default: `10`)
- `ATC_RULES_MAX_ALTITUDE_M` - Max allowed altitude in meters (default: `121`)
- `ATC_RULES_MIN_ALTITUDE_M` - Min allowed altitude in meters (default: `10`)
- `ATC_LOG_FORMAT` - Logging format (`text` or `json`, default: `text`)

## Project Status

**MVP Complete** ✅

- [x] Telemetry ingestion and state management
- [x] Conflict detection with CPA prediction
- [x] Automatic reroute generation
- [x] Command dispatch and acknowledgement
- [x] Geofence CRUD and route checking
- [x] ENU coordinate math (no more degree-as-meters bugs)
- [x] Command expiration and cooldown
- [x] Distance-based phase transitions in simulation
- [x] WebSocket real-time updates

### Roadmap
- [x] Persistence layer (SQLite)
- [x] Time-aware strategic planning (4D flight plans)
- [ ] MAVLink bridge for real drone integration
- [x] WebSocket command push (replace polling)

## License

MIT

## Author

Ezra Khuzadi
