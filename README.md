# ATC-Drone

**Autonomous Traffic Control for Unmanned Aircraft Systems**

A Rust-based strategic conflict detection and resolution system for drone traffic management. This project serves as the "brain" of a micro-UTM (Unmanned Traffic Management) system, providing real-time conflict detection, automatic avoidance routing, and command dispatch to drones.

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
- **Command types**: Reroute, Hold, Resume, Land, AltitudeChange
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
| POST | `/v1/commands/{id}/ack` | Acknowledge command receipt |
| POST | `/v1/admin/reset` | Reset all server state (for demos) |
| GET | `/ws` | WebSocket for real-time updates |

## Configuration

Environment variables:
- `ATC_HOST` - Server bind address (default: `0.0.0.0`)
- `ATC_PORT` - Server port (default: `3000`)
- `BLENDER_URL` - Flight Blender URL (optional)
- `BLENDER_TOKEN` - Flight Blender auth token (optional)

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
- [ ] Persistence layer (SQLite/PostgreSQL)
- [ ] Time-aware strategic planning (4D flight plans)
- [ ] MAVLink bridge for real drone integration
- [ ] WebSocket command push (replace polling)

## License

MIT

## Author

Ezra Khuzadi
