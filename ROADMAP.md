# ATC Drone System - Project Roadmap

Local traffic management system for cooperative UAVs using OpenUTM.

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                      atc-drone workspace                     │
├─────────────────────────────────────────────────────────────┤
│  crates/                                                     │
│  ├── atc-core/      Pure logic (no networking)              │
│  │   ├── models     DroneState, Telemetry, Mission, Command │
│  │   ├── conflict   Conflict detection (10-30s lookahead)   │
│  │   ├── routing    Route suggestions ("Waze options")      │
│  │   └── rules      Safety thresholds, altitude bands       │
│  │                                                           │
│  ├── atc-blender/   Flight Blender API client               │
│  │   ├── client     HTTP client for Blender                 │
│  │   ├── sync_*     Track/geofence sync to Blender          │
│  │                                                           │
│  ├── atc-server/    Always-on backend (Axum)                │
│  │   ├── api/       REST + WebSocket endpoints              │
│  │   ├── state/     In-memory store (DashMap)               │
│  │   └── loops/     Background conflict detection           │
│  │                                                           │
│  └── atc-sdk/       Drone integration SDK                   │
│      ├── client     Register + connect                      │
│      ├── telemetry  Stream position updates                 │
│      └── commands   Receive/ACK commands                    │
│                                                              │
│  src/bin/           Legacy CLIs (still work)                │
│  deploy/            Docker config                            │
│  docs/              Architecture docs                        │
└─────────────────────────────────────────────────────────────┘
```

---

## Milestones

### ✅ Milestone A: Hello World - Blender Up + API Reachable
- [x] Install Docker and dev tools
- [x] Run Flight Blender
- [x] Verify /ping responds

### ✅ Milestone B: First Track in Spotlight
- [x] Generate JWT tokens
- [x] Write telemetry injector
- [x] See drone in Spotlight UI

### 🚧 Milestone C: Multi-drone + Conflict Detection
- [x] Multi-drone simulator
- [x] Conflict detection engine (10-30s lookahead)
- [x] **Rewrote in Rust** 🦀
- [/] **atc-server always-on backend**
- [ ] WebSocket streaming to UI
- [ ] Sync conflicts to Blender as geofences

### ⏳ Milestone D: Commands (Hold/Alt/Reroute)
- [ ] HOLD command implementation
- [ ] Altitude change command
- [ ] Reroute command
- [ ] Simulator responds to commands

### ⏳ Milestone E: SDK + "Waze" Workflow
- [ ] atc-sdk Python reference client
- [ ] Route request → options → select flow
- [ ] Pre-flight checklist (green checks / red X)

### ⏳ Milestone F: Reliability + Demo
- [ ] docker-compose full stack
- [ ] One-command demo script
- [ ] Latency logging (<500ms target)

---

## Quick Start (New Structure)

```bash
cd /home/uci/Project/atc-drone

# Build entire workspace
cargo build

# Run the always-on server (port 3000)
cargo run -p atc-server

# Run old CLIs (still work)
cargo run --bin send_one_track -- --duration 60
```

---

## API Endpoints (atc-server)

| Method | Endpoint | Description |
|--------|----------|-------------|
| POST | `/v1/drones/register` | Register a drone |
| POST | `/v1/telemetry` | Send position update |
| GET | `/v1/drones` | List all drones |
| GET | `/v1/conflicts` | Get active conflicts |
| WS | `/v1/stream` | Real-time updates (TODO) |
