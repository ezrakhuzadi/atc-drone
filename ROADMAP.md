# ATC Drone System - Project Roadmap

This document tracks the development milestones for the ATC Drone/ATCGS project.

## Project Goal

Build a local traffic management system for cooperative UAVs that:
- Tracks drones in ~1 km radius
- Predicts conflicts 10-30s ahead
- Issues HOLD / REROUTE / ALTITUDE commands
- Keeps latency under 500ms

---

## Milestone A: "Hello World" - Blender Up + API Reachable ✅

- [x] Install Docker and dev tools on Arch server
- [x] Create `.env` file for Flight Blender based on sample
- [x] Build Flight Blender Docker image
- [x] Run Flight Blender with `docker compose up`
- [x] Verify homepage loads at `http://localhost:8000`
- [x] Verify `/ping` endpoint responds

---

## Milestone B: "First Track in Spotlight" - Simulated Drone Shows on 3D Globe 🚧

- [x] Generate dummy JWT token for Blender API access
- [x] Create `atc-drone/` project folder structure
- [x] Write telemetry injector script (`send_one_track.py`)
- [x] Verify track appears in Blender's `GET /get_air_traffic/{session_id}` response
- [x] Configure and run Flight Spotlight
- [x] Connect Spotlight to local Blender instance
- [x] Verify simulated drone appears in Spotlight UI

---

## Milestone C: Multi-drone + Conflict Detection

- [ ] Write multi-drone simulator (extend `send_one_track.py`)
- [ ] Implement conflict detection engine (10-30s lookahead)
- [ ] Create crossing scenario test
- [ ] Surface conflict alerts via geofence API

---

## Milestone D: Commands (Hold/Alt/Reroute)

- [ ] Design command protocol (HOLD, REROUTE, ALTITUDE_CHANGE)
- [ ] Implement command handler in simulator
- [ ] Test command execution flow
- [ ] (Optional) MAVLink integration for real autopilots

---

## Milestone E: Reliability + Metrics

- [ ] Add latency logging (target: <500ms end-to-end)
- [ ] Implement fail-safe behavior for link/sensor issues
- [ ] Create repeatable demo script

---

## Related Repositories

| Repo | Description |
|------|-------------|
| [atc-drone](https://github.com/ezrakhuzadi/atc-drone) | This repo - ATC brain and telemetry adapter |
| [flight-blender-irvine](https://github.com/ezrakhuzadi/flight-blender-irvine) | UTM Backend (forked from OpenUTM) |
| [flight-spotlight-irvine](https://github.com/ezrakhuzadi/flight-spotlight-irvine) | 3D UI (forked from OpenUTM) |

---

## Contributing

Pick an unchecked item, create a branch, implement, and submit a PR!
