# ATC Drone Audit Megalist

Comprehensive audit performed 2026-01-18 after reviewing project requirements from `~/Downloads` and doing deep code inspection.

## Intent Anchors
- Final report sets <500ms latency, LoRa/STM32 pipeline, Node 20 + React 18 UI, and Flask/FastAPI gateway expectations. /home/uci/Downloads/159A-Final-Report__1_.txt:74, /home/uci/Downloads/159A-Final-Report__1_.txt:88, /home/uci/Downloads/159A-Final-Report__1_.txt:107, /home/uci/Downloads/159A-Final-Report__1_.txt:110
- Poster calls out ~1km radius monitoring, 10–30s prediction horizon, and LoRa telemetry. /home/uci/Downloads/EECS-159A-Poster.txt:10, /home/uci/Downloads/EECS-159A-Poster.txt:13, /home/uci/Downloads/EECS-159A-Poster.txt:31
- Transcript emphasizes Waze-like low-friction UX plus preflight compliance checks (weather/population/obstacles/battery/strategic deconfliction). /home/uci/Downloads/Transcript.txt:73, /home/uci/Downloads/Transcript.txt:97
- Individual reflection highlights React/Node dashboard work and <500ms WebSocket latency goals. /home/uci/Downloads/EECS_159A_-_Final_Individual.txt:103, /home/uci/Downloads/EECS_159A_-_Final_Individual.txt:104

## System/Architecture Gaps (Requirements-level)

### Scope/Stack Gaps
- LoRa/STM32 embedded pipeline is described, but ingestion is HTTP-only and no firmware/gateway code exists. /home/uci/Downloads/EECS-159A-Poster.txt:31, /home/uci/Downloads/159A-Final-Report__1_.txt:88, atc-drone/crates/atc-server/src/api/routes.rs:65
- MAVLink compatibility is a requirement but remains a TODO; telemetry ingestion is HTTP JSON only with no UDP/MAVLink bridge. /home/uci/Downloads/159A-Final-Report__1_.txt:89, atc-drone/README.md:178, atc-drone/crates/atc-server/src/api/routes.rs:246

### Operational/Performance Mismatches
- <500ms latency target vs 1s conflict loop + 2s RID polling; pipeline cannot meet target as written. /home/uci/Downloads/159A-Final-Report__1_.txt:74, atc-drone/crates/atc-server/src/loops/conflict_loop.rs:41, atc-drone/crates/atc-server/src/loops/rid_sync_loop.rs:19
- 1km radius operational scope is not enforced; telemetry validation only checks global lat/lon bounds. /home/uci/Downloads/EECS-159A-Poster.txt:10, atc-drone/crates/atc-server/src/api/routes.rs:327
- Altitude-band compliance is stated but only defined, not enforced in validation. /home/uci/Downloads/EECS-159A-Poster.txt:52, atc-drone/crates/atc-core/src/rules.rs:22, atc-drone/crates/atc-server/src/api/routes.rs:327
- Waze-like low-friction pilot UX is a stated goal, but the UI is an ops control center with multiple admin panels. /home/uci/Downloads/Transcript.txt:73, atc-frontend/README.md:1

### Integration/Data Flow Gaps
- Split-brain routing: frontend A* path planner uses terrain/building context, but backend conflict reroute uses simple geometric avoidance without map context; server reroutes can intersect obstacles. atc-frontend/static/planner/src/route-engine.js:298, atc-drone/crates/atc-core/src/routing.rs:136, atc-drone/crates/atc-server/src/loops/conflict_loop.rs:246
- Conflict geofence sync relies on name heuristics; Blender conflict geofences created by ATC can be re-ingested as external if naming drifts, causing feedback or duplicate tracking. atc-drone/crates/atc-server/src/loops/conflict_loop.rs:110, atc-drone/crates/atc-server/src/loops/geofence_sync_loop.rs:189, atc-drone/crates/atc-server/src/loops/geofence_sync_loop.rs:375
- (Fixed) Standalone planner now pre-creates a Blender declaration and injects `blender_declaration_id` before ATC submission for owner flights, with compliance checks sourced from ATC `/v1/compliance/evaluate`. atc-frontend/views/flight-planner.ejs:34, atc-frontend/views/flight-planner.ejs:310, atc-frontend/static/planner/src/planner.js:1525, atc-frontend/static/planner/src/planner.js:1578, atc-drone/crates/atc-server/src/api/routes.rs:44
- Compliance thresholds are duplicated in frontend and backend; drift risk for pass/fail decisions. atc-frontend/server.js:37, atc-frontend/static/js/mission-plan.js:17, atc-drone/crates/atc-server/src/config.rs:158
  - Fixed: ATC now exposes `/v1/compliance/limits` and the frontend pulls + injects the shared limits for server/client use. atc-drone/crates/atc-server/src/api/routes.rs:35, atc-drone/crates/atc-server/src/api/routes.rs:437, atc-frontend/server.js:37, atc-frontend/views/layouts/main.ejs:35, atc-frontend/views/flight-planner.ejs:37
- Compliance evaluation is duplicated (frontend /api/compliance/* vs backend evaluate_compliance), yielding two sources of truth. atc-frontend/static/js/mission-plan.js:947, atc-drone/crates/atc-server/src/compliance.rs:184
  - Fixed: ATC exposes `/v1/compliance/evaluate` and the planner/mission UI now calls it for compliance results. atc-drone/crates/atc-server/src/api/routes.rs:44, atc-frontend/static/js/mission-plan.js:947, atc-frontend/views/flight-planner.ejs:34
- Hazards are hardcoded twice (server + mission planner) with no shared source of truth. atc-frontend/static/js/mission-plan.js:708, atc-drone/crates/atc-server/src/compliance.rs:896
  - Fixed: Mission planning now consumes hazard lists from ATC compliance reports instead of local constants. atc-drone/crates/atc-server/src/compliance.rs:896, atc-frontend/static/js/mission-plan.js:708, atc-frontend/views/flight-planner.ejs:148
- Flight plan persistence drops trajectory_log and metadata, losing 4D detail after restart. atc-drone/crates/atc-server/src/persistence/flight_plans.rs:21, atc-drone/crates/atc-server/src/persistence/flight_plans.rs:140
  - Fixed: Persist `trajectory_log` and `metadata` to SQLite and restore them on load; init migration updated and runtime column guard added. atc-drone/crates/atc-server/src/persistence/flight_plans.rs:9, atc-drone/crates/atc-server/src/persistence/db.rs:63, atc-drone/crates/atc-server/migrations/001_init.sql:43
- ATC server syncs telemetry/RID/geofences with Blender but does not ingest Blender flight declarations into ATC flight plans. atc-drone/crates/atc-server/src/loops/mod.rs:3, atc-drone/crates/atc-server/src/loops/blender_sync_loop.rs:1, atc-drone/crates/atc-server/src/loops/rid_sync_loop.rs:1
  - Fixed: Added a flight declaration sync loop that pulls Blender declarations and upserts ATC flight plans. atc-drone/crates/atc-server/src/loops/flight_declaration_sync_loop.rs:1, atc-drone/crates/atc-server/src/loops/mod.rs:4, atc-drone/crates/atc-server/src/main.rs:103, atc-drone/crates/atc-blender/src/client.rs:404

### Config/Deployment Drift
- Port mismatch: docs/compose say 5050, server default is 5000, unified compose maps 5000. atc-frontend/README.md:22, atc-frontend/server.js:1432, atc-frontend/docker-compose.yml:23, docker-compose.unified.yml:12
- Unified compose sets ATC_WS_URL=ws://atc-drone:3000/v1/ws, which the browser cannot resolve; realtime will fail. docker-compose.unified.yml:19, atc-frontend/static/js/map.js:452
- .env declares AUTH_STRATEGY/BLENDER_SESSION_ID/MAPBOX_KEY/PASSPORT_/OIDC_ but server only reads a subset; config drift. atc-frontend/.env:3, atc-frontend/.env:7, atc-frontend/.env:26, atc-frontend/.env:29, atc-frontend/.env:37, atc-frontend/server.js:19
- CORS defaults allow localhost:5000/3000 only; docs/compose often use 5050 -> direct calls can be blocked. atc-drone/crates/atc-server/src/config.rs:103, atc-frontend/README.md:22
- Frontend expects BLENDER_SESSION_ID in .env but never uses it; backend reads its own session ID. atc-frontend/.env:7, atc-frontend/server.js:19, atc-drone/crates/atc-server/src/config.rs:96
- CockroachDB runs with `--insecure` and exposes port 26257; no TLS/auth on the DSS store. docker-compose.unified.yml:47
- Hardcoded/default secrets in compose (ATC_REGISTRATION_TOKEN=change-me, BLENDER_AUTH_TOKEN empty, SECRET_KEY set); risk of deploying with demo credentials. docker-compose.unified.yml:16, docker-compose.unified.yml:18, docker-compose.unified.yml:168
- Vendored upstream services (atc-blender, interuss-dss) live in repo; prefer upstream Docker images to reduce drift/patch debt. atc-blender/README.md:1, interuss-dss/README.md:1

### Auth/Access Foot-guns
- Guest login is always shown but crashes if guest user is not seeded; route does not guard null. atc-frontend/views/login.ejs:207, atc-frontend/server.js:335, atc-frontend/README.md:46
- Admin token comparison uses `==` (timing attack); should be constant-time compare. atc-drone/crates/atc-server/src/api/auth.rs:22
- Drone session tokens are UUIDs stored in memory; restarts invalidate all drones and there is no persistence or expiry. atc-drone/crates/atc-server/src/api/routes.rs:183, atc-drone/crates/atc-server/src/state/store.rs:280
- ATC_ALLOW_DEFAULT_USERS seeds admin/admin123 when enabled; risk if env is mis-set in production. atc-frontend/server.js:260, atc-frontend/server.js:281
- DEMO_MODE unlocks /api/rid/demo injection route; fake traffic can be posted if enabled. atc-frontend/server.js:630
- AUTH_STRATEGY=dev_bypass and OIDC envs imply alternate auth, but only session/local auth is implemented. atc-frontend/.env:3, atc-frontend/.env:37, atc-frontend/util/auth.js:1
- Frontend Blender auth ignores PASSPORT_* credentials and uses dummy JWT unless BLENDER_AUTH_TOKEN is set. atc-frontend/server.js:139, atc-frontend/.env:29
  - Risk: dummy JWT signature is hardcoded ("signature"), masking auth misconfig. atc-frontend/server.js:176
- Backend Blender client also uses dummy JWT with fixed audience unless BLENDER_AUTH_TOKEN is supplied; no env-based audience/issuer config. atc-drone/crates/atc-blender/src/client.rs:16, atc-drone/crates/atc-server/src/config.rs:102
  - Risk: dummy JWT signature is literal "dummy"; breaks trust boundary if Blender verifies tokens. atc-drone/crates/atc-blender/src/client.rs:30

## Code-Level Findings (Implementation-level)

### Critical
- Corridor Visualization vs Grid Sampling Mismatch
  - Issue: Cyan corridor was 8m radius but grid sampling used 5m spacing. Buildings between sample points passed through visual corridor.
  - Fixed: Added `sampleCorridorHeights()` with 3m spacing within 10m radius.
  - Evidence: atc-frontend/static/planner/src/planner.js:801-912

### High
- Fire-and-forget DB writes update memory even if SQLite write fails; state can regress after restart. atc-drone/crates/atc-server/src/state/store.rs:187
- Conflict detector samples at 1-second intervals; fast movers can miss violations between ticks. atc-drone/crates/atc-core/src/conflict.rs:212
- Unbounded drone registration/telemetry can grow in-memory state; no max drone cap or eviction policy. atc-drone/crates/atc-server/src/api/routes.rs:183, atc-drone/crates/atc-server/src/state/store.rs:23
- SAFETY_BUFFER_M mismatch between modules
  - route-engine.js: 15m; planner.js and route-planner.js: 20m.
  - Impact: A* path clears buildings by 15m, but validation requires 20m; routes can fail visual safety.
  - Evidence: atc-frontend/static/planner/src/route-engine.js:26
- route-planner.js lacks corridor sampling fix
  - Fix is only in planner.js (iframe).
  - Impact: Mission plan routes may still pass through buildings.
- Altitude ellipsoid vs AGL confusion
  - Waypoint API uses "meters above ellipsoid" but UI displays "m AGL".
  - No consistent conversion between reference frames.

### Medium
- Conflict detection is O(N^2); scales poorly beyond demo fleet sizes. atc-drone/crates/atc-core/src/conflict.rs:254
- Geofence checks scan every geofence/segment; no spatial index for point or route checks. atc-drone/crates/atc-server/src/api/geofences.rs:164, atc-drone/crates/atc-server/src/api/geofences.rs:206
- Route engine line-of-sight uses point sampling; thin obstacles can be skipped between samples. atc-frontend/static/planner/src/route-engine.js:213
- Route engine A* runs on the main UI thread; dense routes can freeze the browser. atc-frontend/static/planner/src/route-engine.js:298
- Frontend map runs polling loops even when WebSocket is active; duplicate load and jitter. atc-frontend/static/js/map.js:165, atc-frontend/static/js/map.js:367
- Loop error handling logs and continues without backoff/circuit breaker; external outages can spam logs. atc-drone/crates/atc-server/src/loops/rid_sync_loop.rs:67
- File-based session store in Docker container can cause IO contention under load. atc-frontend/server.js:17, atc-frontend/server.js:313
- Mixed geodesic vs flat-earth math; error bounds are undefined for larger sectors. atc-drone/crates/atc-core/src/spatial.rs:259, atc-drone/crates/atc-core/src/spatial.rs:358
- DAA endpoint exists but overlaps conflict advisory logic with no clear behavioral separation. atc-drone/crates/atc-server/src/api/daa.rs:1, atc-drone/crates/atc-server/src/loops/conflict_loop.rs:141
- Geofence advisory zone filtering inconsistent
  - /v1/geofences/check-route includes advisory; flight plan validation and route engine do not.
- Failsafe only issues HOLD (120s), not LAND or RTH
  - Requirements call for hold/land/return-to-home.
  - Evidence: atc-drone/crates/atc-server/src/loops/conflict_loop.rs:65-74
- Health telemetry not surfaced in UI
  - Requirements call for GPS lock, IMU status, link quality; not shown.
- setInterval without clearInterval
  - 12+ intervals across frontend JS; no cleanup on navigation.
- Conflict prediction horizon not configurable
  - Requirements target 10-30 seconds; implementation is fixed.

### Low
- Hardcoded Irvine coordinates in routing defaults; should be configurable. atc-drone/crates/atc-core/src/routing.rs:7
- Cesium Ion token hardcoded in multiple files; no centralized config.
- Console logs left in production (planner/route engine logging).
- Deprecated NPM packages in package-lock.json: request, older uuid, etc.

## Requirements Not Implemented (Summary)
| Requirement | Status |
|-------------|--------|
| STM32 firmware | ❌ Not implemented |
| GPS/IMU UART drivers | ❌ Not implemented |
| LoRa telemetry link | ❌ Not implemented |
| Gateway service (Flask/FastAPI) | ❌ Not implemented |
| MAVLink protocol/bridge | ❌ Not implemented |
| Python multi-drone simulation | ⚠️ Rust only |
| Health telemetry UI | ❌ Not implemented |
| Hard failsafe (LAND/RTH) | ⚠️ HOLD only |

## Priority Fixes
1. Fix fire-and-forget DB writes (await/transactional writes before state update).
2. Replace conflict detection sampling with continuous CPA + spatial index to avoid missed collisions and O(N^2) scaling.
3. Consolidate routing so backend owns obstacle-aware routing (port route engine + OSM ingestion to atc-core).
4. Harden auth (constant-time admin token compare; remove dummy JWT fallbacks; persist drone sessions).
5. Align SAFETY_BUFFER_M -> change route-engine.js from 15m to 20m.
6. Port corridor sampling fix to route-planner.js.
7. Add failsafe escalation -> HOLD -> LAND -> RTH.
8. Clear intervals on page unload.
9. Update deprecated npm packages.
