# ATC Drone System (Rust)

Local traffic management system for cooperative UAVs using OpenUTM (Flight Blender + Flight Spotlight).

## Overview

This project provides the "ATC brain" that sits between drones and the OpenUTM stack:
- **Telemetry injection** - Sends drone positions to Flight Blender
- **Conflict detection** - Predicts collisions 10-30s ahead
- **Multi-drone scenarios** - Crossing, parallel, and converging flight paths

## Project Structure

```
atc-drone-rs/
├── src/
│   ├── lib.rs              # Library root
│   ├── auth/mod.rs         # JWT token generation
│   ├── conflict/mod.rs     # Conflict detection engine
│   └── sim/                # Telemetry simulation
│       ├── mod.rs
│       ├── client.rs       # Flight Blender HTTP client
│       ├── paths.rs        # Circular/Linear flight paths
│       └── scenarios.rs    # Test scenarios
└── src/bin/
    ├── generate_token.rs   # JWT generator CLI
    ├── send_one_track.rs   # Single drone simulator
    └── send_multi_track.rs # Multi-drone scenarios
```

## Quick Start

```bash
# Build the project
cargo build --release

# Generate a token
./target/release/generate_token

# Run single drone simulator
./target/release/send_one_track --duration 60 --rate 1

# Run multi-drone scenario (crossing paths)
./target/release/send_multi_track --scenario crossing --duration 60
```

## CLI Options

### generate_token
```
--scopes    Space-separated scopes (default: "flightblender.read flightblender.write")
--audience  Token audience (default: "testflight.flightblender.com")
--expiry    Token expiry in hours (default: 24)
```

### send_one_track
```
--url       Flight Blender URL (default: http://localhost:8000)
--session   Session UUID
--icao      Drone identifier (default: DRONE001)
--lat/--lon Center coordinates (default: Irvine, CA)
--radius    Circle radius in meters (default: 200)
--altitude  Flight altitude in meters (default: 50)
--duration  Duration in seconds (default: 60)
--rate      Update rate in Hz (default: 1.0)
```

### send_multi_track
```
--scenario  Scenario type: crossing, parallel, converging
            (other options same as send_one_track)
```

## Library Usage

```rust
use atc_drone::{ConflictDetector, DronePosition, ConflictSeverity};

fn main() {
    let mut detector = ConflictDetector::default();
    
    detector.update_position(DronePosition::new("DRONE001", 33.68, -117.82, 50.0));
    detector.update_position(DronePosition::new("DRONE002", 33.68, -117.82, 55.0));
    
    for conflict in detector.detect_conflicts() {
        println!("{:?}: {} vs {}", 
            conflict.severity, 
            conflict.drone1_id, 
            conflict.drone2_id
        );
    }
}
```

## Related Repos

- [flight-blender-irvine](https://github.com/ezrakhuzadi/flight-blender-irvine) - UTM Backend
- [flight-spotlight-irvine](https://github.com/ezrakhuzadi/flight-spotlight-irvine) - 3D UI

## Requirements

- Rust 1.70+
- Flight Blender running on localhost:8000 (for simulator tools)
