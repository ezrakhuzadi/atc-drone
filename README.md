# ATC Drone System

Local traffic management system for cooperative UAVs using OpenUTM (Flight Blender + Flight Spotlight).

## Overview

This project provides the "ATC brain" that sits between drones and the OpenUTM stack:
- **Telemetry injection** - Sends drone positions to Flight Blender
- **Conflict detection** - Predicts collisions 10-30s ahead (planned)
- **Command issuance** - Sends HOLD/REROUTE/ALTITUDE commands (planned)

## Project Structure

```
atc-drone/
├── scripts/
│   └── generate_token.py   # JWT token generator for Blender API
├── sim/
│   └── send_one_track.py   # Simulated drone telemetry injector
├── gateway/                 # (Planned) FastAPI telemetry gateway
├── conflict/                # (Planned) Conflict detection engine
└── requirements.txt
```

## Quick Start

```bash
# Install dependencies
pip install -r requirements.txt

# Generate a token
python scripts/generate_token.py

# Run the simulator (sends drone telemetry to Blender)
python sim/send_one_track.py --duration 60 --rate 1
```

## Related Repos

- [flight-blender-irvine](https://github.com/ezrakhuzadi/flight-blender-irvine) - UTM Backend
- [flight-spotlight-irvine](https://github.com/ezrakhuzadi/flight-spotlight-irvine) - 3D UI

## Requirements

- Python 3.10+
- Flight Blender running on localhost:8000
