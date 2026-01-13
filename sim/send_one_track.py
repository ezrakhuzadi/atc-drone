#!/usr/bin/env python3
"""
Send simulated drone telemetry to Flight Blender.

This script sends a single drone track (position updates) to the
Flight Blender set_air_traffic API endpoint.

API Endpoint: POST /set_air_traffic/{session_id}

Required observation fields:
  - lat_dd: float (latitude in decimal degrees)
  - lon_dd: float (longitude in decimal degrees)
  - altitude_mm: float (altitude in millimeters)
  - icao_address: str (unique drone identifier)
  - traffic_source: int (0-11, source type enum)
  - timestamp: int (Unix timestamp in seconds)
  - source_type: int (optional, 0-9)
  - metadata: dict (optional)

Traffic source values:
  0 = Unknown
  1 = ADS-B
  2 = FLARM
  3 = Radar
  4 = MLAT
  5-11 = Various others (including telemetry/RemoteID)
"""

import requests
import time
import argparse
import math
import sys
import os

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from scripts.generate_token import generate_dummy_token


def send_observation(
    blender_url: str,
    session_id: str,
    token: str,
    lat: float,
    lon: float,
    altitude_m: float,
    icao_address: str,
    traffic_source: int = 1,
    source_type: int = 1
) -> dict:
    """Send a single observation to Flight Blender.
    
    Args:
        blender_url: Base URL of Flight Blender (e.g., http://localhost:8000)
        session_id: UUID session identifier
        token: JWT bearer token
        lat: Latitude in decimal degrees
        lon: Longitude in decimal degrees  
        altitude_m: Altitude in meters
        icao_address: Unique drone identifier
        traffic_source: Traffic source type (default: 1 = ADS-B)
        source_type: Source type (default: 1)
        
    Returns:
        Response JSON from Blender API
    """
    url = f"{blender_url}/flight_stream/set_air_traffic/{session_id}"
    
    observation = {
        "observations": [{
            "lat_dd": lat,
            "lon_dd": lon,
            "altitude_mm": int(altitude_m * 1000),  # Convert m to mm
            "icao_address": icao_address,
            "traffic_source": traffic_source,
            "source_type": source_type,
            "timestamp": int(time.time()),
            "metadata": {
                "heading": 0,
                "speed_mps": 10,
                "aircraft_type": "UAV"
            }
        }]
    }
    
    headers = {
        "Content-Type": "application/json",
        "Authorization": f"Bearer {token}"
    }
    
    response = requests.post(url, json=observation, headers=headers)
    return response.json(), response.status_code


def simulate_circular_flight(
    blender_url: str,
    session_id: str,
    token: str,
    icao_address: str,
    center_lat: float,
    center_lon: float,
    radius_m: float = 200,
    altitude_m: float = 50,
    duration_s: int = 60,
    update_rate_hz: float = 1.0
):
    """Simulate a drone flying in a circle and send updates to Blender.
    
    Args:
        blender_url: Base URL of Flight Blender
        session_id: UUID session identifier
        token: JWT bearer token
        icao_address: Unique drone identifier
        center_lat: Center latitude of circle
        center_lon: Center longitude of circle
        radius_m: Circle radius in meters (default: 200)
        altitude_m: Flight altitude in meters (default: 50)
        duration_s: Flight duration in seconds (default: 60)
        update_rate_hz: Position update rate in Hz (default: 1)
    """
    # Approximate degrees per meter at given latitude
    lat_deg_per_m = 1 / 111320
    lon_deg_per_m = 1 / (111320 * math.cos(math.radians(center_lat)))
    
    radius_lat = radius_m * lat_deg_per_m
    radius_lon = radius_m * lon_deg_per_m
    
    angular_velocity = 2 * math.pi / 60  # One full circle per minute
    
    start_time = time.time()
    update_count = 0
    
    print(f"Starting circular flight simulation for {icao_address}")
    print(f"  Center: ({center_lat}, {center_lon})")
    print(f"  Radius: {radius_m}m, Altitude: {altitude_m}m")
    print(f"  Duration: {duration_s}s, Update rate: {update_rate_hz}Hz")
    print()
    
    while time.time() - start_time < duration_s:
        elapsed = time.time() - start_time
        angle = angular_velocity * elapsed
        
        lat = center_lat + radius_lat * math.sin(angle)
        lon = center_lon + radius_lon * math.cos(angle)
        
        result, status = send_observation(
            blender_url=blender_url,
            session_id=session_id,
            token=token,
            lat=lat,
            lon=lon,
            altitude_m=altitude_m,
            icao_address=icao_address
        )
        
        update_count += 1
        print(f"[{update_count}] Sent position ({lat:.6f}, {lon:.6f}) -> Status: {status}")
        
        time.sleep(1.0 / update_rate_hz)
    
    print(f"\nSimulation complete. Sent {update_count} position updates.")


def main():
    parser = argparse.ArgumentParser(description="Send drone telemetry to Flight Blender")
    parser.add_argument("--url", default="http://localhost:8000",
                        help="Flight Blender URL (default: http://localhost:8000)")
    parser.add_argument("--session", default="00000000-0000-0000-0000-000000000001",
                        help="Session UUID")
    parser.add_argument("--icao", default="DRONE001",
                        help="Drone ICAO address/identifier")
    parser.add_argument("--lat", type=float, default=33.6846,
                        help="Center latitude (default: 33.6846 - UCI)")
    parser.add_argument("--lon", type=float, default=-117.8265,
                        help="Center longitude (default: -117.8265 - UCI)")
    parser.add_argument("--radius", type=float, default=200,
                        help="Circle radius in meters (default: 200)")
    parser.add_argument("--altitude", type=float, default=50,
                        help="Altitude in meters (default: 50)")
    parser.add_argument("--duration", type=int, default=60,
                        help="Duration in seconds (default: 60)")
    parser.add_argument("--rate", type=float, default=1.0,
                        help="Update rate in Hz (default: 1.0)")
    
    args = parser.parse_args()
    
    print("Generating authentication token...")
    token = generate_dummy_token()
    
    simulate_circular_flight(
        blender_url=args.url,
        session_id=args.session,
        token=token,
        icao_address=args.icao,
        center_lat=args.lat,
        center_lon=args.lon,
        radius_m=args.radius,
        altitude_m=args.altitude,
        duration_s=args.duration,
        update_rate_hz=args.rate
    )


if __name__ == "__main__":
    main()
