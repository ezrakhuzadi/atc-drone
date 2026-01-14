#!/usr/bin/env python3
"""
Multi-drone simulator with conflict scenarios.

This script simulates multiple drones on configurable flight paths,
including crossing scenarios for testing conflict detection.

Scenarios:
  - crossing: Two drones on collision course
  - parallel: Two drones flying parallel paths
  - converging: Multiple drones converging on a point
"""

import requests
import time
import argparse
import math
import sys
import os
import threading
from typing import List, Tuple, Callable
from dataclasses import dataclass

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from scripts.generate_token import generate_dummy_token


@dataclass
class DroneState:
    """Current state of a simulated drone."""
    id: str
    lat: float
    lon: float
    altitude_m: float
    heading: float  # degrees
    speed_mps: float


class FlightPath:
    """Base class for flight paths."""
    
    def get_position(self, t: float) -> Tuple[float, float, float]:
        """Get (lat, lon, alt) at time t seconds from start."""
        raise NotImplementedError


class CircularPath(FlightPath):
    """Circular flight path around a center point."""
    
    def __init__(self, center_lat: float, center_lon: float, 
                 radius_m: float, altitude_m: float, speed_mps: float = 10,
                 start_angle: float = 0, clockwise: bool = True):
        self.center_lat = center_lat
        self.center_lon = center_lon
        self.radius_m = radius_m
        self.altitude_m = altitude_m
        self.speed_mps = speed_mps
        self.start_angle = start_angle
        self.clockwise = clockwise
        
        # Calculate angular velocity
        circumference = 2 * math.pi * radius_m
        self.period = circumference / speed_mps  # seconds per revolution
        
    def get_position(self, t: float) -> Tuple[float, float, float]:
        # Angular position
        angle_rad = self.start_angle + (2 * math.pi * t / self.period)
        if self.clockwise:
            angle_rad = -angle_rad
            
        # Convert radius from meters to degrees (approximate)
        lat_offset = (self.radius_m / 111320) * math.cos(angle_rad)
        lon_offset = (self.radius_m / (111320 * math.cos(math.radians(self.center_lat)))) * math.sin(angle_rad)
        
        return (
            self.center_lat + lat_offset,
            self.center_lon + lon_offset,
            self.altitude_m
        )


class LinearPath(FlightPath):
    """Linear flight path between two points."""
    
    def __init__(self, start_lat: float, start_lon: float,
                 end_lat: float, end_lon: float,
                 altitude_m: float, speed_mps: float = 15):
        self.start_lat = start_lat
        self.start_lon = start_lon
        self.end_lat = end_lat
        self.end_lon = end_lon
        self.altitude_m = altitude_m
        self.speed_mps = speed_mps
        
        # Calculate total distance and duration
        self.distance_m = self._haversine_distance(
            start_lat, start_lon, end_lat, end_lon
        )
        self.duration = self.distance_m / speed_mps
        
    def _haversine_distance(self, lat1: float, lon1: float, 
                            lat2: float, lon2: float) -> float:
        """Calculate distance between two points in meters."""
        R = 6371000  # Earth radius in meters
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)
        
        a = math.sin(dphi/2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda/2)**2
        return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1-a))
        
    def get_position(self, t: float) -> Tuple[float, float, float]:
        # Clamp progress to [0, 1]
        progress = min(1.0, max(0.0, t / self.duration)) if self.duration > 0 else 0
        
        lat = self.start_lat + progress * (self.end_lat - self.start_lat)
        lon = self.start_lon + progress * (self.end_lon - self.start_lon)
        
        return (lat, lon, self.altitude_m)


def create_crossing_scenario(center_lat: float, center_lon: float) -> List[Tuple[str, FlightPath]]:
    """
    Create two drones on collision course.
    
    Drone 1: Flying West to East through center
    Drone 2: Flying South to North through center
    They will cross paths at the center point.
    """
    offset = 0.003  # ~300m offset in degrees
    
    # Drone 1: West to East
    drone1_path = LinearPath(
        start_lat=center_lat,
        start_lon=center_lon - offset,
        end_lat=center_lat,
        end_lon=center_lon + offset,
        altitude_m=50,
        speed_mps=10
    )
    
    # Drone 2: South to North  
    drone2_path = LinearPath(
        start_lat=center_lat - offset,
        start_lon=center_lon,
        end_lat=center_lat + offset,
        end_lon=center_lon,
        altitude_m=50,
        speed_mps=10
    )
    
    return [
        ("DRONE001", drone1_path),
        ("DRONE002", drone2_path)
    ]


def create_parallel_scenario(center_lat: float, center_lon: float) -> List[Tuple[str, FlightPath]]:
    """Create two drones flying parallel paths (no conflict)."""
    offset = 0.003
    separation = 0.001  # ~100m separation
    
    drone1_path = LinearPath(
        start_lat=center_lat,
        start_lon=center_lon - offset,
        end_lat=center_lat,
        end_lon=center_lon + offset,
        altitude_m=50,
        speed_mps=10
    )
    
    drone2_path = LinearPath(
        start_lat=center_lat + separation,
        start_lon=center_lon - offset,
        end_lat=center_lat + separation,
        end_lon=center_lon + offset,
        altitude_m=50,
        speed_mps=10
    )
    
    return [
        ("DRONE001", drone1_path),
        ("DRONE002", drone2_path)
    ]


def create_converging_scenario(center_lat: float, center_lon: float) -> List[Tuple[str, FlightPath]]:
    """Create multiple drones converging on a central point."""
    offset = 0.003
    
    paths = []
    for i, angle in enumerate([0, 90, 180, 270]):  # 4 drones from cardinal directions
        angle_rad = math.radians(angle)
        start_lat = center_lat + offset * math.cos(angle_rad)
        start_lon = center_lon + offset * math.sin(angle_rad)
        
        path = LinearPath(
            start_lat=start_lat,
            start_lon=start_lon,
            end_lat=center_lat,
            end_lon=center_lon,
            altitude_m=50,
            speed_mps=8
        )
        paths.append((f"DRONE{i+1:03d}", path))
    
    return paths


SCENARIOS = {
    "crossing": create_crossing_scenario,
    "parallel": create_parallel_scenario,
    "converging": create_converging_scenario,
}


def send_observation(
    blender_url: str,
    session_id: str,
    token: str,
    drone_id: str,
    lat: float,
    lon: float,
    altitude_m: float,
    heading: float = 0,
    speed_mps: float = 10
) -> int:
    """Send a single observation to Flight Blender."""
    url = f"{blender_url}/flight_stream/set_air_traffic/{session_id}"
    
    observation = {
        "observations": [{
            "lat_dd": lat,
            "lon_dd": lon,
            "altitude_mm": int(altitude_m * 1000),
            "icao_address": drone_id,
            "traffic_source": 1,
            "source_type": 1,
            "timestamp": int(time.time()),
            "metadata": {
                "heading": heading,
                "speed_mps": speed_mps,
                "aircraft_type": "UAV"
            }
        }]
    }
    
    headers = {
        "Content-Type": "application/json",
        "Authorization": f"Bearer {token}"
    }
    
    try:
        response = requests.post(url, json=observation, headers=headers, timeout=5)
        return response.status_code
    except requests.exceptions.RequestException as e:
        print(f"[{drone_id}] Error: {e}")
        return 0


def simulate_multi_drone(
    blender_url: str,
    session_id: str,
    drones: List[Tuple[str, FlightPath]],
    duration_s: float,
    update_rate_hz: float = 1.0
):
    """
    Simulate multiple drones simultaneously.
    
    Args:
        blender_url: Flight Blender base URL
        session_id: Session ID for API calls
        drones: List of (drone_id, flight_path) tuples
        duration_s: Simulation duration in seconds
        update_rate_hz: Position update rate
    """
    print("Generating authentication token...")
    token = generate_dummy_token()
    
    print(f"\nStarting multi-drone simulation")
    print(f"  Drones: {len(drones)}")
    print(f"  Duration: {duration_s}s, Update rate: {update_rate_hz}Hz\n")
    
    for drone_id, _ in drones:
        print(f"  - {drone_id}")
    print()
    
    start_time = time.time()
    update_count = 0
    
    while True:
        elapsed = time.time() - start_time
        if elapsed >= duration_s:
            break
            
        update_count += 1
        
        # Update all drones
        for drone_id, path in drones:
            lat, lon, alt = path.get_position(elapsed)
            status = send_observation(
                blender_url, session_id, token,
                drone_id, lat, lon, alt
            )
            print(f"[{update_count:3d}] {drone_id}: ({lat:.6f}, {lon:.6f}) -> {status}")
        
        # Sleep until next update
        time.sleep(1.0 / update_rate_hz)
    
    print(f"\nSimulation complete. Sent {update_count * len(drones)} total updates.")


def main():
    parser = argparse.ArgumentParser(
        description="Multi-drone simulator with conflict scenarios"
    )
    parser.add_argument(
        "--url",
        default="http://localhost:8000",
        help="Flight Blender base URL"
    )
    parser.add_argument(
        "--session",
        default="00000000-0000-0000-0000-000000000000",
        help="Session ID (optional)"
    )
    parser.add_argument(
        "--scenario",
        choices=list(SCENARIOS.keys()),
        default="crossing",
        help="Scenario to simulate"
    )
    parser.add_argument(
        "--lat",
        type=float,
        default=33.6846,
        help="Center latitude (default: Irvine, CA)"
    )
    parser.add_argument(
        "--lon",
        type=float,
        default=-117.8265,
        help="Center longitude (default: Irvine, CA)"
    )
    parser.add_argument(
        "--duration",
        type=int,
        default=60,
        help="Simulation duration in seconds"
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=1.0,
        help="Update rate in Hz"
    )
    
    args = parser.parse_args()
    
    # Create scenario
    scenario_func = SCENARIOS[args.scenario]
    drones = scenario_func(args.lat, args.lon)
    
    print(f"Scenario: {args.scenario}")
    
    simulate_multi_drone(
        blender_url=args.url,
        session_id=args.session,
        drones=drones,
        duration_s=args.duration,
        update_rate_hz=args.rate
    )


if __name__ == "__main__":
    main()
