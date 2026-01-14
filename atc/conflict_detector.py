"""
Conflict Detection Engine for ATC-Drone.

This module provides real-time conflict detection with lookahead prediction
for multiple drones operating in the same airspace.

Conflict detection uses:
- Position extrapolation based on current velocity
- Configurable separation minimums (horizontal/vertical)
- Lookahead window (default 10-30 seconds)
"""

import math
import time
from dataclasses import dataclass, field
from typing import List, Dict, Optional, Tuple
from enum import Enum


class ConflictSeverity(Enum):
    """Severity levels for detected conflicts."""
    INFO = "info"           # Drones on converging paths but far
    WARNING = "warning"     # Conflict predicted within lookahead window
    CRITICAL = "critical"   # Immediate separation violation


@dataclass
class DronePosition:
    """Current position and velocity of a drone."""
    drone_id: str
    lat: float
    lon: float
    altitude_m: float
    heading_deg: float = 0.0
    speed_mps: float = 0.0
    timestamp: float = field(default_factory=time.time)
    
    def to_dict(self) -> dict:
        return {
            "drone_id": self.drone_id,
            "lat": self.lat,
            "lon": self.lon,
            "altitude_m": self.altitude_m,
            "heading_deg": self.heading_deg,
            "speed_mps": self.speed_mps,
            "timestamp": self.timestamp
        }


@dataclass
class Conflict:
    """Detected conflict between two drones."""
    drone1_id: str
    drone2_id: str
    severity: ConflictSeverity
    distance_m: float
    time_to_closest: float  # seconds
    closest_distance_m: float
    timestamp: float = field(default_factory=time.time)
    
    def to_dict(self) -> dict:
        return {
            "drone1_id": self.drone1_id,
            "drone2_id": self.drone2_id,
            "severity": self.severity.value,
            "distance_m": self.distance_m,
            "time_to_closest": self.time_to_closest,
            "closest_distance_m": self.closest_distance_m,
            "timestamp": self.timestamp
        }


class ConflictDetector:
    """
    Real-time conflict detection engine.
    
    Uses position extrapolation to predict conflicts within a
    configurable lookahead window.
    """
    
    def __init__(
        self,
        lookahead_seconds: float = 20.0,
        separation_horizontal_m: float = 50.0,
        separation_vertical_m: float = 30.0,
        warning_multiplier: float = 2.0
    ):
        """
        Initialize conflict detector.
        
        Args:
            lookahead_seconds: How far ahead to predict (default 20s)
            separation_horizontal_m: Minimum horizontal separation
            separation_vertical_m: Minimum vertical separation  
            warning_multiplier: Multiplier for warning threshold
        """
        self.lookahead_seconds = lookahead_seconds
        self.separation_horizontal_m = separation_horizontal_m
        self.separation_vertical_m = separation_vertical_m
        self.warning_multiplier = warning_multiplier
        
        # Track drone positions
        self.drones: Dict[str, DronePosition] = {}
        
        # Track active conflicts
        self.active_conflicts: Dict[Tuple[str, str], Conflict] = {}
        
    def update_position(self, position: DronePosition) -> None:
        """Update tracked position for a drone."""
        self.drones[position.drone_id] = position
        
    def remove_drone(self, drone_id: str) -> None:
        """Remove a drone from tracking."""
        if drone_id in self.drones:
            del self.drones[drone_id]
            
        # Remove any conflicts involving this drone
        to_remove = [k for k in self.active_conflicts if drone_id in k]
        for key in to_remove:
            del self.active_conflicts[key]
    
    def _haversine_distance(
        self, lat1: float, lon1: float, lat2: float, lon2: float
    ) -> float:
        """Calculate horizontal distance between two points in meters."""
        R = 6371000  # Earth radius in meters
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)
        
        a = math.sin(dphi/2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda/2)**2
        return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1-a))
    
    def _predict_position(
        self, drone: DronePosition, time_offset_s: float
    ) -> Tuple[float, float, float]:
        """
        Predict drone position after time_offset_s seconds.
        
        Uses simple linear extrapolation based on heading and speed.
        """
        if drone.speed_mps <= 0:
            return (drone.lat, drone.lon, drone.altitude_m)
            
        # Distance traveled
        distance_m = drone.speed_mps * time_offset_s
        
        # Convert heading to radians (0 = North, clockwise)
        heading_rad = math.radians(drone.heading_deg)
        
        # Calculate offset in meters
        north_m = distance_m * math.cos(heading_rad)
        east_m = distance_m * math.sin(heading_rad)
        
        # Convert to lat/lon offset
        lat_offset = north_m / 111320
        lon_offset = east_m / (111320 * math.cos(math.radians(drone.lat)))
        
        return (
            drone.lat + lat_offset,
            drone.lon + lon_offset,
            drone.altitude_m  # Assuming constant altitude
        )
    
    def _check_separation(
        self, pos1: Tuple[float, float, float], pos2: Tuple[float, float, float]
    ) -> Tuple[float, float]:
        """
        Check separation between two positions.
        
        Returns:
            (horizontal_distance_m, vertical_distance_m)
        """
        horizontal = self._haversine_distance(pos1[0], pos1[1], pos2[0], pos2[1])
        vertical = abs(pos1[2] - pos2[2])
        return (horizontal, vertical)
    
    def _find_closest_approach(
        self, drone1: DronePosition, drone2: DronePosition
    ) -> Tuple[float, float]:
        """
        Find time and distance of closest approach.
        
        Returns:
            (time_to_closest_s, closest_distance_m)
        """
        min_distance = float('inf')
        time_of_min = 0
        
        # Sample at 1-second intervals
        for t in range(int(self.lookahead_seconds) + 1):
            pos1 = self._predict_position(drone1, t)
            pos2 = self._predict_position(drone2, t)
            h_dist, v_dist = self._check_separation(pos1, pos2)
            
            # 3D distance (weighted vertical)
            distance = math.sqrt(h_dist**2 + v_dist**2)
            
            if distance < min_distance:
                min_distance = distance
                time_of_min = t
                
        return (time_of_min, min_distance)
    
    def detect_conflicts(self) -> List[Conflict]:
        """
        Check all tracked drones for conflicts.
        
        Returns:
            List of detected conflicts
        """
        conflicts = []
        drone_list = list(self.drones.values())
        
        # Check all pairs
        for i in range(len(drone_list)):
            for j in range(i + 1, len(drone_list)):
                drone1 = drone_list[i]
                drone2 = drone_list[j]
                
                # Check current separation
                h_dist, v_dist = self._check_separation(
                    (drone1.lat, drone1.lon, drone1.altitude_m),
                    (drone2.lat, drone2.lon, drone2.altitude_m)
                )
                current_distance = math.sqrt(h_dist**2 + v_dist**2)
                
                # Check for current violation
                if (h_dist < self.separation_horizontal_m and 
                    v_dist < self.separation_vertical_m):
                    conflicts.append(Conflict(
                        drone1_id=drone1.drone_id,
                        drone2_id=drone2.drone_id,
                        severity=ConflictSeverity.CRITICAL,
                        distance_m=current_distance,
                        time_to_closest=0,
                        closest_distance_m=current_distance
                    ))
                    continue
                
                # Check predicted conflicts
                time_to_closest, closest_distance = self._find_closest_approach(
                    drone1, drone2
                )
                
                # Warning threshold
                warning_h = self.separation_horizontal_m * self.warning_multiplier
                warning_v = self.separation_vertical_m * self.warning_multiplier
                
                if closest_distance < self.separation_horizontal_m:
                    severity = ConflictSeverity.CRITICAL
                elif closest_distance < warning_h:
                    severity = ConflictSeverity.WARNING
                else:
                    continue  # No conflict
                    
                conflicts.append(Conflict(
                    drone1_id=drone1.drone_id,
                    drone2_id=drone2.drone_id,
                    severity=severity,
                    distance_m=current_distance,
                    time_to_closest=time_to_closest,
                    closest_distance_m=closest_distance
                ))
        
        # Update active conflicts
        self.active_conflicts = {}
        for conflict in conflicts:
            key = tuple(sorted([conflict.drone1_id, conflict.drone2_id]))
            self.active_conflicts[key] = conflict
            
        return conflicts
    
    def get_all_positions(self) -> List[DronePosition]:
        """Get all tracked drone positions."""
        return list(self.drones.values())


# Global detector instance for easy access
_detector: Optional[ConflictDetector] = None


def get_detector() -> ConflictDetector:
    """Get the global conflict detector instance."""
    global _detector
    if _detector is None:
        _detector = ConflictDetector()
    return _detector


def reset_detector() -> None:
    """Reset the global detector."""
    global _detector
    _detector = None
