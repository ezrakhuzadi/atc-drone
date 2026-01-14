"""
Test suite for crossing scenario conflict detection.

This tests that the conflict detector correctly identifies when
two drones are on a collision course.
"""

import sys
import os
import pytest
import math

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from atc.conflict_detector import (
    ConflictDetector, DronePosition, Conflict, ConflictSeverity
)


class TestConflictDetector:
    """Test the ConflictDetector class."""
    
    def setup_method(self):
        """Set up fresh detector for each test."""
        self.detector = ConflictDetector(
            lookahead_seconds=20,
            separation_horizontal_m=50,
            separation_vertical_m=30
        )
    
    def test_no_conflict_when_far_apart(self):
        """Drones far apart should not trigger conflict."""
        # Two drones 1km apart
        self.detector.update_position(DronePosition(
            drone_id="DRONE001",
            lat=33.6846,
            lon=-117.8265,
            altitude_m=50,
            heading_deg=90,
            speed_mps=0
        ))
        self.detector.update_position(DronePosition(
            drone_id="DRONE002",
            lat=33.6946,  # ~1km north
            lon=-117.8265,
            altitude_m=50,
            heading_deg=270,
            speed_mps=0
        ))
        
        conflicts = self.detector.detect_conflicts()
        assert len(conflicts) == 0
    
    def test_immediate_conflict_when_too_close(self):
        """Drones within separation minimum should trigger CRITICAL."""
        # Two drones 30m apart (below 50m minimum)
        self.detector.update_position(DronePosition(
            drone_id="DRONE001",
            lat=33.6846,
            lon=-117.8265,
            altitude_m=50,
            heading_deg=0,
            speed_mps=0
        ))
        self.detector.update_position(DronePosition(
            drone_id="DRONE002",
            lat=33.684627,  # ~30m north
            lon=-117.8265,
            altitude_m=50,
            heading_deg=0,
            speed_mps=0
        ))
        
        conflicts = self.detector.detect_conflicts()
        assert len(conflicts) == 1
        assert conflicts[0].severity == ConflictSeverity.CRITICAL
    
    def test_crossing_paths_detected(self):
        """Two drones on crossing paths should trigger WARNING/CRITICAL."""
        # Drone 1: Flying east through intersection
        self.detector.update_position(DronePosition(
            drone_id="DRONE001",
            lat=33.6846,
            lon=-117.8275,  # 100m west of intersection
            altitude_m=50,
            heading_deg=90,  # Heading east
            speed_mps=10
        ))
        # Drone 2: Flying north through same intersection
        self.detector.update_position(DronePosition(
            drone_id="DRONE002",
            lat=33.6836,  # 100m south of intersection
            lon=-117.8265,
            altitude_m=50,
            heading_deg=0,  # Heading north
            speed_mps=10
        ))
        
        conflicts = self.detector.detect_conflicts()
        assert len(conflicts) == 1
        # Should predict conflict as they're converging
        assert conflicts[0].severity in [ConflictSeverity.WARNING, ConflictSeverity.CRITICAL]
        assert conflicts[0].time_to_closest < 20  # Within lookahead
    
    def test_parallel_paths_no_conflict(self):
        """Drones on parallel paths should not trigger conflict."""
        # Two drones flying east, 200m apart
        self.detector.update_position(DronePosition(
            drone_id="DRONE001",
            lat=33.6846,
            lon=-117.8275,
            altitude_m=50,
            heading_deg=90,
            speed_mps=10
        ))
        self.detector.update_position(DronePosition(
            drone_id="DRONE002",
            lat=33.6866,  # ~200m north
            lon=-117.8275,
            altitude_m=50,
            heading_deg=90,
            speed_mps=10
        ))
        
        conflicts = self.detector.detect_conflicts()
        assert len(conflicts) == 0
    
    def test_vertical_separation_safe(self):
        """Drones separated vertically should not conflict."""
        # Same horizontal position but 100m vertical separation
        self.detector.update_position(DronePosition(
            drone_id="DRONE001",
            lat=33.6846,
            lon=-117.8265,
            altitude_m=50,
            heading_deg=90,
            speed_mps=10
        ))
        self.detector.update_position(DronePosition(
            drone_id="DRONE002",
            lat=33.6846,
            lon=-117.8265,
            altitude_m=150,  # 100m higher
            heading_deg=90,
            speed_mps=10
        ))
        
        conflicts = self.detector.detect_conflicts()
        assert len(conflicts) == 0
    
    def test_three_drones_pairwise_check(self):
        """With 3 drones, should check all pairs."""
        # Three drones very close together
        base_lat, base_lon = 33.6846, -117.8265
        for i in range(3):
            self.detector.update_position(DronePosition(
                drone_id=f"DRONE{i+1:03d}",
                lat=base_lat + i * 0.0001,  # ~10m apart
                lon=base_lon,
                altitude_m=50,
                heading_deg=0,
                speed_mps=0
            ))
        
        conflicts = self.detector.detect_conflicts()
        # 3 drones = 3 pairs, all should be in conflict if close enough
        assert len(conflicts) >= 1  # At least some conflicts


class TestDronePosition:
    """Test DronePosition dataclass."""
    
    def test_to_dict(self):
        """Position should serialize to dict correctly."""
        pos = DronePosition(
            drone_id="TEST001",
            lat=33.6846,
            lon=-117.8265,
            altitude_m=100
        )
        d = pos.to_dict()
        assert d["drone_id"] == "TEST001"
        assert d["lat"] == 33.6846
        assert d["altitude_m"] == 100


class TestConflict:
    """Test Conflict dataclass."""
    
    def test_to_dict(self):
        """Conflict should serialize to dict correctly."""
        conflict = Conflict(
            drone1_id="DRONE001",
            drone2_id="DRONE002",
            severity=ConflictSeverity.WARNING,
            distance_m=75,
            time_to_closest=10,
            closest_distance_m=25
        )
        d = conflict.to_dict()
        assert d["severity"] == "warning"
        assert d["distance_m"] == 75
        assert d["closest_distance_m"] == 25


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
