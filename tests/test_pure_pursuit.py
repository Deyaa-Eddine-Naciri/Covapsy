"""Unit tests for the Pure Pursuit controller and Trajectory modules."""

import sys
from math import isclose, pi, radians
from pathlib import Path
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "common"))
import pure_pursuit as pp
import trajectory as traj_mod
from config import PARAMS


class DummyTrajectory:
    """Mock trajectory for deterministic tracking tests."""

    def __init__(self, target_point=(2.0, 0.0), speed=2.5, yaw=0.0):
        self.target_point = target_point
        self.speed = speed
        self.yaw = yaw

    def find_lookahead_point(self, px, py, lookahead_m):
        return self.target_point[0], self.target_point[1], self.speed, self.yaw


def test_transform_to_vehicle_frame_straight():
    # Car at (0, 0) facing +X (yaw = 0)
    # Point at (2, 1) -> x_local=2, y_local=1 (left)
    xl, yl = pp.transform_to_vehicle_frame(2.0, 1.0, 0.0, 0.0, 0.0)
    assert isclose(xl, 2.0, abs_tol=1e-5)
    assert isclose(yl, 1.0, abs_tol=1e-5)


def test_transform_to_vehicle_frame_rotated():
    # Car at (1, 1) facing +Y (yaw = pi/2)
    # Target at (1, 3) -> 2m ahead of car -> x_local=2.0, y_local=0.0
    xl, yl = pp.transform_to_vehicle_frame(1.0, 3.0, 1.0, 1.0, pi / 2.0)
    assert isclose(xl, 2.0, abs_tol=1e-5)
    assert isclose(yl, 0.0, abs_tol=1e-5)


def test_lookahead_distance_bounds():
    p = {"pp_lookahead_ratio_s": 0.5, "pp_lookahead_min_m": 0.6, "pp_lookahead_max_m": 2.0}
    # At 0 speed -> clamped to min
    assert isclose(pp.compute_lookahead_distance(0.0, p), 0.6)
    # At 2 m/s -> 0.5 * 2 = 1.0 m
    assert isclose(pp.compute_lookahead_distance(2.0, p), 1.0)
    # At 10 m/s -> clamped to max
    assert isclose(pp.compute_lookahead_distance(10.0, p), 2.0)


def test_pure_pursuit_straight_ahead():
    # Target directly on vehicle axis -> steer should be 0.0
    mock_traj = DummyTrajectory(target_point=(2.0, 0.0), speed=2.0)
    steer, speed, pt, idx = pp.pure_pursuit_step(0.0, 0.0, 0.0, 2.0, mock_traj, PARAMS)
    assert isclose(steer, 0.0, abs_tol=1e-3)
    assert speed == 2.0
    assert idx == 0


def test_pure_pursuit_steers_left():
    # Target to the left (y > 0) -> positive steering (+deg = left)
    mock_traj = DummyTrajectory(target_point=(2.0, 0.5), speed=2.0)
    steer, speed, pt, idx = pp.pure_pursuit_step(0.0, 0.0, 0.0, 2.0, mock_traj, PARAMS)
    assert steer > 0.0
    assert steer <= PARAMS["max_steer_deg"]


def test_pure_pursuit_steers_right():
    # Target to the right (y < 0) -> negative steering (-deg = right)
    mock_traj = DummyTrajectory(target_point=(2.0, -0.5), speed=2.0)
    steer, speed, pt, idx = pp.pure_pursuit_step(0.0, 0.0, 0.0, 2.0, mock_traj, PARAMS)
    assert steer < 0.0
    assert steer >= -PARAMS["max_steer_deg"]


def test_pure_pursuit_steer_clamped():
    # Target with sharp lateral offset -> should clamp to max_steer_deg (18.0)
    mock_traj = DummyTrajectory(target_point=(0.2, 0.65), speed=1.0)
    steer, speed, pt, idx = pp.pure_pursuit_step(0.0, 0.0, 0.0, 1.0, mock_traj, PARAMS)
    assert isclose(steer, PARAMS["max_steer_deg"], abs_tol=1e-5)


def test_trajectory_loading():
    raceline_file = Path(__file__).resolve().parent.parent / "common" / "maps" / "raceline.csv"
    if raceline_file.is_file():
        traj = traj_mod.load_raceline(raceline_file)
        assert traj.num_points > 100
        assert traj.total_length > 20.0
        # Check closest index query
        idx = traj.find_closest_index(0.0, -2.5)
        assert 0 <= idx < traj.num_points
        # Check lookahead point
        tx, ty, tspd, tyaw, closest_idx = traj.find_lookahead_point(0.0, -2.5, 1.0)
        assert tspd > 0.0
        assert 0 <= closest_idx < traj.num_points
