"""Unit tests for the follow-the-gap driver.

Run: python -m pytest tests/test_gap_follower.py -v
"""

import math
import os
import sys

# shared module is in ../common
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "common"))

import gap_follower as gf

MAXR = gf.DEFAULT_PARAMS["max_range_m"]


def scan(default=MAXR):
    """360-element scan, all-open by default."""
    return [default] * 360


def put(s, deg, dist):
    """Set the beam at car-angle deg (0=front, +left, -right)."""
    s[deg % 360] = dist


def block_arc(s, center_deg, half_width_deg, dist):
    for d in range(center_deg - half_width_deg, center_deg + half_width_deg + 1):
        s[d % 360] = dist


def test_open_field_goes_straight_and_fast():
    steer, speed = gf.gap_follow(scan())
    assert abs(steer) < 2.0, f"should aim straight, got {steer}"
    assert speed > 0.5, f"open field should be near max speed, got {speed}"


def test_wall_on_right_steers_left():
    s = scan()
    block_arc(s, -60, 25, 0.4)   # wall on the right
    steer, speed = gf.gap_follow(s)
    assert steer > 0.0, f"wall on right -> steer left (positive), got {steer}"
    assert speed > 0.0


def test_wall_on_left_steers_right():
    s = scan()
    block_arc(s, 60, 25, 0.4)    # wall on the left
    steer, speed = gf.gap_follow(s)
    assert steer < 0.0, f"wall on left -> steer right (negative), got {steer}"


def test_boxed_front_rear_clear_reverses():
    s = scan(0.25)               # everything close, boxed in front
    steer, speed = gf.gap_follow(s, rear_blocked=False)
    assert speed < 0.0, f"blocked & rear clear -> reverse, got speed {speed}"


def test_boxed_front_and_rear_stops():
    s = scan(0.25)
    steer, speed = gf.gap_follow(s, rear_blocked=True)
    assert speed == 0.0 and steer == 0.0, \
        f"blocked both ways -> full stop, got ({steer},{speed})"


def test_front_wall_only_with_open_sides_does_not_crash_forward():
    s = scan()
    block_arc(s, 0, 12, 0.2)     # wall dead ahead, sides and rear open
    steer, speed = gf.gap_follow(s, rear_blocked=False)
    # front blocked, rear clear: reverse instead of driving forward
    assert speed <= 0.0, f"wall ahead should not produce forward speed, got {speed}"


def test_reverse_hysteresis_maintains_reverse_until_exit_threshold():
    # Car is at 0.45m front clearance.
    # Case 1: When not reversing, 0.45m > 0.30m (emergency_front_m), so it drives forward
    s = scan(3.0)
    block_arc(s, 0, 15, 0.45)
    steer, speed = gf.gap_follow(s, rear_blocked=False, is_reversing=False)
    assert speed > 0.0, "initially forward when front > emergency_front_m"

    # Case 2: When already reversing, 0.45m < 0.65m (reverse_exit_m), so it stays in reverse
    steer, speed = gf.gap_follow(s, rear_blocked=False, is_reversing=True)
    assert speed < 0.0, "continues reversing until clearance exceeds reverse_exit_m"

    # Case 3: Once clearance reaches 0.70m (> 0.65m), reverse disengages
    s_clear = scan(3.0)
    block_arc(s, 0, 15, 0.70)
    steer, speed = gf.gap_follow(s_clear, rear_blocked=False, is_reversing=True)
    assert speed > 0.0, "switches back to forward once clear of reverse_exit_m"


def test_speed_never_exceeds_cap():
    cap = gf.DEFAULT_PARAMS["max_speed_mps"]
    steer, speed = gf.gap_follow(scan())
    assert speed <= cap + 1e-9


def test_steer_clamped():
    s = scan()
    block_arc(s, -90, 40, 0.3)   # force a hard left turn
    steer, speed = gf.gap_follow(s)
    limit = gf.DEFAULT_PARAMS["max_steer_deg"]
    assert -limit - 1e-9 <= steer <= limit + 1e-9


def test_handles_zeros_and_nan_as_no_return():
    s = scan()
    s[10] = 0.0
    s[20] = float("nan")
    s[30] = float("inf")
    steer, speed = gf.gap_follow(s)   # must not raise
    assert speed > 0.0


def test_narrow_gap_between_two_walls_aims_center():
    s = scan()
    # two walls leaving a corridor straight ahead
    block_arc(s, 70, 30, 0.6)
    block_arc(s, -70, 30, 0.6)
    steer, speed = gf.gap_follow(s)
    assert abs(steer) < 10.0, f"corridor ahead -> mostly straight, got {steer}"
    assert speed > 0.0

def test_prefers_straight_ahead_over_larger_side_opening():
    # Direct test of _largest_gap scoring mechanism:
    # Forward gap of width 35 centered around mid (index 100)
    # Side open area of width 60 far to the side (index 130 to 190)
    dists = [0.0] * 201
    for i in range(85, 120):
        dists[i] = 3.0
    for i in range(130, 190):
        dists[i] = 4.0

    gap = gf._largest_gap(dists, 0.05)
    assert gap == (85, 119), f"expected forward gap (85, 119), got {gap}"


def test_tight_right_turn_beats_wider_shallow_left_gap():
    # Approaching a tight right turn:
    # - Front wall at 0.7m across [-35, +35] deg
    # - Inner right apex at 0.25m near -35 deg
    # - Narrower (25 deg) but deep (3.8m) right-turn exit around -70 deg
    # - Wider (45 deg) but shallower (1.8m) left pocket around +60 deg
    s = scan(0.7)
    block_arc(s, -35, 5, 0.25)
    block_arc(s, -70, 20, 3.8)
    block_arc(s, 60, 25, 1.8)
    steer, speed = gf.gap_follow(s)
    assert steer < 0.0, f"tight right turn should steer right (negative), got {steer}"
    assert speed > 0.0


def test_straight_corridor_adaptive_cone_unlocks_top_speed():
    # Open straight ahead (4.5m across [-15, +15] deg), with side walls from +/-16..100 deg at 1.2m.
    # A fixed +/-20 deg cone would clip the side walls at 1.2m and cap speed to ~1.2 m/s;
    # the adaptive cone (+/-10 deg when target_angle ~ 0) looks down the open corridor and reaches max_speed_mps.
    s = scan(4.5)
    block_arc(s, 58, 42, 1.2)
    block_arc(s, -58, 42, 1.2)
    steer, speed = gf.gap_follow(s)
    assert abs(steer) < 2.0
    assert speed >= gf.DEFAULT_PARAMS["max_speed_mps"] * 0.98, \
        f"expected full straightaway speed, got {speed}"
