"""Pure Pursuit path-tracking controller for the 1/10 Ackermann car.

Pure, stateless implementation shared between Webots simulation and on-car Pi runtime.
"""

from math import atan, atan2, cos, degrees, hypot, pi, radians, sin
from typing import Any, Dict, List, Tuple


def transform_to_vehicle_frame(px: float, py: float, car_x: float, car_y: float, car_yaw: float) -> Tuple[float, float]:
    """Transform global point (px, py) into vehicle local coordinates (x_forward, y_left)."""
    dx = px - car_x
    dy = py - car_y
    # Rotate by -car_yaw
    c = cos(car_yaw)
    s = sin(car_yaw)
    x_local = dx * c + dy * s
    y_local = -dx * s + dy * c
    return x_local, y_local


def compute_lookahead_distance(current_speed: float, params: Dict[str, Any]) -> float:
    """Calculate speed-dependent lookahead distance L."""
    ratio = float(params.get("pp_lookahead_ratio_s", 0.40))
    min_l = float(params.get("pp_lookahead_min_m", 0.65))
    max_l = float(params.get("pp_lookahead_max_m", 1.80))
    raw_l = ratio * max(current_speed, 0.0)
    return max(min_l, min(max_l, raw_l))


def pure_pursuit_step(car_x: float, car_y: float, car_yaw: float,
                      current_speed: float,
                      trajectory: Any,
                      params: Dict[str, Any],
                      prev_idx: int = None) -> Tuple[float, float, Tuple[float, float], int]:
    """Execute one step of Pure Pursuit path tracking.

    Args:
        car_x, car_y: Current vehicle position in metres.
        car_yaw: Vehicle heading in radians (counter-clockwise from +X).
        current_speed: Measured vehicle speed in m/s.
        trajectory: Trajectory object providing find_lookahead_point().
        params: Configuration dictionary (wheelbase, lookahead limits, steer clamps).
        prev_idx: Optional index of previously tracked waypoint for monotonic search.

    Returns:
        steer_deg: Commanded steering angle in degrees (+left, -right).
        target_speed_mps: Commanded target speed in m/s.
        lookahead_point: (target_x, target_y) in world coordinates.
        closest_idx: Index of closest waypoint on path.
    """
    wheelbase_m = float(params.get("wheelbase_m", 0.257))
    max_steer_deg = float(params.get("max_steer_deg", 18.0))

    # 1. Determine dynamic lookahead distance
    L = compute_lookahead_distance(current_speed, params)

    # 2. Find target waypoint on the path
    try:
        res = trajectory.find_lookahead_point(car_x, car_y, L, pyaw=car_yaw, prev_idx=prev_idx)
    except TypeError:
        # Fallback for mock/simplified trajectory objects
        res = trajectory.find_lookahead_point(car_x, car_y, L)

    if len(res) == 5:
        target_x, target_y, target_speed, _, closest_idx = res
    elif len(res) == 4:
        target_x, target_y, target_speed, _ = res
        closest_idx = prev_idx if prev_idx is not None else 0
    else:
        target_x, target_y = res[0], res[1]
        target_speed = res[2] if len(res) > 2 else current_speed
        closest_idx = prev_idx if prev_idx is not None else 0

    # 3. Transform to vehicle local frame (+x forward, +y left)
    x_local, y_local = transform_to_vehicle_frame(target_x, target_y, car_x, car_y, car_yaw)

    # Actual Euclidean distance to lookahead point
    actual_dist = hypot(x_local, y_local)
    if actual_dist < 1e-4:
        return 0.0, target_speed, (target_x, target_y), closest_idx

    # 4. Pure Pursuit curvature: gamma = 2 * y_local / (L^2)
    # Ackermann steering angle: delta = atan(gamma * L_wb)
    gamma = 2.0 * y_local / (actual_dist ** 2)
    delta_rad = atan(gamma * wheelbase_m)
    steer_deg = degrees(delta_rad)

    # 5. Strict physical clamp
    steer_deg = max(-max_steer_deg, min(max_steer_deg, steer_deg))

    return steer_deg, target_speed, (target_x, target_y), closest_idx
