"""Trajectory optimization and raceline generation utilities.

Provides utilities to:
1. Load raw path waypoints from CSV.
2. Clean duplicates and resample with uniform arc-length spacing.
3. Compute path headings, tangent vectors, and curvature.
4. Generate a curvature-constrained speed profile with acceleration/braking passes.
5. Export clean raceline waypoints for Pure Pursuit path tracking.
"""

from math import atan2, cos, hypot, pi, sin, sqrt
from pathlib import Path
from typing import List, Tuple


def _normalize_angle(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while angle > pi:
        angle -= 2.0 * pi
    while angle < -pi:
        angle += 2.0 * pi
    return angle


class Trajectory:
    """Smooth closed-circuit raceline with curvature and speed profile."""

    def __init__(self, s: List[float], x: List[float], y: List[float],
                 yaw: List[float], curvature: List[float], speed: List[float]):
        self.s = s
        self.x = x
        self.y = y
        self.yaw = yaw
        self.curvature = curvature
        self.speed = speed
        self.total_length = s[-1] if s else 0.0
        self.num_points = len(x)

    def find_closest_index(self, px: float, py: float, prev_idx: int = None, window: int = 50) -> int:
        """Find the index of the closest waypoint to (px, py), optionally localized near prev_idx."""
        if self.num_points == 0:
            return 0
        n = self.num_points

        if prev_idx is not None:
            # Search within forward window of previous index to prevent track-jumping
            best_dist_sq = float("inf")
            best_idx = prev_idx
            for offset in range(-5, window):
                idx = (prev_idx + offset) % n
                dx = self.x[idx] - px
                dy = self.y[idx] - py
                d2 = dx * dx + dy * dy
                if d2 < best_dist_sq:
                    best_dist_sq = d2
                    best_idx = idx
            if best_dist_sq < 4.0:
                return best_idx

        # Global search on first step or if track lost
        best_dist_sq = float("inf")
        best_idx = 0
        for i in range(n):
            dx = self.x[i] - px
            dy = self.y[i] - py
            d2 = dx * dx + dy * dy
            if d2 < best_dist_sq:
                best_dist_sq = d2
                best_idx = i
        return best_idx

    def find_lookahead_point(self, px: float, py: float, lookahead_m: float,
                             pyaw: float = None, prev_idx: int = None) -> Tuple[float, float, float, float, int]:
        """Find the target point on the path at lookahead_m forward from the car.

        Searches strictly forward along the raceline within a bounded local window
        to avoid jumping across parallel track sections.
        Returns (target_x, target_y, target_speed, target_yaw, closest_idx).
        """
        if self.num_points == 0:
            return px, py, 0.0, 0.0, 0

        closest_idx = self.find_closest_index(px, py, prev_idx=prev_idx)
        n = self.num_points

        # Search window is strictly bounded: at 0.10m waypoint spacing,
        # lookahead_m corresponds to ~10-20 waypoints. Limit window to max 35 waypoints (~3.5m).
        max_search = min(n // 4, max(15, int(2.5 * lookahead_m / 0.10)))

        c = cos(pyaw) if pyaw is not None else None
        s = sin(pyaw) if pyaw is not None else None

        best_any = None

        for offset in range(1, max_search + 1):
            idx = (closest_idx + offset) % n
            dx = self.x[idx] - px
            dy = self.y[idx] - py
            dist = hypot(dx, dy)

            # Check if point satisfies lookahead distance
            if dist >= lookahead_m:
                if c is not None:
                    x_local = dx * c + dy * s
                    if x_local > -0.05:  # In front or alongside vehicle
                        return self.x[idx], self.y[idx], self.speed[idx], self.yaw[idx], closest_idx
                    elif best_any is None:
                        best_any = (self.x[idx], self.y[idx], self.speed[idx], self.yaw[idx], closest_idx)
                else:
                    return self.x[idx], self.y[idx], self.speed[idx], self.yaw[idx], closest_idx

        # If a point at dist >= L was found within the local window, use it
        if best_any is not None:
            return best_any

        # Fallback: advance along track by lookahead equivalent waypoints
        fallback_offset = min(max_search, max(1, int(round(lookahead_m / 0.10))))
        fb_idx = (closest_idx + fallback_offset) % n
        return self.x[fb_idx], self.y[fb_idx], self.speed[fb_idx], self.yaw[fb_idx], closest_idx


def clean_raw_points(raw_points: List[Tuple[float, float]], min_dist: float = 0.04) -> List[Tuple[float, float]]:
    """Filter duplicate or ultra-close consecutive points."""
    if not raw_points:
        return []
    cleaned = [raw_points[0]]
    for p in raw_points[1:]:
        if hypot(p[0] - cleaned[-1][0], p[1] - cleaned[-1][1]) >= min_dist:
            cleaned.append(p)
    return cleaned


def resample_equidistant(points: List[Tuple[float, float]], step_m: float = 0.10) -> Tuple[List[float], List[float], List[float]]:
    """Resample closed loop with uniform arc length step_m.

    Returns (s, x, y).
    """
    if len(points) < 3:
        raise ValueError("Need at least 3 points to resample trajectory")

    # Ensure closed loop by wrapping back to first point
    loop_pts = list(points)
    if hypot(loop_pts[0][0] - loop_pts[-1][0], loop_pts[0][1] - loop_pts[-1][1]) > 0.05:
        loop_pts.append(loop_pts[0])

    # Compute cumulative arc lengths
    cum_s = [0.0]
    for i in range(1, len(loop_pts)):
        d = hypot(loop_pts[i][0] - loop_pts[i - 1][0], loop_pts[i][1] - loop_pts[i - 1][1])
        cum_s.append(cum_s[-1] + d)

    total_s = cum_s[-1]
    num_samples = max(10, int(total_s / step_m))
    s_new = [i * (total_s / num_samples) for i in range(num_samples)]

    x_new = []
    y_new = []

    seg_idx = 0
    for target_s in s_new:
        while seg_idx < len(cum_s) - 2 and cum_s[seg_idx + 1] < target_s:
            seg_idx += 1
        s0, s1 = cum_s[seg_idx], cum_s[seg_idx + 1]
        ds = s1 - s0
        t = (target_s - s0) / ds if ds > 1e-6 else 0.0
        x_interp = loop_pts[seg_idx][0] + t * (loop_pts[seg_idx + 1][0] - loop_pts[seg_idx][0])
        y_interp = loop_pts[seg_idx][1] + t * (loop_pts[seg_idx + 1][1] - loop_pts[seg_idx][1])
        x_new.append(x_interp)
        y_new.append(y_interp)

    return s_new, x_new, y_new


def smooth_points(x: List[float], y: List[float], window_size: int = 5) -> Tuple[List[float], List[float]]:
    """Moving average smoothing over closed periodic boundary."""
    n = len(x)
    half = window_size // 2
    xs = []
    ys = []
    for i in range(n):
        sum_x = 0.0
        sum_y = 0.0
        for w in range(-half, half + 1):
            idx = (i + w) % n
            sum_x += x[idx]
            sum_y += y[idx]
        xs.append(sum_x / window_size)
        ys.append(sum_y / window_size)
    return xs, ys


def compute_heading_and_curvature(x: List[float], y: List[float]) -> Tuple[List[float], List[float]]:
    """Calculate path headings and signed curvatures for a closed loop."""
    n = len(x)
    yaw = []
    curvature = []

    for i in range(n):
        prev_i = (i - 1) % n
        next_i = (i + 1) % n

        # Central difference for tangent
        dx = x[next_i] - x[prev_i]
        dy = y[next_i] - y[prev_i]
        heading = atan2(dy, dx)
        yaw.append(heading)

        # Menger curvature of triangle formed by prev_i, i, next_i
        x1, y1 = x[prev_i], y[prev_i]
        x2, y2 = x[i], y[i]
        x3, y3 = x[next_i], y[next_i]

        area = 0.5 * ((x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1))
        d12 = hypot(x2 - x1, y2 - y1)
        d23 = hypot(x3 - x2, y3 - y2)
        d31 = hypot(x1 - x3, y1 - y3)

        denom = d12 * d23 * d31
        k = (4.0 * area / denom) if denom > 1e-6 else 0.0
        curvature.append(k)

    return yaw, curvature


def generate_speed_profile(curvature: List[float], s: List[float],
                           max_speed_mps: float = 3.20,
                           min_speed_mps: float = 1.00,
                           max_lat_accel: float = 4.5,
                           max_accel: float = 2.0,
                           max_decel: float = 2.5) -> List[float]:
    """Compute curvature-capped velocity profile with acceleration & braking passes."""
    n = len(curvature)

    # 1. Curvature speed limit: v_max = sqrt(a_lat / |kappa|)
    v_target = []
    for k in curvature:
        abs_k = abs(k)
        if abs_k > 1e-3:
            v_k = sqrt(max_lat_accel / abs_k)
        else:
            v_k = max_speed_mps
        v_target.append(max(min_speed_mps, min(max_speed_mps, v_k)))

    # 2. Backward deceleration pass (ensure we brake early enough before sharp turns)
    v_profile = list(v_target)
    for _ in range(2):  # Two passes over closed loop for convergence
        for i in range(n - 1, -1, -1):
            next_i = (i + 1) % n
            ds = hypot(s[next_i] - s[i], 0.0) if next_i > i else (s[-1] - s[i] + s[0])
            v_max_allowable = sqrt(v_profile[next_i] ** 2 + 2.0 * max_decel * ds)
            v_profile[i] = min(v_profile[i], v_max_allowable)

    # 3. Forward acceleration pass (limit acceleration out of corners)
    for _ in range(2):
        for i in range(n):
            prev_i = (i - 1) % n
            ds = hypot(s[i] - s[prev_i], 0.0) if i > prev_i else (s[i] + s[-1] - s[prev_i])
            v_max_allowable = sqrt(v_profile[prev_i] ** 2 + 2.0 * max_accel * ds)
            v_profile[i] = max(min_speed_mps, min(v_profile[i], v_max_allowable))

    return v_profile


def optimize_raceline(csv_in: Path, csv_out: Path, step_m: float = 0.10,
                      max_speed_mps: float = 3.20, min_speed_mps: float = 1.00,
                      max_lat_accel: float = 4.5) -> Trajectory:
    """Full optimization pipeline: load -> clean -> resample -> smooth -> profile -> save."""
    raw_points = []
    with open(csv_in, "r") as f:
        header = f.readline()
        for line in f:
            parts = line.strip().split(",")
            if len(parts) >= 2:
                raw_points.append((float(parts[0]), float(parts[1])))

    cleaned = clean_raw_points(raw_points, min_dist=0.03)
    s, x, y = resample_equidistant(cleaned, step_m=step_m)
    x_smooth, y_smooth = smooth_points(x, y, window_size=5)
    yaw, curvature = compute_heading_and_curvature(x_smooth, y_smooth)
    speed = generate_speed_profile(curvature, s, max_speed_mps=max_speed_mps,
                                   min_speed_mps=min_speed_mps, max_lat_accel=max_lat_accel)

    # Save to output CSV
    csv_out.parent.mkdir(parents=True, exist_ok=True)
    with open(csv_out, "w") as f:
        f.write("s_m,x_m,y_m,yaw_rad,curv_radpm,speed_mps\n")
        for i in range(len(s)):
            f.write(f"{s[i]:.4f},{x_smooth[i]:.4f},{y_smooth[i]:.4f},{yaw[i]:.4f},{curvature[i]:.4f},{speed[i]:.4f}\n")

    return Trajectory(s, x_smooth, y_smooth, yaw, curvature, speed)


def load_raceline(csv_path: Path) -> Trajectory:
    """Load optimized raceline from CSV file."""
    s, x, y, yaw, curvature, speed = [], [], [], [], [], []
    with open(csv_path, "r") as f:
        _ = f.readline()  # skip header
        for line in f:
            parts = line.strip().split(",")
            if len(parts) >= 6:
                s.append(float(parts[0]))
                x.append(float(parts[1]))
                y.append(float(parts[2]))
                yaw.append(float(parts[3]))
                curvature.append(float(parts[4]))
                speed.append(float(parts[5]))
    return Trajectory(s, x, y, yaw, curvature, speed)
