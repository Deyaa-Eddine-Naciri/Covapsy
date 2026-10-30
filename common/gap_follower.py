"""Follow-the-gap reactive driver. Same function runs in Webots sim and on
the car.

API: gap_follow(ranges_m, rear_blocked=False, params=None) -> (steer_deg, speed_mps)

    ranges_m     LiDAR scan, 360 distances in metres, index = angle in degrees,
                 0 = front, CCW-positive = left (index 90 = left, 270 = right).
                 <=0 or non-finite = no return.
    rear_blocked True if something is close behind (rear IR on car, rear LiDAR
                 sector in sim). Gates reverse; reverse only when rear is clear.
    returns      steer_deg (+left / -right, clamped to +/-max_steer_deg),
                 speed_mps (>0 forward, <0 reverse, 0 = stop).

Pure Python, no numpy or I/O. Algorithm: disparity extension, safety bubble,
largest gap, aim at deepest point, speed adapted to steering and clearance,
with a reverse-when-boxed-in fallback.
"""

import math
import os
import sys

# Make the sibling config.py importable from Pi runtime, Webots, or pytest.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    from config import PARAMS as DEFAULT_PARAMS
except Exception:  # pragma: no cover - fallback so the module is self-contained
    DEFAULT_PARAMS = {
        "max_steer_deg": 18.0, "car_width_m": 0.30, "max_range_m": 5.0,
        "min_valid_m": 0.05, "front_fov_deg": 100, "gap_select_fov_deg": 100,
        "disparity_threshold_m": 0.35, "safety_radius_m": 0.18, "steer_gain": 1.4,
        "aim_deep_bias": 0.75, "clearance_margin_m": 0.10,
        "max_speed_mps": 0.8, "min_speed_mps": 0.8, "reverse_speed_mps": 0.6,
        "ttc_target_s": 1.0, "emergency_front_m": 0.30, "reverse_exit_m": 0.65,
        "emergency_window_deg": 15,
        "side_clear_min_deg": 20, "side_clear_max_deg": 85,
    }


def _clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def _sanitize(ranges_m, p):
    """Clip to [0, max_range]; map no-return / noise to max_range."""
    mx = p["max_range_m"]
    mn = p["min_valid_m"]
    out = [0.0] * 360
    for i in range(360):
        r = ranges_m[i] if i < len(ranges_m) else 0.0
        if r is None or not math.isfinite(r) or r < mn:
            out[i] = mx
        else:
            out[i] = r if r < mx else mx
    return out


def _front_sector(ranges, p):
    """Return (angles_rad, dists_m) over -fov..+fov degrees, in angle order.
    Distance at car-angle deg is ranges[deg % 360]: deg=+60 is index 60 (left),
    deg=-60 is index 300 (right)."""
    fov = int(p["front_fov_deg"])
    angles = []
    dists = []
    for deg in range(-fov, fov + 1):
        angles.append(math.radians(deg))
        dists.append(ranges[deg % 360])
    return angles, dists


def _disparity_extension(angles, dists, p):
    """Inflate the closer side of each depth discontinuity by half a car width
    so the car never aims through a gap it cannot fit."""
    half_w = 0.5 * p["car_width_m"] + p.get("clearance_margin_m", 0.0)
    step = math.radians(1.0)  # 1 sample per degree
    thr = p["disparity_threshold_m"]
    n = len(dists)
    # Detect disparities on a snapshot of the original scan; reading the live
    # array would cascade the nearest distance across the sector and erase
    # every opening.
    orig = list(dists)
    for k in range(1, n):
        if abs(orig[k] - orig[k - 1]) > thr:
            close_k = k if orig[k] < orig[k - 1] else k - 1
            close_r = orig[close_k]
            if close_r > p["min_valid_m"]:
                spread = min(int(math.atan2(half_w, close_r) / step), 12)
                lo = max(0, close_k - spread)
                hi = min(n, close_k + spread + 1)
                for j in range(lo, hi):
                    if dists[j] > close_r:
                        dists[j] = close_r


def _safety_bubble(angles, dists, p):
    """Zero out a bubble around the nearest point. Physical radius safety_radius_m,
    angular half-width capped at 12 deg so a very close obstacle cannot carve a
    wide wedge that splits a corner gap."""
    n = len(dists)
    near_k = min(range(n), key=lambda i: dists[i])
    near_d = dists[near_k]
    if near_d >= p["max_range_m"]:
        return
    near_a = angles[near_k]
    half = min(p["safety_radius_m"] / max(near_d, p["min_valid_m"]),
               math.radians(12.0))
    for i in range(n):
        if abs(angles[i] - near_a) < half:
            dists[i] = 0.0


def _mask_shoulders(angles, dists, p):
    """Forbid choosing a heading outside the gap-selection FOV."""
    lim = math.radians(p["gap_select_fov_deg"])
    for i in range(len(dists)):
        if abs(angles[i]) > lim:
            dists[i] = 0.0


def _largest_gap(dists, min_valid):
    """Return (start, end) inclusive of the valid gap with the highest score.
    Scores each gap as width * penalty, where penalty reduces the score based
    on how far the gap's center index is from the straight-ahead index
    (len(dists)//2), strongly prioritizing forward paths over side openings."""
    mid = len(dists) // 2
    max_dist = max(mid, 1)

    # 1. Collect all continuous runs of valid beams
    candidates = []
    start = None
    for i, d in enumerate(dists):
        if d > min_valid and start is None:
            start = i
        elif d <= min_valid and start is not None:
            candidates.append((start, i - 1))
            start = None
    if start is not None:
        candidates.append((start, len(dists) - 1))

    if not candidates:
        return None

    # 2. Score all candidate gaps and gather metrics
    best = None
    best_score = -float("inf")
    gap_metrics = []

    for s, e in candidates:
        width = e - s
        center = (s + e) / 2.0
        gap_slice = dists[s:e + 1]
        mean_depth = sum(gap_slice) / len(gap_slice) if gap_slice else 0.0
        # Cap effective depth at 3.5m so a 5m off-track void gets no bonus over a 3.5m corridor,
        # while a 3.5m corner exit still strongly beats a 1.5m shallow pocket.
        eff_depth = min(mean_depth, 3.5)
        # Bell-shaped forward preference: 1.0 straight ahead, flattening near ~0.35 past +/-50 deg
        # so tight corners (-70 deg) aren't unfairly crushed vs slightly less tight ones (+55 deg).
        penalty = 0.35 + 0.65 * math.exp(-((center - mid) / (0.35 * max_dist)) ** 2)
        score = width * (eff_depth ** 1.5) * penalty

        if score > best_score:
            best_score = score
            best = (s, e)

        gap_metrics.append({
            "start": s,
            "end": e,
            "width": width,
            "center": center,
            "offset_deg": center - mid,
            "mean_depth": mean_depth,
            "score": score,
        })

    # 3. Debug print: iterate through every candidate gap right before final selection
    '''print(f"[GAP SELECTION] {len(candidates)} candidate gap(s) evaluated:")
    for idx, m in enumerate(gap_metrics):
        selected_flag = " <== [SELECTED]" if (m["start"], m["end"]) == best else ""
        print(f"  Candidate #{idx + 1}: indices=[{m['start']:3d}, {m['end']:3d}] | "
              f"angular_width={m['width']:3d}° | "
              f"center_idx={m['center']:5.1f} (offset={m['offset_deg']:+5.1f}°) | "
              f"mean_depth={m['mean_depth']:4.2f}m | "
              f"score={m['score']:6.2f}{selected_flag}")'''

    return best


def side_clearances(ranges_m, params=None):
    """Median clearance on the left and right shoulders. Used to pick steer
    direction while reversing out of a dead end."""
    p = params or DEFAULT_PARAMS
    ranges = _sanitize(ranges_m, p)
    lo = p["side_clear_min_deg"]
    hi = p["side_clear_max_deg"]
    left = [ranges[d % 360] for d in range(lo, hi + 1)]
    right = [ranges[(-d) % 360] for d in range(lo, hi + 1)]
    left.sort()
    right.sort()
    lc = left[len(left) // 2] if left else p["max_range_m"]
    rc = right[len(right) // 2] if right else p["max_range_m"]
    return lc, rc


def front_clearance(ranges_m, params=None, half_window_deg=None):
    """Nearest obstacle distance in a cone of +/-half_window_deg around front."""
    p = params or DEFAULT_PARAMS
    ranges = _sanitize(ranges_m, p)
    hw = int(half_window_deg if half_window_deg is not None
             else p["emergency_window_deg"])
    return min(ranges[d % 360] for d in range(-hw, hw + 1))


def gap_follow(ranges_m, rear_blocked=False, params=None, is_reversing=False):
    """See module docstring. Returns (steer_deg, speed_mps)."""
    p = params or DEFAULT_PARAMS
    ranges = _sanitize(ranges_m, p)

    # Path straight ahead blocked? Uses hysteresis: if already in reverse,
    # stay in reverse until clearance exceeds reverse_exit_m (default 0.65m)
    # instead of immediately switching back forward at emergency_front_m (0.30m).
    front_min = front_clearance(ranges, p, p["emergency_window_deg"])
    threshold = p.get("reverse_exit_m", 0.65) if is_reversing else p["emergency_front_m"]
    front_is_blocked = front_min < threshold

    # Build and condition the front sector.
    angles, dists = _front_sector(ranges, p)
    _disparity_extension(angles, dists, p)
    _safety_bubble(angles, dists, p)
    _mask_shoulders(angles, dists, p)

    # Require beams in a gap to be deeper than nearby walls so a front/side wall
    # isn't merged or mistaken for an open corridor.
    max_d = max(dists) if dists else 0.0
    gap_min_depth = max(p["min_valid_m"], min(1.5, 0.45 * max_d))
    gap = _largest_gap(dists, gap_min_depth)

    # No usable gap, or wall right in front: handle the dead end.
    max_steer = min(float(p.get("max_steer_deg", 18.0)), 18.0)
    if gap is None or front_is_blocked:
        if not rear_blocked:
            # Reverse, steering toward the more open side.
            lc, rc = side_clearances(ranges, p)
            steer = max_steer if lc >= rc else -max_steer
            return steer, -p["reverse_speed_mps"]
        # Boxed front and rear: stop.
        return 0.0, 0.0

    # Aim within the largest gap: blend its center (mid-corridor on straights)
    # with the center of its deepest beams (points down a corner exit, commits
    # the turn earlier). Using the deepest-plateau center, not a single max
    # beam, avoids snapping to one edge on a uniform scan. aim_deep_bias=0 is
    # pure centering.
    g0, g1 = gap
    center_k = (g0 + g1) // 2
    maxd = max(dists[g0:g1 + 1])
    plateau_tol = max(0.06, min(0.20, 0.04 * maxd))
    deep_band = [i for i in range(g0, g1 + 1) if dists[i] >= maxd - plateau_tol]
    deep_k = deep_band[len(deep_band) // 2]
    bias = p["aim_deep_bias"]
    aim_k = int(round((1.0 - bias) * center_k + bias * deep_k))
    target_angle = angles[aim_k]
    steer_deg = _clamp(p["steer_gain"] * math.degrees(target_angle),
                       -max_steer, max_steer)

    # Speed: slow for hard steering and for low clearance ahead.
    # Adaptive clearance cone: narrow (+/-10 deg) on straights so parallel side
    # walls don't falsely trigger TTC braking, widening smoothly to +/-20 deg as
    # target_angle commits to a corner.
    turn_ratio = abs(steer_deg) / max_steer
    steer_factor = 1.0 - 0.30 * (turn_ratio ** 2)
    target_deg = abs(math.degrees(target_angle))
    cone_deg = int(round(10.0 + 10.0 * _clamp(target_deg / 22.0, 0.0, 1.0)))
    projected = front_clearance(ranges, p, cone_deg)
    clear_factor = _clamp((projected - p["emergency_front_m"]) / 1.2, 0.0, 1.0)
    speed = p["min_speed_mps"] + (p["max_speed_mps"] - p["min_speed_mps"]) \
        * steer_factor * clear_factor
    # Time-to-collision cap: never close faster than ttc_target_s allows.
    ttc_cap = projected / max(p["ttc_target_s"], 0.1)
    speed = _clamp(min(speed, ttc_cap), 0.0, p["max_speed_mps"])
    # Don't crawl to a stop while a valid gap exists.
    speed = max(speed, p["min_speed_mps"])

    return steer_deg, speed
