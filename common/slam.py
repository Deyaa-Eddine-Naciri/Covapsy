"""2D LiDAR Occupancy Grid Mapping and Scan-Matching Localization (SLAM).

Pure Python standard-library implementation (math, array, struct, zlib) so it runs
natively inside Snap Webots, on the host PC, and on the Jetson Orin Nano / Pi
with zero external dependencies.
"""

from array import array
from math import cos, hypot, isfinite, pi, radians, sin, sqrt
from pathlib import Path
import struct
from typing import List, Tuple
import zlib


def _normalize_angle(angle: float) -> float:
    """Wrap angle in radians to [-pi, pi]."""
    return (angle + pi) % (2.0 * pi) - pi


def _edt_1d(f: List[float], n: int) -> List[float]:
    """Exact 1D squared Euclidean Distance Transform (Felzenszwalb & Huttenlocher)."""
    v = [0] * n
    z = [0.0] * (n + 1)
    d = [0.0] * n
    k = 0
    z[0] = -1e20
    z[1] = 1e20
    for q in range(1, n):
        vk = v[k]
        s = ((f[q] + q * q) - (f[vk] + vk * vk)) / (2.0 * (q - vk))
        while s <= z[k]:
            k -= 1
            vk = v[k]
            s = ((f[q] + q * q) - (f[vk] + vk * vk)) / (2.0 * (q - vk))
        k += 1
        v[k] = q
        z[k] = s
        z[k + 1] = 1e20
    k = 0
    for q in range(n):
        while z[k + 1] < q:
            k += 1
        vk = v[k]
        dx = q - vk
        d[q] = dx * dx + f[vk]
    return d


def compute_2d_edt(walls_flat: List[bool], width: int, height: int, resolution: float) -> List[float]:
    """Compute exact 2D Euclidean Distance Transform (in metres) to nearest True cell."""
    inf = 1e12
    f = [0.0 if w else inf for w in walls_flat]

    # Pass 1: columns
    for x in range(width):
        col = [f[y * width + x] for y in range(height)]
        d_col = _edt_1d(col, height)
        for y in range(height):
            f[y * width + x] = d_col[y]

    # Pass 2: rows
    out = [0.0] * (width * height)
    for y in range(height):
        row_start = y * width
        row = f[row_start:row_start + width]
        d_row = _edt_1d(row, width)
        for x in range(width):
            out[row_start + x] = sqrt(d_row[x]) * resolution

    return out


class OccupancyGridMap:
    """2D Log-Odds Occupancy Grid Map and Likelihood Distance Field."""

    def __init__(self, x_min: float = -6.5, x_max: float = 6.5,
                 y_min: float = -5.0, y_max: float = 6.5,
                 resolution: float = 0.04):
        self.x_min = float(x_min)
        self.x_max = float(x_max)
        self.y_min = float(y_min)
        self.y_max = float(y_max)
        self.resolution = float(resolution)

        self.width = int(round((self.x_max - self.x_min) / self.resolution))
        self.height = int(round((self.y_max - self.y_min) / self.resolution))
        n_cells = self.width * self.height

        self.log_odds: List[float] = [0.0] * n_cells
        self.wall_dist_m: List[float] = [5.0] * n_cells

        # Unobstructed front/side FOV (-135 to +135 deg, skipping rear body occlusion around 180)
        self._beam_indices = list(range(0, 136, 2)) + list(range(225, 360, 2))
        self._beam_angles = [radians(idx) for idx in self._beam_indices]

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        gx = int((x - self.x_min) / self.resolution)
        gy = int((y - self.y_min) / self.resolution)
        return gx, gy

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        x = self.x_min + (gx + 0.5) * self.resolution
        y = self.y_min + (gy + 0.5) * self.resolution
        return x, y

    def integrate_scan(self, car_x: float, car_y: float, car_yaw: float,
                       scan_360: List[float],
                       min_range: float = 0.25, max_range: float = 4.2) -> None:
        """Integrate a 360-deg LiDAR scan into the log-odds occupancy grid."""
        w = self.width
        h = self.height
        x_min = self.x_min
        y_min = self.y_min
        inv_res = 1.0 / self.resolution
        log_odds = self.log_odds

        for b_idx, b_ang in zip(self._beam_indices, self._beam_angles):
            r = scan_360[b_idx]
            if not (min_range <= r <= max_range and isfinite(r)):
                continue
            ang = car_yaw + b_ang
            ca = cos(ang)
            sa = sin(ang)

            # Free-space raycasting along the beam
            d = 0.08
            free_limit = r - 0.05
            while d < free_limit:
                gx = int((car_x + d * ca - x_min) * inv_res)
                gy = int((car_y + d * sa - y_min) * inv_res)
                if 0 <= gx < w and 0 <= gy < h:
                    cell = gy * w + gx
                    val = log_odds[cell] - 0.25
                    log_odds[cell] = -5.0 if val < -5.0 else val
                d += 0.04

            # Occupied endpoint (wall)
            wgx = int((car_x + r * ca - x_min) * inv_res)
            wgy = int((car_y + r * sa - y_min) * inv_res)
            if 0 <= wgx < w and 0 <= wgy < h:
                cell = wgy * w + wgx
                val = log_odds[cell] + 0.85
                log_odds[cell] = 5.0 if val > 5.0 else val

    def build_distance_field(self, occ_threshold: float = 0.7) -> None:
        """Filter isolated speckle noise and compute exact 2D Euclidean Distance Field."""
        w = self.width
        h = self.height
        log_odds = self.log_odds
        raw_walls = [val > occ_threshold for val in log_odds]
        clean_walls = [False] * (w * h)

        for y in range(1, h - 1):
            row = y * w
            for x in range(1, w - 1):
                idx = row + x
                if not raw_walls[idx]:
                    continue
                # Require at least 1 neighbor in 3x3 window to reject stray single pixels
                n_cnt = (
                    raw_walls[idx - w - 1] + raw_walls[idx - w] + raw_walls[idx - w + 1] +
                    raw_walls[idx - 1] + raw_walls[idx + 1] +
                    raw_walls[idx + w - 1] + raw_walls[idx + w] + raw_walls[idx + w + 1]
                )
                if n_cnt >= 1:
                    clean_walls[idx] = True

        if any(clean_walls):
            self.wall_dist_m = compute_2d_edt(clean_walls, w, h, self.resolution)

    def get_wall_distance(self, x: float, y: float) -> float:
        """Bilinearly interpolated distance in metres from (x, y) to the nearest wall."""
        fx = (x - self.x_min) / self.resolution - 0.5
        fy = (y - self.y_min) / self.resolution - 0.5
        gx0 = int(fx)
        gy0 = int(fy)
        w = self.width
        h = self.height
        if 0 <= gx0 < w - 1 and 0 <= gy0 < h - 1:
            tx = fx - gx0
            ty = fy - gy0
            wd = self.wall_dist_m
            row0 = gy0 * w + gx0
            row1 = row0 + w
            d00 = wd[row0]
            d10 = wd[row0 + 1]
            d01 = wd[row1]
            d11 = wd[row1 + 1]
            return (1.0 - tx) * ((1.0 - ty) * d00 + ty * d01) + tx * ((1.0 - ty) * d10 + ty * d11)
        gx, gy = self.world_to_grid(x, y)
        if 0 <= gx < w and 0 <= gy < h:
            return self.wall_dist_m[gy * w + gx]
        return 0.0

    def raycast_wall_distance(self, x: float, y: float, angle_rad: float,
                              max_dist: float = 2.5, step: float = 0.025,
                              occ_threshold: float = 0.7) -> float:
        """Cast a ray from (x, y) along angle_rad and return distance to first wall."""
        c = cos(angle_rad)
        s = sin(angle_rad)
        w = self.width
        h = self.height
        x_min = self.x_min
        y_min = self.y_min
        inv_res = 1.0 / self.resolution
        log_odds = self.log_odds
        wall_dist = self.wall_dist_m
        hit_thresh = self.resolution * 0.75

        d = 0.06
        while d < max_dist:
            gx = int((x + d * c - x_min) * inv_res)
            gy = int((y + d * s - y_min) * inv_res)
            if not (0 <= gx < w and 0 <= gy < h):
                return d
            idx = gy * w + gx
            if log_odds[idx] > occ_threshold or wall_dist[idx] <= hit_thresh:
                return d
            d += step
        return max_dist

    def save(self, path: Path) -> None:
        """Save map to compact zlib-compressed binary file."""
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        header = struct.pack(
            "<5d2i",
            self.x_min, self.x_max, self.y_min, self.y_max, self.resolution,
            self.width, self.height
        )
        lo_bytes = array("f", self.log_odds).tobytes()
        wd_bytes = array("f", self.wall_dist_m).tobytes()
        payload = zlib.compress(header + lo_bytes + wd_bytes, level=6)
        with open(path, "wb") as f:
            f.write(payload)

    @classmethod
    def load(cls, path: Path) -> "OccupancyGridMap":
        """Load map from compressed binary file."""
        with open(Path(path), "rb") as f:
            raw = zlib.decompress(f.read())
        hdr_size = struct.calcsize("<5d2i")
        x_min, x_max, y_min, y_max, res, w, h = struct.unpack("<5d2i", raw[:hdr_size])
        obj = cls(x_min=x_min, x_max=x_max, y_min=y_min, y_max=y_max, resolution=res)
        n_cells = w * h
        byte_len = n_cells * 4
        lo_arr = array("f")
        lo_arr.frombytes(raw[hdr_size:hdr_size + byte_len])
        wd_arr = array("f")
        wd_arr.frombytes(raw[hdr_size + byte_len:hdr_size + 2 * byte_len])
        obj.log_odds = list(lo_arr)
        obj.wall_dist_m = list(wd_arr)
        return obj


class LidarLocalizer:
    """Real-time 2D LiDAR Scan-to-Map Localizer (Correlative + Coordinate Refinement)."""

    def __init__(self, grid_map: OccupancyGridMap,
                 init_x: float = 0.0, init_y: float = 0.0, init_yaw: float = 0.0):
        self.map = grid_map
        self.x = float(init_x)
        self.y = float(init_y)
        self.yaw = float(init_yaw)

        # Subsample every 4th beam in unobstructed FOV (-135..+135 deg) -> ~68 beams
        self._beam_indices = list(range(0, 136, 4)) + list(range(225, 360, 4))
        self._beam_angles = [radians(i) for i in self._beam_indices]

        # Candidate search grid offsets
        offsets = (-0.10, -0.05, -0.02, 0.0, 0.02, 0.05, 0.10)
        self._cand_xy = [(dx, dy, 0.12 * hypot(dx, dy)) for dx in offsets for dy in offsets]
        self._cand_dyaw = (-0.04, -0.015, 0.0, 0.015, 0.04)

    def update(self, scan_360: List[float], speed_mps: float, imu_yaw: float, dt: float) -> Tuple[float, float, float]:
        """Predict pose with motion model and align LiDAR scan against the map distance field."""
        mid_yaw = _normalize_angle(0.5 * (self.yaw + imu_yaw))
        pred_x = self.x + speed_mps * cos(mid_yaw) * dt
        pred_y = self.y + speed_mps * sin(mid_yaw) * dt
        pred_yaw = imu_yaw

        valid_beams = []
        for b_idx, b_ang in zip(self._beam_indices, self._beam_angles):
            r = scan_360[b_idx]
            if 0.25 <= r <= 4.2 and isfinite(r):
                valid_beams.append((r, b_ang))

        if len(valid_beams) < 12:
            self.x, self.y, self.yaw = pred_x, pred_y, pred_yaw
            return self.x, self.y, self.yaw

        w = self.map.width
        h = self.map.height
        x_min = self.map.x_min
        y_min = self.map.y_min
        inv_res = 1.0 / self.map.resolution
        dist_grid = self.map.wall_dist_m
        inv_n = 1.0 / len(valid_beams)

        best_cost = float("inf")
        best_x, best_y, best_yaw = pred_x, pred_y, pred_yaw

        for dyaw in self._cand_dyaw:
            cand_yaw = pred_yaw + dyaw
            yaw_pen = 0.15 * abs(dyaw)
            # Pre-project beam offsets for this candidate yaw
            proj = [
                ( (r * cos(cand_yaw + a) - x_min) * inv_res,
                  (r * sin(cand_yaw + a) - y_min) * inv_res )
                for r, a in valid_beams
            ]
            for dx, dy, xy_pen in self._cand_xy:
                cx_res = (pred_x + dx) * inv_res
                cy_res = (pred_y + dy) * inv_res
                cost_sum = 0.0
                for px_r, py_r in proj:
                    gx = int(cx_res + px_r)
                    gy = int(cy_res + py_r)
                    if 0 <= gx < w and 0 <= gy < h:
                        d_val = dist_grid[gy * w + gx]
                        cost_sum += 0.25 if d_val > 0.25 else d_val
                    else:
                        cost_sum += 0.25
                total = cost_sum * inv_n + xy_pen + yaw_pen
                if total < best_cost:
                    best_cost = total
                    best_x = pred_x + dx
                    best_y = pred_y + dy
                    best_yaw = _normalize_angle(cand_yaw)

        self.x = best_x
        self.y = best_y
        self.yaw = best_yaw
        return self.x, self.y, self.yaw
