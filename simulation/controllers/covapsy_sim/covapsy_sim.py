# Copyright 2026 COVAPSY Team
#
# Webots controller for the TT-02 car. Runs the shared follow-the-gap code
# (common/gap_follower.py): read LiDAR, call gap_follow(), drive.
#
# Usage:
#   1. Set the TT02 robot's "controller" field to "covapsy_sim".
#   2. Click the 3D view, press 'A' to drive autonomously, 'N' to stop.

import os
import sys
from collections import deque
from math import radians, isnan, hypot
from pathlib import Path

from vehicle import Driver
from controller import Lidar, GPS

# --- Import the shared code ------------------------------------------------
# Shared follow-the-gap code lives in common/. Walk up the tree to find it so
# the import works regardless of where Webots places the controller.
_COMMON = None
for _p in Path(__file__).resolve().parents:
    if (_p / "common" / "gap_follower.py").is_file():
        _COMMON = _p / "common"
        break
if _COMMON is None:
    raise RuntimeError("covapsy_sim: could not locate common/ (shared code)")
sys.path.insert(0, str(_COMMON))
import config  # noqa: E402
import gap_follower  # noqa: E402

PARAMS = config.PARAMS

# Optional file logging. Set COVAPSY_LOGFILE=<path> to enable (Webots GUI does
# not pipe stdout to a terminal on Windows); harmless when unset.
_LOGF = None
_logpath = os.environ.get("COVAPSY_LOGFILE")
if _logpath:
    _LOGF = open(_logpath, "w", buffering=1)


def _log(msg):
    print(msg, flush=True)
    if _LOGF:
        _LOGF.write(msg + "\n")
        _LOGF.flush()


_log(f"[COVAPSY] controller import OK; brain={_COMMON}")
_log(f"[COVAPSY] params: max_speed={PARAMS['max_speed_mps']} "
     f"steer_gain={PARAMS['steer_gain']} "
     f"aim_deep_bias={PARAMS['aim_deep_bias']} "
     f"ttc_target={PARAMS['ttc_target_s']}")

# --- Lap timer setup -------------------------------------------------------
# Virtual start/finish line configuration:
# In standalone.wbt and multicar.wbt, the bottom straight runs along Y in
# [-3.5, -1.5] from X = -5.0 to +3.5. Cars start around X = -3.2, Y = -3.0
# heading in the +X direction. We place the finish line across this straight
# at X = 0.0.
FINISH_LINE_X = 0.0
FINISH_LINE_Y_MIN = -3.5
FINISH_LINE_Y_MAX = -1.5
LAP_DEBOUNCE_S = 5.0  # Minimum seconds before a new lap can trigger


class LapTimer:
    """Lap timer tracking start/finish line crossings via GPS & driver.getTime()."""

    def __init__(self, line_x=FINISH_LINE_X, y_min=FINISH_LINE_Y_MIN,
                 y_max=FINISH_LINE_Y_MAX, debounce_s=LAP_DEBOUNCE_S,
                 start_on_first_cross=True):
        self.line_x = line_x
        self.y_min = y_min
        self.y_max = y_max
        self.debounce_s = debounce_s
        self.start_on_first_cross = start_on_first_cross
        self.prev_x = None
        self.prev_y = None
        self.lap_start_time = None if start_on_first_cross else 0.0
        self.last_lap_trigger_time = -float("inf")
        self.lap_count = 0
        self.last_lap_duration = None

    def update(self, current_time, x, y):
        """Check for line crossing in +X direction and compute lap time."""
        if self.prev_x is not None:
            # Traveling in +X direction across line_x within track corridor [y_min, y_max]
            crossed = (self.prev_x < self.line_x <= x) and (self.y_min <= y <= self.y_max)
            if crossed and (current_time - self.last_lap_trigger_time >= self.debounce_s):
                self.last_lap_trigger_time = current_time
                if self.lap_start_time is None:
                    # First crossing marks the start line crossing (out-lap completion)
                    self.lap_start_time = current_time
                    _log(f"\033[31m[LAP TIMER] Start line crossed at {current_time:.2f}s -- timing Lap 1!\033[0m")
                else:
                    lap_time = current_time - self.lap_start_time
                    self.lap_start_time = current_time
                    self.lap_count += 1
                    self.last_lap_duration = lap_time
                    # ANSI escape \033[31m prints in RED, \033[0m resets color
                    _log(f"\033[31m[LAP TIMER] Lap {self.lap_count} completed: {lap_time:.2f} s "
                         f"(sim time: {current_time:.2f}s)\033[0m")

        self.prev_x = x
        self.prev_y = y


# --- Webots setup ----------------------------------------------------------

driver = Driver()
basic_step = int(driver.getBasicTimeStep())
sensor_step = 4 * basic_step

lidar = Lidar("RpLidarA2")
lidar.enable(sensor_step)

gps = GPS("gps")
gps.enable(sensor_step)

try:
    from controller import InertialUnit
    imu = InertialUnit("imu")
    imu.enable(sensor_step)
except Exception:
    imu = None

keyboard = driver.getKeyboard()
keyboard.enable(sensor_step)

lap_timer = LapTimer()
record_path = os.environ.get("COVAPSY_RECORD_PATH")
if record_path in ("1", "true", "True"):
    record_path = str(_COMMON / "maps" / "reference_path.csv")
recorded_points = []
path_saved = False


def _save_record_path():
    global path_saved
    _log(f"[RECORDER] _save_record_path called: target={record_path}, points={len(recorded_points)}")
    if record_path and not path_saved and recorded_points:
        p = Path(record_path)
        p.parent.mkdir(parents=True, exist_ok=True)
        with open(p, "w") as f:
            f.write("x,y,yaw,speed_mps,steer_deg\n")
            for px, py, pyaw, pspd, pstr in recorded_points:
                f.write(f"{px:.4f},{py:.4f},{pyaw:.4f},{pspd:.4f},{pstr:.4f}\n")
        _log(f"[RECORDER] Successfully saved {len(recorded_points)} waypoints to {record_path}")
        path_saved = True


def read_scan():
    """Webots LiDAR to canonical scan: 360 floats in metres, index 0 = front,
    increasing index = counter-clockwise (index 90 = left, 270 = right).
    No-return or invalid beams are 0.0 (downstream treats that as max range).
    """
    raw = lidar.getRangeImage()
    scan = [0.0] * 360
    if raw is None:
        return scan
    n = len(raw)
    for i in range(360):
        d = raw[-i % n]
        # Webots returns inf for no-return; clamp inf and <=0 to 0.0.
        if d == d and 0.0 < d < float("inf"):  # d == d rejects NaN
            scan[(i - 180) % 360] = d
        else:
            scan[(i - 180) % 360] = 0.0
    return scan


def rear_nearest(scan, ignore_self=True):
    """Nearest real obstacle behind the car, within +/- rear_sector_deg of
    index 180. By default ignore returns nearer than rear_self_ignore_m (the
    TT-02 shell occludes the rear LiDAR with a constant close return)."""
    sec = int(PARAMS["rear_sector_deg"])
    mx = PARAMS["max_range_m"]
    self_r = PARAMS["rear_self_ignore_m"]
    nearest = mx
    for d in range(-sec, sec + 1):
        r = scan[(180 + d) % 360]
        if r <= 0.0:                       # no return, treat as clear
            continue
        if ignore_self and r < self_r:     # own body, ignore
            continue
        if r < nearest:
            nearest = r
    return nearest


def rear_is_blocked(scan):
    """True if a real obstacle is within rear_block_m behind the car."""
    return rear_nearest(scan) < PARAMS["rear_block_m"]


# --- Main loop -------------------------------------------------------------

print("=" * 52)
print("  COVAPSY sim controller -- shared gap_follower brain")
print("  Click the 3D view, then:")
print("    'A' = enable autonomous driving")
print("    'N' = stop (zero speed + steering, disable auto)")
print("=" * 52)

# COVAPSY_AUTOSTART=1 enables auto mode at boot; normally press 'A' in the
# 3D view.
auto_mode = os.environ.get("COVAPSY_AUTOSTART") == "1"
headless_tune = os.environ.get("COVAPSY_HEADLESS_TUNE") == "1"
if auto_mode:
    print("[COVAPSY] autostart (COVAPSY_AUTOSTART=1)")
if headless_tune:
    print("[COVAPSY] headless tuning mode enabled (COVAPSY_HEADLESS_TUNE=1)")
driver.setSteeringAngle(0.0)
driver.setCruisingSpeed(0.0)

step_count = 0
pos_history = deque()
reverse_start_time = None
reverse_count = 0
is_reversing = False
current_steer_cmd = 0.0
last_steer_time = None
while driver.step() != -1:
    step_count += 1
    # Keyboard
    while True:
        key = keyboard.getKey()
        if key == -1:
            break
        if key in (ord("a"), ord("A")):
            if not auto_mode:
                auto_mode = True
                print("[COVAPSY] Auto mode ENABLED")
        elif key in (ord("n"), ord("N")):
            if auto_mode:
                auto_mode = False
                print("[COVAPSY] Auto mode DISABLED")

    # Read GPS coordinates (X, Y) and update lap timer
    gps_coords = gps.getValues()
    x = gps_coords[0] if gps_coords and not isnan(gps_coords[0]) else None
    y = gps_coords[1] if gps_coords and not isnan(gps_coords[1]) else None
    sim_time = driver.getTime()
    if x is not None and y is not None:
        lap_timer.update(sim_time, x, y)
        pos_history.append((sim_time, x, y))
        while pos_history and pos_history[0][0] < sim_time - 2.5:
            pos_history.popleft()

    # Fail-fast abort in headless tuning mode
    if headless_tune:
        if lap_timer.lap_count >= 1:
            _save_record_path()
            if _LOGF:
                _LOGF.flush()
            sys.stdout.flush()
            driver.simulationQuit(0)
            break
        lap_elapsed = (sim_time - lap_timer.lap_start_time) if lap_timer.lap_start_time is not None else 0.0
        if (lap_timer.lap_start_time is None and sim_time > 15.0) or lap_elapsed > 55.0 or sim_time > 60.0:
            _log(f"[TUNE_RESULT] DNF (timeout: sim_time={sim_time:.2f}s, lap_elapsed={lap_elapsed:.2f}s)")
            sys.stdout.flush()
            driver.simulationQuit(1)
            break
        if sim_time > 4.0 and pos_history and (sim_time - pos_history[0][0] >= 3.0):
            t0, x0, y0 = pos_history[0]
            max_moved = max(hypot(px - x0, py - y0) for _, px, py in pos_history)
            if max_moved < 0.10:
                _log(f"[TUNE_RESULT] DNF (stuck at sim_time={sim_time:.2f}s, x={x:.2f}, y={y:.2f}, max_moved={max_moved:.3f}m in 3.0s)")
                sys.stdout.flush()
                driver.simulationQuit(1)
                break

    # Wait 0.5s at boot before driving so the 12 Hz rotating RpLidarA2 completes
    # multiple full 360-deg revolutions (prevents partial-scan phantom gaps on step 1).
    if not auto_mode or sim_time < 0.5:
        driver.setSteeringAngle(0.0)
        driver.setCruisingSpeed(0.0)
        current_steer_cmd = 0.0
        last_steer_time = sim_time
        continue

    scan = read_scan()
    rear_blocked = rear_is_blocked(scan)
    steer_deg, speed_mps = gap_follower.gap_follow(scan, rear_blocked, PARAMS, is_reversing=is_reversing)
    is_reversing = speed_mps < 0.0

    # Track reverse maneuvers for logging/telemetry
    if speed_mps < 0.0:
        if reverse_start_time is None:
            reverse_start_time = sim_time
    else:
        if reverse_start_time is not None and (sim_time - reverse_start_time) >= 0.2:
            reverse_count += 1
        reverse_start_time = None

    # Only abort if reversing continuously for an excessively long time (>12s) without recovering
    if headless_tune and sim_time > 1.5:
        if reverse_start_time is not None and (sim_time - reverse_start_time) > 12.0:
            _log(f"[TUNE_RESULT] DNF (continuous reverse timeout at sim_time={sim_time:.2f}s)")
            sys.stdout.flush()
            driver.simulationQuit(1)
            break

    # Webots steering sign is inverted vs our convention (+left, -right). In
    # reverse, mirror the steer so the nose swings toward the open side.
    # Strictly clamp to [-18.0, 18.0] to prevent Webots Ackermann motor warnings.
    target_steer = max(-18.0, min(18.0, steer_deg if speed_mps < 0 else -steer_deg))

    # Slew-rate limiter: smooth steering commands to mimic real RC servo dynamics
    # and prevent instant slip-angle spikes, understeer wash-out, and tire scrub.
    dt = (sim_time - last_steer_time) if last_steer_time is not None else (basic_step / 1000.0)
    last_steer_time = sim_time
    max_rate = float(PARAMS.get("max_steer_rate_dps", 0.0))
    if max_rate > 0.0 and dt > 0.0:
        max_delta = max_rate * dt
        delta = target_steer - current_steer_cmd
        if delta > max_delta:
            current_steer_cmd += max_delta
        elif delta < -max_delta:
            current_steer_cmd -= max_delta
        else:
            current_steer_cmd = target_steer
    else:
        current_steer_cmd = target_steer

    steer_cmd = max(-18.0, min(18.0, current_steer_cmd))
    driver.setSteeringAngle(radians(steer_cmd))
    driver.setCruisingSpeed(speed_mps * 3.6)  # m/s to km/h (negative = reverse)

    # Path recording for Pure Pursuit raceline extraction
    if record_path and not path_saved and x is not None and y is not None:
        if lap_timer.lap_start_time is not None and lap_timer.lap_count == 0:
            yaw = 0.0
            if imu is not None:
                rpy = imu.getRollPitchYaw()
                if rpy:
                    yaw = rpy[2]
            recorded_points.append((x, y, yaw, speed_mps, steer_cmd))
        elif lap_timer.lap_count >= 1 and recorded_points:
            _save_record_path()

_save_record_path()
