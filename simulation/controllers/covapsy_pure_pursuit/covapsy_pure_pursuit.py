# Copyright 2026 COVAPSY Team
#
# Webots controller for the TT-02 car using Pure Pursuit path tracking
# on an optimized raceline with ground-truth GPS + IMU heading.
#
# Usage:
#   1. Set TT02 robot's "controller" field to "covapsy_pure_pursuit"
#   2. Click the 3D view, press 'A' to drive autonomously, 'N' to stop.

import os
import sys
from collections import deque
from math import hypot, isnan, radians, tan
from pathlib import Path

from controller import GPS, InertialUnit
from vehicle import Driver

# --- Import shared modules from common/ ------------------------------------
_COMMON = None
for _p in Path(__file__).resolve().parents:
    if (_p / "common" / "pure_pursuit.py").is_file():
        _COMMON = _p / "common"
        break
if _COMMON is None:
    raise RuntimeError("covapsy_pure_pursuit: could not locate common/")
sys.path.insert(0, str(_COMMON))

import config  # noqa: E402
import pure_pursuit as pp  # noqa: E402
import trajectory as traj_mod  # noqa: E402

PARAMS = config.PARAMS

# Load raceline trajectory
RACELINE_CSV = _COMMON / "maps" / "raceline.csv"
if not RACELINE_CSV.is_file():
    # If raceline.csv not found, build it from reference_path.csv
    REF_CSV = _COMMON / "maps" / "reference_path.csv"
    if REF_CSV.is_file():
        trajectory = traj_mod.optimize_raceline(REF_CSV, RACELINE_CSV)
    else:
        raise FileNotFoundError(f"Neither {RACELINE_CSV} nor {REF_CSV} found!")
else:
    trajectory = traj_mod.load_raceline(RACELINE_CSV)

# Optional file logging
_LOGF = None
_logpath = os.environ.get("COVAPSY_LOGFILE")
if _logpath:
    _LOGF = open(_logpath, "w", buffering=1)


def _log(msg):
    print(msg, flush=True)
    if _LOGF:
        _LOGF.write(msg + "\n")
        _LOGF.flush()


_log(f"[PURE PURSUIT] Loaded raceline with {trajectory.num_points} waypoints ({trajectory.total_length:.2f} m)")

# --- Virtual start/finish line configuration -------------------------------
FINISH_LINE_X = 0.0
FINISH_LINE_Y_MIN = -3.5
FINISH_LINE_Y_MAX = -1.5
LAP_DEBOUNCE_S = 5.0


class LapTimer:
    """Lap timer tracking start/finish line crossings via GPS & driver.getTime()."""

    def __init__(self, line_x=FINISH_LINE_X, y_min=FINISH_LINE_Y_MIN,
                 y_max=FINISH_LINE_Y_MAX, debounce_s=LAP_DEBOUNCE_S):
        self.line_x = line_x
        self.y_min = y_min
        self.y_max = y_max
        self.debounce_s = debounce_s
        self.prev_x = None
        self.prev_y = None
        self.lap_start_time = None
        self.last_lap_trigger_time = -float("inf")
        self.lap_count = 0
        self.last_lap_duration = None

    def update(self, current_time, x, y):
        if self.prev_x is not None:
            crossed = (self.prev_x < self.line_x <= x) and (self.y_min <= y <= self.y_max)
            if crossed and (current_time - self.last_lap_trigger_time >= self.debounce_s):
                self.last_lap_trigger_time = current_time
                if self.lap_start_time is None:
                    self.lap_start_time = current_time
                    _log(f"\033[32m[LAP TIMER] Start line crossed at {current_time:.2f}s -- Timing Pure Pursuit Lap 1!\033[0m")
                else:
                    lap_time = current_time - self.lap_start_time
                    self.lap_start_time = current_time
                    self.lap_count += 1
                    self.last_lap_duration = lap_time
                    _log(f"\033[32m[LAP TIMER] Lap {self.lap_count} completed: {lap_time:.2f} s "
                         f"(sim time: {current_time:.2f}s)\033[0m")
        self.prev_x = x
        self.prev_y = y


# --- Webots setup ----------------------------------------------------------
driver = Driver()
basic_step = int(driver.getBasicTimeStep())
sensor_step = 2 * basic_step

gps = GPS("gps")
gps.enable(sensor_step)

try:
    imu = InertialUnit("imu")
    imu.enable(sensor_step)
except Exception:
    imu = None

keyboard = driver.getKeyboard()
keyboard.enable(sensor_step)

lap_timer = LapTimer()

auto_mode = os.environ.get("COVAPSY_AUTOSTART") == "1"
headless_tune = os.environ.get("COVAPSY_HEADLESS_TUNE") == "1"
if auto_mode:
    _log("[PURE PURSUIT] Autostart enabled (COVAPSY_AUTOSTART=1)")

driver.setSteeringAngle(0.0)
driver.setCruisingSpeed(0.0)

current_steer_cmd = 0.0
last_steer_time = None
pos_history = deque()
measured_speed = 0.0
last_pos = None
last_pos_time = None
current_waypoint_idx = None

print("=" * 60)
print("  COVAPSY Pure Pursuit Controller -- Waypoint Tracking")
print("  Click the 3D view, then:")
print("    'A' = enable autonomous pure pursuit")
print("    'N' = stop (neutral throttle and steering)")
print("=" * 60)

step_count = 0
while driver.step() != -1:
    step_count += 1
    # Keyboard control
    while True:
        key = keyboard.getKey()
        if key == -1:
            break
        if key in (ord("a"), ord("A")):
            if not auto_mode:
                auto_mode = True
                _log("[PURE PURSUIT] Auto mode ENABLED")
        elif key in (ord("n"), ord("N")):
            if auto_mode:
                auto_mode = False
                _log("[PURE PURSUIT] Auto mode DISABLED")

    # Read GPS & IMU
    gps_coords = gps.getValues()
    x = gps_coords[0] if gps_coords and not isnan(gps_coords[0]) else None
    y = gps_coords[1] if gps_coords and not isnan(gps_coords[1]) else None
    sim_time = driver.getTime()

    yaw = 0.0
    if imu is not None:
        rpy = imu.getRollPitchYaw()
        if rpy:
            yaw = rpy[2]

    # Track speed & lap timing
    if x is not None and y is not None:
        lap_timer.update(sim_time, x, y)
        pos_history.append((sim_time, x, y))
        while pos_history and pos_history[0][0] < sim_time - 2.5:
            pos_history.popleft()

        spd_val = gps.getSpeed()
        if spd_val is not None and not isnan(spd_val) and spd_val >= 0.0:
            measured_speed = spd_val
        else:
            measured_speed = max(0.0, driver.getCurrentSpeed() / 3.6)

    # Headless benchmark mode quit
    if headless_tune:
        if lap_timer.lap_count >= 1:
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
                _log(f"[TUNE_RESULT] DNF (stuck at sim_time={sim_time:.2f}s, max_moved={max_moved:.3f}m)")
                sys.stdout.flush()
                driver.simulationQuit(1)
                break

    if not auto_mode or sim_time < 0.3 or x is None or y is None:
        driver.setSteeringAngle(0.0)
        driver.setCruisingSpeed(0.0)
        current_steer_cmd = 0.0
        last_steer_time = sim_time
        continue

    # Pure Pursuit calculation
    steer_deg, target_speed_mps, (tx, ty), current_waypoint_idx = pp.pure_pursuit_step(
        x, y, yaw, measured_speed, trajectory, PARAMS, prev_idx=current_waypoint_idx
    )

    # In Webots: steering sign convention is inverted (+left in pp -> -deg in Webots)
    target_steer = max(-18.0, min(18.0, -steer_deg))

    # Apply slew-rate limiter
    dt = (sim_time - last_steer_time) if last_steer_time is not None else (basic_step / 1000.0)
    last_steer_time = sim_time
    max_rate = float(PARAMS.get("max_steer_rate_dps", 180.0))
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

    # Dynamic cornering speed clamp: constrain lateral acceleration below tire grip limit (2.2 m/s^2).
    # Prevents power understeer when lookahead point is on a straight but vehicle is still negotiating a turn.
    steer_rad = radians(abs(steer_cmd))
    if steer_rad > radians(2.0):
        max_lat_accel = 2.20  # m/s^2
        steer_spd_cap = (max_lat_accel * 0.257 / tan(steer_rad)) ** 0.5
        target_speed_mps = max(1.10, min(target_speed_mps, steer_spd_cap))

    driver.setSteeringAngle(radians(steer_cmd))
    driver.setCruisingSpeed(target_speed_mps * 3.6)  # m/s to km/h

    if step_count % 30 == 0:
        _log(f"[PP] t={sim_time:.2f}s wp={current_waypoint_idx}/{trajectory.num_points} pos=({x:.2f},{y:.2f}) yaw={yaw:.2f} spd={measured_speed:.2f} steer_cmd={steer_cmd:.1f} target=({tx:.2f},{ty:.2f}) tgt_spd={target_speed_mps:.2f}")
