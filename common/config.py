"""Tuning constants shared by the Webots simulator and the on-car runtime.

See gap_follower.py for how each key is used.

LiDAR convention: list of 360 distances in metres, index = angle in degrees,
index 0 = straight ahead, increasing index = counter-clockwise (left), so
index 90 = 90 deg left, index 270 = 90 deg right. Value <= 0 or non-finite
means no return and is treated as max range.
"""

PARAMS = {
    # --- car geometry ---
    "wheelbase_m": 0.257,         # TT-02 wheelbase, metres
    "max_steer_deg": 18.0,        # steering clamp, deg (keeps inside Ackermann wheel <= 0.35 rad limit)
    "car_width_m": 0.30,          # used to inflate obstacles (disparity extension)

    # --- pure pursuit path tracking ---
    "pp_lookahead_ratio_s": 0.30, # dynamic lookahead gain: L = ratio * speed
    "pp_lookahead_min_m": 0.60,   # minimum lookahead distance floor (sharp corner tracking)
    "pp_lookahead_max_m": 1.80,   # maximum lookahead distance ceiling

    # --- lidar conditioning ---
    "max_range_m": 5.0,           # clip ranges beyond this (track is < 5 m across)
    "min_valid_m": 0.05,          # readings closer than this treated as noise

    # --- gap finding ---
    # FOV kept < 180 deg, centred front: Pi and battery behind the LiDAR
    # occlude the rear, so rear/side-rear beams are unreliable.
    "front_fov_deg": 100,         # sector +/-100 deg so a corner-exit gap near +/-90 isn't truncated
    "gap_select_fov_deg": 100,    # max heading +/-100 deg so tight corner exits aren't clipped at +/-90
    "disparity_threshold_m": 0.35,# depth jump that triggers obstacle inflation
    "safety_radius_m": 0.18,      # bubble carved around nearest obstacle
    "clearance_margin_m": 0.055,  # extra berth on half-car-width when inflating walls
                                  # (raise if it clips corners, lower if it won't fit gaps)
    "steer_gain": 1.347,          # proportional gain, gap angle to steering angle
    "aim_deep_bias": 0.8,       # 0 = aim gap center, 1 = aim deepest point (pulls toward corner exit)

    # --- speed (conservative, to finish safely) ---
    "max_speed_mps": 3.143,       # cruise pace; ramp with --speed-cap. 2.0 = firmware soft cap
    "min_speed_mps": 0.977,       # floor speed in tight spots
    "reverse_speed_mps": 0.9,     # reverse speed, sim only (real car uses reverse_pwm_us)
    "reverse_pwm_us": 1250,       # real-car reverse pulse width, us; lower = faster reverse
    "ttc_target_s": 1.082,        # keep time-to-collision above this; raise = brake earlier

    # --- blocked / reverse detection ---
    "emergency_front_m": 0.30,    # front clearance below this == blocked, trigger reverse
    "reverse_exit_m": 0.65,       # hysteresis: must back up until clearance exceeds this before going forward
    "emergency_window_deg": 15,   # half-angle of the front-blocked cone
    "side_clear_min_deg": 20,     # side-clearance sampling window (for recovery dir)
    "side_clear_max_deg": 85,

    # --- runtime-only knobs (pi/drive.py and sim controller,
    #     NOT the pure gap_follow function) ---
    "loop_hz": 15,                # control loop rate, Hz (inside 250 ms watchdog)
    "max_steer_rate_dps": 180.0,  # steering slew-rate limit, deg/s (prevents instant snap & tire scrub; 150-250 deg/s realistic for RC servo)
    "steer_sign": 1,              # flip to -1 if the car steers the wrong way
    "steer_trim_deg": 0.0,        # steering trim offset (+left, -right) applied to all modes
    "lidar_front_offset_deg": 0,  # raw-lidar-angle to car-front calibration
    "rear_unknown_blocks": True,  # if rear sensor data missing, assume blocked
    "rear_sector_deg": 35,        # sim: +/- this around 180 deg == behind the car
    "rear_block_m": 0.5,          # closer than this behind == rear blocked
    "rear_self_ignore_m": 0.22,   # rear returns nearer than this = own body, ignore
                                  # (TT-02 shell occludes rear LiDAR; real car uses rear IR)
    "stuck_speed_mps": 0.05,      # measured speed below this while commanding fwd...
    "stuck_time_s": 2.0,          # ...for this long == stuck, trigger recovery

    # --- race control (race.py): stuck detection + recovery ---
    # Stuck = commanding forward but wheel encoder shows no rotation.
    "wheel_moving_pps": 5,        # encoder pulses/s below this == wheels not turning
    "stuck_secs": 3.0,            # commanding fwd but stalled this long == stuck
    "recovery_secs": 1.5,         # duration of the reverse-out recovery maneuver
    "control_port": 5005,         # TCP port for remote START/STOP
    "control_dir": "/tmp/covapsy",# control-file dir (touch start|stop|quit)

    # --- manual mode (race.py --gamepad): Xbox controller via /dev/input/js* ---
    "manual_max_speed_mps": 0.8,  # full RT = this speed; override with --manual-speed-cap
    "manual_reverse_enabled": True,  # LT = reverse (fixed reverse_pwm_us pulse on the real car)
    "manual_deadzone": 0.08,      # stick / trigger deadzone (fraction of full travel)
    "manual_trigger_on": 0.30,    # LT press past this fraction == reverse requested
    # Joystick axis/button numbers. Defaults = Xbox Series X over Bluetooth with
    # the stock kernel driver (hid-microsoft). Run tests/test_gamepad.py to check.
    # USB (xpad) / xpadneo layout: lt=2, rt=5, a=0, b=1, x=2, rb=5.
    "gp_axis_steer": 0,           # left stick X (negative = left)
    "gp_axis_rt": 4,              # right trigger (throttle)
    "gp_axis_lt": 5,              # left trigger (reverse)
    "gp_btn_a": 0,                # START autonomous
    "gp_btn_b": 1,                # STOP (neutral)
    "gp_btn_x": 3,                # enter MANUAL
    "gp_btn_rb": 7,               # arm / disarm manual throttle (press to toggle)
    "gp_axis_dpad_x": 6,          # D-pad left/right axis (-32767 left, +32767 right; Bluetooth or USB)
    "gp_btn_dpad_left": 13,       # D-pad button fallback (if mapped as buttons on some drivers)
    "gp_btn_dpad_right": 14,
    "gp_trim_step_deg": 0.25,     # steering trim step per D-pad press
}

# Allow overriding any parameter via TUNE_<key> environment variables
# (e.g., TUNE_max_speed_mps=2.8 or TUNE_steer_gain=1.25) or via
# common/.tune_overrides.json (used when Snap Webots isolates env/tmp).
import json as _json
import os as _os
from pathlib import Path as _Path

_overrides_file = _Path(__file__).resolve().parent / ".tune_overrides.json"
if _overrides_file.is_file():
    try:
        with open(_overrides_file, "r") as _f:
            _data = _json.load(_f)
        for _k, _v in _data.get("env", {}).items():
            _os.environ.setdefault(_k, str(_v))
        for _k, _v in _data.get("tune", {}).items():
            _os.environ[f"TUNE_{_k}"] = str(_v)
    except Exception:
        pass

for _env_key, _env_val in _os.environ.items():
    if _env_key.startswith("TUNE_"):
        _param_name = _env_key[len("TUNE_"):]
        if _param_name in PARAMS:
            _orig = PARAMS[_param_name]
            if isinstance(_orig, bool):
                PARAMS[_param_name] = _env_val.strip().lower() in ("1", "true", "yes")
            elif isinstance(_orig, str):
                PARAMS[_param_name] = _env_val
            elif _param_name == "max_steer_deg":
                PARAMS[_param_name] = min(float(_env_val), 18.0)
            else:
                PARAMS[_param_name] = float(_env_val)
        else:
            try:
                _val = float(_env_val)
                PARAMS[_param_name] = min(_val, 18.0) if _param_name == "max_steer_deg" else _val
            except ValueError:
                PARAMS[_param_name] = _env_val

