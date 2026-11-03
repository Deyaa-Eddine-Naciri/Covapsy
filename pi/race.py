"""Autonomous race control for the COVAPSY car.

States: IDLE (neutral, waiting for START), RUNNING (LiDAR gap-follow with rear-IR
reverse and stuck recovery), STOPPED (neutral, START re-arms to RUNNING). Boots IDLE.
MANUAL (only with --gamepad): Xbox controller drives; RB arms/disarms throttle.
Start/stop come from a TCP control server, control files in control_dir
(start/stop/quit), or the keyboard (s/x/q) on a TTY. The STM32 250 ms comms
watchdog neutralises the car if this program dies or USB drops.

Usage (Pi, wheels off the ground for first runs):
    python3 race.py                 # autonomous, boots IDLE
    python3 race.py --dry-run       # steering tracks LiDAR, throttle 0
    python3 race.py --camera        # also read RGB wrong-way (advisory only)
    python3 race.py --gamepad       # enable Xbox controller (X = manual, A = auto, B = stop)
"""

import argparse
import os
import queue
import socket
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "common"))
import gap_follower as gf          # noqa: E402
from config import PARAMS          # noqa: E402

from lidar import Lidar            # noqa: E402
from stm32_link import STM32Link   # noqa: E402


# --------------------------------------------------------------------------
# Remote start/stop: TCP server, control files, and keyboard feed one
# thread-safe command queue. Commands: START / STOP / QUIT / STATUS.
# --------------------------------------------------------------------------
class Control:
    def __init__(self, port: int, control_dir: str):
        self.port = port
        self.control_dir = control_dir
        try:
            os.makedirs(control_dir, exist_ok=True)
        except OSError:
            pass
        self._events = queue.Queue()
        self._state = "IDLE"
        self._running = True
        threading.Thread(target=self._serve, daemon=True).start()
        threading.Thread(target=self._watch_files, daemon=True).start()
        if sys.stdin and sys.stdin.isatty():
            threading.Thread(target=self._watch_keys, daemon=True).start()

    def set_state(self, s: str) -> None:
        self._state = s

    def poll(self):
        out = []
        while True:
            try:
                out.append(self._events.get_nowait())
            except queue.Empty:
                break
        return out

    def stop(self) -> None:
        self._running = False

    # command sources
    def _serve(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            srv.bind(("0.0.0.0", self.port))
            srv.listen(5)
        except OSError as e:
            print(f"[ctl] cannot bind TCP {self.port}: {e} (file/keyboard still work)")
            return
        srv.settimeout(0.5)
        while self._running:
            try:
                conn, _ = srv.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            threading.Thread(target=self._handle, args=(conn,), daemon=True).start()
        srv.close()

    def _handle(self, conn):
        with conn:
            conn.settimeout(2.0)
            try:
                cmd = conn.recv(64).decode("ascii", "replace").strip().upper()
            except Exception:
                return
            if cmd in ("START", "STOP", "QUIT", "MANUAL"):
                self._events.put(cmd)
                conn.sendall(f"OK {cmd} (state={self._state})\n".encode())
            elif cmd == "STATUS":
                conn.sendall(f"state={self._state}\n".encode())
            else:
                conn.sendall(b"ERR use START/STOP/MANUAL/STATUS/QUIT\n")

    def _watch_files(self):
        while self._running:
            for name, cmd in (("start", "START"), ("stop", "STOP"), ("quit", "QUIT")):
                p = os.path.join(self.control_dir, name)
                if os.path.exists(p):
                    try:
                        os.remove(p)
                    except OSError:
                        pass
                    self._events.put(cmd)
            time.sleep(0.1)

    def _watch_keys(self):
        for line in sys.stdin:
            c = line.strip().lower()
            if c in ("s", "start"):
                self._events.put("START")
            elif c in ("x", "stop"):
                self._events.put("STOP")
            elif c in ("q", "quit"):
                self._events.put("QUIT")
            elif c in ("m", "manual"):
                self._events.put("MANUAL")


def parse_args():
    ap = argparse.ArgumentParser(description="COVAPSY race control")
    ap.add_argument("--dry-run", action="store_true",
                    help="stream steering only; throttle forced to 0")
    ap.add_argument("--no-steer", action="store_true", help="force steering 0 too")
    ap.add_argument("--camera", action="store_true",
                    help="read the RGB camera for wrong-way (advisory only)")
    ap.add_argument("--speed-cap", type=float, default=None)
    ap.add_argument("--autostart", action="store_true",
                    help="boot straight into RUNNING (TESTING ONLY -- unsafe on the ground)")
    ap.add_argument("--planner", choices=["ftg", "pp"], default="ftg",
                    help="Trajectory planning algorithm: ftg (Follow-the-Gap, default) or pp (Pure Pursuit)")
    ap.add_argument("--lidar-port", default="/dev/ttyUSB0")
    ap.add_argument("--stm32-port", default="/dev/ttyACM0")
    ap.add_argument("--gamepad", nargs="?", const="/dev/input/js0", default=None,
                    metavar="DEV", help="enable Xbox controller manual mode (default /dev/input/js0)")
    ap.add_argument("--manual-speed-cap", type=float, default=None,
                    help="manual mode top speed at full RT, m/s (default manual_max_speed_mps)")
    return ap.parse_args()


def _engage_reverse(link):
    """Brake-then-neutral double-tap that arms WP-1060 reverse before the reverse pulse.

    Without it the ESC only brakes. Raw PWM re-sent at ~25 Hz to feed the 250 ms
    watchdog. Blocks ~0.45 s, only on the transition into reverse.
    """
    for us, dur in (("P1150", 0.30), ("P1500", 0.15)):   # brake (while rolling), then neutral
        t0 = time.monotonic()
        while time.monotonic() - t0 < dur:
            link.send_raw(us)
            time.sleep(0.04)


def main():
    args = parse_args()
    p = dict(PARAMS)
    if args.speed_cap is not None:
        p["max_speed_mps"] = args.speed_cap
        # When a user sets an explicit speed cap lower than the default cornering
        # floor speed (min_speed_mps), ensure the floor doesn't override the cap.
        p["min_speed_mps"] = min(p["min_speed_mps"], args.speed_cap)
    if args.manual_speed_cap is not None:
        p["manual_max_speed_mps"] = args.manual_speed_cap
    period = 1.0 / p["loop_hz"]
    steer_sign = p["steer_sign"]

    ctl = Control(p["control_port"], p["control_dir"])
    state = "RUNNING" if args.autostart else "IDLE"

    planner_mode = args.planner.lower()
    trajectory = None
    pp_idx = None
    if planner_mode == "pp":
        try:
            import pure_pursuit as pp_mod  # noqa: F401
            import trajectory as traj_mod
            raceline_path = Path(__file__).resolve().parent.parent / "common" / "maps" / "raceline.csv"
            if raceline_path.is_file():
                trajectory = traj_mod.load_raceline(raceline_path)
                print(f"[race] Pure Pursuit initialized: {trajectory.num_points} waypoints ({trajectory.total_length:.2f}m)")
            else:
                print(f"[race] WARNING: {raceline_path} not found! Falling back to FTG.")
                planner_mode = "ftg"
        except Exception as e:
            print(f"[race] Pure Pursuit init failed ({e}), falling back to FTG.")
            planner_mode = "ftg"

    print("=" * 62)
    print(" COVAPSY RACE CONTROL  --  WHEELS OFF THE GROUND for first runs")
    print(f" mode: {'DRY-RUN' if args.dry_run else 'LIVE'}   "
          f"planner: {planner_mode.upper()}   "
          f"cap {p['max_speed_mps']:.2f} m/s   ctl TCP :{p['control_port']}   "
          f"files {p['control_dir']}/(start|stop|quit)")
    print(" START -> drive,  STOP -> neutral.  (covapsy_ctl.py start|stop)")
    if args.gamepad:
        print(f" gamepad {args.gamepad}: X = MANUAL, RB = arm/disarm throttle, "
              f"A = START, B = STOP   manual cap {p['manual_max_speed_mps']:.2f} m/s")
    print("=" * 62)

    # Stuck-recovery state.
    stuck_since = None
    recovery_until = 0.0
    recovery_steer = 0.0
    reverse_armed = False   # WP-1060 reverse brake-then-neutral tap already done?

    cam = None
    last_cam = 0.0
    wrong_way = False

    # Manual mode (only with --gamepad).
    gp = None
    gp_drops = 0
    manual_armed = False
    if args.gamepad:
        from gamepad import Gamepad, manual_command
        gp = Gamepad(args.gamepad, p).start()

    with Lidar(args.lidar_port, front_offset_deg=p["lidar_front_offset_deg"]) as lidar, \
            STM32Link(args.stm32_port) as link:

        if args.camera:
            try:
                from camera import Camera
                cam = Camera().start()
                print(f" camera: {cam.device}")
            except Exception as e:
                print(f" camera unavailable ({e}); continuing without it")

        print(" waiting for LiDAR...")
        t0 = time.monotonic()
        while not lidar.healthy and (time.monotonic() - t0) < 8.0:
            link.stop()
            time.sleep(0.1)
        if not lidar.healthy:
            print(" ERROR: no LiDAR scans. Check /dev/ttyUSB0.")
            ctl.stop()
            return 1
        print(f" LiDAR OK. State = {state}.")

        last_log = 0.0
        try:
            while True:
                tick = time.monotonic()

                cmds = ctl.poll()
                if gp is not None:
                    for b in gp.pop_presses():
                        if b == p["gp_btn_b"]:
                            cmds.append("STOP")
                        elif b == p["gp_btn_a"]:
                            cmds.append("START")
                        elif b == p["gp_btn_x"]:
                            cmds.append("MANUAL")
                        elif b == p["gp_btn_rb"] and state == "MANUAL":
                            manual_armed = not manual_armed
                            print(f" >> manual throttle {'ARMED' if manual_armed else 'disarmed'}")
                    if state == "MANUAL" and (not gp.connected or gp.drops != gp_drops):
                        state = "STOPPED"
                        manual_armed = False
                        print(" !! gamepad lost -> STOPPED")
                    gp_drops = gp.drops

                for cmd in cmds:
                    if cmd == "START" and state != "RUNNING":
                        state = "RUNNING"
                        manual_armed = False
                        stuck_since = None
                        recovery_until = 0.0
                        print(" >> START -> RUNNING")
                    elif cmd == "STOP" and state != "STOPPED":
                        state = "STOPPED"
                        manual_armed = False
                        print(" >> STOP -> STOPPED")
                    elif cmd == "MANUAL" and state != "MANUAL":
                        if gp is None:
                            print(" !! MANUAL needs --gamepad; ignored")
                        elif not gp.connected:
                            print(f" !! no gamepad on {args.gamepad}; MANUAL ignored")
                        else:
                            state = "MANUAL"
                            manual_armed = False
                            print(" >> MANUAL (throttle disarmed, press RB to arm)")
                    elif cmd == "QUIT":
                        raise KeyboardInterrupt
                ctl.set_state(state)

                scan = lidar.get_scan()
                rear_blocked = not link.rear_clear()
                front = gf.front_clearance(scan, p)

                if state == "RUNNING" and lidar.healthy:
                    is_reversing = (prev_speed_mps < 0.0) if 'prev_speed_mps' in locals() else False
                    if planner_mode == "pp" and trajectory is not None:
                        # Pure Pursuit strategy: requires pose (car_x, car_y, car_yaw)
                        # In Stage 2, this pose will be continuously estimated by the LiDAR SLAM node.
                        # Default fallback to FTG if pose estimation is not yet active.
                        steer_deg, speed_mps = gf.gap_follow(scan, rear_blocked, p, is_reversing=is_reversing)
                    else:
                        steer_deg, speed_mps = gf.gap_follow(scan, rear_blocked, p, is_reversing=is_reversing)
                    prev_speed_mps = speed_mps

                    # Stuck: commanding forward but wheel encoder shows no
                    # turning for stuck_secs.
                    if speed_mps > 0.05 and link.wheels_stalled(p["wheel_moving_pps"]):
                        if stuck_since is None:
                            stuck_since = tick
                        elif (tick - stuck_since) > p["stuck_secs"] and tick > recovery_until:
                            # Back out toward the more open side if the rear is clear.
                            lc, rc = gf.side_clearances(scan, p)
                            recovery_steer = p["max_steer_deg"] if lc >= rc else -p["max_steer_deg"]
                            recovery_until = tick + p["recovery_secs"]
                            stuck_since = None
                            print(" !! stuck (wheels not turning) -> recovery reverse")
                    else:
                        stuck_since = None

                    # Active recovery overrides the gap follower.
                    if tick < recovery_until:
                        if not rear_blocked:
                            steer_deg, speed_mps = recovery_steer, -p["reverse_speed_mps"]
                        else:
                            steer_deg, speed_mps = 0.0, 0.0   # boxed both ways, hold
                elif state == "MANUAL":
                    steer_deg, speed_mps = manual_command(
                        gp.steer, gp.rt, gp.lt, manual_armed, p,
                        p["manual_max_speed_mps"])
                else:
                    steer_deg, speed_mps = 0.0, 0.0

                cmd_speed = speed_mps
                if args.no_steer:
                    steer_deg = 0.0
                if args.dry_run:
                    speed_mps = 0.0

                # Send to the car. Reverse arms once via the brake-neutral tap, then
                # drives reverse. In autonomous mode, steering is inverted in reverse
                # (kinematics mirrored to point nose toward opening). In manual mode,
                # steering remains direct and intuitive for the human driver.
                if speed_mps < 0.0 and not rear_blocked:
                    if not reverse_armed:
                        _engage_reverse(link)
                        reverse_armed = True
                    # Scale reverse PWM proportionally between neutral (1440us) and reverse_pwm_us (e.g. 1250us)
                    # based on commanded reverse speed.
                    rev_ratio = min(1.0, abs(speed_mps) / max(0.01, p.get("reverse_speed_mps", 0.9)))
                    neutral_rev_us = 1440.0
                    target_rev_us = float(p["reverse_pwm_us"])
                    pulse_us = int(round(neutral_rev_us - rev_ratio * (neutral_rev_us - target_rev_us)))

                    actual_steer = steer_deg if state == "MANUAL" else -steer_deg
                    link.set_steering(steer_sign * actual_steer)
                    link.send_raw("P%d" % pulse_us)
                else:
                    reverse_armed = False
                    link.set_steering(steer_sign * steer_deg)
                    link.set_speed(max(speed_mps, 0.0))

                # Camera wrong-way: advisory telemetry only, not a control input.
                if cam is not None and (tick - last_cam) > 0.2:
                    try:
                        wrong_way = bool(cam.read().get("wrong_way", False))
                    except Exception:
                        wrong_way = False
                    last_cam = tick

                if tick - last_log >= 0.5:
                    head = link.heading_deg
                    act = link.speed_mps or 0.0
                    print(f" [{state:7}] front={front:4.2f}m steer={steer_deg:+6.1f} "
                          f"cmd={cmd_speed:+4.2f} act={act:4.2f}m/s "
                          f"rear={'BLK' if rear_blocked else 'clr'} "
                          f"head={head if head is not None else '--'} "
                          f"{'WRONG-WAY' if wrong_way else ''}"
                          f"{(' ARMED' if manual_armed else ' disarmed') if state == 'MANUAL' else ''}")
                    last_log = tick

                dt = time.monotonic() - tick
                if dt < period:
                    time.sleep(period - dt)
        except KeyboardInterrupt:
            print("\n quit -> neutralising.")
        finally:
            link.stop()
            ctl.stop()
            if cam is not None:
                cam.stop()
            if gp is not None:
                gp.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
