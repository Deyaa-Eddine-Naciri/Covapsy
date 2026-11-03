"""
Xbox (Series X / One) controller reader for manual mode, stdlib only.

Reads the Linux joystick API (/dev/input/js0), which works the same for a
controller on USB or paired over Bluetooth. A daemon thread decodes 8-byte
js_event records into the latest axis values and a queue of button presses;
the control loop polls them without blocking. If the device disappears
(battery, Bluetooth drop, cable) `connected` goes False and the thread keeps
retrying the open once a second.

Axis values are normalised by the kernel to [-32767, +32767]. Triggers rest
at -32767 (released) and reach +32767 fully pressed.

Axis/button numbers depend on the driver (xpad USB vs Bluetooth); they come
from the gp_* keys in common/config.py. tests/test_gamepad.py prints them.
"""

import math
import os
import select
import struct
import threading
import time

DEFAULT_DEVICE = "/dev/input/js0"

_EVENT = struct.Struct("<IhBB")   # u32 time_ms, s16 value, u8 type, u8 number
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80
_AXIS_MAX = 32767.0


def stick_unit(raw: int) -> float:
    """Stick raw value to [-1, 1]."""
    return max(-1.0, min(1.0, raw / _AXIS_MAX))


def trigger_unit(raw: int) -> float:
    """Trigger raw value (rest -32767) to [0, 1]."""
    return max(0.0, min(1.0, (raw + _AXIS_MAX) / (2.0 * _AXIS_MAX)))


def _apply_deadzone(v: float, dz: float) -> float:
    """Zero inside the deadzone, rescale the rest back to full range."""
    if abs(v) <= dz:
        return 0.0
    return math.copysign((abs(v) - dz) / (1.0 - dz), v)


def manual_command(steer: float, rt: float, lt: float, armed: bool,
                   p: dict, speed_cap: float):
    """Map controller state to (steer_deg, speed_mps). Pure function.

    steer: left stick X in [-1, 1] (negative = left). rt, lt: triggers [0, 1].
    +steer_deg = LEFT, matching the rest of the codebase. Throttle is analog
    (speed = rt * speed_cap). LT provides proportional reverse up to
    reverse_speed_mps. Pressing both triggers, or being disarmed, gives speed 0.
    """
    dz = p["manual_deadzone"]
    max_steer = min(float(p.get("max_steer_deg", 18.0)), 18.0)
    steer_deg = -_apply_deadzone(steer, dz) * max_steer

    if not armed:
        return steer_deg, 0.0

    throttle = _apply_deadzone(rt, dz)
    rev_input = _apply_deadzone(lt, dz)
    reverse = p["manual_reverse_enabled"] and rev_input > 0.0

    if reverse and throttle > 0.0:
        return steer_deg, 0.0                      # conflicting inputs: hold
    if reverse:
        rev_cap = float(p.get("reverse_speed_mps", 0.9))
        return steer_deg, -rev_input * rev_cap
    return steer_deg, throttle * max(speed_cap, 0.0)


class Gamepad:
    def __init__(self, device: str = DEFAULT_DEVICE, params: dict = None):
        p = params or {}
        self.device = device
        self.ax_steer = int(p.get("gp_axis_steer", 0))
        self.ax_rt = int(p.get("gp_axis_rt", 4))
        self.ax_lt = int(p.get("gp_axis_lt", 5))
        self._lock = threading.Lock()
        self._axes = {}
        self._presses = []
        self._connected = False
        self.drops = 0               # disconnect count, so brief drops aren't missed
        self._running = False
        self._thread = None
        self._fd = None

    # lifecycle

    def start(self) -> "Gamepad":
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        return self

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        self._close()

    def __enter__(self) -> "Gamepad":
        return self.start()

    def __exit__(self, exc_type, exc, tb) -> None:
        self.stop()

    # public read API

    @property
    def connected(self) -> bool:
        with self._lock:
            return self._connected

    def axis(self, n: int, default: int = 0) -> int:
        with self._lock:
            return self._axes.get(n, default)

    @property
    def steer(self) -> float:
        return stick_unit(self.axis(self.ax_steer, 0))

    @property
    def rt(self) -> float:
        return trigger_unit(self.axis(self.ax_rt, -32767))   # unseen = released

    @property
    def lt(self) -> float:
        return trigger_unit(self.axis(self.ax_lt, -32767))

    def pop_presses(self) -> list:
        """Button numbers pressed (down edges) since the last call."""
        with self._lock:
            out, self._presses = self._presses, []
            return out

    # event decoding (separate from I/O so it is unit-testable)

    def handle_event(self, data: bytes) -> None:
        _t, value, etype, number = _EVENT.unpack(data)
        kind = etype & ~JS_EVENT_INIT
        with self._lock:
            if kind == JS_EVENT_AXIS:
                self._axes[number] = value
            elif kind == JS_EVENT_BUTTON:
                # INIT events report the state at open time, not a real press.
                if value == 1 and not (etype & JS_EVENT_INIT):
                    self._presses.append(number)

    # background reader

    def _close(self) -> None:
        if self._fd is not None:
            try:
                os.close(self._fd)
            except OSError:
                pass
            self._fd = None
        with self._lock:
            if self._connected:
                self.drops += 1
            self._connected = False
            self._axes = {}           # forget stale stick/trigger positions

    def _loop(self) -> None:
        while self._running:
            if self._fd is None:
                try:
                    self._fd = os.open(self.device, os.O_RDONLY | os.O_NONBLOCK)
                    with self._lock:
                        self._connected = True
                        self._presses = []
                except OSError:
                    time.sleep(1.0)
                    continue
            try:
                r, _, _ = select.select([self._fd], [], [], 0.1)
                if not r:
                    continue
                data = os.read(self._fd, _EVENT.size * 16)
                if not data:
                    raise OSError("device closed")
                for i in range(0, len(data) - _EVENT.size + 1, _EVENT.size):
                    self.handle_event(data[i:i + _EVENT.size])
            except BlockingIOError:
                continue
            except (OSError, ValueError):
                self._close()
