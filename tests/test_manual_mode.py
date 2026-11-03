"""Unit tests for manual mode (pi/gamepad.py). No controller or car needed."""

import os
import struct
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "pi"))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "common"))

from config import PARAMS                                          # noqa: E402
from gamepad import (Gamepad, manual_command, stick_unit,           # noqa: E402
                     trigger_unit, JS_EVENT_AXIS, JS_EVENT_BUTTON, JS_EVENT_INIT)


def _ev(value, etype, number):
    return struct.pack("<IhBB", 0, value, etype, number)


P = dict(PARAMS)
CAP = 0.8


def test_unit_conversions():
    assert stick_unit(-32767) == -1.0 and stick_unit(32767) == 1.0
    assert trigger_unit(-32767) == 0.0 and trigger_unit(32767) == 1.0
    assert abs(trigger_unit(0) - 0.5) < 1e-3


def test_unseen_triggers_read_released():
    gp = Gamepad(params=P)
    assert gp.rt == 0.0 and gp.lt == 0.0 and gp.steer == 0.0


def test_axis_and_button_events():
    gp = Gamepad(params=P)
    gp.handle_event(_ev(32767, JS_EVENT_AXIS, P["gp_axis_rt"]))
    gp.handle_event(_ev(-32767, JS_EVENT_AXIS, P["gp_axis_steer"]))
    assert gp.rt == 1.0 and gp.steer == -1.0
    gp.handle_event(_ev(1, JS_EVENT_BUTTON | JS_EVENT_INIT, 3))   # init state, not a press
    gp.handle_event(_ev(1, JS_EVENT_BUTTON, 7))                   # press
    gp.handle_event(_ev(0, JS_EVENT_BUTTON, 7))                   # release
    assert gp.pop_presses() == [7]
    assert gp.pop_presses() == []


def test_disarmed_gives_zero_speed_but_steers():
    steer, speed = manual_command(-1.0, 1.0, 0.0, False, P, CAP)
    assert speed == 0.0
    assert steer == 18.0                       # stick left -> +deg (left)


def test_analog_throttle():
    _, full = manual_command(0.0, 1.0, 0.0, True, P, CAP)
    _, half = manual_command(0.0, 0.54, 0.0, True, P, CAP)
    _, rest = manual_command(0.0, 0.02, 0.0, True, P, CAP)   # inside deadzone
    assert abs(full - CAP) < 1e-9
    assert 0.0 < half < full
    assert rest == 0.0


def test_reverse_and_conflict():
    _, rev = manual_command(0.0, 0.0, 1.0, True, P, CAP)
    assert rev < 0.0
    _, both = manual_command(0.0, 1.0, 1.0, True, P, CAP)
    assert both == 0.0


def test_steer_deadzone_and_right():
    steer, _ = manual_command(0.05, 0.0, 0.0, True, P, CAP)
    assert steer == 0.0
    steer, _ = manual_command(1.0, 0.0, 0.0, True, P, CAP)
    assert steer == -18.0                      # stick right -> -deg (right)

