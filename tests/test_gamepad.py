"""
Xbox controller mapping check. No car hardware needed, only the controller.

Prints the controller name, then every axis/button event with its number, so
you can confirm (or fix) the gp_* numbers in common/config.py. Ctrl+C quits.

    python3 tests/test_gamepad.py               # /dev/input/js0
    python3 tests/test_gamepad.py /dev/input/js1

Move left stick left/right -> gp_axis_steer. Squeeze RT / LT -> gp_axis_rt /
gp_axis_lt (they rest at -32767). Press A, B, X, RB -> gp_btn_*.
"""

import array
import fcntl
import os
import struct
import sys

_EVENT = struct.Struct("<IhBB")
_JSIOCGNAME = 0x80006A13 + (128 << 16)   # _IOC(_IOC_READ, 'j', 0x13, 128)
_JSIOCGAXES = 0x80016A11
_JSIOCGBUTTONS = 0x80016A12


def main() -> int:
    dev = sys.argv[1] if len(sys.argv) > 1 else "/dev/input/js0"
    try:
        fd = os.open(dev, os.O_RDONLY)
    except OSError as e:
        print(f"cannot open {dev}: {e}")
        print("  USB: plug it in.  Bluetooth: pair with bluetoothctl first.  ls /dev/input/js*")
        return 1

    name = array.array("B", [0] * 128)
    n_ax = array.array("B", [0])
    n_bt = array.array("B", [0])
    try:
        fcntl.ioctl(fd, _JSIOCGNAME, name)
        fcntl.ioctl(fd, _JSIOCGAXES, n_ax)
        fcntl.ioctl(fd, _JSIOCGBUTTONS, n_bt)
        label = name.tobytes().split(b"\0", 1)[0].decode(errors="replace")
        print(f"{dev}: {label}  ({n_ax[0]} axes, {n_bt[0]} buttons)")
    except OSError:
        print(f"{dev}: (name query failed)")

    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "common"))
    from config import PARAMS
    print("config: steer axis %d, RT axis %d, LT axis %d | A %d, B %d, X %d, RB %d" % (
        PARAMS["gp_axis_steer"], PARAMS["gp_axis_rt"], PARAMS["gp_axis_lt"],
        PARAMS["gp_btn_a"], PARAMS["gp_btn_b"], PARAMS["gp_btn_x"], PARAMS["gp_btn_rb"]))
    print("move sticks / triggers / buttons (Ctrl+C to quit)\n")

    last_axis = {}
    try:
        while True:
            data = os.read(fd, _EVENT.size)
            if len(data) < _EVENT.size:
                break
            _t, value, etype, number = _EVENT.unpack(data)
            init = " (init)" if etype & 0x80 else ""
            kind = etype & 0x7F
            if kind == 0x01:
                print(f"button {number:2d} {'DOWN' if value else 'up  '}{init}")
            elif kind == 0x02:
                # Only print meaningful axis changes to keep the output readable.
                if init or abs(value - last_axis.get(number, 0)) > 4000 or value in (-32767, 32767):
                    print(f"axis   {number:2d} = {value:+6d}{init}")
                    last_axis[number] = value
    except KeyboardInterrupt:
        pass
    except OSError as e:
        print(f"\nread error: {e} (controller disconnected?)")
    finally:
        os.close(fd)
    return 0


if __name__ == "__main__":
    sys.exit(main())
