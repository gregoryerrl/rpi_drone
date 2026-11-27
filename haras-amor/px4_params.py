"""Save, compare and restore the flight controller's PX4 parameters.

  px4_params.py save FILE            every parameter, in QGroundControl's .params format
  px4_params.py diff FILE            what differs between the flight controller and FILE
  px4_params.py restore FILE [NAME]  set the differing parameters (or only NAME...) back to FILE

QGroundControl can load the same file (Parameters, Tools, Load from file). restore refuses while
PX4 is armed and prints every change. Stop the ch8-takeover service first: it holds the FC port.
"""
import argparse
import math
import struct
import time

from pymavlink.dialects.v20 import common as mav
from serial import Serial

DEVICE = "/dev/serial/by-id/usb-MicoAir_MicoAir743v2_0-if00"
INTEGER_TYPES = {mav.MAV_PARAM_TYPE_UINT8, mav.MAV_PARAM_TYPE_INT8, mav.MAV_PARAM_TYPE_UINT16,
                 mav.MAV_PARAM_TYPE_INT16, mav.MAV_PARAM_TYPE_UINT32, mav.MAV_PARAM_TYPE_INT32}
# Counters and identifiers PX4 changes by itself; restoring them would be meaningless.
VOLATILE = {"LND_FLIGHT_T_HI", "LND_FLIGHT_T_LO", "COM_FLIGHT_UUID", "SYS_RESTART_TYPE"}


def decode(value, kind):
    """PX4 sends integer parameters bytewise inside the float field."""
    return struct.unpack("<i", struct.pack("<f", value))[0] if kind in INTEGER_TYPES else value


def encode(value, kind):
    return struct.unpack("<f", struct.pack("<i", int(value)))[0] if kind in INTEGER_TYPES else float(value)


def format_params(params):
    """QGroundControl .params text for {name: (value, type)}."""
    lines = ["# Onboard parameters for Vehicle 1", "#", "# Vehicle-Id Component-Id Name Value Type"]
    for name in sorted(params):
        value, kind = params[name]
        text = str(int(value)) if kind in INTEGER_TYPES else repr(float(value))
        lines.append(f"1\t1\t{name}\t{text}\t{kind}")
    return "\n".join(lines) + "\n"


def parse_params(text):
    params = {}
    for line in text.splitlines():
        if not line.strip() or line.startswith("#"):
            continue
        _, _, name, value, kind = line.split("\t")
        kind = int(kind)
        params[name] = (int(value) if kind in INTEGER_TYPES else float(value), kind)
    return params


def differences(saved, current):
    """{name: (saved value, current value)} for parameters that differ, ignoring PX4's counters."""
    out = {}
    for name, (value, kind) in saved.items():
        if name in VOLATILE or name not in current:
            continue
        now = current[name][0]
        same = now == value if kind in INTEGER_TYPES else math.isclose(now, value, rel_tol=1e-6, abs_tol=1e-9)
        if not same:
            out[name] = (value, now)
    return out


class Link:
    def __init__(self):
        self.port = Serial(DEVICE, 57600, timeout=0.02, exclusive=True)
        self.wire = mav.MAVLink(self.port, srcSystem=255, srcComponent=190)
        self.wire.robust_parsing = True
        self.armed = None

    def messages(self):
        for msg in self.wire.parse_buffer(self.port.read(8192)) or ():
            if msg.get_type() != "BAD_DATA" and msg.get_srcComponent() == 1:
                if msg.get_type() == "HEARTBEAT":
                    self.armed = bool(msg.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED)
                yield msg

    def read_all(self):
        params, count, end = {}, None, time.monotonic() + 3
        self.wire.param_request_list_send(1, 1)
        while time.monotonic() < end and (count is None or len(params) < count):
            for msg in self.messages():
                if msg.get_type() == "PARAM_VALUE":
                    params[msg.param_id] = (decode(msg.param_value, msg.param_type), msg.param_type, msg.param_index)
                    count, end = msg.param_count, time.monotonic() + 1.5
        for _ in range(5):  # ask again for any the list stream dropped
            have = {index for _, _, index in params.values()}
            missing = [i for i in range(count or 0) if i not in have]
            if not missing:
                break
            for index in missing:
                self.wire.param_request_read_send(1, 1, b"", index)
                end = time.monotonic() + 0.3
                while time.monotonic() < end:
                    for msg in self.messages():
                        if msg.get_type() == "PARAM_VALUE":
                            params[msg.param_id] = (decode(msg.param_value, msg.param_type), msg.param_type,
                                                    msg.param_index)
        if count is None or len(params) < count:
            raise SystemExit(f"Only {len(params)} of {count} parameters arrived; try again.")
        return {name: (value, kind) for name, (value, kind, _) in params.items()}

    def set(self, name, value, kind):
        self.wire.param_set_send(1, 1, name.encode(), encode(value, kind), kind)
        end = time.monotonic() + 2
        while time.monotonic() < end:
            for msg in self.messages():
                if msg.get_type() == "PARAM_VALUE" and msg.param_id == name:
                    return decode(msg.param_value, msg.param_type)
        return None

    def close(self):
        self.port.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("action", choices=("save", "diff", "restore"))
    parser.add_argument("file")
    parser.add_argument("names", nargs="*", help="restore only these parameters")
    args = parser.parse_args()
    link = Link()
    try:
        current = link.read_all()
        if args.action == "save":
            with open(args.file, "w") as out:
                out.write(format_params(current))
            print(f"Saved {len(current)} parameters to {args.file}")
            return
        with open(args.file) as saved_file:
            saved = parse_params(saved_file.read())
        changed = differences(saved, current)
        if args.names:
            changed = {n: v for n, v in changed.items() if n in args.names}
        for name, (old, now) in sorted(changed.items()):
            print(f"{name}: now {now:g}, saved {old:g}")
        if not changed:
            print("No differences.")
        if args.action == "restore" and changed:
            if link.armed is not False:
                raise SystemExit("PX4 is armed or silent: nothing restored. Disarm with CH5 low.")
            for name, (old, _) in sorted(changed.items()):
                print(f"{name} -> {link.set(name, old, saved[name][1])}")
    finally:
        link.close()


if __name__ == "__main__":
    main()
