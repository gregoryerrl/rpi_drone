"""Check PX4's motor layout on the jig by spinning motor pairs and watching which way the drone tips.

Props on, drone in the jig, disarmed, CH6 low, nobody near the props, the ch8-takeover service
stopped (it holds the FC port). The script reads PX4's motor layout (CA_ROTOR0-3_PX and _PY) and
spins, one after another for SPIN_SECONDS at POWER, the pair PX4 places at its front, its back,
its left and its right. A spinning pair lifts its own side: the front pair should raise the nose
(pitch up), the back pair lower it, the left pair roll the drone right (positive roll) and the
right pair roll it left. A pair that tips the drone the other way means PX4 has that axis
mirrored: CA_ROTOR0-3_PX for front/back, CA_ROTOR0-3_PY for left/right. Nothing is changed on
the flight controller.
"""
import argparse
import math
import time

import numpy as np
from pymavlink.dialects.v20 import common as mav
from serial import Serial

DEVICE = "/dev/serial/by-id/usb-MicoAir_MicoAir743v2_0-if00"
SPIN_SECONDS, POWER, GAP = 1.0, 0.20, 3.0
FIRST_SPIN = 2.0      # s after start, for PX4 and the radio
MIN_CHANGE = math.radians(1.0)  # smaller tips are too small to tell
# name, layout parameter, side of it, attitude axis, expected sign of the tip
SIDES = (("front", "PX", +1, "pitch", +1), ("back", "PX", -1, "pitch", -1),
         ("left", "PY", -1, "roll", +1), ("right", "PY", +1, "roll", -1))


def pairs_from_layout(layout):
    """[(side, motors, axis, expected sign)] from {(motor number, "PX" or "PY"): value}."""
    pairs = []
    for side, axis_param, sign, axis, expected in SIDES:
        motors = tuple(sorted(m for (m, p), value in layout.items() if p == axis_param and value * sign > 0))
        if len(motors) != 2:
            raise ValueError(f"expected two motors on PX4's {side}, got {motors or 'none'}")
        pairs.append((side, motors, axis, expected))
    return pairs


def verdict(results):
    """Report lines and the fix for [(side, motors, axis, expected sign, change in radians)]."""
    lines, mirrored, unclear = [], set(), False
    for side, motors, axis, expected, change in results:
        name = "M" + "+M".join(map(str, motors))
        if abs(change) < MIN_CHANGE:
            state, unclear = "too small to tell", True
        elif change * expected > 0:
            state = "as PX4 expects"
        else:
            state = "OPPOSITE to what PX4 expects"
            mirrored.add("PX" if axis == "pitch" else "PY")
        lines.append(f"PX4's {side} pair {name}: {axis} {math.degrees(change):+.1f} deg "
                     f"(expected {'+' if expected > 0 else '-'}) - {state}")
    if mirrored:
        what = {"PX": "front and back (flip the sign of CA_ROTOR0-3_PX)",
                "PY": "left and right (flip the sign of CA_ROTOR0-3_PY)"}
        lines.append("PX4's motor layout has " + " and ".join(what[m] for m in sorted(mirrored)) + " mirrored.")
    elif unclear:
        lines.append("Some tips were too small to tell; check that the jig lets the drone tilt that way.")
    else:
        lines.append("PX4's motor layout matches the drone.")
    return lines


def main():
    parser = argparse.ArgumentParser(description="Spin PX4's motor pairs on the jig and check its motor layout.")
    parser.add_argument("--axis", choices=("both", "pitch", "roll"), default="both",
                        help="which pairs to spin: front/back (pitch), left/right (roll) or all four")
    parser.add_argument("--power", type=float, default=POWER * 100, metavar="PERCENT",
                        help="motor power for each spin (default %(default)g, at most 50)")
    parser.add_argument("--seconds", type=float, default=SPIN_SECONDS, help="length of each spin (default %(default)g, at most 3)")
    args = parser.parse_args()
    if not 5 <= args.power <= 50 or not 0.2 <= args.seconds <= 3:
        parser.error("--power must be 5 to 50 percent and --seconds 0.2 to 3")
    power, seconds = args.power / 100, args.seconds
    port = Serial(DEVICE, 57600, timeout=0.01, exclusive=True)
    wire = mav.MAVLink(port, srcSystem=255, srcComponent=190)
    wire.robust_parsing = True
    names = {f"CA_ROTOR{i}_{p}": (i + 1, p) for i in range(4) for p in ("PX", "PY")}
    layout, attitude, state = {}, [], {"heartbeat": None, "ch6": None}

    def pump():
        for msg in wire.parse_buffer(port.read(4096)) or ():
            kind = msg.get_type()
            if kind == "BAD_DATA" or msg.get_srcComponent() != 1:
                continue
            if kind == "PARAM_VALUE" and msg.param_id in names:
                layout[names[msg.param_id]] = msg.param_value
            elif kind == "HEARTBEAT":
                state["heartbeat"] = msg
            elif kind == "RC_CHANNELS":
                state["ch6"] = msg.chan6_raw
            elif kind == "ATTITUDE":
                attitude.append((time.monotonic(), msg.roll, msg.pitch))

    def spin(motors, value, timeout):
        for motor in motors:
            wire.command_long_send(1, 1, mav.MAV_CMD_ACTUATOR_TEST, 0, value, timeout, 0, 0, motor, 0, 0)

    start = time.monotonic()
    try:
        for name in names:
            wire.param_request_read_send(1, 1, name.encode(), -1)
        for message, interval in ((mav.MAVLINK_MSG_ID_RC_CHANNELS, 50000), (mav.MAVLINK_MSG_ID_ATTITUDE, 10000)):
            wire.command_long_send(1, 1, mav.MAV_CMD_SET_MESSAGE_INTERVAL, 0, message, interval, 0, 0, 0, 0, 0)
        while time.monotonic() - start < FIRST_SPIN:
            pump()
        try:
            pairs = pairs_from_layout(layout)
        except ValueError as exc:
            raise SystemExit(f"Could not read PX4's motor layout: {exc}")
        heartbeat = state["heartbeat"]
        if heartbeat is None or heartbeat.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED:
            raise SystemExit("PX4 is armed or silent. Disarm with CH5 low.")
        if state["ch6"] is None or state["ch6"] > 1500:
            raise SystemExit(f"CH6 (kill) reads {state['ch6']} (none means no radio data). Lower it; "
                             "PX4 blocks motor tests while killed.")
        if not attitude:
            raise SystemExit("No attitude from PX4.")
        pairs = [pair for pair in pairs if args.axis in ("both", pair[2])]
        schedule = [(FIRST_SPIN + i * (seconds + GAP), pair) for i, pair in enumerate(pairs)]
        for at, (side, motors, _, _) in schedule:
            print(f"{at:4.1f} s: PX4's {side} pair M" + "+M".join(map(str, motors)) + f" at {power:.0%} for {seconds:g} s")
        next_send, end = 0.0, schedule[-1][0] + seconds + 1.0
        while (now := time.monotonic() - start) < end:
            pump()
            if state["ch6"] is not None and state["ch6"] > 1500:
                raise SystemExit("CH6 raised: stopped.")
            active = next((pair[1] for at, pair in schedule if at <= now < at + seconds), None)
            if active and now >= next_send:
                spin(active, power, 0.4)
                next_send = now + 0.15
    finally:
        for _ in range(3):
            spin((1, 2, 3, 4), 0, 0)
            time.sleep(0.05)
        port.flush()
        port.close()

    times = np.array([a[0] for a in attitude]) - start
    angles = {"roll": np.array([a[1] for a in attitude]), "pitch": np.array([a[2] for a in attitude])}
    results = []
    for at, (side, motors, axis, expected) in schedule:
        before = np.interp(at - 0.05, times, angles[axis])
        after = np.interp(at + seconds + 0.2, times, angles[axis])
        results.append((side, motors, axis, expected, after - before))
    print("\n".join(verdict(results)))


if __name__ == "__main__":
    main()
