"""Measure how the aircraft camera's picture slides when the drone rolls, for the follow camera.

Run from haras-amor with the ch8-takeover service stopped (it owns the FC port and the camera),
the props on, the drone in the roll jig, CH5 low and CH6 low:

    systemctl --user stop ch8-takeover
    venv/bin/python calibrate_camera.py [--nose top|bottom|left|right]
    systemctl --user start ch8-takeover

It reads PX4's motor layout (CA_ROTOR0-3_PY) and spins the motors PX4 places on its left, then
those on its right, at 20% for 1 s each, as in the jig checks. PX4's roll must go positive for
the left side and negative for the right side. If it does not, PX4's orientation and motor
layout disagree and nothing is saved. Otherwise the
picture's shift per unit tan(roll) gives the camera's roll column and focal length, and --nose,
the side of the picture the drone's nose appears on, gives the pitch column. The camera looks up,
so pitching nose-up slides the scene toward the nose side. The result goes to MOUNT_FILE, which
follow_camera reads when the service starts.
"""
import argparse
import datetime
import json
import math
from pathlib import Path
import threading
import time

import cv2
import numpy as np
from pymavlink.dialects.v20 import common as mav
from serial import Serial

from follow_camera.camera import MOUNT_FILE, SIZE, camera_frames

DEVICE = "/dev/serial/by-id/usb-MicoAir_MicoAir743v2_0-if00"
NOSE = {"top": (0, -1), "bottom": (0, 1), "left": (-1, 0), "right": (1, 0)}
SPIN_SECONDS, POWER = 1.0, 0.20


def spins_from_layout(lateral):
    """(motors, start time, expected roll direction) for PX4's left side, then its right side.

    lateral maps motor number to CA_ROTORn_PY: negative on PX4's left. Lifting the left side
    rolls the drone right (positive roll)."""
    left = tuple(sorted(motor for motor, y in lateral.items() if y < 0))
    right = tuple(sorted(motor for motor, y in lateral.items() if y > 0))
    if len(left) != 2 or len(right) != 2:
        raise ValueError(f"expected two motors on each side, got {lateral}")
    return [(left, 3.0, 1), (right, 8.0, -1)]
MIN_ROLL = math.radians(2)


def picture_shift(before, after, window):
    """How far the picture content moved from before to after, in pixels (x right, y down)."""
    return np.array(cv2.phaseCorrelate(before, after, window)[0])


def fit_column(tilts, shifts):
    """Least-squares picture shift per unit of tilt, through the origin."""
    tilts, shifts = np.asarray(tilts, float), np.asarray(shifts, float)
    return (tilts @ shifts) / (tilts @ tilts)


def mount_from(roll_column, nose):
    """Mount matrix from the measured roll column and the side of the picture the nose is on."""
    roll_column = np.asarray(roll_column, float)
    focal = float(np.hypot(*roll_column))
    direction = np.array(NOSE[nose], float)
    if abs(roll_column @ direction) / focal > 0.35:
        raise ValueError("rolling slid the picture toward the nose side; check --nose")
    return np.column_stack([roll_column, focal * direction])


def main():
    parser = argparse.ArgumentParser(description="Calibrate the follow camera on the roll jig.")
    parser.add_argument("--nose", choices=NOSE, default="top",
                        help="side of the camera picture the drone's nose is on (default top)")
    parser.add_argument("--debug", type=Path, help="folder to save each spin's before/after frames")
    args = parser.parse_args()

    frames, stop = [], threading.Event()

    def grab():
        for image, stamp in camera_frames(stop):
            frames.append((stamp, cv2.cvtColor(image, cv2.COLOR_BGR2GRAY).astype(np.float32)))

    grabber = threading.Thread(target=grab, daemon=True)
    grabber.start()
    port = Serial(DEVICE, 57600, timeout=0.01, exclusive=True)
    wire = mav.MAVLink(port, srcSystem=255, srcComponent=190)
    wire.robust_parsing = True
    rolls, state, lateral = [], {"heartbeat": None, "ch6": None}, {}

    def pump():
        for msg in wire.parse_buffer(port.read(4096)) or ():
            kind = msg.get_type()
            if kind == "BAD_DATA" or msg.get_srcComponent() != 1:
                continue
            if kind == "PARAM_VALUE" and msg.param_id in [f"CA_ROTOR{i}_PY" for i in range(4)]:
                lateral[int(msg.param_id[8]) + 1] = msg.param_value
            elif kind == "HEARTBEAT":
                state["heartbeat"] = msg
            elif kind == "RC_CHANNELS":
                state["ch6"] = msg.chan6_raw
            elif kind == "ATTITUDE":
                rolls.append((time.monotonic(), msg.roll))

    def spin(motors, value, timeout):
        for motor in motors:
            wire.command_long_send(1, 1, mav.MAV_CMD_ACTUATOR_TEST, 0, value, timeout, 0, 0, motor, 0, 0)

    start = time.monotonic()
    try:
        for rotor in range(4):
            wire.param_request_read_send(1, 1, f"CA_ROTOR{rotor}_PY".encode(), -1)
        wire.command_long_send(1, 1, mav.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                               mav.MAVLINK_MSG_ID_RC_CHANNELS, 50000, 0, 0, 0, 0, 0)
        # At least 2 s for the camera and PX4, and up to 3 s for the first radio data.
        while time.monotonic() - start < 2.0 or (state["ch6"] is None and time.monotonic() - start < 3.0):
            pump()
        try:
            spins = spins_from_layout(lateral)
        except ValueError as exc:
            raise SystemExit(f"Could not read PX4's motor layout: {exc}")
        print("PX4's left side: M" + "+M".join(map(str, spins[0][0])) +
              ", right side: M" + "+M".join(map(str, spins[1][0])))
        heartbeat = state["heartbeat"]
        if heartbeat is None or heartbeat.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED:
            raise SystemExit("PX4 is armed or silent. Disarm with CH5 low.")
        if state["ch6"] is None or state["ch6"] > 1500:
            raise SystemExit(f"CH6 (kill) reads {state['ch6']} (none means no radio data). Lower it; "
                             "PX4 blocks motor tests while killed.")
        if len(frames) < 10:
            raise SystemExit("No camera frames. Is the ch8-takeover service still holding the camera?")
        next_send = 0.0
        while (now := time.monotonic() - start) < 11.0:
            pump()
            if state["ch6"] is not None and state["ch6"] > 1500:
                raise SystemExit("CH6 raised: stopped.")
            active = next((motors for motors, at, _ in spins if at <= now < at + SPIN_SECONDS), None)
            if active and now >= next_send:
                spin(active, POWER, 0.4)
                next_send = now + 0.15
    finally:
        for _ in range(3):
            spin((1, 2, 3, 4), 0, 0)
            time.sleep(0.05)
        port.flush()
        port.close()
        stop.set()
        grabber.join(timeout=5)  # close the camera before Python exits

    times, angles = np.array([r[0] for r in rolls]), np.array([r[1] for r in rolls])
    window = cv2.createHanningWindow(SIZE, cv2.CV_32F)
    tilts, shifts = [], []
    for motors, at, sign in spins:
        begin = start + at
        reference = [f for f in frames if f[0] < begin]
        during = [f for f in frames if begin <= f[0] <= begin + SPIN_SECONDS + 0.3]
        if not reference or not during:
            raise SystemExit("Missing camera frames around a spin.")
        before_stamp, before = reference[-1]
        roll_before = np.interp(before_stamp, times, angles)
        change = np.interp(during[-1][0], times, angles) - roll_before
        print(f"M{'+M'.join(map(str, motors))}: PX4 roll change {math.degrees(change):+.1f} deg "
              f"(expected {'+' if sign > 0 else '-'})")
        if sign * change < MIN_ROLL:
            raise SystemExit("PX4's roll moved the wrong way or too little: fix PX4's orientation "
                             "or motor layout first. Nothing saved.")
        spin_tilts, spin_shifts = [], []
        for stamp, image in during:
            spin_tilts.append(math.tan(np.interp(stamp, times, angles)) - math.tan(roll_before))
            spin_shifts.append(picture_shift(before, image, window))
        last = spin_shifts[-1]
        print(f"   {len(during)} frames; picture moved ({last[0]:+.0f}, {last[1]:+.0f}) px by the end, "
              f"{fit_column(spin_tilts, spin_shifts).round(0).tolist()} px per unit tan(roll)")
        if args.debug:
            args.debug.mkdir(parents=True, exist_ok=True)
            name = "M" + "M".join(map(str, motors))
            cv2.imwrite(str(args.debug / f"{name}-before.png"), before.astype(np.uint8))
            cv2.imwrite(str(args.debug / f"{name}-after.png"), during[-1][1].astype(np.uint8))
        tilts += spin_tilts
        shifts += spin_shifts
    column = fit_column(tilts, shifts)
    try:
        mount = mount_from(column, args.nose)
    except ValueError as exc:
        raise SystemExit(f"Measured {column.round(1).tolist()} px per unit tan(roll), but {exc}. Nothing saved.")
    focal = float(np.hypot(*column))
    MOUNT_FILE.parent.mkdir(parents=True, exist_ok=True)
    MOUNT_FILE.write_text(json.dumps({
        "roll": mount[:, 0].round(1).tolist(), "pitch": mount[:, 1].round(1).tolist(),
        "focal_px": round(focal, 1), "lens_hfov_deg": round(math.degrees(2 * math.atan(SIZE[0] / 2 / focal)), 1),
        "nose": args.nose, "measured": datetime.datetime.now().isoformat(timespec="seconds"),
    }, indent=2) + "\n")
    right = "right" if column[0] > 0 else "left"
    print(f"Rolling right slides the picture {right}: {column.round(1).tolist()} px per unit tan(roll).")
    print(f"Focal length {focal:.0f} px, about {math.degrees(2 * math.atan(SIZE[0] / 2 / focal)):.0f} "
          f"degrees across. Saved {MOUNT_FILE}. Restart the service to use it.")


if __name__ == "__main__":
    main()
