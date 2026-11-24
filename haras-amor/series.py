#!/usr/bin/env python3
# series.py — PROPS OFF. Full motor test series on PX4 (MAV_CMD_ACTUATOR_TEST).
# Usage: venv/bin/python series.py [gap_seconds]   (default 1.5, 0 = no pauses)
import sys, time
from pymavlink import mavutil


def main():
    ON = 2.0      # seconds each step runs (PX4 max 3)
    GAP = float(sys.argv[1]) if len(sys.argv) > 1 else 1.5   # pause between steps

    STEPS = (
        [("motor %d alone, 10%%" % i, [i], 0.10) for i in (1, 2, 3, 4)] +
        [("motor %d alone, 30%%" % i, [i], 0.30) for i in (1, 2, 3, 4)] +
        [("all 4, %d%%" % int(t * 100), [1, 2, 3, 4], t) for t in (0.10, 0.30, 0.50, 0.75, 1.00)]
    )

    m = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600,
                                   source_system=255, source_component=190)

    def hb():
        m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                             mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    t0 = time.time()
    while time.time() - t0 < 5:
        hb()
        h = m.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if h and h.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID:
            break
    else:
        sys.exit("No heartbeat from flight controller")

    def run(motors, throttle):
        pending = set(motors)
        for attempt in range(2):                     # one resend if an ACK is lost
            for mot in pending:
                m.mav.command_long_send(m.target_system, 1, 310, 0,
                                        throttle, ON, 0, 0, mot, 0, 0)
            t = time.time()
            acked = 0
            while time.time() - t < 1 and acked < len(pending):
                a = m.recv_match(type='COMMAND_ACK', condition='COMMAND_ACK.command==310',
                                 blocking=True, timeout=0.3)
                if a:
                    if a.result != 0:
                        return f"REJECTED (result {a.result})"
                    acked += 1
            if acked >= len(pending):
                return "OK"
        return f"only {acked}/{len(pending)} acked"

    for n, (name, motors, thr) in enumerate(STEPS, 1):
        res = run(motors, thr)
        print(f"[{n:2}/{len(STEPS)}] {name:22} {res}", flush=True)
        t = time.time()
        while time.time() - t < ON + GAP:
            hb()
            time.sleep(0.5)
    print("series done")


if __name__ == "__main__":
    main()
