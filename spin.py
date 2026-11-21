#!/usr/bin/env python3
# spin.py — props OFF, battery plugged in
import sys, time
from pymavlink import mavutil

motors   = [int(x) for x in sys.argv[1].split(',')]            # 1  or 1,3  or 1,2,3,4
throttle = float(sys.argv[2]) if len(sys.argv) > 2 else 0.10   # 0.0 to 1.0
seconds  = min(float(sys.argv[3]) if len(sys.argv) > 3 else 3, 3)  # max 3
throttle = max(0.0, min(throttle, 0.3))                          # safety cap

m = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600,
                               source_system=255, source_component=190)

def hb():
    m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                         mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

# introduce the Pi, wait for the flight controller
t0 = time.time()
while time.time() - t0 < 5:
    hb()
    h = m.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
    if h and h.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID:
        break
else:
    sys.exit("No heartbeat from flight controller")

for motor in motors:
    # MAV_CMD_ACTUATOR_TEST: param5 = ACTUATOR_OUTPUT_FUNCTION, Motor1 = 1
    m.mav.command_long_send(m.target_system, 1, 310, 0,
                            throttle, seconds, 0, 0, motor, 0, 0)
    ack = m.recv_match(type='COMMAND_ACK', condition='COMMAND_ACK.command==310',
                       blocking=True, timeout=2)
    print(f"motor {motor}: {'OK' if ack and ack.result == 0 else ack}")

# keep the link alive until the motors time out
t0 = time.time()
while time.time() - t0 < seconds + 0.5:
    hb()
    time.sleep(0.5)
