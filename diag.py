#!/usr/bin/env python3
# diag.py — props OFF, battery plugged in
import time
from pymavlink import mavutil

m = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600,
                               source_system=255, source_component=190)

def hb():
    m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                         mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

# introduce the Pi and wait for the flight controller's own heartbeat
fc = None
t0 = time.time()
while time.time() - t0 < 5 and fc is None:
    hb()
    h = m.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
    if h and h.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID:
        fc = h
print("FC heartbeat:", fc)
print("from system", m.target_system, "component", m.target_component)

# motor 1, 10%, 3 seconds
m.mav.command_long_send(m.target_system, 1, 310, 0,
                        0.10, 3, 0, 0, 1, 0, 0)

# print every reply for 4 seconds
t0 = time.time()
while time.time() - t0 < 4:
    hb()
    msg = m.recv_match(type=['COMMAND_ACK', 'STATUSTEXT'],
                       blocking=True, timeout=0.5)
    if msg:
        print(msg)

