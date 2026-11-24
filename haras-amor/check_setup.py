#!/usr/bin/env python3
"""Read-only software and device inventory. No frames captured or actuators commanded."""
import os
from pathlib import Path

from pymavlink import mavutil
from serial import Serial


def main():
    print(f"Python drivers available: {mavutil.__name__}, {Serial.__name__}")
    device = Path("/dev/ttyACM0")
    if not device.exists():
        print(f"{device}: absent")
    elif os.access(device, os.R_OK | os.W_OK):
        print(f"{device}: available to current user")
    else:
        print(f"{device}: present, check user permissions")


if __name__ == "__main__":
    main()
