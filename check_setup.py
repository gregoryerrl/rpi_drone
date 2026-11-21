#!/usr/bin/env python3
"""Read-only software and device inventory. No frames captured or actuators commanded."""
import os
from pathlib import Path

import cv2
import numpy as np
from picamera2 import Picamera2
from pymavlink import mavutil
from serial import Serial
from smbus2 import SMBus

import face_id


def main():
    # Loading and running the actual weights catches missing files and incompatible OpenCV builds.
    image = np.zeros((face_id.SIZE[1], face_id.SIZE[0], 3), np.uint8)
    face_id.faces_in(image)
    features = face_id.recognizer().feature(np.zeros((112, 112, 3), np.uint8))
    if features.shape != (1, 128) or not np.isfinite(features).all():
        raise RuntimeError("Unexpected recognition model output")
    print(f"OpenCV {cv2.__version__}: both models ran on synthetic inputs.")
    print(f"Python drivers available: {mavutil.__name__}, {Serial.__name__}, {SMBus.__name__}")
    cameras = Picamera2.global_camera_info()
    print("Cameras:", ", ".join(str(camera["Model"]) for camera in cameras) or "none detected")
    for device in (Path("/dev/i2c-1"), Path("/dev/ttyACM0")):
        if not device.exists():
            print(f"{device}: absent")
        elif os.access(device, os.R_OK | os.W_OK):
            print(f"{device}: available to current user")
        else:
            print(f"{device}: present, check user permissions")


if __name__ == "__main__":
    main()
