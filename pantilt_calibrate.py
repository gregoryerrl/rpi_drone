#!/usr/bin/env python3
# pantilt_calibrate.py — use the camera to work out which PCA9685 channel is pan/tilt
# and which pulse direction moves the view left/right/up/down.
#   /usr/bin/python3 pantilt_calibrate.py [outdir]
# Method: for each channel, step pulse away from center and measure the global image
# shift with phase correlation. Scene moving LEFT in the image => camera turned RIGHT;
# scene moving DOWN in the image => camera tilted UP.
import os, sys, time
import cv2, numpy as np
from picamera2 import Picamera2
from servo_test import PCA9685, CHANNELS, MID_US

OUT = sys.argv[1] if len(sys.argv) > 1 else "calib"
STEPS = (1300, 1400, 1500, 1600, 1700)
os.makedirs(OUT, exist_ok=True)

cam = Picamera2()
cam.configure(cam.create_still_configuration(main={"size": (1280, 720), "format": "RGB888"}))
cam.start(); time.sleep(1.5)
pca = PCA9685()


def grab(name):
    time.sleep(0.8)                      # let servo settle / motion blur clear
    cam.capture_array()                  # drop a possibly stale frame
    img = cam.capture_array()
    cv2.imwrite(os.path.join(OUT, name + ".jpg"), img)
    g = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY).astype(np.float32)
    return cv2.GaussianBlur(g, (5, 5), 0)


win = None
try:
    results = {}
    for ch in CHANNELS:
        for c in CHANNELS:
            pca.set_pulse_us(c, MID_US)
        ref = grab(f"ch{ch}_ref")
        if win is None:
            win = cv2.createHanningWindow(ref.shape[::-1], cv2.CV_32F)
        rows = []
        for us in STEPS:
            pca.set_pulse_us(ch, us)
            (dx, dy), resp = cv2.phaseCorrelate(ref, grab(f"ch{ch}_{us}"), win)
            rows.append((us, dx, dy, resp))
            print(f"ch{ch} {us}us: image shift dx={dx:+7.1f}px dy={dy:+7.1f}px (conf {resp:.2f})",
                  flush=True)
        results[ch] = rows
    for c in CHANNELS:
        pca.set_pulse_us(c, MID_US)

    print("\n=== summary ===")
    for ch, rows in results.items():
        lo, hi = rows[0], rows[-1]
        ddx, ddy = hi[1] - lo[1], hi[2] - lo[2]
        span = hi[0] - lo[0]
        if abs(ddx) >= abs(ddy):
            cam_dir = "RIGHT" if ddx < 0 else "LEFT"
            print(f"ch{ch}: PAN  — increasing pulse turns camera {cam_dir} "
                  f"({abs(ddx)/span:.2f} px/us, cross-talk dy {ddy:+.0f}px)")
        else:
            cam_dir = "UP" if ddy > 0 else "DOWN"
            print(f"ch{ch}: TILT — increasing pulse tilts camera {cam_dir} "
                  f"({abs(ddy)/span:.2f} px/us, cross-talk dx {ddx:+.0f}px)")
finally:
    for c in CHANNELS:
        pca.set_pulse_us(c, MID_US)
    time.sleep(0.5)
    pca.close()
    cam.stop()
