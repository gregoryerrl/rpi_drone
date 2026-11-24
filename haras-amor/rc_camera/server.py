"""Radio-controlled live pixel tracking on the Pi's two-axis camera stand."""
import argparse
import json
from pathlib import Path
import signal
import sys
from threading import Event, Lock, Thread
import time

import cv2
from flask import Flask, Response, jsonify, send_from_directory
import numpy as np

from .radio import Decoder, Feed
from .research import CENTER, MANUAL_SPEED, MAX_GAP, LiveTrial, PixelLock
from .mount import RadioRig

HERE = Path(__file__).resolve().parent
MAX_MANUAL_RATE = 1800.0  # µs of servo command per second at full stick


def manual_rates(pan, tilt, maximum):
    stick = np.array([pan, tilt], dtype=float)
    fine = min(MANUAL_SPEED, maximum)
    # Shape each axis separately so a small tilt correction stays fine during a fast pan.
    rates = fine * stick + (maximum - fine) * stick**3
    return rates / max(1.0, np.linalg.norm(rates) / maximum)


class CameraControl:
    def __init__(self, trial):
        self.trial = trial
        self.decoder = Decoder()
        self.high = self.cycle_switch = False
        self.manual_pose = None
        self.max_manual_rate = min(MAX_MANUAL_RATE, trial.rig.catchup_speed)

    def unlock(self):
        if self.trial.tracker.state != "Manual":
            self.trial.tracker = PixelLock()
        self.high = False

    def step(self, frame, packet, now):
        trial = self.trial
        controls = self.decoder.read(packet, now)
        dt = frame.stamp - trial.frame.stamp
        fresh = 0 < dt <= MAX_GAP and 0 <= now - frame.stamp <= MAX_GAP
        enabled = controls is not None and fresh and trial.rig.mount_ready
        if not enabled:
            self.cycle_switch |= self.high
            self.unlock()
        elif not controls.follow:
            self.cycle_switch = False
            self.unlock()
        trial.advance(frame, (0, 0), authorized=enabled and not self.cycle_switch)
        if enabled and controls.follow and not self.cycle_switch:
            if not self.high:
                trial.lock(frame.index, trial.size)
                self.high = True  # A lost or textureless target needs a new switch edge.
            self.manual_pose = None
        elif enabled and not controls.follow and (controls.pan or controls.tilt):
            if self.manual_pose is None:
                self.manual_pose = trial.rig.position.astype(float)
            rates = manual_rates(controls.pan, controls.tilt, self.max_manual_rate)
            step = -np.sign(trial.gains) * rates * min(dt, 0.1)
            self.manual_pose = np.clip(self.manual_pose + step, *trial.rig.limits)
            target = np.rint(self.manual_pose)
            if np.any(target != trial.rig.position):
                trial.rig.move(target)
        else:
            self.manual_pose = None
        state = trial.state()
        state.update(pulses=controls.pulses if controls else None,
                     follow_switch=controls.follow if controls else None, mount_ready=trial.rig.mount_ready)
        if not trial.rig.mount_ready:
            self.cycle_switch |= self.high
            self.unlock()
            self.manual_pose = None
            state["mode"] = "Mount offline"
            state["point"] = None
            state["message"] = "Camera live. Waiting for the PCA9685 servo board on I²C."
        elif controls is None:
            state["message"] = "Waiting for fresh right-stick and CH8 inputs."
        elif not fresh or self.cycle_switch:
            state["message"] = "Camera or RC input interrupted. Lower SG before starting another lock."
        elif not controls.follow:
            state["message"] = "Aim with the right stick. Raise SG to lock the crosshair."
        return state


class Preview:
    def __init__(self):
        self.guard = Lock()
        self.view = self.cached = None
        self.error = None

    def publish(self, image, state):
        with self.guard:
            self.view = (image, state)

    def jpeg(self):
        # Sensor frames are immutable. Encode on demand so radio-only use is inexpensive.
        with self.guard:
            if self.view is None or self.error:
                return None
            image, state = self.view
            if self.cached is None or self.cached[1]["frame"] != state["frame"]:
                picture = image.copy()
                x, y = map(int, CENTER)
                r = state["size"] // 2
                cv2.rectangle(picture, (x-r, y-r), (x+r, y+r), (230,230,230), 1)
                cv2.drawMarker(picture, (x,y), (20,20,20), cv2.MARKER_CROSS, 17, 3)
                cv2.drawMarker(picture, (x,y), (255,255,255), cv2.MARKER_CROSS, 17, 1)
                if state["point"] is not None:
                    px, py = map(round, state["point"])
                    color = (90,230,120) if state["mode"] == "Following" else (40,180,255)
                    cv2.rectangle(picture, (px-r,py-r), (px+r,py+r), color, 2)
                ok, data = cv2.imencode(".jpg", picture, [cv2.IMWRITE_JPEG_QUALITY, 78])
                if not ok:
                    raise RuntimeError("JPEG encoding failed")
                self.cached = (data.tobytes(), state)
            return self.cached


def web_app(preview):
    app = Flask(__name__, static_folder=None)

    @app.get("/")
    @app.get("/live")
    def page():
        return send_from_directory(HERE, "index.html", max_age=0)

    @app.get("/api/state")
    def state():
        with preview.guard:
            if preview.error or preview.view is None:
                return jsonify(error=preview.error or "Starting camera"), 503
            return jsonify(preview.view[1])

    @app.get("/frame.jpg")
    def frame():
        result = preview.jpeg()
        if result is None:
            return jsonify(error=preview.error or "Starting camera"), 503
        image, info = result
        return Response(image, mimetype="image/jpeg", headers={
            "Cache-Control":"no-store", "X-Camera":json.dumps(info, separators=(",", ":"))})

    return app


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8001)
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--device", default="/dev/ttyACM0", help="Read-only flight-controller USB port")
    source.add_argument("--rc-url", help="Share an existing RC monitor on this Pi via /api/inputs")
    parser.add_argument("--camera-gains", type=float, nargs=2, default=(0.51,0.46))
    parser.add_argument("--start-pose", type=int, nargs=2, default=(1500,1114))
    args = parser.parse_args()
    if not all(np.isfinite(args.camera_gains)) or 0 in args.camera_gains:
        parser.error("Camera gains must be finite and nonzero.")
    import logging
    logging.getLogger("werkzeug").setLevel(logging.ERROR)
    cv2.setNumThreads(2)
    rig = RadioRig(args.start_pose)
    feed = None
    worker = None
    stopping = Event()
    try:
        controller = CameraControl(LiveTrial(rig, args.camera_gains))
        if args.rc_url:
            feed = Feed(args.rc_url)
        else:
            from rc_monitor.server import Receiver
            feed = Receiver(args.device)
            feed.thread.start()
        preview = Preview()

        def run():
            try:
                trial = controller.trial
                while not stopping.is_set():
                    frame = trial.capture(trial.frame.index + 1)
                    if stopping.is_set():
                        break
                    info = controller.step(frame, feed.state(), time.monotonic())
                    preview.publish(frame.image, info)
            except Exception as exc:
                with preview.guard:
                    preview.error = str(exc)
                print(f"Camera stopped: {exc}", file=sys.stderr, flush=True)

        worker = Thread(target=run, name="rc-pan-tilt", daemon=True)
        worker.start()
        signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
        web_app(preview).run(host=args.bind, port=args.port, threaded=True, use_reloader=False)
    finally:
        stopping.set()
        if worker is not None:
            worker.join()
        if feed is not None:
            feed.close()
        rig.close()


if __name__ == "__main__":
    main()
