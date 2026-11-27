"""Aircraft camera with a centre box for the CH8 handover in ch8_takeover.py.

CH8 low leaves the camera unlocked. Moving CH8 from low to the middle locks the object inside
the centre box with the pinned pixel-lock tracker. CH8 high keeps the lock, and while the Pi has
the motors, ch8_takeover.py tilts the drone (roll and pitch) to keep the object in the middle.
The camera keeps a short history of the drone's roll and pitch and looks up the attitude at the
middle of each frame's exposure. The exact rotation geometry predicts where the drone's own
turning moves the object, so tilting does not lose the lock. The same geometry turns the object's
place in the picture into the attitude that centres it, at any tilt. A brief uncertain match holds
the last target. A radio or camera interruption unlocks it; lower CH8 and move it to the middle to
lock again. The camera website, with the live view and the camera, lock, CH8 and drone status, is
served on port 8004.
"""
import bisect
from collections import deque
import json
import math
from pathlib import Path
from threading import Event, Lock, Thread
import time

import cv2
import numpy as np

from rc_camera.research import CENTER, MAX_GAP, SIZE, PixelLock

HERE = Path(__file__).resolve().parent
BOX = 64                     # centre box side in image pixels: the tracker's largest patch
FRAME_RATE = 60              # the global-shutter sensor's top rate: least delay and motion blur
LOW, SWITCH_ON = 1250, 1750  # CH8 thresholds shared with ch8_takeover.py
ATTITUDE_AGE = 0.25          # s; older roll and pitch count as unknown (treated as level)
ATTITUDE_HISTORY = 1.0       # s of roll and pitch kept to look up the attitude at each frame
# How the fixed camera's picture slides when the drone tilts: a 2x2 matrix whose columns are the
# picture shift in pixels (x right, y down) per unit tan(roll) and per unit tan(pitch). For the
# pinhole lens the same matrix places any direction above the camera in the picture (see ray).
# calibrate_camera.py measures it on the roll jig and saves it to MOUNT_FILE. Until then the
# camera is assumed to look straight up, nose at the top of the picture, through a ~100 degree
# lens: rolling right slides the scene right and pitching nose-up slides it up.
MOUNT_FILE = Path.home() / ".config" / "rpi_drone" / "haras-amor" / "camera-mount.json"
DEFAULT_FOCAL = (SIZE[0] / 2) / math.tan(math.radians(100 / 2))
DEFAULT_MOUNT = np.array([[DEFAULT_FOCAL, 0.0], [0.0, -DEFAULT_FOCAL]])
MOTION_SMOOTHING = 0.15      # s; smoothing of how fast the object's angle changes
MODES = {"Manual": "Unlocked", "Acquiring": "Locking", "Following": "Locked", "Holding": "Holding",
         "Lost": "Lost"}
MESSAGES = {
    "Unlocked": "Put the object in the box, then move CH8 to the middle to lock it.",
    "Locking": "Keep the object in the box until its frame turns green.",
    "Locked": "Locked. With the Pi in control, CH8 high tilts the drone to keep it in the middle.",
    "Following": "Following. While the Pi has the motors it tilts the drone to keep the object in the middle.",
    "Holding": "Match uncertain. The Pi holds its tilt until the object is found again.",
    "Lost": "Object lost. Lower CH8, then move it to the middle to lock again.",
}


def camera_frames(stop):
    """Aircraft camera frames, each with the monotonic time of the middle of its exposure."""
    from picamera2 import Picamera2
    with Picamera2() as camera:
        camera.configure(camera.create_video_configuration(
            main={"size": SIZE, "format": "RGB888"}, controls={"FrameRate": FRAME_RATE},
            buffer_count=3, queue=False))
        camera.start()
        while not stop.is_set():
            request = camera.wait(camera.capture_request(wait=False), timeout=2)
            try:
                image = request.make_array("main")
                metadata = request.get_metadata()
            finally:
                request.release()
            # libcamera stamps the end of this global-shutter exposure (the start of the readout:
            # the delay to here does not grow with the exposure time) on CLOCK_BOOTTIME; the
            # attitude history and the radio checks use monotonic time.
            stamp = (metadata["SensorTimestamp"] / 1e9 - metadata.get("ExposureTime", 0) / 2e6
                     + time.monotonic() - time.clock_gettime(time.CLOCK_BOOTTIME))
            yield image, stamp


def position(ch8):
    """CH8 as "low", "middle" or "high", or None without fresh radio data."""
    if ch8 is None:
        return None
    return "low" if ch8 < LOW else "middle" if ch8 <= SWITCH_ON else "high"


def load_mount(path=MOUNT_FILE):
    """The calibrated camera mount matrix, or DEFAULT_MOUNT without a usable calibration."""
    try:
        data = json.loads(Path(path).read_text())
        mount = np.array([data["roll"], data["pitch"]], dtype=float).T
        if mount.shape == (2, 2) and np.all(np.isfinite(mount)) and abs(np.linalg.det(mount)) > 1:
            return mount
    except (OSError, ValueError, KeyError, TypeError):
        pass
    return DEFAULT_MOUNT.copy()


def ray(mount, offset):
    """Body-frame unit vector (x forward, y right, z down) toward the point seen at this offset,
    in pixels, from the centre of the picture. The camera looks up the body's -z axis, and the
    mount matrix maps a direction's tangents to pixels, so this is exact at any angle."""
    across, along = np.linalg.solve(mount, np.asarray(offset, float))
    direction = np.array([along, -across, -1.0])
    return direction / np.linalg.norm(direction)


def picture_offset(mount, direction):
    """Where, in pixels from the centre of the picture, a body-frame direction appears, or None
    when it is not above the camera's horizon. The inverse of ray."""
    x, y, z = direction
    if z > -0.05:
        return None
    return mount @ np.array([y / z, x / -z])


def level_from_body(roll, pitch):
    """Rotation from body axes to level axes that keep the drone's heading (yaw left out)."""
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    return np.array([[cp, sp * sr, sp * cr], [0.0, cr, -sr], [-sp, cp * sr, cp * cr]])


def centring_attitude(mount, offset, roll=0.0, pitch=0.0):
    """The (roll, pitch) that puts the point seen at this offset from the centre of the picture,
    taken with the drone at this roll and pitch, in the middle of the picture. Exact at any tilt."""
    x, y, z = level_from_body(roll, pitch) @ ray(mount, offset)
    return math.asin(max(-1.0, min(1.0, y))), math.atan2(-x, -z)


def rotation_shift(mount, offset, before, after):
    """How far, in pixels, the drone turning from attitude `before` to `after` (each roll, pitch)
    moves a still point seen at this offset from the centre of the picture."""
    direction = level_from_body(*after).T @ level_from_body(*before) @ ray(mount, offset)
    moved = picture_offset(mount, direction)
    return np.zeros(2) if moved is None else moved - np.asarray(offset, float)


def attitude_at(history, stamp):
    """Roll and pitch at a monotonic time from (time, roll, pitch) samples, oldest first:
    interpolated between neighbours, else the nearest sample within ATTITUDE_AGE, else None."""
    if not history:
        return None
    i = bisect.bisect_left([sample[0] for sample in history], stamp)
    if 0 < i < len(history):
        (t0, r0, p0), (t1, r1, p1) = history[i - 1], history[i]
        k = (stamp - t0) / (t1 - t0)
        return r0 + k * (r1 - r0), p0 + k * (p1 - p0)
    nearest = history[0] if i == 0 else history[-1]
    return tuple(nearest[1:]) if abs(stamp - nearest[0]) <= ATTITUDE_AGE else None


class FollowCamera:
    def __init__(self, frames=camera_frames, say=print, mount=None):
        self.frames, self.say = frames, say
        if mount is None:
            self.mount = load_mount()
            measured = not np.array_equal(self.mount, DEFAULT_MOUNT)
            say(f"Camera mount: {'measured, from ' + str(MOUNT_FILE) if measured else 'default estimate; run calibrate_camera.py'}")
        else:
            self.mount = np.asarray(mount, float)
        self.guard = Lock()
        self.ch8 = self.drone = None
        self.attitudes = deque()  # (monotonic time, roll, pitch) from the flight controller
        self.origin = np.zeros(2)  # where this picture sits in the tracker's scene-fixed coordinates
        self.last_attitude = None  # (roll, pitch) when the previous frame was taken
        self.last_seen = None     # where the tracked object was in the previous picture
        self.switch = None        # CH8 position at the previous frame
        self.tracker = PixelLock()
        self.notice = None
        self.view = self.cached = None  # (number, image, state, stamp) of the latest frame
        self.followed = None      # (offset, target, motion, stamp) of the locked object
        self.last_target = None   # (target, stamp) from the previous followed frame
        self.rate = (0.0, 0.0)
        self.stamps = deque(maxlen=FRAME_RATE)
        self.mode = "Unlocked"
        self.number = 0
        self.error = "Opening camera…"
        self.stop = Event()
        self.worker = Thread(target=self.run, name="follow-camera", daemon=True)

    def start(self):
        self.worker.start()

    def close(self):
        self.stop.set()
        self.worker.join(timeout=3)

    def radio(self, ch8):
        """The latest CH8 pulse, or None while the radio data is stale."""
        with self.guard:
            self.ch8 = ch8

    def vehicle(self, drone):
        """Flight-controller status from ch8_takeover.py for the website, or None without it."""
        with self.guard:
            self.drone = drone

    def attitude(self, roll, pitch, stamp=None):
        """The drone's roll and pitch in radians, measured at this monotonic time (now by default),
        or None when unknown."""
        with self.guard:
            if roll is None:
                self.attitudes.clear()
                return
            stamp = time.monotonic() if stamp is None else stamp
            if self.attitudes and stamp <= self.attitudes[-1][0]:
                if stamp == self.attitudes[-1][0]:
                    self.attitudes[-1] = (stamp, roll, pitch)
                return
            self.attitudes.append((stamp, roll, pitch))
            while stamp - self.attitudes[0][0] > ATTITUDE_HISTORY:
                self.attitudes.popleft()

    def _fresh(self):
        with self.guard:
            followed = self.followed
        if followed is None or time.monotonic() - followed[3] > MAX_GAP:
            return None
        return followed

    def offset(self):
        """The locked object's (x, y) offset from the centre of the picture, each -1 at the left
        or top edge to 1 at the right or bottom edge, or None."""
        followed = self._fresh()
        return None if followed is None else followed[0]

    def target(self):
        """The (roll, pitch) in radians that would put the locked object in the centre, or None."""
        followed = self._fresh()
        return None if followed is None else followed[1]

    def motion(self):
        """How fast that centring (roll, pitch) is changing, in rad/s, smoothed, or None."""
        followed = self._fresh()
        return None if followed is None else followed[2]

    def _motion(self, target, stamp):
        if self.last_target is None or stamp <= self.last_target[1]:
            self.rate = (0.0, 0.0)
        else:
            dt = stamp - self.last_target[1]
            k = min(1.0, dt / MOTION_SMOOTHING)
            self.rate = tuple(rate + k * ((now - before) / dt - rate)
                              for rate, now, before in zip(self.rate, target, self.last_target[0]))
        self.last_target = (target, stamp)
        return self.rate

    def _live(self):
        """The latest state with its frame age and the drone status. Call with the guard held."""
        if self.error or self.view is None:
            state = dict(mode="Camera off", message=self.error or "Opening camera…", ch8=self.ch8,
                         point=None, offset=None, box=BOX, fps=None, size=list(SIZE), age_ms=None)
        else:
            state = dict(self.view[2], age_ms=round((time.monotonic() - self.view[3]) * 1000))
        state["drone"] = self.drone
        return state

    def state(self):
        with self.guard:
            return self._live()

    def summary(self):
        state = self.state()
        offset = state["offset"]
        return state["mode"] + ("" if offset is None else " {:+.2f},{:+.2f}".format(*offset))

    def run(self):
        while not self.stop.is_set():
            try:
                previous = None
                for image, stamp in self.frames(self.stop):
                    self.process(image, stamp, previous)
                    previous = stamp
            except Exception as exc:  # libcamera can be busy or disappear; keep retrying
                self.unlock(None)
                self.stamps.clear()
                with self.guard:
                    self.error, self.view, self.followed = f"Camera unavailable: {exc}", None, None
                self.announce("Camera off")
                self.stop.wait(2)

    def unlock(self, notice):
        if self.tracker.state != "Manual":
            self.tracker = PixelLock()
        self.notice = notice
        self.origin = np.zeros(2)  # only meaningful while locked

    def process(self, image, stamp, previous):
        with self.guard:
            ch8, history = self.ch8, list(self.attitudes)
        attitude = attitude_at(history, stamp)  # when this frame was taken, not now
        # Tracker coordinates stay fixed to the scene: move each picture's origin by how far the
        # drone's own turning since the previous frame moved the tracked point, so tilting does
        # not look like the object moving.
        if attitude is not None and self.last_attitude is not None:
            spot = CENTER if self.last_seen is None else self.last_seen
            self.origin = self.origin - rotation_shift(self.mount, spot - CENTER, self.last_attitude, attitude)
        self.last_attitude = attitude
        origin = self.origin
        switch = position(ch8)
        if switch is None:
            self.unlock("No fresh radio data. Lower CH8, then move it to the middle to lock.")
        elif previous is not None and stamp - previous > MAX_GAP:
            self.unlock("The camera stalled. Lower CH8, then move it to the middle to lock again.")
        elif switch == "low":
            self.unlock(None)
        elif self.switch == "low" and self.tracker.state == "Manual":
            # CH8 has just left low: lock whatever is inside the centre box.
            if not self.tracker.start(image, origin, origin + CENTER, BOX, stamp):
                self.notice = f"{self.tracker.message} Lower CH8 to try again."
        elif self.tracker.state not in ("Manual", "Lost"):
            self.tracker.update(image, origin, origin + CENTER, stamp)
        self.switch = switch
        tracker = self.tracker
        mode = MODES[tracker.state]
        if mode == "Locked" and switch == "high":
            mode = "Following"
        followed = tracker.state == "Following"
        shown = tracker.point is not None and tracker.state not in ("Manual", "Lost")
        seen = tracker.point - origin if shown else None  # where it is in this picture
        self.last_seen = seen
        offset = target = motion = None
        held = None
        if tracker.state == "Holding" and self.followed is not None:
            # An uncertain match keeps the last target, without extrapolating its motion.
            held = (*self.followed[:2], (0.0, 0.0), self.followed[3])
        if tracker.state not in ("Following", "Holding"):
            self.last_target = None
        if followed:
            offset = tuple(round(float(v), 3) for v in (seen - CENTER) / CENTER)
            # Where it is in this picture, with the attitude when the picture was taken, gives
            # the attitude that centres it.
            target = centring_attitude(self.mount, seen - CENTER, *(attitude or (0.0, 0.0)))
            motion = self._motion(target, stamp)
        message = self.notice or MESSAGES[mode]
        if mode == "Unlocked" and switch in ("middle", "high") and not self.notice:
            message = "Lower CH8, then move it to the middle to lock the object in the box."
        self.stamps.append(stamp)
        span = self.stamps[-1] - self.stamps[0]
        state = dict(mode=mode, message=message, ch8=ch8,
                     point=seen.tolist() if shown else None, offset=offset and list(offset), box=BOX,
                     fps=round((len(self.stamps) - 1) / span, 1) if span > 0 else None,
                     size=list(SIZE))
        with self.guard:
            self.number += 1
            self.error = None
            self.view = (self.number, image, state, stamp)
            self.followed = (offset, target, motion, stamp) if followed else held
        self.announce(mode)

    def announce(self, mode):
        if mode != self.mode:
            self.mode = mode
            self.say(f"Camera: {mode}")

    def jpeg(self):
        """The latest frame with the centre box and the tracked box drawn on it, and its state."""
        with self.guard:
            if self.view is None:
                return None
            number, image, state, _ = self.view
            if self.cached is None or self.cached[0] != number:
                picture = image.copy()
                x, y, r = int(CENTER[0]), int(CENTER[1]), BOX // 2
                cv2.rectangle(picture, (x - r, y - r), (x + r, y + r), (235, 235, 235), 1)
                if state["point"] is not None:
                    px, py = (round(v) for v in state["point"])
                    green = state["mode"] in ("Locked", "Following")
                    cv2.rectangle(picture, (px - r, py - r), (px + r, py + r),
                                  (90, 230, 120) if green else (40, 180, 255), 2)
                ok, data = cv2.imencode(".jpg", picture, [cv2.IMWRITE_JPEG_QUALITY, 78])
                if not ok:
                    raise RuntimeError("JPEG encoding failed")
                self.cached = (number, data.tobytes())
            return self.cached[1], self._live()


def app(camera):
    from flask import Flask, Response, jsonify, send_from_directory
    web = Flask(__name__, static_folder=None)

    @web.get("/")
    def page():
        return send_from_directory(HERE, "index.html", max_age=0)

    @web.get("/api/state")
    def state():
        return jsonify(camera.state())

    @web.get("/frame.jpg")
    def frame():
        result = camera.jpeg()
        if result is None:
            return jsonify(error=camera.state()["message"]), 503
        data, info = result
        return Response(data, mimetype="image/jpeg", headers={
            "Cache-Control": "no-store", "X-Camera": json.dumps(info, separators=(",", ":"))})

    return web


def serve(camera, host="0.0.0.0", port=8004):
    """Serve the read-only camera website from a background thread. Returns the server."""
    import logging
    from werkzeug.serving import make_server
    logging.getLogger("werkzeug").setLevel(logging.ERROR)
    server = make_server(host, port, app(camera), threaded=True)
    Thread(target=server.serve_forever, name="follow-camera-web", daemon=True).start()
    return server
