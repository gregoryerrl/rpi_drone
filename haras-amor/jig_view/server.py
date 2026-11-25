"""Haras Amor's CH8-operated camera experiment with passive MAVLink telemetry."""
import argparse
import asyncio
from dataclasses import dataclass
import json
import logging
from pathlib import Path
from threading import Event, Lock, Thread
import time

from aiohttp import web
import cv2
from numpy import ndarray

from rc_camera.radio import MAX_RC_AGE, SWITCH_HOLD
from rc_camera.research import FPS, MAX_GAP, SIZE, PixelLock
from rc_monitor.server import Receiver


HERE = Path(__file__).resolve().parent
DEVICE = "/dev/serial/by-id/usb-MicoAir_MicoAir743v2_0-if00"
PATCH = 40
LOG = logging.getLogger(__name__)


def radio_live(radio, now):
    stamp = radio["received_at"]
    return radio["status"] == "Receiving" and stamp is not None and 0 <= now - stamp <= MAX_RC_AGE


def channel_eight(radio, now):
    if not radio_live(radio, now):
        return None
    return next((ch["value"] for ch in radio["channels"]
                 if ch["number"] == 8 and 800 <= ch["value"] <= 2200), None)


def report(mode, hint, switch=None, box=None):
    return {"mode": mode, "hint": hint, "switch": switch, "box": box}


class RadioLock:
    """A distinct radio switch edge selects the center once for every viewer."""
    def __init__(self):
        self.reset()

    def reset(self):
        self.tracker = None
        self.level = self.candidate = self.last_packet = None
        self.since = 0.0
        self.can_lock = False
        self.last_frame = None

    def step(self, image, stamp, radio, now):
        pulse = channel_eight(radio, now)
        if pulse is None or not 0 <= now - stamp <= MAX_GAP:
            self.reset()
            return report("Waiting", "Waiting for fresh camera and CH8 input.")
        packet = radio["received_at"]
        if ((self.last_frame is not None and not 0 < stamp - self.last_frame <= MAX_GAP)
                or (self.last_packet is not None and not 0 <= packet - self.last_packet <= MAX_RC_AGE)):
            self.reset()
        self.last_frame = stamp
        position = "Low" if pulse < 1300 else "High" if pulse > 1700 else "Middle"
        if self.last_packet != packet:
            self.last_packet = packet
            if self.candidate != position:
                self.candidate, self.since = position, packet
            elif packet - self.since >= SWITCH_HOLD:
                self.level = position
        if self.level is None:
            return report("Waiting", "Reading the CH8 switch position.")
        if self.level != "High":
            self.tracker = None
            self.can_lock = True
            if self.level == "Low":
                return report("Off", "Move CH8 to middle to prepare the lock.", self.level)
            return report("Ready", "Center the object, then switch CH8 high.", self.level)

        height, width = image.shape[:2]
        center = (width / 2, height / 2)
        if self.can_lock:
            self.can_lock = False
            self.tracker = PixelLock()
            self.tracker.start(image, (0, 0), center, PATCH, stamp)
        elif self.tracker is None:
            return report("Switch high", "Move CH8 to middle, then high to select the center.", self.level)
        else:
            self.tracker.update(image, (0, 0), center, stamp)

        mode = self.tracker.state
        box = None
        if mode in {"Following", "Acquiring"}:
            x, y = self.tracker.point
            box = [float(x / width), float(y / height), PATCH / width, PATCH / height]
        hints = {
            "Acquiring": "Keep the object near the center while the lock settles.",
            "Following": "The video follows the object. Move CH8 to middle to release.",
            "Holding": "Visual match uncertain. The video is holding its framing.",
            "Lost": "Object lost. Move CH8 to middle, center the object, then switch high.",
        }
        if mode == "Manual":
            return report("No detail", "Choose a textured spot. Cycle CH8 through middle to try again.", self.level)
        return report(mode, hints[mode], self.level, box)


def camera_frames(stop):
    from picamera2 import Picamera2
    with Picamera2() as camera:
        config = camera.create_video_configuration(main={"size": SIZE, "format": "RGB888"},
                                                 controls={"FrameRate": FPS}, buffer_count=3, queue=False)
        camera.configure(config)
        camera.start()
        while not stop.is_set():
            request = camera.wait(camera.capture_request(wait=False), timeout=2)
            try:
                image = request.make_array("main")
                stamp = request.get_metadata()["SensorTimestamp"] / 1e9
                stamp += time.monotonic() - time.clock_gettime(time.CLOCK_BOOTTIME)
            finally:
                request.release()
            yield image, stamp


@dataclass
class Frame:
    number: int
    image: ndarray
    stamp: float
    tracking: dict


class Jig:
    def __init__(self, receiver, camera=camera_frames):
        self.receiver, self.camera = receiver, camera
        self.lock = Lock()
        self.encode_lock = Lock()
        self.frame = self.cached = None
        self.error = "Opening camera…"
        self.stop = Event()
        self.worker = Thread(target=self.run, name="jig-camera", daemon=True)

    def run(self):
        number = 0
        while not self.stop.is_set():
            tracking = RadioLock()
            try:
                self.receiver.state()
                for image, stamp in self.camera(self.stop):
                    state = tracking.step(image, stamp, self.receiver.state(), time.monotonic())
                    number += 1
                    with self.lock:
                        self.frame = Frame(number, image, stamp, state)
                        self.error = None
            except Exception as exc:
                # libcamera can disappear or be busy; the HTTP and RC feeds stay live.
                with self.lock:
                    if self.error != str(exc):
                        LOG.warning("Camera unavailable: %s", exc)
                    self.frame, self.error = None, str(exc)
                self.stop.wait(1)

    def latest(self):
        with self.lock:
            frame = self.frame
            if frame is not None and 0 <= time.monotonic() - frame.stamp <= MAX_GAP:
                return frame
        return None

    def state(self):
        radio = self.receiver.state()
        frame = self.latest()
        now = time.monotonic()
        if frame is None:
            tracking = report("Waiting", "Waiting for live camera frames.")
        elif channel_eight(radio, now) is None:
            tracking = report("Waiting", "Waiting for fresh CH8 input.")
        else:
            tracking = frame.tracking
        return {"radio": radio, "rc_live": radio_live(radio, now),
                "camera_live": frame is not None, "tracking": tracking}

    def jpeg(self, after):
        # Several browsers share a single encoding of each requested frame.
        with self.encode_lock:
            frame = self.latest()
            if frame is None or frame.number == after:
                return frame, None
            if self.cached is None or self.cached[0] != frame.number:
                ok, data = cv2.imencode(".jpg", frame.image, [cv2.IMWRITE_JPEG_QUALITY, 82])
                if not ok:
                    raise RuntimeError("Could not encode camera frame.")
                self.cached = frame.number, data.tobytes()
            return frame, self.cached[1]


def make_app(jig):
    async def asset(request):
        return web.FileResponse(HERE / request.match_info.get("name", "index.html"))

    async def state(request):
        return web.json_response(jig.state())

    async def frame(request):
        try:
            after = int(request.query.get("after", -1))
        except ValueError:
            raise web.HTTPBadRequest(text="Invalid frame number.")
        current, jpeg = await asyncio.to_thread(jig.jpeg, after)
        if current is None:
            raise web.HTTPServiceUnavailable(text="Waiting for camera")
        if jpeg is None:
            return web.Response(status=204)
        metadata = {"frame": current.number, **current.tracking}
        return web.Response(body=jpeg, content_type="image/jpeg",
                            headers={"X-Jig-Frame": json.dumps(metadata, separators=(",", ":"))})

    async def start(app):
        jig.receiver.thread.start()
        jig.worker.start()

    async def cleanup(app):
        jig.stop.set()
        await asyncio.to_thread(jig.worker.join)
        await asyncio.to_thread(jig.receiver.close)

    async def headers(request, response):
        response.headers.update({
            "Cache-Control": "no-store",
            "X-Content-Type-Options": "nosniff",
            "X-Frame-Options": "DENY",
            "Referrer-Policy": "no-referrer",
            "Content-Security-Policy": "default-src 'self'; img-src 'self' data:; "
                                       "object-src 'none'; frame-ancestors 'none'; base-uri 'none'",
        })

    app = web.Application()
    app.router.add_get("/", asset)
    app.router.add_get(r"/{name:index\.html|view\.js|style\.css|switch-sim\.html|switch-sim\.js}", asset)
    app.router.add_get("/api/state", state)
    app.router.add_get("/frame.jpg", frame)
    app.on_startup.append(start)
    app.on_cleanup.append(cleanup)
    app.on_response_prepare.append(headers)
    return app


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8003)
    parser.add_argument("--device", default=DEVICE, help="Read-only MAVLink serial device")
    args = parser.parse_args()
    cv2.setNumThreads(2)
    web.run_app(make_app(Jig(Receiver(args.device))), host=args.bind, port=args.port, access_log=None)


if __name__ == "__main__":
    main()
