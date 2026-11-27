"""Live Haras Amor jig telemetry with an optional CH5 motor bench override."""
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

from jig_view.ch5 import Ch5Receiver
from motor_preview.stop import stop_marker
from rc_monitor.server import Receiver, SWITCH_TRACE_GAP_S


HERE = Path(__file__).resolve().parent
DEVICE = "/dev/serial/by-id/usb-MicoAir_MicoAir743v2_0-if00"
FRAME_SIZE = (640, 360)
FRAME_RATE = 24
MAX_FRAME_AGE_S = 0.45
LOG = logging.getLogger(__name__)


def radio_live(radio, now):
    stamp = radio["received_at"]
    return radio["status"] == "Receiving" and stamp is not None and 0 <= now - stamp <= SWITCH_TRACE_GAP_S


def camera_frames(stop):
    from picamera2 import Picamera2
    with Picamera2() as camera:
        config = camera.create_video_configuration(main={"size": FRAME_SIZE, "format": "RGB888"},
                                                 controls={"FrameRate": FRAME_RATE}, buffer_count=3, queue=False)
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
            try:
                for image, stamp in self.camera(self.stop):
                    number += 1
                    with self.lock:
                        self.frame = Frame(number, image, stamp)
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
            if frame is not None and 0 <= time.monotonic() - frame.stamp <= MAX_FRAME_AGE_S:
                return frame
        return None

    def state(self):
        radio = self.receiver.state()
        return {"radio": radio, "rc_live": radio_live(radio, time.monotonic()),
                "camera_live": self.latest() is not None}

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
        metadata = {"frame": current.number}
        return web.Response(body=jpeg, content_type="image/jpeg",
                            headers={"X-Jig-Frame": json.dumps(metadata, separators=(",", ":"))})

    async def start(app):
        jig.receiver.thread.start()
        jig.worker.start()

    async def cleanup(app):
        jig.stop.set()
        jig.receiver.stop.set()
        await asyncio.to_thread(jig.receiver.close)
        await asyncio.to_thread(jig.worker.join)

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
    parser.add_argument("--device", default=DEVICE, help="MAVLink serial device")
    parser.add_argument("--ch5-override", action="store_true",
                        help="Enable the CH8 15%% motor test while physical CH5 is high and PX4 is disarmed")
    args = parser.parse_args()
    cv2.setNumThreads(2)
    receiver = Ch5Receiver(args.device, stop_marker()) if args.ch5_override else Receiver(args.device, continuous=True)
    web.run_app(make_app(Jig(receiver)), host=args.bind, port=args.port, access_log=None)


if __name__ == "__main__":
    main()
