"""Serve the receiver channels reported by a USB flight controller."""
import argparse
from contextlib import contextmanager
from dataclasses import dataclass
import fcntl
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import select
import signal
import sys
import termios
import threading
import time
import tty

from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mav

MOVE_US = 25
HIGHLIGHT_S = 0.8
STALE_S = 1.5
IDLE_S = 5.0


@dataclass
class Channel:
    value: int
    anchor: int
    moved_at: float | None = None


class Inputs:
    def __init__(self):
        self.reset("Waiting for receiver")

    def reset(self, status):
        self.system = None
        self.armed = None
        self.mode = None
        self.heartbeat_at = None
        self.attitude = None
        self.attitude_stamp = None
        self.attitude_at = None
        self.reset_channels(status)

    def reset_channels(self, status):
        self.status = status
        self.channels = {}
        self.latest = []
        self.last_packet = None
        self.unhealthy_at = None

    def update(self, message, now):
        if message.get_srcComponent() != mav.MAV_COMP_ID_AUTOPILOT1:
            return
        if self.system is not None and message.get_srcSystem() != self.system:
            return
        kind = message.get_type()
        if kind == "HEARTBEAT" and message.autopilot != mav.MAV_AUTOPILOT_INVALID:
            self.system = message.get_srcSystem()
            self.armed = bool(message.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED)
            mode = mavutil.mode_string_v10(message)
            self.mode = "Hold" if message.autopilot == mav.MAV_AUTOPILOT_PX4 and mode == "LOITER" else mode
            self.heartbeat_at = now
            return
        if kind == "ATTITUDE" and self.system is not None:
            if message.time_boot_ms != self.attitude_stamp:
                self.attitude_stamp = message.time_boot_ms
                self.attitude_at = now
                self.attitude = None
                if math.isfinite(message.roll) and math.isfinite(message.pitch):
                    self.attitude = {"roll": round(math.degrees(message.roll), 1),
                                     "pitch": round(math.degrees(message.pitch), 1)}
            return
        if kind == "SYS_STATUS":
            flag = mav.MAV_SYS_STATUS_SENSOR_RC_RECEIVER
            monitored = message.onboard_control_sensors_present & message.onboard_control_sensors_enabled
            if monitored & flag and not message.onboard_control_sensors_health & flag:
                self.reset_channels("Receiver signal lost")
                self.unhealthy_at = now
            else:
                self.unhealthy_at = None
            return
        if kind != "RC_CHANNELS" or (self.unhealthy_at is not None and now - self.unhealthy_at < 3):
            return
        if self.last_packet is not None and now - self.last_packet > STALE_S:
            self.reset_channels("Waiting for receiver")
        self.last_packet = now
        channels = {}
        changes = []
        for number in range(1, min(message.chancount, 18) + 1):
            value = getattr(message, f"chan{number}_raw")
            if value in (0, 65535):
                continue
            ch = self.channels.get(number)
            if ch is None:
                ch = Channel(value, value)
            ch.value = value
            # Compare to the last deliberate movement so gradual stick travel accumulates.
            if abs(value - ch.anchor) >= MOVE_US:
                changes.append({"number": number, "before": ch.anchor, "after": value})
                ch.anchor, ch.moved_at = value, now
            channels[number] = ch
        self.channels = channels
        self.latest = changes or [c for c in self.latest if c["number"] in channels]
        self.status = "Receiving" if channels else "Waiting for RC input"

    def snapshot(self, now):
        if self.last_packet is not None and now - self.last_packet > STALE_S:
            self.reset_channels("RC data stopped")
        heartbeat_fresh = self.heartbeat_at is not None and 0 <= now - self.heartbeat_at <= STALE_S
        attitude_fresh = self.attitude_at is not None and 0 <= now - self.attitude_at <= .5
        return {
            "status": self.status,
            "fc_armed": self.armed if heartbeat_fresh else None,
            "fc_mode": self.mode if heartbeat_fresh else None,
            "attitude_deg": self.attitude if heartbeat_fresh and attitude_fresh else None,
            "received_at": self.last_packet,  # Pi monotonic clock, stamped when the FC packet arrived
            "channels": [
                {"number": n, "value": ch.value,
                 "active": ch.moved_at is not None and now - ch.moved_at < HIGHLIGHT_S}
                for n, ch in self.channels.items()
            ],
            "latest": self.latest,
        }


@contextmanager
def listen_port(path):
    fd = os.open(path, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        previous = termios.tcgetattr(fd)
        try:
            tty.setraw(fd, termios.TCSANOW)
            yield fd
        finally:
            try:
                termios.tcsetattr(fd, termios.TCSANOW, previous)
            except (OSError, termios.error):
                pass  # A disconnected USB device has no terminal settings to restore.
    finally:
        os.close(fd)


class Receiver:
    def __init__(self, path):
        self.path = path
        self.inputs = Inputs()
        self.lock = threading.Lock()
        self.wake = threading.Event()
        self.stop = threading.Event()
        self.view_until = 0.0
        self.thread = threading.Thread(target=self._run, name="rc-input", daemon=True)

    def state(self):
        now = time.monotonic()
        with self.lock:
            self.view_until = now + IDLE_S
            self.wake.set()
            return self.inputs.snapshot(now)

    def _receive(self):
        with listen_port(self.path) as fd:
            decoder = mav.MAVLink(None)
            decoder.robust_parsing = True
            with self.lock:
                self.inputs.reset("Waiting for RC input")
            while not self.stop.is_set():
                with self.lock:
                    if time.monotonic() >= self.view_until:
                        return
                if not select.select([fd], [], [], 0.25)[0]:
                    continue
                data = os.read(fd, 8192)
                if not data:
                    return
                messages = decoder.parse_buffer(data) or []
                now = time.monotonic()
                with self.lock:
                    for message in messages:
                        self.inputs.update(message, now)

    def _run(self):
        while not self.stop.is_set():
            self.wake.wait()
            self.wake.clear()
            with self.lock:
                if self.stop.is_set() or time.monotonic() >= self.view_until:
                    continue
            status = "Waiting for flight controller"
            try:
                self._receive()
            except BlockingIOError:
                status = "Serial port in use"
            except PermissionError:
                status = "Serial permission denied"
            except (OSError, termios.error):
                pass
            with self.lock:
                self.inputs.reset(status)
            self.stop.wait(0.5)

    def close(self):
        self.stop.set()
        self.wake.set()
        self.thread.join()


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/":
            body, mime = self.server.page, "text/html; charset=utf-8"
        elif self.path == "/api/inputs":
            state = self.server.receiver.state()
            body = json.dumps(dict(state, labels=self.server.labels, source=self.server.source)).encode()
            mime = "application/json"
        else:
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type", mime)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def log_message(self, *args):
        pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8002)
    parser.add_argument("--device", default="/dev/ttyACM0")
    parser.add_argument("--label", nargs=2, metavar=("CHANNEL", "NAME"), action="append", default=[])
    args = parser.parse_args()
    labels = {}
    for number, name in args.label:
        if not number.isdecimal() or not 1 <= int(number) <= 18:
            parser.error("channel must be between 1 and 18")
        labels[int(number)] = name
    page = Path(__file__).with_name("index.html").read_bytes()
    with ThreadingHTTPServer((args.bind, args.port), Handler) as server:
        server.page, server.labels, server.source = page, labels, Path(args.device).name
        server.receiver = Receiver(args.device)
        server.receiver.thread.start()
        signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
        try:
            server.serve_forever()
        finally:
            server.receiver.close()


if __name__ == "__main__":
    main()
