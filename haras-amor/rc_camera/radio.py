"""Read the local RC monitor. Channel timestamps use this Pi's monotonic clock."""
from collections import deque
from dataclasses import dataclass
import json
import math
from http.client import HTTPException
from statistics import median
from threading import Event, Thread
from urllib.request import ProxyHandler, build_opener


MAX_RC_AGE = 0.25
SWITCH_HOLD = 0.10
DEADBAND_US = 30
FULL_DEFLECTION_US = 450  # Full right/up on this TX16S reads about 1955–1962 µs.


def axis(pulse):
    delta = pulse - 1500
    magnitude = min(1.0, max(0.0, (abs(delta) - DEADBAND_US) / (FULL_DEFLECTION_US - DEADBAND_US)))
    return math.copysign(magnitude, delta)


@dataclass(frozen=True)
class Controls:
    pan: float
    tilt: float
    follow: bool
    pulses: tuple


class Decoder:
    def __init__(self):
        self.samples = deque(maxlen=3)
        self.clear()

    def clear(self):
        self.samples.clear()
        self.last = self.candidate = self.level = None
        self.since = 0.0

    def read(self, packet, now):
        stamp = packet.get("received_at")
        if (packet.get("status") != "Receiving" or not isinstance(stamp, (int, float))
                or not 0 <= now - stamp <= MAX_RC_AGE):
            self.clear()
            return None
        values = {ch["number"]: ch["value"] for ch in packet.get("channels", [])}
        pulses = tuple(values.get(ch) for ch in (1, 2, 8))
        if any(type(v) is not int or not 800 <= v <= 2200 for v in pulses):
            self.clear()
            return None
        if self.last is not None and not 0 <= stamp - self.last <= MAX_RC_AGE:
            self.clear()
        if stamp != self.last:
            self.last = stamp
            self.samples.append(pulses)
            # Wait for distinct receiver packets. Repeated HTTP reads and isolated
            # one-frame jumps on this receiver must not start a lock or jerk the rig.
            wanted = True if pulses[2] >= 1800 else False if pulses[2] <= 1650 else None
            if wanted != self.candidate:
                self.candidate, self.since = wanted, stamp
            if wanted is not None and stamp - self.since >= SWITCH_HOLD:
                self.level = wanted
        if self.level is None or len(self.samples) < 3:
            return None
        pan, tilt, switch = (int(median(vals)) for vals in zip(*self.samples))
        return Controls(axis(pan), -axis(tilt), self.level, (pan, tilt, switch))


class Feed:
    def __init__(self, url):
        self.url = url
        self.packet = {}
        self.stop = Event()
        self.thread = Thread(target=self._run, name="camera-rc-input", daemon=True)
        self.thread.start()

    def _run(self):
        opener = build_opener(ProxyHandler({}))
        while not self.stop.is_set():
            try:
                with opener.open(self.url, timeout=0.2) as reply:
                    packet = json.load(reply)
                self.packet = packet if isinstance(packet, dict) else {}
            except (OSError, ValueError, HTTPException):
                self.packet = {}
            self.stop.wait(0.035)

    def state(self):
        return self.packet

    def close(self):
        self.stop.set()
        self.thread.join()
