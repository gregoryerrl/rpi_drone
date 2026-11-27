"""Disarmed PX4 motor test triggered by CH8 while CH5 is enabled."""
import asyncio
import time

from pymavlink.dialects.v20 import common as mav

from motor_preview.motors import (BenchError, HEARTBEAT_AGE, MotorLink, MotorStopped,
                                  TIMED_BENCH_PERCENT, TIMED_BENCH_SECONDS)
from rc_monitor.server import Receiver, SWITCH_TRACE_GAP_S, switch_band


PERCENT = TIMED_BENCH_PERCENT
REFRESH_S = 0.10
RUN_SECONDS = TIMED_BENCH_SECONDS


class Ch5Receiver(Receiver):
    def __init__(self, path, stop_marker):
        super().__init__(path, continuous=True)
        self.marker = stop_marker
        self.report = (False, "Waiting for the flight controller.")

    def state(self):
        with self.lock:
            radio = self.inputs.snapshot(time.monotonic())
            active, detail = self.report
            radio["ch5_override"] = {"percent": PERCENT if active else 0, "detail": detail}
            return radio

    def _blocked(self, link, now):
        if link.fault or link.system is None or now - link.last_heartbeat > HEARTBEAT_AGE:
            return "Waiting for a fresh PX4 heartbeat."
        if link.jig.armed:
            return "PX4 is armed. Bench motor control requires a disarmed controller."
        if self.rc_lost or self.rc_at is None or not 0 <= now - self.rc_at <= SWITCH_TRACE_GAP_S:
            return "Waiting for fresh receiver input."
        if self.switches is None:
            return "Waiting for valid CH5, CH6 and CH8 pulses."
        if self.switches[1] != "lower":
            return "CH6 stop. Clear CH6 and cycle CH8 to start again."
        try:
            self.marker.stat()
        except FileNotFoundError:
            return None
        except OSError:
            return "The Pi could not read the motor-stop state."
        return "Motor stop is latched. Reset it in the SSH motor preview."

    def _request(self, link, wanted, detail):
        if self.wanted != wanted:
            self.changed.set()
            if not wanted:
                self.release_due = True
                self.applied = False
                # An ACK can be outstanding for any motor in the batch. Inhibit
                # the writer and release even that motor before reopening USB.
                if self.sending:
                    link.stop_outputs()
        self.wanted = wanted
        self.report = (self.applied, detail)

    def _reset_switches(self):
        self.enabled_at = self.run_until = None
        self.trigger_ready = False

    def _evaluate(self, link, now, *, new_sample=False):
        reason = self._blocked(link, now)
        if reason or self.switches[0] != "high":
            self._reset_switches()
            self._request(link, False, reason or "Raise CH5 with CH8 low or middle to enable the test.")
            return
        if self.enabled_at is None:
            self.enabled_at = now
        ch8 = self.switches[2]
        if ch8 != "high":
            self.run_until = None
            if ch8 == "lower" and new_sample:
                self.trigger_ready = True
        # CH5 may also be mapped to a PX4 switch. Confirm the actual arm state
        # after enabling before acting on a CH8 edge.
        if link.last_heartbeat < self.enabled_at:
            self._request(link, False, "CH5 is on. Waiting for PX4 to confirm disarmed.")
            return
        if ch8 == "high" and self.trigger_ready:
            self.trigger_ready = False
            self.run_until = now + RUN_SECONDS
        if self.run_until is None:
            detail = ("CH5 enabled. Raise CH8 to start." if self.trigger_ready
                      else "CH5 enabled. Move CH8 low or middle, then high to start.")
            self._request(link, False, detail)
        elif now >= self.run_until:
            self._request(link, False, f"{RUN_SECONDS:g}-second cutoff reached. Cycle CH8 for another run.")
        else:
            detail = "PX4 accepted 15% on M1–M4." if self.applied else "Requesting 15% on M1–M4."
            self._request(link, True, f"{detail} {self.run_until - now:.1f} s to cutoff.")

    def observe(self, link, msg, now):
        # MotorLink has already selected the PX4 system on this serial port.
        with self.lock:
            self.inputs.update(msg, now)
            if msg.get_srcComponent() != mav.MAV_COMP_ID_AUTOPILOT1:
                return
            kind = msg.get_type()
            if kind == "SYS_STATUS":
                self.rc_lost = self.inputs.unhealthy_at is not None
            elif kind == "RC_CHANNELS":
                stamp = msg.time_boot_ms
                elapsed_ms = (stamp - self.rc_stamp) % (1 << 32) if self.rc_stamp is not None else None
                advancing = elapsed_ms is None or 0 < elapsed_ms < (1 << 31)
                if self.rc_at is None or now - self.rc_at > SWITCH_TRACE_GAP_S or not advancing:
                    self._reset_switches()
                    self._request(link, False, "Receiver continuity lost.")
                self.rc_stamp = stamp
                self.rc_at = now if advancing and msg.rssi != 0 else None
                pulses = (msg.chan5_raw, msg.chan6_raw, msg.chan8_raw)
                self.switches = (tuple(map(switch_band, pulses)) if msg.chancount >= 8
                                 and all(800 <= value <= 2200 for value in pulses) else None)
            elif kind != "HEARTBEAT":
                return
            self._evaluate(link, now, new_sample=kind == "RC_CHANNELS")

    def pulse_duration(self, link, fraction, duration):
        # Recheck at each actual write, including after a delayed motor ACK.
        now = time.monotonic()
        reason = self._blocked(link, now)
        if self.stop.is_set() or not self.wanted or reason or fraction != PERCENT / 100:
            raise MotorStopped(reason or "CH8 motor test released.")
        if self.run_until is None or now >= self.run_until:
            raise MotorStopped("CH8 motor test complete.")
        return min(duration, self.run_until - now)

    async def _control(self):
        self.changed = asyncio.Event()
        self._reset_switches()
        self.rc_stamp = self.rc_at = self.switches = None
        self.rc_lost = self.wanted = False
        self.release_due = self.applied = self.sending = False
        with self.lock:
            self.inputs.reset("Waiting for RC input")
        link = MotorLink(self.path, radio_override=self)
        try:
            await link.ready()
            next_send = 0.0
            while not self.stop.is_set():
                self.changed.clear()
                link.check()
                if link.outputs_stopped:
                    break
                now = time.monotonic()
                self._evaluate(link, now)
                if self.release_due:
                    self.release_due = False
                    await link.set_percent(0)
                    next_send = 0.0
                    continue
                if self.wanted and now >= next_send:
                    self.sending = True
                    try:
                        await link.set_percent(PERCENT)
                    finally:
                        self.sending = False
                    self.applied = self.wanted and not link.outputs_stopped
                    next_send = time.monotonic() + REFRESH_S
                    continue
                try:
                    await asyncio.wait_for(self.changed.wait(), .04)
                except TimeoutError:
                    pass
        except BenchError as exc:
            if not link.outputs_stopped:
                self.report = (False, str(exc))
        finally:
            try:
                link.close()
            finally:
                active, detail = self.report
                self.report = (False, "CH8 motor test stopped." if active or self.stop.is_set() else detail)

    def _receive(self):
        asyncio.run(self._control())
