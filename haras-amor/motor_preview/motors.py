"""Momentary collective motor tests on the disarmed Haras Amor flight controller."""
import asyncio
from collections import deque
import struct
import time

from pymavlink.dialects.v20 import common as mav
from serial import Serial

from .jig import JIG_PARAMS, NativeJig
from .leveling import GEOMETRY_PARAMS, LevelingPreview


DEFAULT_DEVICE = "/dev/serial/by-id/usb-MicoAir_MicoAir743v2_0-if00"
MAX_PERCENT = 50
MOTORS = (1, 2, 3, 4)
TIMED_BENCH_PERCENT = 15
TIMED_BENCH_SECONDS = 20
PULSE_SECONDS = 0.6
BATCH_SECONDS = 0.24
HEARTBEAT_AGE = 1.6


class BenchError(Exception):
    """A condition that ends the current operator session."""


class MotorStopped(BenchError):
    """This USB session can supply telemetry but cannot energize a motor."""


class MotorLink:
    def __init__(self, device, *, radio_override=None):
        self.loop = asyncio.get_running_loop()
        self.port = Serial(device, 57600, timeout=0, write_timeout=0.03, exclusive=True)
        try:
            self.port.reset_input_buffer()
            self.wire = mav.MAVLink(self.port, srcSystem=255, srcComponent=190)
            self.wire.robust_parsing = True
            self.system = None
            self.radio_override = radio_override
            self.last_heartbeat = 0.0
            self.seen = asyncio.Event()
            self.fault = None
            self.waiting = None
            self.command = None
            self.active = set()
            self.bench_cutoff = None
            self.outputs_stopped = False
            self.leveling = LevelingPreview()
            self.jig = NativeJig(self)
            self.geometry_queue = deque()
            self.next_geometry_read = float("inf")
            self.loop.add_reader(self.port.fileno(), self._read)
        except BaseException:
            self.port.close()
            raise

    def _fail(self, reason):
        self.fault = reason
        self.seen.set()
        if self.waiting is not None and not self.waiting.done():
            self.waiting.set_result(None)

    def _read(self):
        try:
            data = self.port.read(16384)
            for msg in self.wire.parse_buffer(data) or ():
                if self.system is not None and msg.get_srcSystem() != self.system:
                    continue
                now = time.monotonic()
                if msg.get_srcComponent() == mav.MAV_COMP_ID_AUTOPILOT1:
                    self._observe_fc(msg, now)
                if self.radio_override is not None and self.system is not None and not self.fault:
                    self.radio_override.observe(self, msg, now)
            self._read_missing_geometry(time.monotonic())
        except (OSError, mav.MAVError) as exc:
            self.loop.remove_reader(self.port.fileno())
            self._fail(f"Flight controller disconnected: {exc}")

    def _observe_fc(self, msg, now):
        kind = msg.get_type()
        if kind == "HEARTBEAT" and (msg.autopilot != mav.MAV_AUTOPILOT_PX4 or msg.type != mav.MAV_TYPE_QUADROTOR):
            self._fail("Expected a PX4 quadrotor on this USB port.")
            return
        if self.system is not None or kind == "HEARTBEAT":
            self.jig.observe(msg, now)
        if kind == "HEARTBEAT":
            if self.jig.armed and self.radio_override is None and not (self.jig.selected or self.jig.stop_pending):
                self._fail("Flight controller armed. Bench control stopped.")
            else:
                self.system = msg.get_srcSystem()
                self.last_heartbeat = now
                self.seen.set()
        elif (kind == "COMMAND_ACK" and self.waiting is not None and not self.waiting.done()
              and msg.command == self.command and msg.target_system in (0, 255) and msg.target_component in (0, 190)):
            self.waiting.set_result(msg.result)
        elif self.system is not None and kind == "ATTITUDE":
            self.leveling.observe(msg.time_boot_ms, msg.roll, msg.pitch, msg.rollspeed, msg.pitchspeed, now)
        elif self.system is not None and kind == "PARAM_VALUE":
            self._geometry_value(msg)

    def _geometry_value(self, msg):
        name = msg.param_id
        if name not in GEOMETRY_PARAMS and name not in JIG_PARAMS:
            return
        integer = name in ("CA_ROTOR_COUNT", "COM_RC_IN_MODE")
        if integer and msg.param_type == mav.MAV_PARAM_TYPE_INT32:
            # PX4 encodes this integer byte-for-byte in the MAVLink float field.
            value, = struct.unpack("<i", struct.pack("<f", msg.param_value))
        elif not integer and msg.param_type == mav.MAV_PARAM_TYPE_REAL32:
            value = msg.param_value
        else:
            self.leveling.parameters.pop(name, None)
            return
        self.leveling.parameters[name] = value

    def _read_missing_geometry(self, now):
        if now < self.next_geometry_read or self.active or self.jig.armed or self.fault or self.system is None:
            return
        if not self.geometry_queue:
            self.geometry_queue.extend(n for n in (*JIG_PARAMS, *GEOMETRY_PARAMS) if n not in self.leveling.parameters)
        if self.geometry_queue:
            name = self.geometry_queue.popleft()
            self.wire.param_request_read_send(self.system, 1, name.encode("ascii"), -1)
        self.next_geometry_read = now + (0.06 if self.geometry_queue else 4)

    def check(self):
        if self.fault:
            raise BenchError(self.fault)
        if self.system is None or time.monotonic() - self.last_heartbeat > HEARTBEAT_AGE:
            raise BenchError("Flight controller heartbeat lost.")

    def _send(self, command, *params):
        if (command == mav.MAV_CMD_ACTUATOR_TEST and params[:2] != (0, 0)
                and (self.outputs_stopped or self.jig.selected or self.jig.stop_pending or self.jig.armed)):
            raise MotorStopped("Bench motor stop latched.")
        if command == mav.MAV_CMD_ACTUATOR_TEST and params[1] > 0 and self.radio_override is not None:
            duration = self.radio_override.pulse_duration(self, params[0], params[1])
            params = (params[0], duration, *params[2:])
        if (command == mav.MAV_CMD_COMPONENT_ARM_DISARM and params[0]
                and (self.outputs_stopped or not self.jig.selected or not self.jig.confirmed)):
            raise MotorStopped("Jig arming blocked.")
        self.wire.command_long_send(self.system, 1, command, 0, *params)

    async def _confirmed(self, timeout, command, *params):
        self.check()
        if timeout <= 0:
            raise BenchError("Motor command timed out.")
        self.command = command
        self.waiting = self.loop.create_future()
        try:
            self._send(command, *params)
            result = await asyncio.wait_for(self.waiting, timeout)
            self.check()
            if result != mav.MAV_RESULT_ACCEPTED:
                raise BenchError(f"Flight controller rejected the command (result {result}).")
        except TimeoutError as exc:
            raise BenchError("Flight controller did not acknowledge in time.") from exc
        finally:
            self.waiting = None

    async def ready(self):
        try:
            await asyncio.wait_for(self.seen.wait(), 3)
        except TimeoutError as exc:
            raise BenchError("No flight controller heartbeat on USB.") from exc
        self.check()
        self.wire.heartbeat_send(mav.MAV_TYPE_GCS, mav.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        await self._confirmed(1.5, mav.MAV_CMD_REQUEST_MESSAGE,
                              mav.MAVLINK_MSG_ID_AUTOPILOT_VERSION, 0, 0, 0, 0, 0, 0)
        if self.radio_override is None:
            self.next_geometry_read = time.monotonic()

    def _cancel_cutoff(self):
        if self.bench_cutoff is not None:
            self.bench_cutoff.cancel()
            self.bench_cutoff = None

    def _expire_timed_run(self):
        self.outputs_stopped = True
        self._fail(f"{TIMED_BENCH_SECONDS:g}-second bench run complete. All motor tests stopped.")
        self._release_active()

    def _bench_pulse(self, percent):
        if percent == TIMED_BENCH_PERCENT and self.bench_cutoff is None:
            self.bench_cutoff = self.loop.call_later(TIMED_BENCH_SECONDS, self._expire_timed_run)
        if self.bench_cutoff is None:
            return PULSE_SECONDS
        # PX4 applies motor-test timeouts in whole milliseconds. Round down so
        # the last renewal cannot request another full pulse past the cutoff.
        remaining_ms = int((self.bench_cutoff.when() - self.loop.time()) * 1000)
        if remaining_ms <= 0:
            self._expire_timed_run()
            raise MotorStopped(self.fault)
        return min(PULSE_SECONDS, remaining_ms / 1000)

    async def set_percent(self, percent):
        if type(percent) is not int or not 0 <= percent <= MAX_PERCENT:
            raise BenchError("Motor level outside the bench range.")
        self.check()
        deadline = time.monotonic() + BATCH_SECONDS
        motors = MOTORS if percent else tuple(sorted(self.active))
        for motor in motors:
            if percent and self.outputs_stopped:
                raise MotorStopped("Bench motor stop latched.")
            pulse = self._bench_pulse(percent) if percent else 0
            self.active.add(motor)
            await self._confirmed(deadline - time.monotonic(), mav.MAV_CMD_ACTUATOR_TEST,
                                  percent / 100, pulse, 0, 0, motor, 0, 0)
            if not percent:
                self.active.discard(motor)
        if not percent:
            self._cancel_cutoff()

    def _release_active(self):
        self._cancel_cutoff()
        for motor in sorted(self.active):
            try:
                self._send(mav.MAV_CMD_ACTUATOR_TEST, 0, 0, 0, 0, motor, 0, 0)
                self.active.discard(motor)
            except OSError:
                pass  # Every active PX4 test also has a finite expiry.

    def stop_outputs(self):
        # Never re-enable an old session: its browser replies or ACKs can be in
        # flight. A reset opens a new session before accepting a new gesture.
        self.outputs_stopped = True
        try:
            if self.jig.stop_pending:
                self.jig.request_disarm()
            else:
                self.jig.end()
        except OSError:
            pass  # Keep the inhibit engaged if USB has gone away.
        if (self.command == mav.MAV_CMD_ACTUATOR_TEST and self.waiting is not None
                and not self.waiting.done()):
            self.waiting.set_exception(MotorStopped("Bench motor stop latched."))
        self._release_active()

    def close(self):
        # On an error, release every potentially active motor before closing USB.
        # Each positive command also expires inside PX4 if the port or Pi disappears.
        self.loop.remove_reader(self.port.fileno())
        try:
            try:
                self.jig.end()
            except OSError:
                pass
            self._release_active()
        finally:
            self.port.close()
