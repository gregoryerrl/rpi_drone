"""Momentary collective motor tests on the disarmed Haras Amor flight controller."""
import asyncio
import time

from pymavlink.dialects.v20 import common as mav
from serial import Serial


DEFAULT_DEVICE = "/dev/serial/by-id/usb-MicoAir_MicoAir743v2_0-if00"
MAX_PERCENT = 50
MOTORS = (1, 2, 3, 4)
PULSE_SECONDS = 0.6
BATCH_SECONDS = 0.24
HEARTBEAT_AGE = 1.6


class BenchError(Exception):
    """A condition that ends the current operator session."""


class MotorLink:
    def __init__(self, device):
        self.loop = asyncio.get_running_loop()
        self.port = Serial(device, 57600, timeout=0, write_timeout=0.03, exclusive=True)
        try:
            self.port.reset_input_buffer()
            self.wire = mav.MAVLink(self.port, srcSystem=255, srcComponent=190)
            self.wire.robust_parsing = True
            self.system = None
            self.last_heartbeat = 0.0
            self.seen = asyncio.Event()
            self.fault = None
            self.waiting = None
            self.command = None
            self.active = set()
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
                if msg.get_srcComponent() != mav.MAV_COMP_ID_AUTOPILOT1:
                    continue
                if self.system is not None and msg.get_srcSystem() != self.system:
                    continue
                if msg.get_type() == "HEARTBEAT":
                    if msg.autopilot != mav.MAV_AUTOPILOT_PX4 or msg.type != mav.MAV_TYPE_QUADROTOR:
                        self._fail("Expected a PX4 quadrotor on this USB port.")
                    elif msg.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED:
                        self._fail("Flight controller armed. Bench control stopped.")
                    else:
                        self.system = msg.get_srcSystem()
                        self.last_heartbeat = time.monotonic()
                        self.seen.set()
                elif (msg.get_type() == "COMMAND_ACK" and self.waiting is not None
                      and not self.waiting.done() and msg.command == self.command
                      and msg.target_system in (0, 255) and msg.target_component in (0, 190)):
                    self.waiting.set_result(msg.result)
        except (OSError, mav.MAVError) as exc:
            self.loop.remove_reader(self.port.fileno())
            self._fail(f"Flight controller disconnected: {exc}")

    def check(self):
        if self.fault:
            raise BenchError(self.fault)
        if self.system is None or time.monotonic() - self.last_heartbeat > HEARTBEAT_AGE:
            raise BenchError("Flight controller heartbeat lost.")

    def _send(self, command, *params):
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

    async def set_percent(self, percent):
        if type(percent) is not int or not 0 <= percent <= MAX_PERCENT:
            raise BenchError("Motor level outside the bench range.")
        self.check()
        deadline = time.monotonic() + BATCH_SECONDS
        motors = MOTORS if percent else tuple(sorted(self.active))
        for motor in motors:
            self.active.add(motor)
            await self._confirmed(deadline - time.monotonic(), mav.MAV_CMD_ACTUATOR_TEST,
                                  percent / 100, PULSE_SECONDS if percent else 0, 0, 0, motor, 0, 0)
            if not percent:
                self.active.remove(motor)

    def close(self):
        # On an error, release every potentially active motor before closing USB.
        # Each positive command also expires inside PX4 if the port or Pi disappears.
        self.loop.remove_reader(self.port.fileno())
        try:
            for motor in sorted(self.active):
                try:
                    self._send(mav.MAV_CMD_ACTUATOR_TEST, 0, 0, 0, 0, motor, 0, 0)
                except OSError:
                    pass
        finally:
            self.port.close()
