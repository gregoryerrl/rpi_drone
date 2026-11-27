import asyncio
from pathlib import Path
import struct
import tempfile
import unittest
from unittest.mock import patch

from pymavlink.dialects.v20 import common as mav

from jig_view.ch5 import Ch5Receiver
from motor_preview.motors import PULSE_SECONDS
from test_motor_preview import SimulatedPX4


class RadioPX4(SimulatedPX4):
    def __init__(self):
        self.ch5, self.ch6, self.ch8 = 1980, 1000, 1980
        self.channel_count = 8
        self.heartbeat_live = self.radio_live = self.advance_stamp = True
        self.stamp = 0
        super().__init__()

    def arm_channel(self, channel):
        value, = struct.unpack("<f", struct.pack("<i", channel))
        self.parameters["RC_MAP_ARM_SW"] = (value, mav.MAV_PARAM_TYPE_INT32)
        self.wire.param_value_send(b"RC_MAP_ARM_SW", value, mav.MAV_PARAM_TYPE_INT32, 1, 0)

    def radio(self):
        if self.advance_stamp:
            self.stamp += 1
        values = [1500, 1500, 990, 1500, self.ch5, self.ch6, 1500, self.ch8] + [65535] * 10
        self.wire.rc_channels_send(self.stamp, self.channel_count, *values, 100)

    def beat(self):
        if self.heartbeat_live:
            super().beat()
        else:
            self.timer = self.loop.call_later(.05, self.beat)
        if self.radio_live:
            self.radio()


class Ch5Tests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        self.fc = RadioPX4()
        self.addCleanup(self.fc.close)
        self.marker = Path(folder.name) / "motor-stop"
        self.rx = Ch5Receiver(self.fc.path, self.marker)
        self.rx.thread.start()
        self.addAsyncCleanup(self.close_receiver)
        await self.until(lambda: "Move CH8 low" in self.report()["detail"])

    async def close_receiver(self):
        self.rx.stop.set()
        await self.until(lambda: not self.rx.thread.is_alive())
        self.rx.close()

    def report(self):
        return self.rx.state()["ch5_override"]

    async def until(self, predicate):
        async with asyncio.timeout(3):
            while not predicate():
                await asyncio.sleep(.005)

    async def disable(self):
        self.fc.ch5 = 1000
        self.fc.radio()
        await self.until(lambda: "Raise CH5" in self.report()["detail"])

    async def enable(self):
        self.fc.ch5 = 1980
        self.fc.ch8 = 1000
        self.fc.radio()
        await self.until(lambda: "Raise CH8 to start" in self.report()["detail"])

    async def start_motors(self):
        await self.enable()
        self.fc.ch8 = 1980
        self.fc.radio()
        await self.until(lambda: self.report()["percent"] == 15)

    async def released(self):
        await self.until(lambda: self.fc.tests[-4:] == [(n, 0, 0) for n in range(1, 5)])
        self.assertEqual(self.report()["percent"], 0)

    async def test_ch5_enables_and_only_a_subsequent_ch8_edge_drives_all_four(self):
        self.assertEqual(self.fc.tests, [])  # Both switches were already high at startup.
        await self.disable()
        for pulse in (1000, 1980):
            self.fc.ch8 = pulse
            self.fc.radio()
        self.fc.ch5 = 1980  # Turning on CH5 with CH8 already high cannot start.
        self.fc.radio()
        await self.until(lambda: "Move CH8 low" in self.report()["detail"])
        await self.enable()
        await asyncio.sleep(.15)
        self.assertEqual(self.fc.tests, [])
        await self.start_motors()
        await self.fc.output_count(12)
        for i, (motor, value, duration) in enumerate(self.fc.tests[:12]):
            self.assertEqual(motor, i % 4 + 1)
            self.assertAlmostEqual(value, .15)
            self.assertAlmostEqual(duration, PULSE_SECONDS)
        self.fc.ch8 = 1500
        self.fc.radio()
        await self.released()
        await self.start_motors()
        await self.disable()
        await self.released()
        count = len(self.fc.tests)
        await asyncio.sleep(.25)
        self.assertEqual(len(self.fc.tests), count)

    async def test_kill_tap_and_radio_loss_require_a_fresh_switch(self):
        await self.start_motors()
        self.fc.ch6 = 1980
        self.fc.radio()
        self.fc.ch6 = 1000
        self.fc.radio()
        await self.released()
        count = len(self.fc.tests)
        await asyncio.sleep(.25)
        self.assertEqual(len(self.fc.tests), count)
        await self.start_motors()
        self.fc.radio_live = False
        await self.released()
        count = len(self.fc.tests)
        self.fc.radio_live = True
        await asyncio.sleep(.25)
        self.assertEqual(len(self.fc.tests), count)

    async def test_fc_arm_state_and_motor_stop_control_the_test(self):
        self.fc.arm_channel(5)
        await self.disable()
        self.fc.heartbeat_live = False
        self.fc.ch5 = 1980
        self.fc.ch8 = 1000
        self.fc.radio()
        await self.until(lambda: "confirm disarmed" in self.report()["detail"])
        self.fc.ch8 = 1980
        self.fc.radio()
        await asyncio.sleep(.12)
        self.assertEqual(self.fc.tests, [])  # The disarmed heartbeat preceded CH5.
        self.fc.armed = True
        self.fc.heartbeat_live = True
        await self.until(lambda: "PX4 is armed" in self.report()["detail"])
        self.fc.armed = False
        self.marker.touch()
        await self.until(lambda: "latched" in self.report()["detail"])
        self.marker.unlink()
        await self.until(lambda: "Move CH8 low" in self.report()["detail"])
        self.assertEqual(self.fc.tests, [])
        await self.start_motors()  # PX4 is disarmed and its arm mapping still reads CH5.
        self.marker.touch()
        await self.released()
        self.marker.unlink()
        await self.start_motors()
        self.fc.armed = True
        await self.released()

    async def test_invalid_or_frozen_receiver_data_releases_motors(self):
        await self.start_motors()
        self.fc.ch6 = 65535
        self.fc.radio()
        await self.released()
        self.fc.ch6 = 1000
        await self.start_motors()
        self.fc.channel_count = 6  # A raw high CH8 outside the channel count is invalid.
        self.fc.radio()
        await self.released()
        self.fc.channel_count = 8
        await self.start_motors()
        self.fc.advance_stamp = False
        await self.released()

    async def test_kill_while_second_motor_ack_is_outstanding(self):
        self.fc.drop_motor = 2
        await self.enable()
        self.fc.ch8 = 1980
        self.fc.radio()
        await self.fc.output_count(2)
        self.fc.ch6 = 1980
        self.fc.radio()
        await self.fc.output_count(4)
        self.assertEqual([n for n, level, _ in self.fc.tests if level], [1, 2])
        self.assertEqual(self.fc.tests[2:], [(1, 0, 0), (2, 0, 0)])
        self.assertEqual(self.report()["percent"], 0)

    async def test_receiver_fault_and_reconnect_do_not_restart_a_held_switch(self):
        await self.start_motors()
        flag = mav.MAV_SYS_STATUS_SENSOR_RC_RECEIVER
        self.fc.wire.sys_status_send(flag, flag, 0, 0, 0, -1, -1, 0, 0, 0, 0, 0, 0)
        await self.released()
        count = len(self.fc.tests)
        self.fc.wire.sys_status_send(flag, flag, flag, 0, 0, -1, -1, 0, 0, 0, 0, 0, 0)
        await self.until(lambda: "Move CH8 low" in self.report()["detail"])
        self.assertEqual(len(self.fc.tests), count)

        self.fc.drop_motor = 2
        await self.enable()
        self.fc.ch8 = 1980
        self.fc.radio()
        await self.until(lambda: len(self.fc.tests) >= count + 4)
        self.assertEqual(self.fc.tests[-2:], [(1, 0, 0), (2, 0, 0)])
        self.fc.drop_motor = None
        await self.until(lambda: "Move CH8 low" in self.report()["detail"])
        self.assertEqual(len(self.fc.tests), count + 4)
        await self.start_motors()
        self.rx.stop.set()
        await self.released()

    async def test_ch8_timer_limits_px4_pulses_and_requires_a_new_edge(self):
        with patch("jig_view.ch5.RUN_SECONDS", .35):
            await self.start_motors()
            await self.released()
            commands = [(motor, duration) for motor, level, duration in self.fc.tests if level]
            self.assertEqual({motor for motor, _ in commands}, {1, 2, 3, 4})
            self.assertTrue(all(0 < duration <= .35 for _, duration in commands))
            self.assertLess(commands[-1][1], commands[0][1])
            count = len(self.fc.tests)
            await asyncio.sleep(.4)
            self.assertEqual(len(self.fc.tests), count)
            await self.start_motors()
