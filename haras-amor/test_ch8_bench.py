import asyncio
import contextlib
import io
import os
import tempfile
import unittest
from unittest.mock import patch

from pymavlink.dialects.v20 import common as mav

import ch8_bench
from motor_preview import motors
from test_motor_preview import SimulatedPX4


class RadioPX4(SimulatedPX4):
    """Simulated PX4 that also streams the TX16S switch channels at 20 Hz."""

    def __init__(self):
        self.ch5 = self.ch6 = self.ch8 = 1000
        self.stamp = 0
        self.intervals = []
        super().__init__()
        self.wire.set_callback(self.received)

    def received(self, msg):
        if msg.get_type() == "COMMAND_LONG" and msg.command == mav.MAV_CMD_SET_MESSAGE_INTERVAL:
            self.intervals.append((int(msg.param1), msg.param2))

    def beat(self):
        super().beat()
        self.stamp += 50
        values = [1500, 1500, 990, 1500, self.ch5, self.ch6, 1500, self.ch8] + [65535] * 10
        self.wire.rc_channels_send(self.stamp, 8, *values, 100)


class Ch8BenchTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        state = tempfile.TemporaryDirectory()
        self.addCleanup(state.cleanup)
        env = patch.dict(os.environ, {"XDG_STATE_HOME": state.name})
        env.start()
        self.addCleanup(env.stop)
        self.fc = RadioPX4()
        self.addCleanup(self.fc.close)

    async def until(self, predicate, timeout=2):
        async with asyncio.timeout(timeout):
            while not predicate():
                await asyncio.sleep(0.01)

    # A shortened driver cutoff would end the run early if it still applied.
    @patch.object(motors, "TIMED_BENCH_SECONDS", 0.2)
    async def test_ch8_run_has_no_cutoff_and_ends_when_ch8_is_lowered(self):
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            run = asyncio.create_task(ch8_bench.once(self.fc.path))
            await self.until(lambda: "Ready" in out.getvalue())
            self.assertEqual(self.fc.intervals, [(mav.MAVLINK_MSG_ID_RC_CHANNELS, 50000)])
            await asyncio.sleep(0.15)  # let the low CH8 position register first
            self.fc.ch8 = 2000
            await self.until(lambda: self.fc.tests)
            await asyncio.sleep(0.6)
            self.assertFalse(run.done())
            sent = len(self.fc.tests)
            await asyncio.sleep(0.2)
            self.assertGreater(len(self.fc.tests), sent)  # still renewing while CH8 is high
            self.fc.ch8 = 1000
            clean = await asyncio.wait_for(run, 2)
        self.assertTrue(clean)
        self.assertIn("CH8 lowered", out.getvalue())
        positive = [test for test in self.fc.tests if test[1:] != (0, 0)]
        self.assertEqual({motor for motor, _, _ in positive}, set(motors.MOTORS))
        for _, value, pulse in positive:
            self.assertAlmostEqual(value, 0.2, places=5)
            self.assertAlmostEqual(pulse, motors.PULSE_SECONDS, places=5)
        self.assertEqual(self.fc.tests[-4:], [(motor, 0, 0) for motor in motors.MOTORS])


if __name__ == "__main__":
    unittest.main()
