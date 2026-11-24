import asyncio
import os
import pty
import tty
import unittest
from unittest.mock import Mock

from aiohttp import WSServerHandshakeError
from aiohttp.test_utils import TestClient, TestServer
from pymavlink.dialects.v20 import common as mav

from motor_preview.motors import BenchError, MAX_PERCENT, MotorLink, PULSE_SECONDS
from motor_preview.server import make_app


class SimulatedPX4:
    """Only the test's pseudo-terminal can receive these commands."""

    def __init__(self):
        self.master, self.slave = pty.openpty()
        tty.setraw(self.slave)
        self.path = os.ttyname(self.slave)
        self.wire = mav.MAVLink(self, srcSystem=1, srcComponent=1)
        self.loop = asyncio.get_running_loop()
        self.tests = []
        self.armed = False
        self.drop_motor = None
        self.loop.add_reader(self.master, self.read)
        self.beat()

    def write(self, data):
        return os.write(self.master, data)

    def beat(self):
        self.wire.heartbeat_send(mav.MAV_TYPE_QUADROTOR, mav.MAV_AUTOPILOT_PX4,
                                 mav.MAV_MODE_FLAG_SAFETY_ARMED if self.armed else 0, 0, 3)
        self.timer = self.loop.call_later(0.05, self.beat)

    def read(self):
        for msg in self.wire.parse_buffer(os.read(self.master, 16384)) or ():
            if msg.get_type() != "COMMAND_LONG":
                continue
            if msg.command == mav.MAV_CMD_ACTUATOR_TEST:
                motor = int(msg.param5)
                self.tests.append((motor, msg.param1, msg.param2))
                if msg.param1 and motor == self.drop_motor:
                    continue
            self.wire.command_ack_send(msg.command, mav.MAV_RESULT_ACCEPTED, 0, 0,
                                       msg.get_srcSystem(), msg.get_srcComponent())

    async def output_count(self, count):
        async with asyncio.timeout(1):
            while len(self.tests) < count:
                await asyncio.sleep(0.005)

    def close(self):
        self.timer.cancel()
        self.loop.remove_reader(self.master)
        os.close(self.master)
        os.close(self.slave)


class PreviewTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.fc = SimulatedPX4()
        self.addCleanup(self.fc.close)
        self.client = TestClient(TestServer(make_app(Mock(), lambda: MotorLink(self.fc.path))))
        await self.client.start_server()
        self.addAsyncCleanup(self.client.close)
        self.origin = str(self.client.make_url("/").origin())

    async def connect(self):
        ws = await self.client.ws_connect("/motors", origin=self.origin)
        self.assertEqual(await ws.receive_json(timeout=2), {"ready": True})
        return ws

    async def apply(self, ws, level, hold=1):
        await ws.send_json({"hold": hold})
        challenge = await ws.receive_json(timeout=1)
        self.assertEqual(challenge["hold"], hold)
        await ws.send_json(dict(challenge, percent=level))
        return challenge

    async def test_full_range_and_release_then_fresh_press(self):
        ws = await self.connect()
        self.assertEqual(self.fc.tests, [])
        for hold, percent in enumerate((50, 1), 1):
            await self.apply(ws, percent, hold)
            await self.fc.output_count(4)
            await ws.send_json({"release": hold})
            self.assertEqual(await ws.receive_json(timeout=1), {"released": hold})
            self.assertEqual(len(self.fc.tests), 8)
            for motor, (number, level, duration) in enumerate(self.fc.tests[:4], 1):
                self.assertEqual(motor, number)
                self.assertAlmostEqual(level, percent / 100)
                self.assertAlmostEqual(duration, PULSE_SECONDS)
            self.assertEqual(self.fc.tests[4:], [(n, 0, 0) for n in range(1, 5)])
            self.fc.tests.clear()
        await ws.close()

    async def test_lost_browser_or_missing_poll_releases_all(self):
        for close_immediately in (True, False):
            ws = await self.connect()
            await self.apply(ws, 7)
            await self.fc.output_count(4)
            if close_immediately:
                await ws.close()
            else:
                await ws.receive_json(timeout=1)  # Drop the next poll.
                self.assertIn("expired", (await ws.receive_json(timeout=1))["error"])
                await ws.close()
            await self.fc.output_count(8)
            self.assertEqual(self.fc.tests[4:], [(n, 0, 0) for n in range(1, 5)])
            self.fc.tests.clear()

    async def test_replayed_poll_cannot_renew(self):
        ws = await self.connect()
        first = await self.apply(ws, 10)
        await self.fc.output_count(4)
        await ws.receive_json(timeout=1)
        await ws.send_json(dict(first, percent=10))
        self.assertIn("error", await ws.receive_json(timeout=1))
        await self.fc.output_count(8)
        self.assertEqual(self.fc.tests[4:], [(n, 0, 0) for n in range(1, 5)])
        await ws.close()

    async def test_sliding_back_to_zero_releases_during_the_hold(self):
        ws = await self.connect()
        await self.apply(ws, 50)
        await self.fc.output_count(4)
        challenge = await ws.receive_json(timeout=1)
        await ws.send_json(dict(challenge, percent=0))
        await self.fc.output_count(8)
        self.assertEqual(self.fc.tests[4:], [(n, 0, 0) for n in range(1, 5)])
        await ws.send_json({"release": 1})
        self.assertEqual(await ws.receive_json(timeout=1), {"released": 1})
        await ws.close()

    async def test_arming_during_a_hold_ends_the_session(self):
        ws = await self.connect()
        await self.apply(ws, 12)
        await self.fc.output_count(4)
        self.fc.armed = True
        async with asyncio.timeout(2):
            while True:
                msg = await ws.receive_json()
                if "error" in msg:
                    self.assertIn("armed", msg["error"])
                    break
                await ws.send_json(dict(msg, percent=12))
        await self.fc.output_count(8)
        self.assertEqual(self.fc.tests[4:], [(n, 0, 0) for n in range(1, 5)])
        await ws.close()

    async def test_armed_flight_controller_and_partial_ack_failure(self):
        self.fc.armed = True
        ws = await self.client.ws_connect("/motors", origin=self.origin)
        self.assertIn("armed", (await ws.receive_json(timeout=2))["error"])
        self.assertEqual(self.fc.tests, [])
        await ws.close()
        self.fc.armed = False
        self.fc.drop_motor = 3
        ws = await self.connect()
        await self.apply(ws, 20)
        self.assertIn("acknowledge", (await ws.receive_json(timeout=1))["error"])
        await self.fc.output_count(6)
        self.assertEqual([n for n, value, _ in self.fc.tests if value], [1, 2, 3])
        self.assertEqual(self.fc.tests[3:], [(n, 0, 0) for n in (1, 2, 3)])
        await ws.close()

    async def test_only_one_local_same_origin_operator(self):
        response = await self.client.get("/", headers={"Host": "192.0.2.1:8000"})
        self.assertIn('data-operator="no"', await response.text())
        for origin, host in (("http://other.invalid", self.client.make_url("/").raw_authority),
                             ("http://192.0.2.1:8000", "192.0.2.1:8000")):
            with self.assertRaises(WSServerHandshakeError) as error:
                await self.client.ws_connect("/motors", origin=origin, headers={"Host": host})
            self.assertEqual(error.exception.status, 403)
        main = await self.connect()
        other = await self.client.ws_connect("/motors", origin=self.origin)
        self.assertIn("Another operator", (await other.receive_json(timeout=1))["error"])
        await other.close()
        await main.close()
        self.assertEqual(self.fc.tests, [])

    async def test_bad_levels_never_reach_the_serial_port(self):
        link = MotorLink(self.fc.path)
        try:
            await link.ready()
            for level in (-1, MAX_PERCENT + 1, True, "10", 1.5):
                with self.assertRaises(BenchError):
                    await link.set_percent(level)
            self.assertEqual(self.fc.tests, [])
        finally:
            link.close()
