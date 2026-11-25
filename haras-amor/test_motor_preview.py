import asyncio
import os
from pathlib import Path
import pty
import struct
import tempfile
import tty
import unittest
from unittest.mock import Mock, patch

from aiohttp import WSServerHandshakeError
from aiohttp.test_utils import TestClient, TestServer
from pymavlink.dialects.v20 import common as mav

from motor_preview.motors import BenchError, MAX_PERCENT, MotorLink, PULSE_SECONDS
from motor_preview.server import make_app
from motor_preview.stop import StopLatch


class SimulatedPX4:
    """Only the test's pseudo-terminal can receive these commands."""

    def __init__(self):
        self.master, self.slave = pty.openpty()
        tty.setraw(self.slave)
        self.path = os.ttyname(self.slave)
        self.wire = mav.MAVLink(self, srcSystem=1, srcComponent=1)
        self.loop = asyncio.get_running_loop()
        self.tests = []
        self.parameters = {}
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
            if msg.get_type() == "PARAM_REQUEST_READ" and msg.param_id in self.parameters:
                value, kind = self.parameters[msg.param_id]
                self.wire.param_value_send(msg.param_id.encode(), value, kind, len(self.parameters), 0)
                continue
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

    async def connect(self, stopped=False):
        ws = await self.client.ws_connect("/motors", origin=self.origin)
        self.assertEqual(await ws.receive_json(timeout=2), {"stopped" if stopped else "ready": True})
        return ws

    async def stop(self):
        response = await self.client.post("/api/motor-stop", headers={"Origin": self.origin})
        self.assertEqual(response.status, 200)
        state = await response.json()
        self.assertTrue(state["stopped"])
        return state["token"]

    async def reset(self, token):
        return await self.client.post("/api/motor-stop/reset", json={"token": token},
                                      headers={"Origin": self.origin})

    async def test_stop_during_hold_keeps_telemetry_and_rejects_old_input(self):
        ws = await self.connect()
        await self.apply(ws, 50)
        await self.fc.output_count(4)
        pending = await ws.receive_json(timeout=1)
        token = await self.stop()  # Separate HTTP connection, with a poll outstanding.
        self.assertEqual(await ws.receive_json(timeout=1), {"stopped": True})
        await self.fc.output_count(8)
        self.assertEqual(self.fc.tests[4:], [(n, 0, 0) for n in range(1, 5)])
        await ws.send_json(dict(pending, percent=50))
        await ws.send_json({"hold": 2})
        self.fc.wire.attitude_send(100, 0.1, -0.2, 0, 0, 0, 0)
        async with asyncio.timeout(1):
            while True:
                state = await (await self.client.get("/api/leveling")).json()
                if "axes" in state:
                    break
                await asyncio.sleep(0.01)
        self.assertEqual(state["sample_ms"], 100)
        await ws.close()
        ws = await self.connect(stopped=True)
        await ws.send_json({"hold": 1})
        self.assertFalse((await (await self.reset(token)).json())["stopped"])
        await ws.close()
        fresh = await self.connect()
        self.assertEqual(len(self.fc.tests), 8)
        await self.apply(fresh, 1)
        await self.fc.output_count(12)
        self.assertAlmostEqual(self.fc.tests[8][1], 0.01)
        await fresh.close()

    async def test_stop_interrupts_unacknowledged_motor_mid_batch(self):
        self.fc.drop_motor = 2
        ws = await self.connect()
        await self.apply(ws, 20)
        await self.fc.output_count(2)
        await self.stop()
        self.assertEqual(await ws.receive_json(timeout=1), {"stopped": True})
        await self.fc.output_count(4)
        self.assertEqual([n for n, value, _ in self.fc.tests if value], [1, 2])
        self.assertEqual(self.fc.tests[2:], [(1, 0, 0), (2, 0, 0)])
        await ws.send_json({"hold": 2})
        await ws.close()
        self.assertEqual(len(self.fc.tests), 4)

    async def test_stop_without_operator_and_only_current_local_reset_allowed(self):
        first = await self.stop()
        second = await self.stop()
        self.assertNotEqual(first, second)
        self.assertEqual((await self.reset(first)).status, 409)
        for headers in ({}, {"Origin": "http://other.invalid"},
                        {"Host": "192.0.2.1:8000", "Origin": "http://192.0.2.1:8000"}):
            for path in ("/api/motor-stop", "/api/motor-stop/reset"):
                response = await self.client.post(path, headers=headers, json={"token": second})
                self.assertEqual(response.status, 403)
        read_only = await self.client.get("/api/motor-stop", headers={"Host": "192.0.2.1:8000"})
        state = await read_only.json()
        self.assertTrue(state["stopped"])
        self.assertNotIn("token", state)
        state = await (await self.client.get("/api/motor-stop")).json()
        self.assertEqual(state["token"], second)
        self.assertFalse((await (await self.reset(second)).json())["stopped"])
        self.assertEqual(self.fc.tests, [])

    async def test_live_leveling_reads_rotor_geometry_without_actuation(self):
        from test_leveling import quad_geometry
        # Swap the two front motor numbers. The UI must follow this FC's geometry.
        params = quad_geometry(((1, -1), (-1, -1), (1, 1), (-1, 1)))
        for name, value in params.items():
            if name == "CA_ROTOR_COUNT":
                encoded, = struct.unpack("<f", struct.pack("<i", value))
                self.fc.parameters[name] = encoded, mav.MAV_PARAM_TYPE_INT32
            else:
                self.fc.parameters[name] = value, mav.MAV_PARAM_TYPE_REAL32
        async def read_state():
            response = await self.client.get("/api/leveling")
            return await response.json()
        self.assertNotIn("axes", await read_state())
        ws = await self.connect()
        stamp = 0
        async with asyncio.timeout(5):
            while True:
                self.fc.wire.attitude_send(stamp, 0.1, 0, 0, 0, 0, 0)
                stamp += 50
                await asyncio.sleep(0.05)
                state = await read_state()
                if state.get("corners"):
                    break
        self.assertEqual(state["axes"]["roll"]["more_lift"], [3, 4])
        self.assertEqual(state["axes"]["pitch"]["more_lift"], [])
        await asyncio.sleep(0.4)
        self.assertNotIn("axes", await read_state())
        await ws.close()
        self.assertNotIn("axes", await read_state())
        self.assertEqual(self.fc.tests, [])

    async def apply(self, ws, level, hold=1):
        await ws.send_json({"hold": hold})
        challenge = await ws.receive_json(timeout=1)
        self.assertEqual(challenge["hold"], hold)
        await ws.send_json(dict(challenge, percent=level))
        return challenge

    async def test_timed_bench_stops_all_four_without_another_request(self):
        link = MotorLink(self.fc.path)
        self.addCleanup(link.close)
        await link.ready()
        with patch("motor_preview.motors.TIMED_BENCH_SECONDS", 0.35):
            await link.set_percent(15)
            await asyncio.sleep(0.10)
            await link.set_percent(15)
            await self.fc.output_count(12)
            self.assertEqual(self.fc.tests[-4:], [(n, 0, 0) for n in range(1, 5)])
            first, renewals = self.fc.tests[:4], self.fc.tests[4:8]
            self.assertEqual([motor for motor, _, _ in first], [1, 2, 3, 4])
            self.assertEqual([motor for motor, _, _ in renewals], [1, 2, 3, 4])
            for (_, value, initial), (_, renewed, remaining) in zip(first, renewals):
                self.assertAlmostEqual(value, 0.15)
                self.assertAlmostEqual(renewed, value)
                self.assertTrue(0 < remaining < initial <= 0.35)
            with self.assertRaises(BenchError):
                await link.set_percent(15)
            self.assertEqual(len(self.fc.tests), 12)

    async def test_early_release_cancels_timeout_before_a_fresh_run(self):
        link = MotorLink(self.fc.path)
        self.addCleanup(link.close)
        await link.ready()
        with patch("motor_preview.motors.TIMED_BENCH_SECONDS", 0.3):
            await link.set_percent(15)
            await link.set_percent(0)
        await asyncio.sleep(0.35)
        self.assertEqual(self.fc.tests[4:], [(n, 0, 0) for n in range(1, 5)])
        start = link.loop.time()
        await link.set_percent(15)
        deadline = link.bench_cutoff.when()
        self.assertLessEqual(start + 20, deadline)
        self.assertLessEqual(deadline, link.loop.time() + 20)
        await link.set_percent(12)
        self.assertEqual(link.bench_cutoff.when(), deadline)
        await link.set_percent(0)

    async def test_cutoff_interrupts_an_outstanding_ack(self):
        link = MotorLink(self.fc.path)
        self.addCleanup(link.close)
        await link.ready()
        self.fc.drop_motor = 2
        with patch("motor_preview.motors.TIMED_BENCH_SECONDS", 0.1):
            with self.assertRaisesRegex(BenchError, "bench run complete"):
                await link.set_percent(15)
        await self.fc.output_count(4)
        self.assertEqual([n for n, value, _ in self.fc.tests if value], [1, 2])
        self.assertEqual(self.fc.tests[2:], [(1, 0, 0), (2, 0, 0)])

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


class StopPersistenceTests(unittest.TestCase):
    def test_restart_preserves_stop_and_invalidates_old_resets(self):
        with tempfile.TemporaryDirectory() as folder:
            marker = Path(folder) / "state" / "motor-stop"
            first = StopLatch(marker)
            self.assertFalse(first.stopped)
            first.engage()
            first.persist()
            self.assertFalse(first.storage_error)
            restarted = StopLatch(marker)
            self.assertTrue(restarted.stopped)
            self.assertFalse(restarted.reset(first.token))
            self.assertTrue(restarted.reset(restarted.token))
            self.assertFalse(StopLatch(marker).stopped)

    def test_unavailable_storage_keeps_stop_engaged(self):
        with tempfile.TemporaryDirectory() as folder:
            parent = Path(folder) / "unusable"
            parent.touch()
            latch = StopLatch(parent / "motor-stop")
            self.assertTrue(latch.stopped)
            latch.persist()
            self.assertTrue(latch.storage_error)
            with self.assertRaises(OSError):
                latch.reset(latch.token)
            self.assertTrue(latch.stopped)
