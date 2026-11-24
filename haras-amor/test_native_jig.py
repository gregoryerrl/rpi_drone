import asyncio
import math
import os
import struct
import unittest
from unittest.mock import Mock

from aiohttp.test_utils import TestClient, TestServer
from pymavlink.dialects.v20 import common as mav

from motor_preview.jig import STABILIZED, stick_axes
from motor_preview.motors import MotorLink
from motor_preview.server import make_app
from test_motor_preview import SimulatedPX4


class JigPX4(SimulatedPX4):
    def __init__(self):
        self.mode = (4, 3)
        self.landed = mav.MAV_LANDED_STATE_ON_GROUND
        self.sticks = []
        self.commands = []
        self.refuse_arm = self.refuse_disarm = False
        self.attitude_fresh = True
        self.attitude = (.02, -.03)
        super().__init__()
        joystick, = struct.unpack("<f", struct.pack("<i", 2))
        self.parameters.update(COM_RC_IN_MODE=(joystick, mav.MAV_PARAM_TYPE_INT32),
                               MPC_MAN_TILT_MAX=(35.0, mav.MAV_PARAM_TYPE_REAL32))

    def beat(self):
        stamp = int(self.loop.time() * 1000) & 0xffffffff
        base = mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        if self.armed:
            base |= mav.MAV_MODE_FLAG_SAFETY_ARMED
        self.wire.heartbeat_send(mav.MAV_TYPE_QUADROTOR, mav.MAV_AUTOPILOT_PX4, base,
                                 (self.mode[0] << 16) | (self.mode[1] << 24), 3)
        self.wire.extended_sys_state_send(0, self.landed)
        if self.attitude_fresh:
            self.wire.attitude_send(stamp, *self.attitude, 0, 0, 0, 0)
        if self.mode == STABILIZED:
            x, y, z = self.sticks[-1] if self.sticks else (0, 0, 0)
            rx, ry = math.radians(35) * y / 1000, -math.radians(35) * x / 1000
            angle = math.hypot(rx, ry)
            scale = math.sin(angle/2) / angle if angle else 0
            quaternion = (math.cos(angle/2), rx * scale, ry * scale, 0)
            self.wire.attitude_target_send(stamp, 0, quaternion, 0, 0, 0, z / 1000)
        self.timer = self.loop.call_later(.03, self.beat)

    def read(self):
        for msg in self.wire.parse_buffer(os.read(self.master, 16384)) or ():
            if msg.get_type() == "MANUAL_CONTROL":
                self.sticks.append((msg.x, msg.y, msg.z))
            elif msg.get_type() == "PARAM_REQUEST_READ" and msg.param_id in self.parameters:
                value, kind = self.parameters[msg.param_id]
                self.wire.param_value_send(msg.param_id.encode(), value, kind, len(self.parameters), 0)
            elif msg.get_type() == "COMMAND_LONG":
                self.commands.append((msg.command, msg.param1, msg.param2, msg.param3))
                result = mav.MAV_RESULT_ACCEPTED
                if msg.command == mav.MAV_CMD_DO_SET_MODE:
                    self.mode = (int(msg.param2), int(msg.param3))
                elif msg.command == mav.MAV_CMD_COMPONENT_ARM_DISARM:
                    denied = self.refuse_arm if msg.param1 else self.refuse_disarm
                    if denied:
                        result = mav.MAV_RESULT_DENIED
                    else:
                        self.armed = bool(msg.param1)
                self.wire.command_ack_send(msg.command, result, 0, 0, msg.get_srcSystem(), msg.get_srcComponent())


class JigTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.fc = JigPX4()
        self.addCleanup(self.fc.close)
        self.client = TestClient(TestServer(make_app(Mock(), lambda: MotorLink(self.fc.path))))
        await self.client.start_server()
        self.addAsyncCleanup(self.client.close)
        self.origin = str(self.client.make_url("/").origin())
        self.ws = await self.client.ws_connect("/motors", origin=self.origin)
        self.assertEqual(await self.ws.receive_json(timeout=2), {"ready": True})
        await asyncio.sleep(.25)  # Both PX4 joystick parameters have arrived.

    async def reply(self, packet, roll=0, pitch=0, throttle=0, arm=False):
        await self.ws.send_json(dict(jig_poll=packet["jig_poll"], roll=roll, pitch=pitch,
                                     throttle=throttle, arm=arm))
        return await self.ws.receive_json(timeout=2)

    async def enable(self):
        await self.ws.send_json({"jig": "on"})
        packet = await self.ws.receive_json(timeout=1)
        async with asyncio.timeout(3):
            while not (packet.get("confirmed") and packet.get("fc_target") is not None):
                packet = await self.reply(packet)
        return packet

    async def arm(self, packet):
        packet = await self.reply(packet, arm=True)
        async with asyncio.timeout(2):
            while not packet.get("armed"):
                packet = await self.reply(packet)
        return packet

    async def test_disarmed_native_mode_accepts_targets_then_restores(self):
        packet = await self.enable()
        self.assertFalse(packet["armed"])
        self.assertEqual(packet["stick_pct"], 0)
        for roll, pitch in ((3, -4), (-5, 0), (0, 5), (0, 0)):
            async with asyncio.timeout(1):
                while True:
                    packet = await self.reply(packet, roll, pitch)
                    if packet["fc_target"] and all(abs(a-b) < .1 for a,b in zip(packet["fc_target"], (roll,pitch))):
                        break
            state = await (await self.client.get("/api/leveling")).json()
            self.assertEqual(state["jig"]["target"], [roll, pitch])
            self.assertEqual(state["jig"]["fc_target"], packet["fc_target"])
            self.assertFalse(state["jig"]["armed"])
            self.assertEqual(state["jig"]["stick_pct"], 0)
        await self.ws.send_json({"jig": "off"})
        self.assertIn("jig_off", await self.ws.receive_json(timeout=1))
        await asyncio.sleep(.05)
        self.assertEqual(self.fc.mode, (4, 3))
        self.assertNotIn("jig", await (await self.client.get("/api/leveling")).json())
        self.assertTrue(all(z == 0 for _, _, z in self.fc.sticks))
        self.assertTrue(all(c not in (mav.MAV_CMD_ACTUATOR_TEST, mav.MAV_CMD_COMPONENT_ARM_DISARM)
                            for c, *_ in self.fc.commands))

    async def test_angle_cap_and_disarmed_throttle_are_enforced_on_wire(self):
        for controls in ({"roll": 4, "pitch": 4}, {"throttle": 1}, {"roll": True}, {"throttle": 51}):
            packet = await self.enable()
            result = await self.reply(packet, **controls)
            self.assertIn("jig_off", result)
            await asyncio.sleep(.06)
        self.assertTrue(all(stick == (0, 0, 0) for stick in self.fc.sticks))
        self.assertFalse(self.fc.armed)

    async def test_operator_stop_overrides_active_jig_and_verifies_disarm(self):
        packet = await self.arm(await self.enable())
        self.assertTrue(packet["armed"])
        self.assertEqual(packet["stick_pct"], 0)
        packet = await self.reply(packet, roll=2, throttle=7)
        self.assertEqual(packet["stick_pct"], 7)
        self.assertEqual(self.fc.sticks[-1][2], 70)
        stop = await self.client.post("/api/motor-stop", headers={"Origin": self.origin})
        self.assertTrue((await stop.json())["stopped"])
        self.assertEqual(await self.ws.receive_json(timeout=1), {"stopped": True})
        await self.ws.send_json(dict(jig_poll=packet["jig_poll"], roll=2, pitch=0, throttle=7, arm=False))
        await asyncio.sleep(.12)
        self.assertEqual(self.fc.sticks[-1], (0, 0, 0))
        self.assertFalse(self.fc.armed)
        self.assertEqual([c[1:3] for c in self.fc.commands if c[0] == mav.MAV_CMD_COMPONENT_ARM_DISARM],
                         [(1, 0), (0, 0)])
        status = await (await self.client.get("/api/motor-stop")).json()
        self.assertIn("reports disarmed", status["fc_stop"])

    async def test_disarm_denial_keeps_stop_latched_and_reset_blocked(self):
        await self.arm(await self.enable())
        self.fc.refuse_disarm = True
        response = await self.client.post("/api/motor-stop", headers={"Origin": self.origin})
        state = await response.json()
        await asyncio.sleep(.12)
        status = await (await self.client.get("/api/motor-stop")).json()
        self.assertIn("motor-power disconnect", status["fc_stop"])
        response = await self.client.post("/api/motor-stop/reset", headers={"Origin": self.origin},
                                          json={"token": state["token"]})
        self.assertEqual(response.status, 409)
        self.assertTrue(self.fc.armed)
        self.assertEqual(self.fc.sticks[-1], (0,0,0))

    async def test_rejected_arm_never_enables_throttle(self):
        self.fc.refuse_arm = True
        packet = await self.reply(await self.enable(), arm=True)
        async with asyncio.timeout(1):
            while not packet["notice"]:
                packet = await self.reply(packet)
        self.assertIn("declined arming", packet["notice"])
        self.assertFalse(packet["armed"])
        self.assertIn("jig_off", await self.reply(packet, throttle=1))
        self.assertTrue(all(z == 0 for _,_,z in self.fc.sticks))

    async def test_browser_loss_while_armed_ends_jig(self):
        packet = await self.arm(await self.enable())
        await self.reply(packet, throttle=2)
        # Drop the next outstanding browser challenge.
        self.assertIn("expired", (await self.ws.receive_json(timeout=1))["jig_off"])
        await asyncio.sleep(.12)
        self.assertFalse(self.fc.armed)
        self.assertEqual(self.fc.sticks[-1], (0,0,0))

    async def test_ground_state_and_fresh_attitude_required(self):
        self.fc.landed = mav.MAV_LANDED_STATE_IN_AIR
        await asyncio.sleep(.07)
        await self.ws.send_json({"jig":"on"})
        self.assertIn("ground", (await self.ws.receive_json(timeout=1))["jig_off"])
        self.fc.landed = mav.MAV_LANDED_STATE_ON_GROUND
        await asyncio.sleep(.07)
        packet = await self.enable()
        self.fc.attitude_fresh = False
        async with asyncio.timeout(1):
            while "jig_poll" in packet:
                packet = await self.reply(packet)
        self.assertIn("Fresh FC attitude", packet["jig_off"])
        self.assertFalse(self.fc.armed)

    async def test_level_at_start_and_measured_tilt_cutoff(self):
        packet = await self.enable()
        self.fc.attitude = (0, math.radians(8))
        await asyncio.sleep(.06)
        result = await self.reply(packet, arm=True)
        self.assertIn("within 5°", result["jig_off"])
        self.assertFalse(self.fc.armed)
        self.fc.attitude = (0, 0)
        await asyncio.sleep(.08)
        packet = await self.arm(await self.enable())
        packet = await self.reply(packet, throttle=3)
        self.fc.attitude = (math.radians(8), math.radians(8))
        await asyncio.sleep(.06)
        result = await self.reply(packet, throttle=3)
        self.assertIn("10°", result["jig_off"])
        self.assertTrue(result["fc_pending"])
        self.assertTrue((await self.ws.receive_json(timeout=1))["jig_disarmed"])
        self.assertFalse(self.fc.armed)
        self.assertEqual(self.fc.sticks[-1], (0, 0, 0))

    async def test_replayed_jig_ticket_cannot_renew_throttle(self):
        old = await self.arm(await self.enable())
        await self.reply(old, throttle=5)
        result = await self.reply(old, throttle=10)
        self.assertIn("expired", result["jig_off"])
        self.assertTrue((await self.ws.receive_json(timeout=1))["jig_disarmed"])
        self.assertNotIn(100, [z for _, _, z in self.fc.sticks])

    async def test_zero_after_throttle_ends_the_run_at_the_server(self):
        packet = await self.arm(await self.enable())
        packet = await self.reply(packet, throttle=2)
        result = await self.reply(packet)
        self.assertIn("released", result["jig_off"])
        self.assertTrue((await self.ws.receive_json(timeout=1))["jig_disarmed"])
        self.assertFalse(self.fc.armed)
        self.assertEqual(self.fc.sticks[-1], (0, 0, 0))

    def test_quantized_axes_remain_inside_five_degree_cone(self):
        for roll, pitch in ((5,0), (-5,0), (3,4), (-4,-3), (0,-5)):
            x, y = stick_axes(roll, pitch, 35)
            self.assertLessEqual(math.hypot(x,y)*35 / 1000, 5)
            if pitch:
                self.assertLess(x * pitch, 0)
            if roll:
                self.assertGreater(y * roll, 0)
