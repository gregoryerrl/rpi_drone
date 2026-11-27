import asyncio
import math
import struct
import threading
import time
import unittest

from pymavlink.dialects.v20 import common as mav

import ch8_takeover
from test_motor_preview import SimulatedPX4

ACRO, OFFBOARD, STABILIZED = 5, ch8_takeover.OFFBOARD, 7


class OffboardPX4(SimulatedPX4):
    """Simulated PX4 with its arm switch on CH5 and its Offboard switch on CH8."""

    def __init__(self, offboard_switch=8):
        self.ch5 = self.ch6 = self.ch8 = 1000
        self.mode = ACRO
        self.altitude = 20.0
        self.roll = self.pitch = 0.0
        self.yaw = 1.0
        self.attitude_live = self.radio_live = self.position_live = True
        self.z = self.vz = 0.0  # PX4 local position: down is positive
        self.thrust = 0.35      # the pilot's current thrust, as PX4 reports it
        self.landed = mav.MAV_LANDED_STATE_ON_GROUND
        self.pilot_mode = ACRO
        self.stamp = 0
        self.setpoints = []
        self.mode_requests = []
        super().__init__()
        value, = struct.unpack("<f", struct.pack("<i", offboard_switch))
        self.parameters["RC_MAP_OFFB_SW"] = (value, mav.MAV_PARAM_TYPE_INT32)
        self.wire.set_callback(self.received)

    def beat(self):
        flags = mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        if self.armed:
            flags |= mav.MAV_MODE_FLAG_SAFETY_ARMED
        self.wire.heartbeat_send(mav.MAV_TYPE_QUADROTOR, mav.MAV_AUTOPILOT_PX4, flags, self.mode << 16, 3)
        self.wire.altitude_send(0, self.altitude, 0, 0, 0, 0, 0)
        if self.attitude_live:
            self.wire.attitude_send(int(self.loop.time() * 1000) & 0xFFFFFFFF,
                                    self.roll, self.pitch, self.yaw, 0, 0, 0)
        if self.position_live:
            self.wire.local_position_ned_send(0, 0, 0, self.z, 0, 0, self.vz)
        self.wire.attitude_target_send(0, 0, [1, 0, 0, 0], 0, 0, 0, self.thrust)
        self.wire.extended_sys_state_send(0, self.landed)
        if self.radio_live:
            self.stamp += 50
            values = [1500, 1500, 1000, 1500, self.ch5, self.ch6, 1500, self.ch8] + [65535] * 10
            self.wire.rc_channels_send(self.stamp, 8, *values, 100)
        self.timer = self.loop.call_later(0.05, self.beat)

    def received(self, msg):
        if msg.get_type() == "SET_ATTITUDE_TARGET":
            self.setpoints.append((time.monotonic(), msg.type_mask, tuple(msg.q), msg.thrust))
        elif msg.get_type() == "COMMAND_LONG" and msg.command == mav.MAV_CMD_DO_SET_MODE:
            self.mode = int(msg.param2)
            self.mode_requests.append(self.mode)

    def arm(self, armed=True):
        self.ch5 = 2000 if armed else 1000
        self.armed = armed

    def switch_ch8(self, value):
        # PX4 enters Offboard on the switch edge only if setpoints are already arriving.
        self.ch8 = value
        offered = self.setpoints and time.monotonic() - self.setpoints[-1][0] < 0.5
        if value > 1750 and self.armed and offered:
            self.pilot_mode, self.mode = self.mode, OFFBOARD
        elif value <= 1750 and self.mode == OFFBOARD:
            self.mode = self.pilot_mode

    def switch_ch6(self, value):
        # Middle selects the Stabilized flight-mode slot; high is the kill switch.
        self.ch6 = value
        if 1250 < value <= 1750:
            self.mode = STABILIZED


class FakeCamera:
    """Stands in for FollowCamera: records CH8 and reports a chosen object offset."""

    def __init__(self):
        self.ch8 = "unset"
        self.goal = self.rate = self.drone = self.tilt = self.tilt_time = None

    def radio(self, ch8):
        self.ch8 = ch8

    def vehicle(self, drone):
        self.drone = drone

    def attitude(self, roll, pitch, stamp=None):
        self.tilt, self.tilt_time = (roll, pitch), stamp

    def target(self):
        return self.goal

    def motion(self):
        return self.rate

    def summary(self):
        return "fake"


class TakeoverTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.fc = OffboardPX4()
        self.addCleanup(self.fc.close)
        self.lines = []

    async def start(self, camera=None, flight=False, max_tilt=ch8_takeover.MAX_TILT, thrust=ch8_takeover.THRUST,
                    max_thrust=ch8_takeover.STAND_MAX_THRUST):
        self.stop = threading.Event()
        self.task = asyncio.create_task(asyncio.to_thread(
            ch8_takeover.run, self.fc.path, self.stop, self.lines.append, camera, flight, max_tilt, thrust,
            max_thrust))
        self.addAsyncCleanup(self.finish)
        await self.until(lambda: any(line.startswith("Ready") for line in self.lines))

    async def finish(self):
        self.stop.set()
        await asyncio.wait_for(self.task, 2)

    async def until(self, predicate, timeout=2):
        async with asyncio.timeout(timeout):
            while not predicate():
                await asyncio.sleep(0.01)

    async def offering(self):
        """Whether 20 Hz setpoints are still arriving after a short wait."""
        await asyncio.sleep(0.2)
        return bool(self.fc.setpoints) and time.monotonic() - self.fc.setpoints[-1][0] < 0.15

    async def arm_and_offer(self):
        self.fc.arm()
        await self.until(lambda: self.fc.setpoints)

    async def flying(self, camera=None):
        """Flight setting, with the pilot flying in Stabilized and PX4 reporting the drone in the air."""
        self.fc.mode, self.fc.landed = STABILIZED, mav.MAV_LANDED_STATE_IN_AIR
        await self.start(camera, flight=True)
        await self.arm_and_offer()

    def logged(self, text):
        return any(text in line for line in self.lines)

    def setpoint_euler(self):
        w, x, y, z = self.fc.setpoints[-1][2]
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        return roll, pitch, yaw

    def setpoint_yaw(self):
        return self.setpoint_euler()[2]

    async def test_ch8_hands_the_motors_to_the_pi_and_back(self):
        await self.start()
        self.assertFalse(await self.offering())  # nothing while disarmed
        await self.arm_and_offer()
        _, mask, q, thrust = self.fc.setpoints[-1]
        self.assertEqual(mask, ch8_takeover.RATES_IGNORED)  # attitude and thrust only
        self.assertEqual(q[1:3], (0, 0))  # level: no roll or pitch
        self.assertAlmostEqual(self.setpoint_yaw(), 1.0, places=5)  # the drone's own heading
        self.assertAlmostEqual(thrust, 0.2, places=5)
        for _ in range(2):  # repeated handovers within one armed session
            self.fc.switch_ch8(2000)
            self.assertEqual(self.fc.mode, OFFBOARD)
            self.assertTrue(await self.offering())
            self.fc.switch_ch8(1000)
            self.assertEqual(self.fc.mode, ACRO)
            self.assertTrue(await self.offering())
        self.assertEqual(self.fc.mode_requests, [])

    async def test_level_hold_keeps_the_heading_from_the_handover(self):
        await self.start()
        await self.arm_and_offer()
        self.fc.yaw = 1.2  # before the handover the setpoint follows the drone
        await self.until(lambda: abs(self.setpoint_yaw() - 1.2) < 1e-4)
        self.fc.switch_ch8(2000)
        await asyncio.sleep(0.15)
        self.fc.yaw = 1.5  # PX4 should turn the drone back to the handover heading
        await asyncio.sleep(0.2)
        self.assertAlmostEqual(self.setpoint_yaw(), 1.2, places=4)
        self.fc.switch_ch8(1000)
        await self.until(lambda: abs(self.setpoint_yaw() - 1.5) < 1e-4)

    async def test_handover_tilts_toward_the_locked_object(self):
        camera = FakeCamera()
        await self.start(camera)
        await self.arm_and_offer()
        self.fc.roll, self.fc.pitch = 0.05, -0.02
        await self.until(lambda: camera.tilt is not None and abs(camera.tilt[0] - 0.05) < 1e-6)
        self.assertAlmostEqual(camera.tilt[1], -0.02, places=6)  # the camera gets the attitude
        self.assertLess(time.monotonic() - camera.tilt_time, 0.2)  # and when it was received
        camera.goal = (-0.1, 0.0)  # the camera says a 0.1 rad left roll centres the object
        self.fc.switch_ch8(2000)
        await asyncio.sleep(0.8)
        roll, pitch, yaw = self.setpoint_euler()
        self.assertAlmostEqual(roll, -0.1, places=4)  # reached and held, not wound up
        self.assertAlmostEqual(pitch, 0, places=4)
        self.assertAlmostEqual(yaw, 1.0, places=4)  # the handover heading is held
        camera.goal = (0.0, -0.5)  # far toward the nose: limited to MAX_TILT
        await asyncio.sleep(1.0)
        roll, pitch, yaw = self.setpoint_euler()
        self.assertAlmostEqual(roll, 0, places=4)
        self.assertAlmostEqual(pitch, -ch8_takeover.MAX_TILT, places=4)
        self.assertAlmostEqual(yaw, 1.0, places=4)
        camera.goal = None  # nothing locked: ease back to level
        await asyncio.sleep(1.0)
        roll, pitch, _ = self.setpoint_euler()
        self.assertAlmostEqual(roll, 0, places=4)
        self.assertAlmostEqual(pitch, 0, places=4)

    async def test_a_larger_tilt_limit_leans_further_and_moves_the_withdrawal(self):
        camera = FakeCamera()
        await self.start(camera, max_tilt=math.radians(25))
        await self.arm_and_offer()
        self.assertTrue(self.logged("tilt up to 25 degrees"))
        camera.goal = (0.6, 0.0)  # far to one side: limited to 25 degrees, not 10
        self.fc.switch_ch8(2000)
        await asyncio.sleep(1.4)
        roll, _, _ = self.setpoint_euler()
        self.assertAlmostEqual(roll, math.radians(25), places=4)
        self.fc.roll = math.radians(35)  # past the default 30 degrees but inside 25 + 15
        self.assertTrue(await self.offering())
        self.fc.roll = math.radians(41)
        await self.until(lambda: self.logged("tilted past 40 degrees"))

    async def test_a_90_degree_limit_caps_nothing_on_the_stand(self):
        camera = FakeCamera()
        await self.start(camera, max_tilt=math.radians(90))
        await self.arm_and_offer()
        camera.goal = (0.9, 0.0)  # 52 degrees, past the old 45-degree cap
        self.fc.switch_ch8(2000)
        await asyncio.sleep(0.8)
        roll, _, _ = self.setpoint_euler()
        self.assertAlmostEqual(roll, 0.9, places=4)

    async def test_the_tilt_target_moves_quickly(self):
        camera = FakeCamera()
        await self.start(camera, max_tilt=math.radians(90))
        await self.arm_and_offer()
        camera.goal = (math.radians(40), 0.0)
        self.fc.switch_ch8(2000)
        await asyncio.sleep(0.5)  # at 180 deg/s, 40 degrees takes about 0.22 s after the handover
        roll, _, _ = self.setpoint_euler()
        self.assertAlmostEqual(math.degrees(roll), 40, places=3)

    def test_the_tilt_limit_is_capped(self):
        with self.assertRaisesRegex(ValueError, "at most 90 degrees"):
            ch8_takeover.Takeover(self.fc.path, max_tilt=math.radians(91))
        with self.assertRaisesRegex(ValueError, "at most 45 degrees in the flight setting"):
            ch8_takeover.Takeover(self.fc.path, flight=True, max_tilt=math.radians(50))
        with self.assertRaisesRegex(ValueError, "between 5% and 50%"):
            ch8_takeover.Takeover(self.fc.path, thrust=0.6)
        with self.assertRaisesRegex(ValueError, "base no higher than the top"):
            ch8_takeover.Takeover(self.fc.path, thrust=0.3, max_thrust=0.25)

    async def test_stand_thrust_rises_while_the_drone_has_to_lean(self):
        camera = FakeCamera()
        await self.start(camera, max_tilt=math.radians(90))
        await self.arm_and_offer()
        self.assertTrue(self.logged("stand setting: 20% to 50% thrust as the lean needs"))
        self.fc.switch_ch8(2000)
        await asyncio.sleep(0.3)
        self.assertAlmostEqual(self.fc.setpoints[-1][3], 0.2, places=3)  # object straight above
        camera.goal = (math.radians(15), 0.0)  # a 15-degree lean, with the drone still level
        await asyncio.sleep(0.6)
        self.assertAlmostEqual(self.fc.setpoints[-1][3], 0.5, places=3)  # 15 degrees behind: the top
        self.fc.roll = math.radians(15)  # it got there and holds it: half of the 30-degree lean
        await asyncio.sleep(0.6)
        self.assertAlmostEqual(self.fc.setpoints[-1][3], 0.35, places=3)
        camera.goal, self.fc.roll = None, 0.0  # the lock is gone: back to level and the base
        await asyncio.sleep(0.6)
        self.assertAlmostEqual(self.fc.setpoints[-1][3], 0.2, places=3)

    async def test_thrust_changes_smoothly(self):
        camera = FakeCamera()
        await self.start(camera, max_tilt=math.radians(90))
        await self.arm_and_offer()
        self.fc.switch_ch8(2000)
        await asyncio.sleep(0.3)
        camera.goal = (math.radians(40), 0.0)
        await asyncio.sleep(0.12)
        thrusts = [point[3] for point in self.fc.setpoints[-12:]]
        steps = [b - a for a, b in zip(thrusts, thrusts[1:])]
        self.assertLessEqual(max(steps), ch8_takeover.THRUST_RATE * ch8_takeover.SETPOINT_PERIOD + 1e-6)

    async def test_stand_thrust_can_be_raised_for_more_leaning_force(self):
        await self.start(max_tilt=math.radians(45), thrust=0.5)
        await self.arm_and_offer()
        self.assertTrue(self.logged("fixed 50% thrust, tilt up to 45 degrees"))
        self.fc.switch_ch8(2000)
        await asyncio.sleep(0.2)
        self.assertAlmostEqual(self.fc.setpoints[-1][3], 0.5, places=5)

    async def test_tipping_past_the_tilt_limit_withdraws(self):
        await self.start()
        await self.arm_and_offer()
        self.fc.roll = math.radians(35)
        await self.until(lambda: any("tilted past" in line for line in self.lines))
        self.assertFalse(await self.offering())

    async def test_camera_website_gets_the_drone_status(self):
        camera = FakeCamera()
        await self.start(camera)
        await self.until(lambda: camera.drone and camera.drone["roll"] is not None)
        before = {key: camera.drone[key] for key in ("armed", "mode", "control", "roll", "pitch",
                                                     "target", "setting", "thrust")}
        self.assertEqual(before, dict(armed=False, mode="Acro", control=None, roll=0.0, pitch=0.0,
                                      target=None, setting="stand", thrust=None))
        await self.arm_and_offer()
        await self.until(lambda: camera.drone["control"] == "pilot")
        self.assertEqual((camera.drone["height"], camera.drone["pi"]), (0.0, "ready"))
        self.fc.switch_ch8(2000)
        await self.until(lambda: camera.drone["control"] == "Pi")
        during = {key: camera.drone[key] for key in ("mode", "target", "thrust", "pi")}
        self.assertEqual(during, dict(mode="Offboard", target=[0.0, 0.0], thrust=0.2, pi="in control"))

    async def test_follow_damping_eases_off_as_the_object_comes_closer(self):
        camera = FakeCamera()
        await self.start(camera)
        await self.arm_and_offer()
        camera.goal, camera.rate = (0.1, 0.0), (-0.1, 0.0)  # its angle shrinks at 0.1 rad/s
        self.fc.switch_ch8(2000)
        await asyncio.sleep(0.6)
        roll, _, _ = self.setpoint_euler()
        self.assertAlmostEqual(roll, 0.1 - ch8_takeover.FOLLOW_DAMPING * 0.1, places=4)

    async def test_flight_setting_offers_only_from_stabilized_in_the_air(self):
        await self.start(flight=True)
        self.fc.arm()  # Acro, on the ground
        await self.until(lambda: self.logged("Pi waiting: fly in Stabilized or Altitude"))
        self.assertFalse(await self.offering())
        self.fc.mode = STABILIZED
        await self.until(lambda: self.logged("Pi waiting: PX4 does not report the drone in the air"))
        self.assertFalse(await self.offering())
        self.fc.landed = mav.MAV_LANDED_STATE_IN_AIR
        await self.until(lambda: self.fc.setpoints)
        self.assertAlmostEqual(self.fc.setpoints[-1][3], 0.35, places=3)  # your thrust: no jump

    async def test_flight_handover_holds_the_height(self):
        await self.flying()
        self.fc.switch_ch8(2000)
        await asyncio.sleep(0.2)
        self.assertAlmostEqual(self.fc.setpoints[-1][3], 0.35, places=2)  # starts from your thrust
        self.fc.z = 0.5  # sinking: 0.5 m below the hold (PX4's z points down)
        await asyncio.sleep(0.3)
        self.assertGreater(self.fc.setpoints[-1][3], 0.36)
        self.fc.z = -0.5  # 0.5 m above the hold
        await asyncio.sleep(0.3)
        self.assertLess(self.fc.setpoints[-1][3], 0.34)
        self.fc.z, self.fc.vz = 0.0, 0.5  # at the hold but descending at 0.5 m/s
        await asyncio.sleep(0.3)
        self.assertGreater(self.fc.setpoints[-1][3], 0.36)
        self.assertLessEqual(max(s[3] for s in self.fc.setpoints), ch8_takeover.THRUST_RANGE[1])

    async def test_flight_handover_gives_control_back_if_the_height_drifts(self):
        await self.flying()
        self.fc.switch_ch8(2000)
        await asyncio.sleep(0.2)
        self.fc.z = 1.2  # 1.2 m below the hold
        await self.until(lambda: self.logged("height drifted 1.2 m from the hold"))
        await self.until(lambda: self.fc.mode_requests)
        self.assertEqual(self.fc.mode_requests, [STABILIZED])  # back to the pilot's mode
        self.assertFalse(await self.offering())

    async def test_flight_handover_needs_height_data(self):
        await self.flying()
        self.fc.switch_ch8(2000)
        await asyncio.sleep(0.2)
        self.fc.position_live = False
        await self.until(lambda: self.logged("Pi control off: no height data from PX4"))
        self.assertFalse(await self.offering())

    async def test_flight_setting_stays_below_the_test_ceiling(self):
        await self.flying()
        self.fc.altitude = 20.0 + ch8_takeover.FLIGHT_CEILING + 0.5  # armed at 20 m
        await self.until(lambda: self.logged("above the 3 m test ceiling"))
        self.assertFalse(await self.offering())

    async def test_camera_gets_ch8_and_none_without_radio(self):
        camera = FakeCamera()
        await self.start(camera)
        self.fc.ch8 = 1500
        await self.until(lambda: camera.ch8 == 1500)
        self.fc.radio_live = False
        await self.until(lambda: camera.ch8 is None)

    async def test_attitude_loss_withdraws(self):
        await self.start()
        await self.arm_and_offer()
        self.fc.attitude_live = False
        await self.until(lambda: any("no attitude data" in line for line in self.lines))
        self.assertFalse(await self.offering())

    async def test_ch8_high_when_arming_needs_a_fresh_ch8_low(self):
        await self.start()
        self.fc.ch8 = 2000
        self.fc.arm()
        self.assertFalse(await self.offering())
        self.fc.switch_ch8(1000)
        await self.until(lambda: self.fc.setpoints)

    async def test_kill_switch_withdraws_and_returns_px4_to_acro(self):
        await self.start()
        await self.arm_and_offer()
        self.fc.switch_ch8(2000)
        self.fc.switch_ch6(2000)
        await self.until(lambda: self.fc.mode_requests)
        self.assertEqual(self.fc.mode_requests, [ACRO])
        self.assertFalse(await self.offering())
        self.fc.switch_ch6(1000)  # CH8 is still high, so the Pi waits for CH8 low
        self.assertFalse(await self.offering())
        self.fc.switch_ch8(1000)
        self.assertTrue(await self.offering())

    async def test_ch6_middle_leaves_px4_in_the_pilots_mode(self):
        await self.start()
        await self.arm_and_offer()
        self.fc.switch_ch8(2000)
        await asyncio.sleep(0.1)  # the Pi sees PX4 in Offboard
        self.fc.switch_ch6(1500)
        await self.until(lambda: any("CH6 raised" in line for line in self.lines))
        await asyncio.sleep(0.1)
        self.assertEqual(self.fc.mode_requests, [])
        self.assertEqual(self.fc.mode, STABILIZED)
        self.assertFalse(await self.offering())

    async def test_radio_loss_withdraws_and_returns_px4_to_acro(self):
        await self.start()
        await self.arm_and_offer()
        self.fc.switch_ch8(2000)
        self.fc.radio_live = False
        await self.until(lambda: self.fc.mode_requests)
        self.assertEqual(self.fc.mode_requests, [ACRO])
        self.assertFalse(await self.offering())

    async def test_stopping_the_script_returns_px4_to_acro(self):
        await self.start()
        await self.arm_and_offer()
        self.fc.switch_ch8(2000)
        await asyncio.sleep(0.1)  # let the Pi see CH8 high
        await self.finish()
        await self.until(lambda: self.fc.mode_requests)
        self.assertEqual(self.fc.mode_requests, [ACRO])

    async def test_climb_ends_the_session_until_px4_is_armed_again(self):
        await self.start()
        await self.arm_and_offer()
        self.fc.altitude += 2
        await self.until(lambda: any("climb" in line for line in self.lines))
        self.fc.altitude -= 2
        self.assertFalse(await self.offering())
        self.fc.arm(False)
        await asyncio.sleep(0.15)
        self.fc.arm()
        self.assertTrue(await self.offering())

    async def test_waits_for_an_arming_it_has_seen(self):
        self.fc.arm()
        await self.start()
        self.assertTrue(any("already armed" in line for line in self.lines))
        self.assertFalse(await self.offering())
        self.fc.arm(False)
        await asyncio.sleep(0.15)
        self.fc.arm()
        self.assertTrue(await self.offering())

    async def test_requires_ch8_as_the_px4_offboard_switch(self):
        fc = OffboardPX4(offboard_switch=0)
        self.addCleanup(fc.close)
        with self.assertRaisesRegex(ch8_takeover.SetupError, "RC_MAP_OFFB_SW is 0"):
            await asyncio.to_thread(ch8_takeover.run, fc.path, threading.Event(), self.lines.append)
        self.assertEqual(fc.setpoints, [])


if __name__ == "__main__":
    unittest.main()
