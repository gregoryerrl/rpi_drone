import math
import unittest

from pymavlink.dialects.v20 import ardupilotmega as mav

from rc_monitor.server import Inputs


def channels(*values):
    sender = mav.MAVLink(None, srcSystem=1, srcComponent=mav.MAV_COMP_ID_AUTOPILOT1)
    raw = list(values) + [65535] * (18 - len(values))
    message = sender.rc_channels_encode(0, len(values), *raw, 255)
    return mav.MAVLink(None).parse_char(message.pack(sender))


class RcInputTests(unittest.TestCase):
    def test_fc_arm_state_and_attitude_have_independent_freshness_from_receiver(self):
        inputs = Inputs()
        sender = mav.MAVLink(None, srcSystem=1, srcComponent=mav.MAV_COMP_ID_AUTOPILOT1)
        def observe(message, now):
            inputs.update(mav.MAVLink(None).parse_char(message.pack(sender)), now)
        def heartbeat(armed, now):
            flags = mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | mav.MAV_MODE_FLAG_AUTO_ENABLED
            flags |= mav.MAV_MODE_FLAG_STABILIZE_ENABLED | mav.MAV_MODE_FLAG_GUIDED_ENABLED
            if armed:
                flags |= mav.MAV_MODE_FLAG_SAFETY_ARMED
            observe(sender.heartbeat_encode(mav.MAV_TYPE_QUADROTOR, mav.MAV_AUTOPILOT_PX4, flags,
                                            (4 << 16) | (3 << 24), 3), now)
        self.assertIsNone(inputs.snapshot(0)["fc_armed"])
        heartbeat(False, 0)
        self.assertIs(inputs.snapshot(0)["fc_armed"], False)
        self.assertEqual(inputs.snapshot(0)["fc_mode"], "Hold")
        attitude = sender.attitude_encode(20, math.radians(2), math.radians(-3), 0, 0, 0, 0)
        observe(attitude, .1)
        self.assertEqual(inputs.snapshot(.2)["attitude_deg"], {"roll": 2, "pitch": -3})
        heartbeat(True, .3)
        flag = mav.MAV_SYS_STATUS_SENSOR_RC_RECEIVER
        observe(sender.sys_status_encode(flag, flag, 0, *([0]*10)), .4)
        self.assertIs(inputs.snapshot(.4)["fc_armed"], True)
        self.assertEqual(inputs.snapshot(.4)["status"], "Receiver signal lost")
        observe(attitude, .8)
        self.assertIsNone(inputs.snapshot(.8)["attitude_deg"])
        self.assertIsNone(inputs.snapshot(2)["fc_armed"])
        self.assertIsNone(inputs.snapshot(2)["fc_mode"])
        heartbeat(True, 3)
        inputs.reset("Waiting for flight controller")
        self.assertIsNone(inputs.snapshot(3)["fc_armed"])

    def test_jitter_is_quiet_and_slow_diagonal_stick_motion_is_visible(self):
        inputs = Inputs()
        inputs.update(channels(1500, 1500), 0)
        inputs.update(channels(1502, 1499), .1)
        self.assertEqual(inputs.snapshot(.1)["latest"], [])
        for n in (1, 2, 3):
            inputs.update(channels(1500 + 10*n, 1500 - 10*n), .1 + n*.05)
        state = inputs.snapshot(.3)
        self.assertEqual([e["number"] for e in state["latest"]], [1, 2])
        self.assertTrue(all(ch["active"] for ch in state["channels"]))

    def test_switch_tap_between_browser_polls_and_fresh_baseline_after_dropout(self):
        inputs = Inputs()
        for stamp, value in ((0, 1000), (.06, 2000), (.12, 1000)):
            inputs.update(channels(65535, value, 0), stamp)
        state = inputs.snapshot(.2)
        self.assertEqual(state["channels"], [{"number": 2, "value": 1000, "active": True}])
        self.assertEqual(state["latest"], [{"number": 2, "before": 2000, "after": 1000}])
        self.assertFalse(inputs.snapshot(1)["channels"][0]["active"])
        self.assertFalse(inputs.snapshot(2)["channels"])
        inputs.update(channels(65535, 2000), 2.1)
        self.assertEqual(inputs.latest, [])
        inputs.update(channels(), 2.2)
        self.assertEqual(inputs.snapshot(2.2)["status"], "Waiting for RC input")

    def test_receiver_health_failure_suppresses_held_values(self):
        inputs = Inputs()
        inputs.update(channels(1000), 0)
        sender = mav.MAVLink(None, srcSystem=1, srcComponent=mav.MAV_COMP_ID_AUTOPILOT1)
        flag = mav.MAV_SYS_STATUS_SENSOR_RC_RECEIVER
        def status(healthy):
            msg = sender.sys_status_encode(flag, flag, flag if healthy else 0, *([0]*10))
            return mav.MAVLink(None).parse_char(msg.pack(sender))
        inputs.update(status(False), .1)
        inputs.update(channels(2000), .2)
        self.assertEqual(inputs.snapshot(.2)["status"], "Receiver signal lost")
        self.assertFalse(inputs.channels)
        inputs.update(status(True), .3)
        inputs.update(channels(2000), .4)
        self.assertEqual(inputs.snapshot(.4)["status"], "Receiving")
        self.assertFalse(inputs.latest)
