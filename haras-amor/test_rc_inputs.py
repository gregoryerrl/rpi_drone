import math
import unittest

from pymavlink.dialects.v20 import ardupilotmega as mav

from rc_monitor.server import BATTERY_STALE_S, Inputs


def channels(*values):
    sender = mav.MAVLink(None, srcSystem=1, srcComponent=mav.MAV_COMP_ID_AUTOPILOT1)
    raw = list(values) + [65535] * (18 - len(values))
    message = sender.rc_channels_encode(0, len(values), *raw, 255)
    return mav.MAVLink(None).parse_char(message.pack(sender))


class BatteryInputTests(unittest.TestCase):
    def setUp(self):
        self.inputs = Inputs()
        self.fc = mav.MAVLink(None, srcSystem=1, srcComponent=mav.MAV_COMP_ID_AUTOPILOT1)
        self.heartbeat(0)

    def receive(self, sender, msg, stamp):
        self.inputs.update(mav.MAVLink(None).parse_char(msg.pack(sender)), stamp)

    def heartbeat(self, stamp):
        msg = self.fc.heartbeat_encode(mav.MAV_TYPE_QUADROTOR, mav.MAV_AUTOPILOT_PX4, 0, 0, 3)
        self.receive(self.fc, msg, stamp)

    def summary(self, stamp, mv, ca, pct, present=False):
        flag = mav.MAV_SYS_STATUS_SENSOR_BATTERY if present else 0
        msg = self.fc.sys_status_encode(flag, flag, flag, 0, mv, ca, pct, 0, 0, 0, 0, 0, 0)
        self.receive(self.fc, msg, stamp)

    def pack(self, stamp, cells, ca=-1, pct=-1, *, extra=(0, 0, 0, 0), state=0, sender=None, ident=0):
        sender = sender or self.fc
        msg = sender.battery_status_encode(ident, 0, 0, 32767, list(cells) + [65535] * (10-len(cells)),
                                           ca, -1, -1, pct, charge_state=state, voltages_ext=extra)
        self.receive(sender, msg, stamp)

    def packs(self, stamp):
        return self.inputs.snapshot(stamp)["batteries"]

    def test_system_summary_preserves_real_zeroes_and_drops_unknowns(self):
        self.summary(.1, 16825, 825, 64)
        pack, = self.packs(.2)
        self.assertEqual((pack["voltage_v"], pack["current_a"], pack["remaining_pct"]), (16.825, 8.25, 64))
        self.assertAlmostEqual(pack["power_w"], 16.825 * 8.25, places=3)
        self.summary(.3, 15700, 0, 0)
        pack, = self.packs(.3)
        self.assertEqual((pack["current_a"], pack["remaining_pct"], pack["power_w"]), (0, 0, 0))
        self.summary(.4, 65535, -1, -1)
        self.assertEqual(self.packs(.4), [])
        self.summary(.5, 65535, 0, 100)
        self.assertEqual(self.packs(.5), [])
        self.summary(.6, 65535, 0, 0, present=True)
        pack, = self.packs(.6)
        self.assertIsNone(pack["voltage_v"])
        self.assertIsNone(pack["power_w"])
        self.assertEqual(pack["remaining_pct"], 0)

    def test_separate_pack_sources_and_no_cross_source_power(self):
        other_component = mav.MAVLink(None, srcSystem=1, srcComponent=mav.MAV_COMP_ID_BATTERY)
        other_aircraft = mav.MAVLink(None, srcSystem=2, srcComponent=mav.MAV_COMP_ID_AUTOPILOT1)
        self.summary(.1, 22000, 1000, 80)
        self.pack(.2, [4010, 3990, 4000], pct=24, state=mav.MAV_BATTERY_CHARGE_STATE_LOW)
        self.pack(.3, [15100], ca=120, sender=other_component)
        self.pack(.4, [50000], sender=other_aircraft, ident=8)
        primary, secondary = self.packs(.5)
        self.assertEqual((primary["voltage_v"], primary["charge_state"], primary["remaining_pct"]), (12, "Low", 24))
        self.assertIsNone(primary["current_a"])
        self.assertIsNone(primary["power_w"])
        self.assertAlmostEqual(secondary["power_w"], 18.12)
        self.assertIsNone(secondary["remaining_pct"])
        self.assertNotEqual(primary["component"], secondary["component"])

    def test_extended_cells_missing_cells_and_charging(self):
        self.pack(.1, [4000]*10, ca=-200, pct=54, extra=(3990, 3980, 0, 0),
                  state=mav.MAV_BATTERY_CHARGE_STATE_CHARGING)
        pack, = self.packs(.1)
        self.assertEqual((pack["voltage_v"], pack["current_a"], pack["power_w"], pack["charge_state"]),
                         (47.97, -2, -95.94, "Charging"))
        self.pack(.2, [4000, 65535, 3990], ca=300)
        pack, = self.packs(.2)
        self.assertIsNone(pack["voltage_v"])
        self.assertIsNone(pack["power_w"])
        self.pack(.3, [])
        self.assertEqual(self.packs(.3), [])

    def test_expiry_fc_reconnect_and_independence_from_rc(self):
        self.pack(.1, [15200], ca=300)
        self.inputs.reset_channels("Receiver signal lost")
        self.assertEqual(self.packs(.2)[0]["voltage_v"], 15.2)
        self.assertEqual(self.packs(2), [])  # FC heartbeat stopped.
        future = BATTERY_STALE_S + 1
        self.heartbeat(future)
        self.summary(future, 14200, -1, -1)
        pack, = self.packs(future)
        self.assertIsNone(pack["id"])
        self.assertEqual(pack["voltage_v"], 14.2)
        self.heartbeat(future * 2)
        self.assertEqual(self.packs(future * 2), [])
        self.pack(future * 2, [16000])
        self.inputs.reset("Waiting for flight controller")
        self.heartbeat(future * 2 + .1)
        self.assertEqual(self.packs(future * 2 + .1), [])


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

    def test_switch_trace_retains_sub_poll_kill_and_resets_on_missing_signal(self):
        inputs = Inputs()

        def observe(t, six, eight):
            inputs.update(channels(1500, 1500, 990, 1500, 999, six, 1500, eight), t)
            return inputs.snapshot(t)["switch_trace"]

        observe(1, 999, 1503)
        observe(1.02, 999, 2000)
        observe(1.04, 2000, 2000)
        trace = observe(1.04, 999, 2000)
        self.assertEqual([(e["at"], e["ch6"], e["ch8"]) for e in trace], [
            (1, "lower", "lower"), (1.02, "lower", "high"),
            (1.04, "high", "high"), (1.04, "lower", "high")])
        self.assertEqual(observe(1.06, 1005, 1985), trace)
        self.assertEqual(observe(1.08, 1760, 2000)[-1]["ch6"], "between")
        self.assertEqual(observe(1.1, 65535, 2000), [])
        self.assertEqual(observe(1.15, 999, 2000),
                         [{"at": 1.15, "ch6": "lower", "ch8": "high"}])
        observe(1.2, 999, 1503)
        self.assertEqual(observe(1.6, 999, 2000),
                         [{"at": 1.6, "ch6": "lower", "ch8": "high"}])

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
