import time
import unittest
from unittest.mock import Mock, patch

import cv2
import numpy as np

from rc_camera.radio import Decoder, MAX_RC_AGE
from rc_camera.server import CameraControl, LiveTrial, Preview, RadioRig, web_app
from rc_camera.mount import HeldPose


def report(stamp, ch1=1500, ch2=1500, ch8=1503):
    return {"received_at": stamp, "status": "Receiving",
            "channels": [{"number": n, "value": v} for n, v in ((1,ch1), (2,ch2), (8,ch8))]}


class TestRig:
    limits = np.array([[400,400], [2000,1800]])
    manual_limits = limits
    catchup_speed = 3000
    mount_ready = True
    rotated = True

    def __init__(self):
        self.position = np.array([1500.,1114.])
        self.t = time.monotonic()
        self.image = np.zeros((360,640,3), np.uint8)
        self.image[160:200,300:340] = np.random.default_rng(8).integers(10,245,(40,40,3),dtype=np.uint8)

    def read(self):
        self.t += 1/24
        return self.image, self.t, self.position.copy()

    def move(self, position):
        self.position = position.copy()


class RCControlTests(unittest.TestCase):
    def setUp(self):
        self.rig = TestRig()
        self.trial = LiveTrial(self.rig, (.51,.46))
        self.camera = CameraControl(self.trial)

    def tick(self, count, ch1=1500, ch2=1500, ch8=1503, stale=False):
        for _ in range(count):
            f = self.trial.capture(self.trial.frame.index + 1)
            stamp = f.stamp - (MAX_RC_AGE + .1 if stale else 0)
            state = self.camera.step(f, report(stamp, ch1, ch2, ch8), f.stamp + .01)
        return state

    def test_proportional_up_right_center_stop_and_low_position(self):
        self.tick(6)
        home = self.rig.position.copy()
        self.tick(18, 1720, 1720)
        half = self.rig.position.copy()
        self.tick(18, 1955, 1962, 999)
        full_delta = self.rig.position - half
        self.assertLess(full_delta[0], 0)
        self.assertGreater(full_delta[1], 0)
        self.assertTrue(np.all(np.abs(full_delta) > np.abs(half-home)*1.6))
        self.tick(3, 1502, 1501)
        centered = self.rig.position.copy()
        self.assertEqual(self.tick(12, 1502, 1501)["mode"], "Manual")
        np.testing.assert_array_equal(self.rig.position, centered)

    def test_switch_edges_acquisition_loss_and_mid_release(self):
        self.tick(6)
        with patch.object(self.trial, "lock", wraps=self.trial.lock) as lock:
            self.tick(1,ch8=2000)
            self.tick(3)
            lock.assert_not_called()
            self.assertEqual(self.tick(22,ch8=2000)["mode"], "Following")
            self.assertEqual(lock.call_count, 1)
            self.assertEqual(self.tick(3,ch8=2000,stale=True)["mode"], "Manual")
            self.tick(15,ch8=2000)
            self.assertEqual(lock.call_count, 1)
            self.tick(8,ch8=1503)
            self.assertEqual(self.tick(21,ch8=2000)["mode"], "Following")
            self.assertEqual(lock.call_count, 2)
            self.assertEqual(self.tick(7,ch8=1503)["mode"], "Manual")

    def test_manual_command_rate_by_stick_throw(self):
        for pulse, expected in ((1530, 0), (1635, 84), (1740, 315), (1845, 838),
                                (1955, 1800), (1040, -1800)):
            with self.subTest(pulse=pulse):
                self.tick(4)
                self.rig.position[:] = [1350, 1100]
                self.tick(3, ch1=pulse)
                before = self.rig.position.copy()
                self.tick(4, ch1=pulse)
                # Four frames at 24 fps. Allow for rounding of the two pulse readings.
                self.assertAlmostEqual((before[0] - self.rig.position[0]) * 6, expected, delta=6)
                self.assertEqual(self.rig.position[1], before[1])

    def test_small_tilt_adjustment_during_fast_pan(self):
        self.tick(5)
        self.tick(3, 1955, 1635)
        before = self.rig.position.copy()
        self.tick(4, 1955, 1635)
        pan, tilt = (self.rig.position - before) * 6
        self.assertAlmostEqual(pan, -1798, delta=6)
        self.assertAlmostEqual(tilt, 84, delta=6)

    def test_duplicate_packets_cannot_engage_switch_or_extend_steering(self):
        decoder = Decoder()
        for i in range(6):
            self.assertIsNone(decoder.read(report(12,ch8=2000), 12+i*.02))
        self.tick(8,1900,1500)
        before = self.rig.position.copy()
        self.assertIsNone(self.tick(12,1900,1500,stale=True)["pulses"])
        np.testing.assert_array_equal(self.rig.position, before)

    def test_mount_outage_keeps_live_video_and_recovers_in_manual(self):
        self.tick(22,ch8=2000)
        self.rig.mount_ready = False
        state = self.tick(3,ch8=2000)
        self.assertEqual(state["mode"], "Mount offline")
        preview = Preview()
        preview.publish(self.rig.image, state)
        api = web_app(preview).test_client()
        self.assertEqual(api.get("/api/state").json["mount_ready"], False)
        reply = api.get("/frame.jpg")
        image = cv2.imdecode(np.frombuffer(reply.data,np.uint8),cv2.IMREAD_COLOR)
        self.assertEqual(image.shape,(360,640,3))
        self.rig.mount_ready = True
        self.assertEqual(self.tick(8,ch8=2000)["mode"], "Manual")
        self.tick(6,ch8=999)
        self.assertEqual(self.tick(21,ch8=2000)["mode"], "Following")

    def test_i2c_reconnect_uses_last_acknowledged_axis_positions(self):
        rig = RadioRig.__new__(RadioRig)
        rig.board, rig.driver, rig.mount = Mock(), Mock(), HeldPose(1270,1093)
        rig.mode_register = 0
        rig.next_probe = 0.0
        old = rig.board
        old.bus.read_byte_data.side_effect = OSError(121, "NACK")
        rig._probe()
        self.assertFalse(rig.mount_ready)
        old.bus.close.assert_called_once()
        rig.next_probe = 0.0
        rig.driver.Servos.return_value = HeldPose(1270,1093)
        with patch("smbus2.SMBus"):
            rig._probe()
        self.assertTrue(rig.mount_ready)
        rig.driver.Servos.assert_called_once_with(rig.board, 1270, 1093)


if __name__ == "__main__":
    unittest.main()
