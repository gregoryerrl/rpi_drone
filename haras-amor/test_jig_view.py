import unittest

import numpy as np

from jig_view.server import RadioLock


class RadioLockTests(unittest.TestCase):
    def setUp(self):
        self.lock = RadioLock()
        self.time = 20.0
        self.patch = np.random.default_rng(42).integers(0, 255, size=(40, 40, 3), dtype=np.uint8)

    def frames(self, switch, count, offset=0, texture=True, packet_time=None, receive=True):
        for _ in range(count):
            self.time += 1 / 24
            image = np.full((120, 200, 3), 30, dtype=np.uint8)
            if texture:
                image[40:80, 80+offset:120+offset] = self.patch
            radio = {"status": "Receiving" if receive else "RC data stopped",
                     "received_at": self.time if packet_time is None else packet_time,
                     "channels": [{"number": 8, "value": switch}]}
            result = self.lock.step(image, self.time, radio, self.time)
        return result

    def test_center_selection_motion_release_and_off(self):
        self.assertEqual(self.frames(1000, 5)["mode"], "Off")
        self.assertEqual(self.frames(1500, 5)["mode"], "Ready")
        self.assertEqual(self.frames(2000, 5)["mode"], "Acquiring")
        self.assertEqual(self.frames(2000, 15)["mode"], "Following")
        for offset in range(0, 32, 2):
            state = self.frames(2000, 1, offset)
        self.assertEqual(state["mode"], "Following")
        self.assertAlmostEqual(state["box"][0] * 200, 130, delta=3)
        for pulse, mode in [(1500, "Ready"), (1000, "Off")]:
            state = self.frames(pulse, 5)
            self.assertEqual(state["mode"], mode)
            self.assertIsNone(state["box"])

    def test_startup_high_spike_and_frozen_packets_do_not_select(self):
        self.assertEqual(self.frames(2000, 20)["mode"], "Switch high")
        self.assertEqual(self.frames(1500, 5)["mode"], "Ready")
        self.assertEqual(self.frames(2000, 1)["mode"], "Ready")
        self.assertEqual(self.frames(1500, 5)["mode"], "Ready")
        self.assertEqual(self.frames(2000, 4, packet_time=self.time)["mode"], "Ready")
        self.assertEqual(self.frames(2000, 9, packet_time=self.time)["mode"], "Waiting")
        self.assertEqual(self.frames(2000, 12)["mode"], "Switch high")

    def test_signal_or_camera_gap_discards_lock_until_new_edge(self):
        self.frames(1500, 6)
        self.assertEqual(self.frames(2000, 17)["mode"], "Following")
        self.assertEqual(self.frames(2000, 1, receive=False)["mode"], "Waiting")
        self.assertEqual(self.frames(2000, 6)["mode"], "Switch high")
        self.frames(1500, 5)
        self.assertEqual(self.frames(2000, 18)["mode"], "Following")
        self.time += 1
        self.assertEqual(self.frames(2000, 6)["mode"], "Switch high")

    def test_flat_selection_and_target_loss_require_reselection(self):
        self.frames(1500, 6)
        self.assertEqual(self.frames(2000, 12, texture=False)["mode"], "No detail")
        self.assertEqual(self.frames(2000, 12)["mode"], "No detail")
        self.frames(1500, 6)
        self.assertEqual(self.frames(2000, 18)["mode"], "Following")
        self.assertEqual(self.frames(2000, 15, texture=False)["mode"], "Lost")
        self.assertEqual(self.frames(2000, 12)["mode"], "Lost")
