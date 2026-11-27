import unittest

import cv2
import numpy as np

from calibrate_camera import fit_column, mount_from, picture_shift, spins_from_layout


class CalibrationTests(unittest.TestCase):
    def test_picture_shift_reports_how_the_content_moved(self):
        rng = np.random.default_rng(3)
        before = cv2.GaussianBlur(rng.random((360, 640)).astype(np.float32), (0, 0), 3)
        after = np.roll(before, (-5, 12), axis=(0, 1))  # content 12 px right and 5 px up
        window = cv2.createHanningWindow((640, 360), cv2.CV_32F)
        np.testing.assert_allclose(picture_shift(before, after, window), (12, -5), atol=0.3)

    def test_fit_column_recovers_the_shift_per_unit_tilt(self):
        tilts = np.linspace(-0.3, 0.3, 25)
        noise = np.random.default_rng(1).normal(0, 1.0, (25, 2))
        shifts = np.outer(tilts, (-412.0, 6.0)) + noise
        np.testing.assert_allclose(fit_column(tilts, shifts), (-412.0, 6.0), atol=3)

    def test_mount_takes_pitch_from_the_nose_side(self):
        mount = mount_from((-400.0, 0.0), "top")
        np.testing.assert_allclose(mount, [[-400.0, 0.0], [0.0, -400.0]])
        with self.assertRaisesRegex(ValueError, "toward the nose"):
            mount_from((0.0, -400.0), "top")  # rolling cannot slide the picture along the nose axis

    def test_spins_follow_px4s_motor_layout(self):
        standard = {1: 1.0, 2: -1.0, 3: -1.0, 4: 1.0}  # quad X: M2 and M3 on the left
        self.assertEqual(spins_from_layout(standard), [((2, 3), 3.0, 1), ((1, 4), 8.0, -1)])
        mirrored = {1: -1.0, 2: 1.0, 3: 1.0, 4: -1.0}
        self.assertEqual(spins_from_layout(mirrored), [((1, 4), 3.0, 1), ((2, 3), 8.0, -1)])
        with self.assertRaisesRegex(ValueError, "two motors on each side"):
            spins_from_layout({1: 1.0, 2: 1.0})


if __name__ == "__main__":
    unittest.main()
