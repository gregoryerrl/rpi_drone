import math
import unittest

from check_layout import pairs_from_layout, verdict

# The layout on the flight controller on 2026-11-27: M1 front-left, M2 back-right,
# M3 front-right, M4 back-left in PX4's frame.
LAYOUT = {(1, "PX"): 1.0, (1, "PY"): -1.0, (2, "PX"): -1.0, (2, "PY"): 1.0,
          (3, "PX"): 1.0, (3, "PY"): 1.0, (4, "PX"): -1.0, (4, "PY"): -1.0}


class LayoutTests(unittest.TestCase):
    def test_pairs_follow_px4s_layout(self):
        pairs = {side: (motors, axis, sign) for side, motors, axis, sign in pairs_from_layout(LAYOUT)}
        self.assertEqual(pairs, {"front": ((1, 3), "pitch", 1), "back": ((2, 4), "pitch", -1),
                                 "left": ((1, 4), "roll", 1), "right": ((2, 3), "roll", -1)})

    def test_an_incomplete_layout_is_refused(self):
        with self.assertRaisesRegex(ValueError, "two motors on PX4's front"):
            pairs_from_layout({(1, "PX"): 1.0, (2, "PX"): -1.0})

    def test_a_pitch_that_tips_the_other_way_means_front_and_back_are_mirrored(self):
        tip = math.radians(5)
        results = [("front", (1, 3), "pitch", 1, -tip), ("back", (2, 4), "pitch", -1, tip),
                   ("left", (1, 4), "roll", 1, tip), ("right", (2, 3), "roll", -1, -tip)]
        lines = verdict(results)
        self.assertIn("OPPOSITE", lines[0])
        self.assertIn("as PX4 expects", lines[2])
        self.assertEqual(lines[-1], "PX4's motor layout has front and back (flip the sign of "
                                    "CA_ROTOR0-3_PX) mirrored.")

    def test_a_matching_layout_and_a_tip_too_small_to_tell(self):
        tip = math.radians(5)
        good = [("front", (1, 3), "pitch", 1, tip), ("back", (2, 4), "pitch", -1, -tip),
                ("left", (1, 4), "roll", 1, tip), ("right", (2, 3), "roll", -1, -tip)]
        self.assertEqual(verdict(good)[-1], "PX4's motor layout matches the drone.")
        good[0] = ("front", (1, 3), "pitch", 1, math.radians(0.3))
        self.assertIn("too small to tell", verdict(good)[0])
        self.assertIn("too small to tell", verdict(good)[-1])


if __name__ == "__main__":
    unittest.main()
