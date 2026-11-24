import math
import unittest

from motor_preview.leveling import (
    LevelingPreview, MAX_ACCELERATION, MAX_SAMPLE_AGE, leveling_acceleration, rotor_corners,
)


def quad_geometry(positions=((1, 1), (-1, -1), (1, -1), (-1, 1))):
    result = {"CA_ROTOR_COUNT": 4}
    for i, (x, y) in enumerate(positions):
        for field, value in (("PX", x), ("PY", y), ("AX", 0), ("AY", 0), ("AZ", -1)):
            result[f"CA_ROTOR{i}_{field}"] = value
    return result


class LevelingTests(unittest.TestCase):
    def test_axis_signs_and_rate_braking(self):
        rad = math.radians
        self.assertEqual(leveling_acceleration(0, 0, 0, 0), (0, 0))
        roll, pitch = leveling_acceleration(rad(10), 0, 0, 0)
        self.assertAlmostEqual(roll, -rad(10))
        self.assertEqual(pitch, 0)
        self.assertAlmostEqual(leveling_acceleration(0, -rad(8), 0, 0)[1], rad(8))
        # With the right side low but rising rapidly, brake the leftward roll.
        self.assertGreater(leveling_acceleration(rad(10), 0, -rad(8), 0)[0], 0)
        self.assertLess(leveling_acceleration(0, 0, 0, rad(4))[1], 0)

    def test_combined_tilt_vector_and_invalid_inputs(self):
        roll, pitch = math.radians(40), math.radians(-30)
        x, y = leveling_acceleration(roll, pitch, 0, 0)
        self.assertAlmostEqual(math.hypot(x, y), MAX_ACCELERATION)
        self.assertAlmostEqual(x / y, math.sin(roll) * math.cos(pitch) / math.sin(pitch))
        for args in ((math.pi, 0, 0, 0), (0, math.pi / 2, 0, 0),
                     (math.nan, 0, 0, 0), (0, 0, math.inf, 0)):
            with self.subTest(args=args), self.assertRaises(ValueError):
                leveling_acceleration(*args)

    def test_ideal_jig_settles_after_disturbance(self):
        # Unit angular-acceleration plant. This exercises the control law without
        # assuming measured gains, motor thrust, friction or inertia for Haras Amor.
        for dt in (0.01, 0.025):
            roll, pitch = map(math.radians, (12, -7))
            p = q = 0
            for step in range(round(12 / dt)):
                if step == round(2 / dt):
                    q += math.radians(8)
                ax, ay = leveling_acceleration(roll, pitch, p, q)
                self.assertLessEqual(math.hypot(ax, ay), MAX_ACCELERATION + 1e-12)
                p, q = p + ax * dt, q + ay * dt
                roll, pitch = roll + p * dt, pitch + q * dt
            self.assertLess(max(map(abs, (roll, pitch, p, q))), math.radians(0.05))

    def test_each_side_at_rest_uses_configured_logical_motors(self):
        preview = LevelingPreview()
        preview.parameters.update(quad_geometry())
        for stamp, (roll, pitch, roll_lift, pitch_lift) in enumerate((
            (5, 0, [1, 4], []), (-5, 0, [2, 3], []),
            (0, 5, [], [2, 4]), (0, -5, [], [1, 3]),
        )):
            preview.observe(stamp, math.radians(roll), math.radians(pitch), 0, 0, 1)
            axes = preview.snapshot(1)["axes"]
            self.assertEqual(axes["roll"]["more_lift"], roll_lift)
            self.assertEqual(axes["pitch"]["more_lift"], pitch_lift)

    def test_partial_or_unsupported_layout_withholds_motor_numbers(self):
        preview = LevelingPreview()
        preview.observe(1, 0.1, 0, 0, 0, 1)
        self.assertIsNone(preview.snapshot(1)["axes"]["roll"]["more_lift"])
        for changes in ({"CA_ROTOR_COUNT": 6}, {"CA_ROTOR0_AZ": 1}, {"CA_ROTOR0_PY": -1},
                        {"CA_ROTOR0_PX": 0}, {"CA_ROTOR1_AX": math.nan}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                rotor_corners(dict(quad_geometry(), **changes))

    def test_wrap_duplicate_stale_clock_reset_and_invalid_attitude(self):
        preview = LevelingPreview()
        def observe(stamp, now, roll=0.1):
            preview.observe(stamp, roll, 0, 0, 0, now)
        preview.parameters.update(quad_geometry())
        observe(0xfffffffe, 0)
        observe(5, .01)
        observe(4, .02)
        observe(5, MAX_SAMPLE_AGE)
        self.assertEqual(preview.snapshot(.03)["sample_ms"], 5)
        self.assertNotIn("axes", preview.snapshot(.02 + MAX_SAMPLE_AGE))
        observe(1, 1)
        self.assertNotIn("axes", preview.snapshot(1))
        self.assertEqual(preview.parameters, {})
        observe(2, 1.01)
        self.assertIn("axes", preview.snapshot(1.01))
        observe(3, 1.02, math.nan)
        self.assertNotIn("axes", preview.snapshot(1.02))
