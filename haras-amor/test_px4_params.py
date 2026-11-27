import unittest

from pymavlink.dialects.v20 import common as mav

from px4_params import decode, differences, encode, format_params, parse_params

INT, REAL = mav.MAV_PARAM_TYPE_INT32, mav.MAV_PARAM_TYPE_REAL32


class ParamFileTests(unittest.TestCase):
    def test_integers_travel_bytewise_in_the_float_field(self):
        for value in (0, 8, 12, -1, 104):
            self.assertEqual(decode(encode(value, INT), INT), value)
        self.assertEqual(decode(encode(0.35, REAL), REAL), 0.35)

    def test_a_saved_file_reads_back_the_same(self):
        params = {"RC_MAP_OFFB_SW": (8, INT), "CA_ROTOR0_PX": (-1.0, REAL), "MC_ROLLRATE_K": (0.35, REAL)}
        text = format_params(params)
        self.assertIn("1\t1\tRC_MAP_OFFB_SW\t8\t6", text)  # QGroundControl's layout
        self.assertEqual(parse_params(text), params)

    def test_differences_ignore_px4s_own_counters_and_float_noise(self):
        saved = {"MC_YAWRATE_K": (1.2, REAL), "SENS_BOARD_ROT": (12, INT), "LND_FLIGHT_T_LO": (5, INT),
                 "MC_ROLL_P": (6.5, REAL)}
        current = {"MC_YAWRATE_K": (0.6, REAL), "SENS_BOARD_ROT": (12, INT), "LND_FLIGHT_T_LO": (900, INT),
                   "MC_ROLL_P": (6.500000001, REAL)}
        self.assertEqual(differences(saved, current), {"MC_YAWRATE_K": (1.2, 0.6)})


if __name__ == "__main__":
    unittest.main()
