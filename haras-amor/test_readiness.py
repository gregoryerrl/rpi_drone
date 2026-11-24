import importlib
import unittest
from unittest.mock import patch


class TemplateTests(unittest.TestCase):
    def test_importing_bench_modules_leaves_hardware_idle(self):
        with patch("pymavlink.mavutil.mavlink_connection") as serial:
            for name in ("diag", "spin", "series"):
                importlib.import_module(name)
        serial.assert_not_called()


if __name__ == "__main__":
    unittest.main()
