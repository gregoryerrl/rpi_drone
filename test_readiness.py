import http.client
import importlib
import json
import threading
import unittest
from unittest.mock import patch

import follow_me


class TemplateTests(unittest.TestCase):
    def test_importing_bench_modules_leaves_hardware_idle(self):
        with patch("pymavlink.mavutil.mavlink_connection") as serial, \
             patch("picamera2.Picamera2") as camera, patch("servo_test.PCA9685") as pwm:
            for name in ("diag", "spin", "series", "pantilt_calibrate"):
                importlib.import_module(name)
        serial.assert_not_called()
        camera.assert_not_called()
        pwm.assert_not_called()

    def test_control_page_over_forwarded_port(self):
        with follow_me.Server(("127.0.0.1", 0), follow_me.Handler) as server:
            worker = threading.Thread(target=server.serve_forever)
            worker.start()
            try:
                port = server.server_address[1]
                conn = http.client.HTTPConnection("127.0.0.1", port, timeout=3)
                conn.request("GET", "/", headers={"Host": "localhost:8000"})
                response = conn.getresponse()
                self.assertEqual(response.status, 200)
                self.assertIn(b"/api/cmd", response.read())
                conn.close()
                for origin, expected in (("http://localhost:8000", 200),
                                         ("http://unrelated.invalid", 403)):
                    conn = http.client.HTTPConnection("127.0.0.1", port, timeout=3)
                    with patch.object(follow_me, "cmds") as commands:
                        conn.request("POST", "/api/cmd", json.dumps({"action": "enroll_cancel"}),
                                     {"Host": "localhost:8000", "Content-Type": "application/json",
                                      "Origin": origin})
                        self.assertEqual(conn.getresponse().status, expected)
                        self.assertEqual(commands.put.called, expected == 200)
                    conn.close()
            finally:
                server.shutdown()
                worker.join()


if __name__ == "__main__":
    unittest.main()
