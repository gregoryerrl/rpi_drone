import time
import unittest
from threading import Event
from unittest.mock import Mock, patch

import cv2
import numpy as np

from jig_view.server import Frame, Jig
from rc_monitor.server import Receiver


class JigViewTests(unittest.TestCase):
    def test_full_frame_jpeg_is_independent_of_receiver(self):
        image = np.full((32, 64, 3), 25, dtype=np.uint8)
        image[:, 24:] = 205

        def camera(stop):
            yield image, time.monotonic()
            stop.set()

        receiver = Mock()
        jig = Jig(receiver, camera=camera)
        jig.run()
        receiver.state.assert_not_called()
        frame, jpeg = jig.jpeg(-1)
        self.assertIs(frame.image, image)
        decoded = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        np.testing.assert_allclose(decoded, image, atol=3)
        self.assertEqual(jig.jpeg(frame.number), (frame, None))

    def test_camera_and_radio_availability_are_independent(self):
        receiver = Mock()
        jig = Jig(receiver)
        jig.frame = Frame(1, np.zeros((8, 8, 3), np.uint8), 20.0)
        with patch("jig_view.server.time.monotonic", return_value=20.1):
            for pulse in (999, 1503, 2000):
                radio = {"received_at": 20.0, "status": "Receiving",
                         "channels": [{"number": 8, "value": pulse}]}
                receiver.state.return_value = radio
                self.assertEqual(jig.state(), {"radio": radio, "rc_live": True, "camera_live": True})
            radio["received_at"] = None
            self.assertFalse(jig.state()["rc_live"])
            self.assertTrue(jig.state()["camera_live"])
        with patch("jig_view.server.time.monotonic", return_value=21.0):
            radio["received_at"] = 21.0
            self.assertTrue(jig.state()["rc_live"])
            self.assertFalse(jig.state()["camera_live"])
            self.assertEqual(jig.jpeg(-1), (None, None))

    def test_continuous_receiver_can_listen_without_frame_or_http_requests(self):
        for continuous in (True, False):
            with self.subTest(continuous=continuous):
                receiver = Receiver("unused", continuous=continuous)
                opened = Event()

                def receive():
                    opened.set()
                    receiver.stop.wait(2)

                with patch.object(receiver, "_receive", side_effect=receive):
                    receiver.thread.start()
                    try:
                        if not continuous:
                            self.assertFalse(opened.wait(.05))
                            receiver.state()
                        self.assertTrue(opened.wait(1))
                    finally:
                        receiver.close()
