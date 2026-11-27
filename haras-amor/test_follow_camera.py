import json
import time
import unittest

import cv2
import numpy as np

import math
from pathlib import Path
import tempfile
import threading

from follow_camera.camera import (BOX, CENTER, DEFAULT_FOCAL as FOCAL, DEFAULT_MOUNT, FollowCamera, app,
                                  attitude_at, centring_attitude, load_mount, picture_offset, ray,
                                  rotation_shift)


class Scene:
    """Synthetic 24 fps video of a textured object on a plain background."""

    def __init__(self):
        self.object = np.random.default_rng(7).integers(0, 256, (48, 48, 3), dtype=np.uint8)
        self.x, self.y = (int(v) for v in CENTER)
        self.hidden = False

    def frames(self, stop):
        while not stop.is_set():
            image = np.full((360, 640, 3), 90, np.uint8)
            if not self.hidden:
                image[self.y - 24:self.y + 24, self.x - 24:self.x + 24] = self.object
            yield image, time.monotonic()
            stop.wait(1 / 24)


class FollowCameraTests(unittest.TestCase):
    def setUp(self):
        self.scene = Scene()
        self.lines = []
        # The default mount keeps these tests independent of a calibration saved on this Pi.
        self.camera = FollowCamera(self.scene.frames, say=self.lines.append, mount=DEFAULT_MOUNT)
        self.camera.start()
        self.addCleanup(self.camera.close)

    def until(self, predicate, timeout=3):
        end = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() > end:
                self.fail(f"timed out; camera state {self.camera.state()}")
            time.sleep(0.02)

    def switch(self, ch8):
        """Set CH8 and wait for a camera frame processed with it."""
        self.camera.radio(ch8)
        self.until(lambda: self.camera.state()["mode"] != "Camera off"
                   and self.camera.state()["ch8"] == ch8)

    def mode(self):
        return self.camera.state()["mode"]

    def lock(self):
        self.switch(1000)
        self.switch(1500)
        self.until(lambda: self.mode() == "Locked")

    def test_middle_locks_the_box_and_high_follows_the_object(self):
        self.lock()
        for centred in self.camera.offset():
            self.assertAlmostEqual(centred, 0, places=1)
        for _ in range(15):  # walk the object 60 px right and 30 px up
            self.scene.x += 4
            self.scene.y -= 2
            time.sleep(1 / 24)
        self.until(lambda: (self.camera.offset() or (0, 0))[0] > 0.15)
        self.until(lambda: self.camera.offset()[1] < -0.1)  # 30 of the 180 px half-height
        self.switch(2000)
        self.assertEqual(self.mode(), "Following")
        self.switch(1000)
        self.assertEqual(self.mode(), "Unlocked")
        self.assertIsNone(self.camera.offset())
        for mode in ("Locking", "Locked", "Following", "Unlocked"):
            self.assertIn(f"Camera: {mode}", self.lines)

    def test_target_attitude_centres_the_object(self):
        self.lock()
        for _ in range(10):  # the object moves 40 px right and 20 px up in the picture
            self.scene.x += 4
            self.scene.y -= 2
            time.sleep(1 / 24)
        self.until(lambda: (self.camera.target() or (0, 0))[0] < -math.atan(30 / FOCAL))
        roll, pitch = self.camera.target()
        self.assertAlmostEqual(roll, -math.atan(40 / FOCAL), delta=0.02)  # right: roll left
        self.assertAlmostEqual(pitch, -math.atan(20 / FOCAL), delta=0.02)  # top: nose down

    def test_motion_reports_how_fast_the_centring_angle_changes(self):
        self.lock()
        for _ in range(12):  # 6 px right every 1/24 s: 144 px/s
            self.scene.x += 6
            time.sleep(1 / 24)
        roll_rate, pitch_rate = self.camera.motion()
        self.assertLess(roll_rate, -0.3)  # right in the picture: the roll target moves left
        self.assertAlmostEqual(pitch_rate, 0, delta=0.1)
        time.sleep(0.6)  # the object stops
        self.assertAlmostEqual(self.camera.motion()[0], 0, delta=0.1)

    def test_tilting_the_drone_keeps_the_lock(self):
        self.lock()
        # Roll right step by step. The fixed upward camera sees the still object slide right;
        # told the attitude, the tracker keeps it, and the centring target stays level.
        for step in range(1, 13):
            roll = math.radians(1.5 * step)
            self.camera.attitude(roll, 0.0)
            self.scene.x = int(CENTER[0] + FOCAL * math.tan(roll))
            time.sleep(1 / 12)
        for _ in range(4):  # the flight controller keeps reporting the attitude
            self.camera.attitude(roll, 0.0)
            time.sleep(0.05)
        self.assertEqual(self.mode(), "Locked")
        self.assertGreater(self.camera.offset()[0], 0.25)  # it really moved in the picture
        roll, pitch = self.camera.target()
        self.assertAlmostEqual(roll, 0, delta=0.02)
        self.assertAlmostEqual(pitch, 0, delta=0.02)

    def test_large_tilts_keep_the_lock_and_the_exact_target(self):
        tilt = [0.0]
        streaming = threading.Event()

        def stream():  # the flight controller's attitude, as ch8_takeover passes it on
            while not streaming.is_set():
                self.camera.attitude(tilt[0], 0.0)
                streaming.wait(0.01)

        feeder = threading.Thread(target=stream, daemon=True)
        feeder.start()
        self.addCleanup(feeder.join)
        self.addCleanup(streaming.set)
        side = math.radians(20)  # the object is 20 degrees to the drone's right of straight up

        def roll_to(degrees):
            tilt[0] = math.radians(degrees)
            # The drone's right is the picture's left; the camera sees it (side - roll) away.
            self.scene.x = int(round(CENTER[0] - FOCAL * math.tan(side - tilt[0])))

        roll_to(20)  # leaning toward it, the drone sees it in the middle and locks it
        self.lock()
        for degrees in range(22, 42, 2):  # out to 40 degrees: 20 degrees past the object
            roll_to(degrees)
            time.sleep(1 / 12)
        time.sleep(0.2)
        self.assertEqual(self.mode(), "Locked")
        # Adding tangents would say 25 degrees here; the exact geometry keeps 20.
        self.assertAlmostEqual(math.degrees(self.camera.target()[0]), 20, delta=1.0)
        for degrees in range(38, -2, -2):  # and back to level
            roll_to(degrees)
            time.sleep(1 / 12)
        time.sleep(0.2)
        self.assertEqual(self.mode(), "Locked")
        self.assertLess(self.camera.offset()[0], -0.25)  # level, it sits well left in the picture
        roll, pitch = self.camera.target()
        self.assertAlmostEqual(math.degrees(roll), 20, delta=1.0)
        self.assertAlmostEqual(pitch, 0, delta=0.02)

    def test_an_uncertain_match_holds_the_last_target_until_the_object_is_lost(self):
        self.lock()
        for _ in range(10):  # 40 px right: a left-roll target
            self.scene.x += 4
            time.sleep(1 / 24)
        self.until(lambda: (self.camera.target() or (0, 0))[0] < -math.atan(30 / FOCAL))
        before = self.camera.target()
        self.scene.hidden = True
        self.until(lambda: self.mode() == "Holding")
        held = self.camera.target()
        self.assertIsNotNone(held)  # the Pi keeps its tilt instead of easing to level
        self.assertAlmostEqual(held[0], before[0], delta=0.01)
        self.assertEqual(self.camera.motion(), (0.0, 0.0))
        self.until(lambda: self.mode() == "Lost")
        self.assertIsNone(self.camera.target())

    def test_lock_needs_ch8_to_come_up_from_low(self):
        self.switch(1500)  # already in the middle when the camera starts
        time.sleep(0.3)
        self.assertEqual(self.mode(), "Unlocked")
        self.assertIn("Lower CH8", self.camera.state()["message"])
        self.lock()

    def test_radio_loss_unlocks_until_ch8_is_cycled(self):
        self.lock()
        self.camera.radio(None)
        self.until(lambda: self.mode() == "Unlocked")
        self.assertIsNone(self.camera.offset())
        self.switch(1500)  # the radio returns in the middle: still unlocked
        time.sleep(0.3)
        self.assertEqual(self.mode(), "Unlocked")
        self.lock()

    def test_preview_draws_the_centre_box(self):
        self.switch(1000)
        data, state = self.camera.jpeg()
        self.assertEqual(state["box"], BOX)
        picture = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        x, y, r = int(CENTER[0]), int(CENTER[1]), BOX // 2
        self.assertEqual(picture.shape, (360, 640, 3))
        self.assertGreater(picture[y - r, x].mean(), 160)   # box edge over the grey background
        self.assertLess(picture[y - r - 20, x].mean(), 120)  # plain background

    def test_website_serves_the_page_status_and_live_frames(self):
        self.switch(1000)
        self.camera.vehicle(dict(armed=False, mode="Acro", control=None))
        self.until(lambda: self.camera.state()["fps"])  # the rate needs two frames
        client = app(self.camera).test_client()
        page = client.get("/")
        self.assertEqual(page.status_code, 200)
        self.assertIn(b"<title>Haras Amor camera</title>", page.data)
        state = client.get("/api/state").get_json()
        self.assertEqual((state["mode"], state["size"]), ("Unlocked", [640, 360]))
        self.assertGreater(state["fps"], 10)
        frame = client.get("/frame.jpg")
        self.assertEqual(frame.mimetype, "image/jpeg")
        info = json.loads(frame.headers["X-Camera"])
        self.assertEqual(info["ch8"], 1000)
        self.assertEqual(info["drone"], dict(armed=False, mode="Acro", control=None))
        self.assertLess(info["age_ms"], 1000)


QUARTER_TURN = np.array([[0.0, 300.0], [-300.0, 0.0]])
SKEWED = np.array([[-420.0, 15.0], [10.0, 410.0]])


class MountTests(unittest.TestCase):
    def test_ray_and_picture_offset_are_inverses(self):
        for mount in (DEFAULT_MOUNT, QUARTER_TURN, SKEWED):
            for offset in ((0.0, 0.0), (300.0, -170.0), (-120.0, 90.0)):
                np.testing.assert_allclose(picture_offset(mount, ray(mount, offset)), offset, atol=1e-9)

    def test_centring_attitude_puts_the_point_in_the_middle_for_any_mount_and_tilt(self):
        offset = np.array([55.0, -30.0])
        for mount in (DEFAULT_MOUNT, QUARTER_TURN, SKEWED):
            for tilt in ((0.0, 0.0), (0.6, -0.3), (-0.8, 0.5)):
                target = centring_attitude(mount, offset, *tilt)
                # Turned from this tilt to the target, the drone sees the point in the middle.
                np.testing.assert_allclose(rotation_shift(mount, offset, tilt, target), -offset, atol=1e-6)

    def test_centring_adds_angles_exactly_at_large_tilts(self):
        # Rolled 20 degrees right, the drone sees the object 30 degrees further right (the
        # picture's left). Centring it takes 50 degrees; adding tangents would give only 43.
        offset = (-FOCAL * math.tan(math.radians(30)), 0.0)
        roll, pitch = centring_attitude(DEFAULT_MOUNT, offset, math.radians(20), 0.0)
        self.assertAlmostEqual(math.degrees(roll), 50, places=6)
        self.assertAlmostEqual(pitch, 0, places=9)

    def test_attitude_is_looked_up_at_the_frame_time(self):
        history = [(10.0, 0.0, 0.2), (10.01, 0.1, 0.1)]
        np.testing.assert_allclose(attitude_at(history, 10.004), (0.04, 0.16))  # between samples
        self.assertEqual(attitude_at(history, 10.2), (0.1, 0.1))  # after the newest: hold it
        self.assertIsNone(attitude_at(history, 10.5))  # too far from any sample to trust
        self.assertIsNone(attitude_at([], 10.0))

    def test_default_mount_is_the_upward_camera_with_the_nose_at_the_top(self):
        roll, pitch = centring_attitude(DEFAULT_MOUNT, (40.0, -20.0))  # right of and above the centre
        self.assertLess(roll, 0)   # roll left
        self.assertLess(pitch, 0)  # nose down

    def test_load_mount_uses_a_calibration_and_falls_back_to_the_default(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "camera-mount.json"
            np.testing.assert_array_equal(load_mount(path), DEFAULT_MOUNT)  # none yet
            path.write_text(json.dumps({"roll": [-410.0, 3.0], "pitch": [0.0, -410.0]}))
            np.testing.assert_allclose(load_mount(path), [[-410.0, 0.0], [3.0, -410.0]])
            path.write_text("not json")
            np.testing.assert_array_equal(load_mount(path), DEFAULT_MOUNT)


class CameraOffTests(unittest.TestCase):
    def test_a_failed_camera_shows_as_off(self):
        def frames(stop):
            raise RuntimeError("no camera")
            yield  # makes this a generator, like the real camera

        camera = FollowCamera(frames, say=lambda line: None, mount=DEFAULT_MOUNT)
        camera.start()
        self.addCleanup(camera.close)
        end = time.monotonic() + 2
        while "no camera" not in camera.state()["message"] and time.monotonic() < end:
            time.sleep(0.02)
        client = app(camera).test_client()
        self.assertEqual(client.get("/api/state").get_json()["mode"], "Camera off")
        frame = client.get("/frame.jpg")
        self.assertEqual(frame.status_code, 503)
        self.assertIn("no camera", frame.get_json()["error"])


if __name__ == "__main__":
    unittest.main()
