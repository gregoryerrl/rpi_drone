"""Keep camera capture independent of the pan-tilt board's I²C connection."""
from dataclasses import dataclass
import sys
import time

import numpy as np
from .research import FPS, SIZE, PiRig, project


@dataclass
class HeldPose:
    pan: float
    tilt: float

    def pose_at(self, stamp):
        return self.pan, self.tilt


class RadioRig(PiRig):
    def __init__(self, start_pose):
        from picamera2 import Picamera2
        with project("pan-tilt-face-tracking"):
            import follow_me
            from servo_test import MODE1

        self.driver = follow_me
        self.mode_register = MODE1
        self.rotated = follow_me.ROTATION_FLAG.is_file()
        self.camera_image = follow_me.camera_image
        self.limits = np.array([follow_me.PAN_RANGE, follow_me.TILT_RANGE]).T
        self.manual_limits = self.limits
        self.catchup_speed = follow_me.MAX_SPEED
        self.board = None
        self.mount = HeldPose(*np.clip(start_pose, *self.limits))
        self.next_probe = 0.0
        self.camera = Picamera2()
        try:
            config = self.camera.create_video_configuration(
                main={"size": SIZE, "format": "RGB888"}, controls={"FrameRate": FPS},
                buffer_count=3, queue=True)
            self.camera.configure(config)
            self.camera.start()
        except Exception:
            self.camera.close()
            raise

    @property
    def mount_ready(self):
        return self.board is not None

    def _drop_board(self, exc):
        self.mount = HeldPose(*self.position)
        if self.board is not None:
            self.board.bus.close()
            self.board = None
            print(f"Pan-tilt I²C disconnected: {exc}", file=sys.stderr, flush=True)
        self.next_probe = time.monotonic() + 1.0

    def _probe(self):
        from smbus2 import SMBus
        if time.monotonic() < self.next_probe:
            return
        self.next_probe = time.monotonic() + 1.0
        try:
            if self.board is not None:
                if self.board.bus.read_byte_data(self.board.addr, self.mode_register) & 0x10:
                    self._drop_board("PWM board reset")
                return
            with SMBus(1) as probe:
                probe.read_byte_data(0x40, self.mode_register)
            self.board = self.driver.RobustPCA()
            self.mount = self.driver.Servos(self.board, *self.position)
            print("Pan-tilt I²C connected.", flush=True)
        except OSError as exc:
            self._drop_board(exc)

    def read(self):
        self._probe()
        return super().read()

    def move(self, position):
        if self.board is not None:
            try:
                super().move(position)
            except OSError as exc:
                self._drop_board(exc)

    def close(self):
        try:
            if self.board is not None:
                try:
                    self.board.close()
                except OSError:
                    self.board.bus.close()
        finally:
            self.camera.close()
