"""Load the pinned camera projects, which expose modules from their own directories."""
from contextlib import contextmanager
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1] / "research" / "cv-research"


@contextmanager
def project(name):
    path = ROOT / name
    if not path.is_dir():
        raise ImportError("Initialize the camera dependency with git submodule update --init --recursive.")
    sys.path.insert(0, str(path))
    try:
        yield
    finally:
        sys.path.remove(str(path))


with project("pixel-lock-follow"):
    from live_trial import CENTER, FPS, MAX_GAP, MANUAL_SPEED, SIZE, LiveTrial, PiRig
    from tracker import PixelLock
