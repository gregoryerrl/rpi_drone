"""Persistent interlock for the Haras Amor browser bench service."""
import os
from pathlib import Path
import secrets


def stop_marker():
    root = Path(os.environ.get("XDG_STATE_HOME") or Path.home() / ".local/state")
    return root / "rpi_drone" / "haras-amor" / "motor-stop"


class StopLatch:
    def __init__(self, marker):
        self.marker = marker
        self.token = None
        self.storage_error = False
        if marker is not None:
            try:
                marker.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
                try:
                    marker.stat()
                except FileNotFoundError:
                    return
            except OSError:
                self.engage()
                self.storage_error = True
            else:
                self.engage()

    @property
    def stopped(self):
        return self.token is not None

    def engage(self):
        # A reset issued for an earlier stop must not clear this one.
        self.token = secrets.token_hex(16)

    def _sync_directory(self):
        fd = os.open(self.marker.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(fd)
        finally:
            os.close(fd)

    def persist(self):
        if self.marker is not None:
            try:
                self.marker.touch(mode=0o600, exist_ok=True)
                self._sync_directory()
                self.storage_error = False
            except OSError:
                self.storage_error = True  # Keep the in-memory inhibit engaged.

    def reset(self, token):
        if not self.stopped or not isinstance(token, str) or token != self.token:
            return False
        if self.marker is not None:
            try:
                self.marker.unlink(missing_ok=True)
                self._sync_directory()
            except OSError:
                # If unlink succeeded but the directory sync failed, restore
                # the marker before reporting that the reset was refused.
                self.persist()
                self.storage_error = True
                raise
        self.token = None
        self.storage_error = False
        return True
