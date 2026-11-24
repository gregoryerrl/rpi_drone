"""Roll/pitch leveling model for the jig display. Angles and rates are in radians."""
import math


NATURAL_FREQUENCY = 1.0  # 1/s in the ideal, critically damped jig model
MAX_ACCELERATION = math.radians(15)  # rad/s², vector magnitude
HINT_THRESHOLD = math.radians(0.5)  # rad/s², suppress small display cues only
MAX_SAMPLE_AGE = 0.35
ROTOR_FIELDS = ("PX", "PY", "AX", "AY", "AZ")
GEOMETRY_PARAMS = ("CA_ROTOR_COUNT",) + tuple(
    f"CA_ROTOR{i}_{field}" for i in range(4) for field in ROTOR_FIELDS
)


def leveling_acceleration(roll, pitch, p, q):
    """Return the proposed body roll and pitch accelerations in rad/s²."""
    if not all(math.isfinite(v) for v in (roll, pitch, p, q)):
        raise ValueError("Invalid attitude or angular rate.")
    if abs(roll) > math.pi or abs(pitch) > math.pi / 2:
        raise ValueError("Attitude outside the expected Euler-angle range.")

    # Earth-down, expressed in forward/right/down body coordinates. The cross
    # product of body-down with earth-down supplies the shortest leveling axis.
    down_x = -math.sin(pitch)
    down_y = math.sin(roll) * math.cos(pitch)
    down_z = math.cos(roll) * math.cos(pitch)
    if down_z <= 1e-6:
        raise ValueError("Outside the upright jig range.")
    horizontal = math.hypot(down_x, down_y)
    tilt = math.atan2(horizontal, down_z)
    error_scale = tilt / horizontal if horizontal else 0.0

    gain = NATURAL_FREQUENCY ** 2 * error_scale
    damping = 2 * NATURAL_FREQUENCY
    roll_accel = -down_y * gain - p * damping
    pitch_accel = down_x * gain - q * damping
    magnitude = math.hypot(roll_accel, pitch_accel)
    if not math.isfinite(magnitude):
        raise ValueError("Invalid angular acceleration.")
    scale = MAX_ACCELERATION / max(MAX_ACCELERATION, magnitude)
    return roll_accel * scale, pitch_accel * scale


def rotor_corners(params):
    """Return logical motor corners for a vertical-thrust X quad, in motor order."""
    if "CA_ROTOR_COUNT" in params and params["CA_ROTOR_COUNT"] != 4:
        raise ValueError("Motor hints require a four-rotor layout.")
    if any(key not in params for key in GEOMETRY_PARAMS):
        raise ValueError("Motor hints are waiting for PX4 rotor geometry.")
    corners = []
    for i in range(4):
        x, y, ax, ay, az = (params[f"CA_ROTOR{i}_{field}"] for field in ROTOR_FIELDS)
        if not all(math.isfinite(v) for v in (x, y, ax, ay, az)):
            raise ValueError("PX4 reported invalid rotor geometry.")
        if max(abs(ax), abs(ay), abs(az + 1)) > 0.001:
            raise ValueError("Motor hints require upward-facing rotor axes.")
        if abs(x) < 1e-6 or abs(y) < 1e-6:
            raise ValueError("Motor hints require an X layout.")
        corners.append(("front" if x > 0 else "rear", "right" if y > 0 else "left"))
    if len(set(corners)) != 4:
        raise ValueError("PX4 does not place one motor in each corner.")
    return corners


class LevelingPreview:
    def __init__(self):
        self.parameters = {}
        self.stamp = None
        self.received = None
        self.sample = None

    def observe(self, stamp, roll, pitch, p, q, now):
        if self.stamp is not None:
            advance = (stamp - self.stamp) & 0xffffffff
            if advance == 0:
                return  # A repeated FC timestamp cannot keep a reading alive.
            if advance >= 0x80000000:
                if now - self.received > MAX_SAMPLE_AGE:
                    # A new FC clock epoch needs another advancing sample and a
                    # fresh geometry read before motor numbers can be displayed.
                    self.sample = None
                    self.parameters.clear()
                    self.stamp, self.received = stamp, now
                return
        self.stamp, self.received = stamp, now
        self.sample = roll, pitch, p, q

    def snapshot(self, now):
        if self.sample is None:
            return {"message": "Waiting for a fresh PX4 attitude sample."}
        if not 0 <= now - self.received <= MAX_SAMPLE_AGE:
            return {"message": "Attitude stream is stale."}
        roll, pitch, p, q = self.sample
        try:
            accelerations = leveling_acceleration(roll, pitch, p, q)
        except ValueError as exc:
            return {"message": str(exc)}
        try:
            corners = rotor_corners(self.parameters)
            message = "Target is level. Corrections are display only."
        except ValueError as exc:
            corners = []
            message = str(exc)

        axes = {}
        for name, angle, rate, accel, positive, negative in (
            ("roll", roll, p, accelerations[0], "left", "right"),
            ("pitch", pitch, q, accelerations[1], "front", "rear"),
        ):
            more_lift = None
            if corners:
                side = positive if accel > 0 else negative
                more_lift = [n for n, corner in enumerate(corners, 1)
                             if abs(accel) >= HINT_THRESHOLD and side in corner]
            axes[name] = {
                "angle_deg": round(math.degrees(angle), 1),
                "rate_deg_s": round(math.degrees(rate), 1),
                "correction_deg_s2": round(math.degrees(accel), 1),
                "more_lift": more_lift,
            }
        return {"message": message, "axes": axes, "sample_ms": self.stamp,
                "corners": ["-".join(corner) for corner in corners]}
