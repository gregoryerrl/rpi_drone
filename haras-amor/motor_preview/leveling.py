"""Roll/pitch leveling model for the jig display. Angles and rates are in radians."""
import math


NATURAL_FREQUENCY = 1.0  # 1/s in the ideal, critically damped jig model
MAX_ACCELERATION = math.radians(15)  # rad/s², vector magnitude
TRIAL_TILT_DEG = 5
HINT_THRESHOLD = math.radians(0.5)  # rad/s², suppress small display cues only
MAX_SAMPLE_AGE = 0.35
ROTOR_FIELDS = ("PX", "PY", "AX", "AY", "AZ")
GEOMETRY_PARAMS = ("CA_ROTOR_COUNT",) + tuple(
    f"CA_ROTOR{i}_{field}" for i in range(4) for field in ROTOR_FIELDS
)


def down_vector(roll, pitch):
    """Earth-down expressed in the forward/right/down body frame."""
    return -math.sin(pitch), math.sin(roll) * math.cos(pitch), math.cos(roll) * math.cos(pitch)


def validate_trial_target(roll_deg, pitch_deg):
    for angle in (roll_deg, pitch_deg):
        if type(angle) not in (int, float) or not -TRIAL_TILT_DEG <= angle <= TRIAL_TILT_DEG:
            raise ValueError(f"Target angles must be between −{TRIAL_TILT_DEG}° and +{TRIAL_TILT_DEG}°.")
    down_z = down_vector(math.radians(roll_deg), math.radians(pitch_deg))[2]
    if down_z < math.cos(math.radians(TRIAL_TILT_DEG)) - 1e-12:
        raise ValueError(f"Combined target tilt exceeds {TRIAL_TILT_DEG}°.")


def leveling_acceleration(roll, pitch, p, q, target_roll=0.0, target_pitch=0.0):
    """Return the proposed body roll and pitch accelerations in rad/s²."""
    if not all(math.isfinite(v) for v in (roll, pitch, p, q, target_roll, target_pitch)):
        raise ValueError("Invalid attitude or angular rate.")
    if abs(roll) > math.pi or abs(pitch) > math.pi / 2:
        raise ValueError("Attitude outside the expected Euler-angle range.")

    down_x, down_y, down_z = down_vector(roll, pitch)
    if down_z <= 1e-6:
        raise ValueError("Outside the upright jig range.")
    tx, ty, tz = down_vector(target_roll, target_pitch)
    # Cross and dot products give the axis and angle between the target and
    # measured gravity directions. Yaw is free on the two-axis jig.
    axis_x, axis_y = ty * down_z - tz * down_y, tz * down_x - tx * down_z
    axis_z = tx * down_y - ty * down_x
    sine = math.hypot(axis_x, axis_y, axis_z)
    cosine = tx * down_x + ty * down_y + tz * down_z
    error_scale = math.atan2(sine, cosine) / sine if sine else 0.0

    gain = NATURAL_FREQUENCY ** 2 * error_scale
    damping = 2 * NATURAL_FREQUENCY
    roll_accel = axis_x * gain - p * damping
    pitch_accel = axis_y * gain - q * damping
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

    def snapshot(self, now, target_roll=0, target_pitch=0):
        if self.sample is None:
            return {"message": "Waiting for a fresh PX4 attitude sample."}
        if not 0 <= now - self.received <= MAX_SAMPLE_AGE:
            return {"message": "Attitude stream is stale."}
        roll, pitch, p, q = self.sample
        try:
            accelerations = leveling_acceleration(roll, pitch, p, q, math.radians(target_roll),
                                                 math.radians(target_pitch))
        except ValueError as exc:
            return {"message": str(exc)}
        try:
            corners = rotor_corners(self.parameters)
            message = "Live attitude. The correction column shows the jig model estimate."
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
