"""CH8 handover from your manual control to the Pi on an armed PX4, with camera follow.

Start this script, then arm with CH5 while CH6 and CH8 are low. You control the motors with
the TX16S. While you stay armed with CH6 low, the Pi streams an Offboard setpoint. PX4 uses CH8
as its Offboard switch (RC_MAP_OFFB_SW=8), so CH8 high hands the motors to the Pi and CH8 low
gives them straight back to you, even if the Pi hangs. CH6 still kills the motors.

Two settings decide the thrust:
  stand (default)   for the roll stand with the frame strapped down. 20% collective thrust
                    (--thrust) with the locked object straight above, rising smoothly to 50%
                    (--max-thrust, at most THRUST_CAP) the further the drone has to lean to
                    centre it, or the further it still is from that lean. At 20% a motor can only
                    swing between idle and 40%; near 50% PX4 has its most leaning force, and the
                    drone pushes about its own weight against the stand.
  flight (--flight) the Pi holds the height the drone had at the handover, starting from the
                    thrust you were flying with. It only offers control while you fly in
                    Stabilized or Altitude, PX4 reports the drone in the air and it is below
                    FLIGHT_CEILING. It gives control back if the height drifts HEIGHT_LIMIT from
                    the hold or PX4's height data stops. Keep your throttle stick at hover while
                    the Pi flies: taking control back returns your stick's thrust at once.

In both settings the setpoint is level (0 degrees roll and pitch) at the heading the drone had
when CH8 went high, and PX4's attitude controller does the levelling.

The follow camera (follow_camera/, website on port 8004) shows a box in the middle of the
aircraft camera, which looks straight up with the drone's nose at the top of the picture.
Moving CH8 from low to the middle locks the object in the box. During the handover the camera
works out the roll and pitch that would put the locked object in the middle of the picture. The
Pi aims there, plus FOLLOW_DAMPING times how fast that angle is changing, so a flying drone
settles under the object instead of swinging past it. The tilt changes at up to TILT_RATE, never
passes the tilt limit (--max-tilt, MAX_TILT by default) combined, and eases back to level when
nothing is locked. A larger limit lets the drone lean further to bring an object far from the
centre back to the middle. On the stand the limit goes up to TILT_CAP, 90 degrees, which in
practice caps nothing. The flight setting stops at FLIGHT_TILT_CAP, because past about 48 degrees
the height hold's 75% thrust ceiling can no longer carry a 50% hover. The camera's mount
geometry lives in follow_camera/camera.py.

The Pi withdraws its setpoint when CH5 drops, CH6 leaves low, the radio or attitude data
stops, the drone tilts past 30 degrees, PX4 disarms, the stand setting sees more than 1.5 m of
climb since arming, or you press Ctrl-C. If PX4 is in Offboard at that moment, the Pi asks it
to return to your mode. After a withdrawal, CH8 must be low again before the Pi offers control.
PX4 refuses to disarm in Offboard while it thinks it is flying, so lower CH8 before CH5, or use
CH6.

Run venv/bin/python ch8_takeover.py [--flight] from haras-amor, or let the ch8-takeover user
service (ch8-takeover.service, stand setting) run it from boot. Either way it holds the FC's
USB port and the camera; --no-camera leaves the camera free. Without the FC it retries every 5 s.
"""
import argparse
import math
import signal
import struct
import threading
import time

from pymavlink.dialects.v20 import common as mav
from serial import Serial

DEVICE = "/dev/serial/by-id/usb-MicoAir_MicoAir743v2_0-if00"
THRUST = 0.2               # stand setting: base thrust, with the locked object straight above
STAND_MAX_THRUST = 0.5     # stand setting: top thrust, for a big lean or a lean still to make
THRUST_CAP = 0.5           # most leaning force: every motor can go fully up or down from here
FULL_THRUST_LEAN = math.radians(30)    # stand setting: the lean that takes the top thrust...
FULL_THRUST_BEHIND = math.radians(15)  # ...or how far behind its lean the drone is
THRUST_RATE = 1.0          # stand setting: fastest thrust change, per second (20% to 50% in 0.3 s)
SETPOINT_PERIOD = 0.05     # 20 Hz; PX4 leaves Offboard if setpoints stop for COM_OF_LOSS_T
RADIO_TIMEOUT = 0.25       # same freshness window as ch8_bench.py
HEARTBEAT_TIMEOUT = 1.6
ALTITUDE_TIMEOUT = 1.0
ATTITUDE_TIMEOUT = 0.5
POSITION_TIMEOUT = 0.5     # PX4 height and climb rate
THRUST_TIMEOUT = 0.5       # PX4's current thrust, read before a flight handover
CLIMB_LIMIT = 1.5          # stand setting: metres of barometric climb since arming
TILT_RATE = math.radians(180)  # fastest change of the Pi's roll and pitch setpoint, per second;
                               # close to PX4's own 220 deg/s roll rate, so it adds little delay
MAX_TILT = math.radians(10)    # default largest combined roll and pitch the Pi asks for
TILT_CAP_DEGREES = 90          # largest --max-tilt on the stand: in practice no cap at all
TILT_CAP = math.radians(TILT_CAP_DEGREES)
FLIGHT_TILT_CAP_DEGREES = 45   # flight setting: past this the drone cannot hold its height
FLIGHT_TILT_CAP = math.radians(FLIGHT_TILT_CAP_DEGREES)
TILT_LIMIT = math.radians(30)  # measured roll or pitch at which the Pi withdraws...
TILT_MARGIN = math.radians(15)  # ...or this much past the tilt limit, whichever is larger
FOLLOW_DAMPING = 0.5           # s: extra tilt per rad/s of change in the object's angle
# Flight setting: height hold, as an upward acceleration turned into thrust around hover.
G = 9.81
HOLD_OMEGA, HOLD_ZETA = 1.0, 0.9   # rad/s and damping ratio of the height hold
MAX_VERTICAL_ACCEL = 2.0           # m/s^2 the height hold may ask for
HOVER_LEARN = 0.02                 # hover-thrust correction per metre-second below the hold
THRUST_RANGE = (0.10, 0.75)        # collective thrust limits while the Pi holds height
HEIGHT_LIMIT = 1.0                 # metres from the hold at which the Pi gives control back
FLIGHT_CEILING = 3.0               # metres above the arming point; no flight handover above it
ASSISTED = (2, 7)                  # PX4 Altitude and Stabilized: the modes a flight handover follows
LOW, HIGH = 1250, 1850     # same switch thresholds as ch8_bench.py
SWITCH_ON = 1750           # PX4's 0.75 threshold for its Offboard and kill switches
OFFBOARD = 6               # PX4 custom main mode
# Attitude setpoint: PX4 uses the quaternion and thrust, not body rates.
RATES_IGNORED = (mav.ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE
                 | mav.ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE
                 | mav.ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE)
MODES = {1: "Manual", 2: "Altitude", 3: "Position", 4: "Auto", 5: "Acro", 6: "Offboard", 7: "Stabilized"}


def say(text):
    print(text, flush=True)


def mode_name(mode):
    return MODES.get(mode[0], f"mode {mode[0]}")


def clamp(value, low, high):
    return max(low, min(high, value))


def quaternion(roll, pitch, yaw):
    """Attitude quaternion [w, x, y, z] for these Euler angles (ZYX order, as PX4 uses)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return [cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy]


class SetupError(Exception):
    """The flight controller is missing or not set up for the handover."""


class Takeover:
    def __init__(self, device=DEVICE, say=say, camera=None, flight=False, max_tilt=MAX_TILT,
                 thrust=THRUST, max_thrust=STAND_MAX_THRUST):
        cap, cap_degrees = (FLIGHT_TILT_CAP, FLIGHT_TILT_CAP_DEGREES) if flight else (TILT_CAP, TILT_CAP_DEGREES)
        if not 0 < max_tilt <= cap:
            raise ValueError(f"the tilt limit must be above 0 and at most {cap_degrees} degrees"
                             + (" in the flight setting" if flight else ""))
        if not 0.05 <= thrust <= max_thrust <= THRUST_CAP:
            raise ValueError(f"the stand thrust must be between 5% and {THRUST_CAP:.0%}, "
                             "with the base no higher than the top")
        self.stand_thrust, self.stand_max_thrust = thrust, max_thrust
        self.say, self.camera, self.flight = say, camera, flight
        self.max_tilt, self.tilt_limit = max_tilt, max(TILT_LIMIT, max_tilt + TILT_MARGIN)
        self.port = Serial(device, 57600, timeout=0.01, write_timeout=0.05, exclusive=True)
        try:
            self.port.reset_input_buffer()
            self.wire = mav.MAVLink(self.port, srcSystem=255, srcComponent=190)
            self.wire.robust_parsing = True
        except BaseException:
            self.port.close()
            raise
        self.system = self.armed = self.mode = self.manual_mode = None
        self.session = self.ch8_low_seen = self.offering = False
        self.heartbeat_time = self.radio_time = self.altitude_time = self.attitude_time = -math.inf
        self.position_time = self.thrust_time = -math.inf
        self.ch5 = self.ch6 = self.ch8 = self.stamp = None
        self.altitude = self.arm_altitude = self.outputs = None
        self.roll = self.pitch = self.yaw = self.held_yaw = None
        self.z = self.vz = self.px4_thrust = self.landed_state = None
        self.tilt = (0.0, 0.0)    # roll and pitch the Pi asks for while it has the motors
        self.hold_z = self.hover = None  # flight setting: height (PX4 z, down) and hover thrust
        self.thrust = thrust
        self.waiting = None       # why the Pi is not offering control, as last reported
        self.next_setpoint = self.next_report = 0.0
        self.params = {}

    def pump(self):
        data = self.port.read(4096)
        now = time.monotonic()
        for msg in self.wire.parse_buffer(data) or ():
            self.observe(msg, now)

    def observe(self, msg, now):
        kind = msg.get_type()
        if kind == "BAD_DATA" or msg.get_srcComponent() != mav.MAV_COMP_ID_AUTOPILOT1:
            return
        if (self.system is None and kind == "HEARTBEAT" and msg.autopilot == mav.MAV_AUTOPILOT_PX4
                and msg.type == mav.MAV_TYPE_QUADROTOR):
            self.system = msg.get_srcSystem()
        if msg.get_srcSystem() != self.system:
            return
        if kind == "HEARTBEAT":
            self.heartbeat(msg, now)
        elif kind == "RC_CHANNELS":
            self.radio(msg, now)
        elif kind == "ALTITUDE" and math.isfinite(msg.altitude_monotonic):
            self.altitude, self.altitude_time = msg.altitude_monotonic, now
            if self.session and self.arm_altitude is None:
                self.arm_altitude = self.altitude
        elif kind == "ATTITUDE" and all(map(math.isfinite, (msg.roll, msg.pitch, msg.yaw))):
            self.roll, self.pitch, self.yaw, self.attitude_time = msg.roll, msg.pitch, msg.yaw, now
        elif kind == "LOCAL_POSITION_NED" and math.isfinite(msg.z) and math.isfinite(msg.vz):
            self.z, self.vz, self.position_time = msg.z, msg.vz, now
        elif kind == "ATTITUDE_TARGET" and math.isfinite(msg.thrust):
            # Your thrust while you fly; the Pi's own setpoint once it has the motors.
            self.px4_thrust, self.thrust_time = msg.thrust, now
        elif kind == "EXTENDED_SYS_STATE":
            self.landed_state = msg.landed_state
        elif kind == "SERVO_OUTPUT_RAW" and msg.port == 0:
            self.outputs = (msg.servo1_raw, msg.servo2_raw, msg.servo3_raw, msg.servo4_raw)
        elif kind == "PARAM_VALUE":
            value = msg.param_value
            if msg.param_type == mav.MAV_PARAM_TYPE_INT32:
                # PX4 encodes integers byte-for-byte in the MAVLink float field.
                value, = struct.unpack("<i", struct.pack("<f", value))
            self.params[msg.param_id] = value
        elif kind == "STATUSTEXT":
            self.say(f"PX4: {msg.text}")

    def heartbeat(self, msg, now):
        armed = bool(msg.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED)
        mode = (msg.custom_mode >> 16 & 0xFF, msg.custom_mode >> 24 & 0xFF)
        if armed and self.armed is False:
            # Only an arming seen by this script starts a session, with its own climb reference.
            self.session, self.ch8_low_seen = True, False
            self.arm_altitude = self.altitude if now - self.altitude_time <= ALTITUDE_TIMEOUT else None
            self.say("PX4 armed. You have the motors.")
        elif self.armed and not armed:
            self.session = False
            self.say("PX4 disarmed.")
        if self.mode is not None and mode != self.mode:
            self.say(f"PX4 mode: {mode_name(mode)}")
        if armed and mode[0] != OFFBOARD:
            self.manual_mode = mode
        self.armed, self.mode, self.heartbeat_time = armed, mode, now

    def radio(self, msg, now):
        switches = (msg.chan5_raw, msg.chan6_raw, msg.chan8_raw)
        advancing = self.stamp is None or 0 < (msg.time_boot_ms - self.stamp) % (1 << 32) < 1 << 31
        self.stamp = msg.time_boot_ms
        if msg.chancount >= 8 and msg.rssi != 0 and advancing and all(800 <= v <= 2200 for v in switches):
            self.ch5, self.ch6, self.ch8 = switches
            self.radio_time = now
        else:
            self.radio_time = -math.inf

    def problem(self, now):
        """Why the Pi must not offer its setpoint now, or None."""
        if now - self.heartbeat_time > HEARTBEAT_TIMEOUT:
            return "PX4 heartbeat lost"
        if not self.armed:
            return "PX4 disarmed"
        if not self.session:
            return "disarm and arm again to start a session"
        if now - self.radio_time > RADIO_TIMEOUT:
            return "radio data stopped"
        if self.ch5 < HIGH:
            return "CH5 low"
        if self.ch6 > LOW:
            return "CH6 raised"
        if now - self.attitude_time > ATTITUDE_TIMEOUT:
            return "no attitude data"
        if max(abs(self.roll), abs(self.pitch)) > self.tilt_limit:
            return f"tilted past {math.degrees(self.tilt_limit):.0f} degrees"
        if self.arm_altitude is None or now - self.altitude_time > ALTITUDE_TIMEOUT:
            return "no barometer data"
        climb = self.altitude - self.arm_altitude
        if self.flight:
            return self.flight_problem(now, climb)
        if climb > CLIMB_LIMIT:
            self.session = False  # A climb ends a stand session until PX4 is armed again.
            return f"barometer shows {climb:.1f} m of climb since arming"
        return None

    def flight_problem(self, now, climb):
        """Extra conditions for the flight setting, or None."""
        if climb > FLIGHT_CEILING:
            return f"above the {FLIGHT_CEILING:g} m test ceiling"
        if now - self.position_time > POSITION_TIMEOUT:
            return "no height data from PX4"
        if self.hold_z is not None:  # the Pi is holding height
            if abs(self.z - self.hold_z) > HEIGHT_LIMIT:
                return f"height drifted {abs(self.z - self.hold_z):.1f} m from the hold"
            return None
        if self.manual_mode is None or self.manual_mode[0] not in ASSISTED:
            return "fly in Stabilized or Altitude for a flight handover"
        if self.landed_state != mav.MAV_LANDED_STATE_IN_AIR:
            return "PX4 does not report the drone in the air"
        if now - self.thrust_time > THRUST_TIMEOUT:
            return "no thrust reading from PX4"
        return None

    def step(self, now):
        if self.camera is not None:
            self.camera.radio(self.ch8 if now - self.radio_time <= RADIO_TIMEOUT else None)
            fresh = now - self.attitude_time <= ATTITUDE_TIMEOUT
            self.camera.attitude(self.roll if fresh else None, self.pitch if fresh else None,
                                 self.attitude_time)
        problem = self.problem(now)
        if problem != self.waiting:
            self.waiting = problem
            if problem and self.armed and self.session and not self.offering:
                self.say(f"Pi waiting: {problem}.")
        if problem:
            self.ch8_low_seen = False
        elif self.ch8 <= LOW:
            self.ch8_low_seen = True
        offer = problem is None and self.ch8_low_seen
        if offer and not self.offering:
            self.offering = True
            lift = ("level, holding this height" if self.flight else f"level, {self.stand_thrust:.0%} thrust"
                    + ("" if self.stand_max_thrust == self.stand_thrust
                       else f", up to {self.stand_max_thrust:.0%} to lean"))
            self.say(f"Pi ready: CH8 high hands the motors to the Pi ({lift}), CH8 low gives them back.")
        elif self.offering and not offer:
            self.release(problem)
        if self.offering and now >= self.next_setpoint:
            self.next_setpoint = now + SETPOINT_PERIOD
            # Follow the drone's heading until CH8 hands over, then hold that heading. While the
            # Pi has the motors it tilts toward the camera's locked object, or back to level.
            if self.ch8 <= SWITCH_ON:
                self.held_yaw, self.tilt, self.hold_z = None, (0.0, 0.0), None
            elif self.held_yaw is None:
                self.held_yaw = self.yaw
                if self.flight:  # start from your height and thrust: no jump at the handover
                    self.hold_z, self.hover = self.z, clamp(self.px4_thrust, *THRUST_RANGE)
            else:
                self.follow()
            self.thrust = self.lift() if self.flight else self.stand_lift()
            yaw = self.yaw if self.held_yaw is None else self.held_yaw
            self.wire.set_attitude_target_send(int(now * 1000) & 0xFFFFFFFF, self.system, 1,
                                               RATES_IGNORED, quaternion(*self.tilt, yaw),
                                               0, 0, 0, self.thrust)
        if self.camera is not None:  # after this step's decisions, so the website matches them
            self.camera.vehicle(self.vehicle(now))
        if self.armed and now >= self.next_report:
            self.next_report = now + 1
            who = "you"
            if self.mode[0] == OFFBOARD:
                who = "Pi at {:.0%} thrust, target roll {:+.1f}° pitch {:+.1f}°".format(
                    self.thrust, *map(math.degrees, self.tilt))
                if self.hold_z is not None:
                    who += f", {self.z - self.hold_z:+.2f} m below the hold"
            tilt = ("" if self.roll is None else
                    f", roll {math.degrees(self.roll):+.1f}° pitch {math.degrees(self.pitch):+.1f}°")
            camera = "" if self.camera is None else f", camera {self.camera.summary()}"
            self.say(f"  {mode_name(self.mode)}: {who} in control{tilt}{camera}, "
                     f"MAIN1-4 outputs {self.outputs}")

    def lift(self):
        """Flight setting: your thrust until the handover, then the height hold's thrust."""
        if self.hold_z is None:
            return clamp(self.px4_thrust, *THRUST_RANGE)
        below = self.z - self.hold_z  # PX4's z and vz point down
        accel = HOLD_OMEGA ** 2 * below + 2 * HOLD_ZETA * HOLD_OMEGA * self.vz
        accel = clamp(accel, -MAX_VERTICAL_ACCEL, MAX_VERTICAL_ACCEL)  # upward, m/s^2
        self.hover = clamp(self.hover + HOVER_LEARN * below * SETPOINT_PERIOD, *THRUST_RANGE)
        # A tilted drone needs more thrust for the same lift.
        level = math.cos(self.tilt[0]) * math.cos(self.tilt[1])
        return clamp(self.hover * (1 + accel / G) / level, *THRUST_RANGE)

    def stand_lift(self):
        """Stand setting: the base thrust until the handover. Then more, up to the top thrust, the
        further the drone has to lean to centre the locked object, or the further it is still
        behind that lean: at 20% thrust a motor can only swing between idle and 40%."""
        if self.held_yaw is None or self.roll is None:
            return self.stand_thrust
        lean = math.hypot(*self.tilt)
        behind = math.hypot(self.tilt[0] - self.roll, self.tilt[1] - self.pitch)
        need = min(1.0, max(lean / FULL_THRUST_LEAN, behind / FULL_THRUST_BEHIND))
        goal = self.stand_thrust + (self.stand_max_thrust - self.stand_thrust) * need
        step = THRUST_RATE * SETPOINT_PERIOD
        return self.thrust + max(-step, min(step, goal - self.thrust))

    def vehicle(self, now):
        """Drone status for the camera website, or None without a fresh PX4 heartbeat."""
        if self.mode is None or now - self.heartbeat_time > HEARTBEAT_TIMEOUT:
            return None
        control = None if not self.armed else "Pi" if self.mode[0] == OFFBOARD else "pilot"
        status = dict(armed=bool(self.armed), mode=mode_name(self.mode), control=control,
                      roll=None, pitch=None, target=None, setting="flight" if self.flight else "stand",
                      max_tilt=round(math.degrees(self.max_tilt), 1), stand_thrust=self.stand_thrust,
                      stand_max_thrust=self.stand_max_thrust,
                      height=None, thrust=None, below_hold=None,
                      pi="in control" if control == "Pi" else "ready" if self.offering else self.waiting)
        if now - self.attitude_time <= ATTITUDE_TIMEOUT:
            status.update(roll=round(math.degrees(self.roll), 1), pitch=round(math.degrees(self.pitch), 1))
        if self.arm_altitude is not None and now - self.altitude_time <= ALTITUDE_TIMEOUT:
            status["height"] = round(self.altitude - self.arm_altitude, 1)
        if control == "Pi":
            status.update(target=[round(math.degrees(angle), 1) for angle in self.tilt],
                          thrust=round(self.thrust, 3))
            if self.hold_z is not None:
                status["below_hold"] = round(self.z - self.hold_z, 2)
        return status

    def follow(self):
        # Aim at the attitude that centres the locked object, damped by how fast its angle is
        # changing, or at level without one. Never past the tilt limit, changing at most TILT_RATE.
        target = self.camera.target() if self.camera is not None else None
        if target is None:
            goal = (0.0, 0.0)
        else:
            motion = self.camera.motion() or (0.0, 0.0)
            goal = tuple(angle + FOLLOW_DAMPING * rate for angle, rate in zip(target, motion))
        scale = min(1.0, self.max_tilt / max(math.hypot(*goal), 1e-9))
        step = TILT_RATE * SETPOINT_PERIOD
        self.tilt = tuple(now + max(-step, min(step, aim * scale - now))
                          for now, aim in zip(self.tilt, goal))

    def release(self, reason):
        self.offering = self.ch8_low_seen = False
        self.held_yaw, self.tilt, self.hold_z = None, (0.0, 0.0), None
        # PX4 enters Offboard on the CH8 edge, before its 1 Hz heartbeat can say so.
        offboard = (self.mode and self.mode[0] == OFFBOARD) or (self.ch8 or 0) > SWITCH_ON
        # CH6 in the middle selects your own flight mode, which PX4 has already applied.
        mode_switch = self.ch6 is not None and LOW < self.ch6 <= SWITCH_ON
        if offboard and self.manual_mode and not mode_switch:
            main, sub = self.manual_mode
            self.wire.command_long_send(self.system, 1, mav.MAV_CMD_DO_SET_MODE, 0,
                                        mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, main, sub, 0, 0, 0, 0)
            self.say(f"Pi control off: {reason}. Asked PX4 to return to {mode_name(self.manual_mode)}.")
        else:
            self.say(f"Pi control off: {reason}.")

    def start(self):
        deadline = time.monotonic() + 3
        while self.armed is None:
            if time.monotonic() > deadline:
                raise SetupError("No PX4 quadrotor heartbeat on USB.")
            self.pump()
        self.wire.param_request_read_send(self.system, 1, b"RC_MAP_OFFB_SW", -1)
        deadline = time.monotonic() + 2
        while "RC_MAP_OFFB_SW" not in self.params:
            if time.monotonic() > deadline:
                raise SetupError("PX4 did not report RC_MAP_OFFB_SW.")
            self.pump()
        if self.params["RC_MAP_OFFB_SW"] != 8:
            raise SetupError(f"PX4 RC_MAP_OFFB_SW is {self.params['RC_MAP_OFFB_SW']}. "
                             "Set it to 8 so CH8 is PX4's Offboard switch.")
        # Fresh switch data for the 0.25 s radio checks, whatever rate another tool left, and the
        # attitude at 100 Hz so the camera can look it up at the moment each frame was taken.
        for message, interval in ((mav.MAVLINK_MSG_ID_RC_CHANNELS, 50000),
                                  (mav.MAVLINK_MSG_ID_ATTITUDE, 10000)):
            self.wire.command_long_send(self.system, 1, mav.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                        message, interval, 0, 0, 0, 0, 0)
        if self.armed:
            self.say("PX4 is already armed. Disarm with CH5, then arm again to start a session.")
        setting = ("flight setting: the Pi holds height" if self.flight
                   else f"stand setting: fixed {self.stand_thrust:.0%} thrust"
                   if self.stand_max_thrust == self.stand_thrust
                   else f"stand setting: {self.stand_thrust:.0%} to {self.stand_max_thrust:.0%} thrust as the lean needs")
        setting += f", tilt up to {math.degrees(self.max_tilt):g} degrees"
        self.say(f"Ready ({setting}). Arm with CH5 while CH6 and CH8 are low. "
                 "Ctrl-C hands control back and exits.")

    def close(self):
        try:
            if self.offering:
                self.release("Pi script stopped")
            self.port.flush()
        finally:
            self.port.close()


def run(device=DEVICE, stop=None, say=say, camera=None, flight=False, max_tilt=MAX_TILT, thrust=THRUST,
        max_thrust=STAND_MAX_THRUST):
    stop = stop or threading.Event()
    link = Takeover(device, say, camera, flight, max_tilt, thrust, max_thrust)
    try:
        link.start()
        while not stop.is_set():
            link.pump()
            link.step(time.monotonic())
    finally:
        link.close()


def main():
    parser = argparse.ArgumentParser(description="CH8 handover to the Pi with the follow camera.")
    parser.add_argument("--max-tilt", type=float, default=math.degrees(MAX_TILT), metavar="DEGREES",
                        help="largest roll and pitch the Pi may ask for while following (default "
                             f"%(default)g; at most {TILT_CAP_DEGREES}, which caps nothing, or "
                             f"{FLIGHT_TILT_CAP_DEGREES} with --flight)")
    parser.add_argument("--thrust", type=float, metavar="PERCENT",
                        help=f"stand setting's base thrust, with the object straight above (default {THRUST * 100:g}%%)")
    parser.add_argument("--max-thrust", type=float, metavar="PERCENT",
                        help="stand setting's top thrust, for leaning (default "
                             f"{STAND_MAX_THRUST * 100:g}%%, at most {THRUST_CAP * 100:g}%%); the stand must "
                             "hold the drone down. Equal to --thrust for a fixed thrust")
    parser.add_argument("--flight", action="store_true",
                        help="hold height for real flight instead of the stand's fixed thrust")
    parser.add_argument("--no-camera", action="store_true", help="leave the camera free")
    parser.add_argument("--port", type=int, default=8004, help="camera preview port")
    args = parser.parse_args()
    if (args.thrust is not None or args.max_thrust is not None) and args.flight:
        parser.error("--thrust and --max-thrust are for the stand setting; --flight holds height itself")
    thrust = THRUST if args.thrust is None else args.thrust / 100
    max_thrust = max(STAND_MAX_THRUST, thrust) if args.max_thrust is None else args.max_thrust / 100
    if not 0.05 <= thrust <= max_thrust <= THRUST_CAP:
        parser.error(f"--thrust and --max-thrust must be between 5 and {THRUST_CAP * 100:g} percent, "
                     "with --thrust no higher than --max-thrust")
    cap = FLIGHT_TILT_CAP_DEGREES if args.flight else TILT_CAP_DEGREES
    if not 0 < args.max_tilt <= cap:
        parser.error(f"--max-tilt must be above 0 and at most {cap} degrees"
                     + (" with --flight" if args.flight else ""))
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    camera = web = None
    if not args.no_camera:
        try:
            from follow_camera.camera import FollowCamera, serve
            camera = FollowCamera(say=say)
            camera.start()
            web = serve(camera, port=args.port)
            say(f"Follow camera preview on port {args.port}.")
        except Exception as exc:  # The handover still works without the camera or its page.
            say(f"Follow camera {'preview' if camera else 'off'}: {exc}")
    try:
        problem = None
        while not stop.is_set():
            started = time.monotonic()
            try:
                run(stop=stop, camera=camera, flight=args.flight, max_tilt=math.radians(args.max_tilt),
                    thrust=thrust, max_thrust=max_thrust)
            except (SetupError, OSError) as exc:
                # Keep the camera running and retry, logging a repeated failure only once.
                if str(exc) != problem or time.monotonic() - started > 30:
                    say(f"{exc} Retrying every 5 s.")
                problem = str(exc)
                stop.wait(5)
    finally:
        if web is not None:
            web.shutdown()
        if camera is not None:
            camera.close()


if __name__ == "__main__":
    main()
