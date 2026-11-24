"""Attended two-axis jig input for PX4's native multicopter Stabilized mode."""
import math
import time

from pymavlink.dialects.v20 import common as mav

from .leveling import MAX_SAMPLE_AGE, TRIAL_TILT_DEG, down_vector, validate_trial_target


STABILIZED = (7, 0)  # PX4 custom main mode, submode
ABORT_TILT_DEG = 2 * TRIAL_TILT_DEG
START_DOWN_Z = math.cos(math.radians(TRIAL_TILT_DEG))
JIG_PARAMS = ("COM_RC_IN_MODE", "MPC_MAN_TILT_MAX")
RESTORABLE_MODES = {(1, 0), (2, 0), (3, 0), (4, 3), STABILIZED}  # manual, altitude, position, hold


def stick_axes(roll_deg, pitch_deg, max_tilt_deg):
    validate_trial_target(roll_deg, pitch_deg)
    if not TRIAL_TILT_DEG <= max_tilt_deg <= 70:
        raise ValueError("PX4 manual tilt setting is unavailable or outside the supported range.")
    x, y, z = down_vector(math.radians(roll_deg), math.radians(pitch_deg))
    horizontal = math.hypot(x, y)
    if not horizontal:
        return 0, 0
    # PX4 maps the stick to a horizontal axis-angle vector. Invert that
    # mapping so both Euler angle sliders remain accurate when used together.
    scale = 1000 * math.atan2(horizontal, z) / (horizontal * math.radians(max_tilt_deg))
    return math.trunc(x * scale), math.trunc(y * scale)


class NativeJig:
    def __init__(self, link):
        self.link = link
        self.mode = None
        self.armed = False
        self.landed = None
        self.landed_at = -math.inf
        self.previous = None
        self.selected = False
        self.mode_sent = False
        self.confirmed = False
        self.started = 0.0
        self.rejection = None
        self.arm_pending = False
        self.arm_authorized = False
        self.arm_time = 0.0
        self.arm_message = ""
        self.last_arm_report = None
        self.stop_pending = False
        self.verify_stop_after = 0.0
        self.last_disarm = -math.inf
        self.stop_message = ""
        self.target = (0, 0)
        self.throttle = 0
        self.fc_target = None
        self.fc_target_at = -math.inf
        self.fc_target_stamp = None
        self.next_target_read = 0.0
        self.next_heartbeat = 0.0
        self.target_since = 0.0
        self.last_warning = ""
        self.warning_at = -math.inf

    def record_arm(self, state, message):
        self.last_arm_report = {"state": state, "message": message, "time": time.time()}

    def observe(self, msg, now):
        kind = msg.get_type()
        if kind == "HEARTBEAT":
            self.mode = ((msg.custom_mode >> 16) & 255, (msg.custom_mode >> 24) & 255)
            was_armed = self.armed
            self.armed = bool(msg.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED)
            if self.armed and self.selected:
                if self.arm_pending and self.arm_authorized:
                    self.record_arm("armed", "PX4 confirmed the jig was armed.")
                self.arm_pending = False
                self.arm_message = ""
                if not self.arm_authorized:
                    self.rejection = "FC armed outside this jig session."
            if was_armed and not self.armed and self.selected:
                self.arm_authorized = False
                self.rejection = "PX4 disarmed. Start a fresh jig session."
            if self.stop_pending:
                if not self.armed and now >= self.verify_stop_after:
                    self.stop_pending = False
                    self.stop_message = "PX4 reports disarmed."
                    self.restore()
                elif now - self.last_disarm >= .45:
                    self.request_disarm()
            if self.selected and not self.confirmed and self.mode_sent and self.mode == STABILIZED:
                self.confirmed = True
                self.fc_target, self.fc_target_at = None, -math.inf
        elif kind == "EXTENDED_SYS_STATE":
            self.landed, self.landed_at = msg.landed_state, now
        elif kind == "COMMAND_ACK" and msg.target_system in (0, 255) and msg.target_component in (0, 190):
            if msg.result not in (mav.MAV_RESULT_ACCEPTED, mav.MAV_RESULT_IN_PROGRESS):
                if self.selected and msg.command == mav.MAV_CMD_DO_SET_MODE and not self.confirmed:
                    self.rejection = f"PX4 declined Stabilized mode (result {msg.result})."
                if self.arm_pending and msg.command == mav.MAV_CMD_COMPONENT_ARM_DISARM:
                    self.arm_pending = self.arm_authorized = False
                    result = mav.enums["MAV_RESULT"].get(msg.result)
                    reason = result.name.removeprefix("MAV_RESULT_").lower().replace("_", " ") if result else str(msg.result)
                    self.arm_message = f"PX4 declined arming ({reason})."
                    self.record_arm("declined", self.arm_message)
                    if self.warning_at >= self.arm_time:
                        self.last_arm_report["warning"] = self.last_warning
                if self.stop_pending and msg.command == mav.MAV_CMD_COMPONENT_ARM_DISARM:
                    self.stop_message = "PX4 did not confirm disarm. Use the motor-power disconnect."
        elif kind == "STATUSTEXT" and msg.severity <= mav.MAV_SEVERITY_WARNING:
            self.last_warning, self.warning_at = msg.text, now
            if (now - self.arm_time <= 3 and self.last_arm_report
                    and self.last_arm_report["state"] == "declined"):
                self.last_arm_report["warning"] = msg.text
        elif kind == "ATTITUDE_TARGET" and msg.time_boot_ms != self.fc_target_stamp:
            w, x, y, z = msg.q
            if (not msg.type_mask & mav.ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE
                    and all(math.isfinite(v) for v in msg.q)
                    and abs(math.hypot(w, x, y, z) - 1) < .01):
                roll = math.atan2(2 * (w*x + y*z), 1 - 2 * (x*x + y*y))
                pitch = math.asin(max(-1, min(1, 2 * (w*y - z*x))))
                self.fc_target = [round(math.degrees(roll), 1), round(math.degrees(pitch), 1)]
                self.fc_target_at, self.fc_target_stamp = now, msg.time_boot_ms

    def ground_and_attitude(self, now):
        if self.landed != mav.MAV_LANDED_STATE_ON_GROUND or now - self.landed_at > 2.5:
            raise ValueError("PX4 must report the jig on the ground.")
        sample = self.link.leveling
        if sample.sample is None or now - sample.received > MAX_SAMPLE_AGE:
            raise ValueError("Fresh FC attitude is required.")
        roll, pitch, p, q = sample.sample
        if not all(math.isfinite(v) for v in (roll, pitch, p, q)):
            raise ValueError("PX4 must report a valid upright attitude.")
        down_z = math.cos(roll) * math.cos(pitch)
        if down_z <= 0:
            raise ValueError("PX4 must report a valid upright attitude.")
        return down_z

    def start_tilt(self, now):
        try:
            down_z = self.ground_and_attitude(now)
        except ValueError as exc:
            return {"within_limit": False, "message": str(exc)}
        return {"degrees": round(math.degrees(math.acos(down_z)), 2),
                "limit_deg": TRIAL_TILT_DEG, "within_limit": down_z >= START_DOWN_Z}

    def begin(self):
        now = time.monotonic()
        self.link.check()
        if self.link.outputs_stopped or self.link.active or self.armed or self.stop_pending:
            raise ValueError("Start with the FC disarmed, the bench button released, and the motor stop reset.")
        self.ground_and_attitude(now)
        params = self.link.leveling.parameters
        if params.get("COM_RC_IN_MODE") not in (1, 2, 3):
            raise ValueError("Waiting for PX4 to report joystick input enabled.")
        stick_axes(0, 0, params.get("MPC_MAN_TILT_MAX", 0))
        if self.mode not in RESTORABLE_MODES:
            raise ValueError("Select a ground standby mode before starting the jig.")
        self.previous, self.selected, self.started = self.mode, True, now
        self.mode_sent = self.confirmed = self.arm_pending = self.arm_authorized = False
        self.rejection, self.arm_message, self.stop_message = None, "", ""
        self.target, self.target_since = (0, 0), now
        self.throttle = 0

    def _stick(self, pitch, roll, throttle):
        if (pitch or roll or throttle) and (self.link.outputs_stopped or not self.selected or not self.confirmed):
            raise ValueError("Jig output is inhibited.")
        self.link.wire.manual_control_send(self.link.system, pitch, roll, throttle, 0, 0)

    def _mode(self, mode):
        self.link._send(mav.MAV_CMD_DO_SET_MODE, mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, *mode, 0, 0, 0, 0)

    def restore(self):
        previous, self.previous = self.previous, None
        if (previous is not None and previous != STABILIZED and self.mode_sent
                and self.mode in (STABILIZED, previous) and not self.armed):
            self._mode(previous)

    def request_disarm(self):
        self.last_disarm = time.monotonic()
        self.link._send(mav.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0)

    def end(self):
        if not self.selected:
            return
        self.selected = self.confirmed = False
        self.target = (0, 0)
        self.throttle = 0
        self.stop_pending = self.armed or self.arm_pending
        if self.arm_pending and self.last_arm_report and self.last_arm_report["state"] == "pending":
            self.record_arm("unconfirmed", "The jig session ended before PX4 confirmed arming.")
        self.verify_stop_after = max(time.monotonic(), self.arm_time + 3 if self.arm_pending else 0)
        self.arm_pending = self.arm_authorized = False
        try:
            self._stick(0, 0, 0)
        finally:
            if self.stop_pending:
                self.stop_message = "Disarm requested. Use the motor-power disconnect if unconfirmed."
                # Regular disarm only. PX4 retains its own landed and arming checks.
                self.request_disarm()
            else:
                self.restore()

    def update(self, roll, pitch, throttle, arm, max_percent):
        now = time.monotonic()
        self.link.check()
        if not self.selected or self.link.outputs_stopped:
            raise ValueError("The jig session has stopped.")
        down_z = self.ground_and_attitude(now)
        if self.rejection:
            raise ValueError(self.rejection)
        if type(throttle) is not int or not 0 <= throttle <= max_percent or type(arm) is not bool:
            raise ValueError("Invalid jig controls.")
        if self.throttle and not throttle:
            raise ValueError("Jig throttle released.")
        params = self.link.leveling.parameters
        if params.get("COM_RC_IN_MODE") not in (1, 2, 3):
            raise ValueError("PX4 joystick input is unavailable.")
        x, y = stick_axes(roll, pitch, params.get("MPC_MAN_TILT_MAX", 0))
        if now >= self.next_heartbeat:
            self.link.wire.heartbeat_send(mav.MAV_TYPE_GCS, mav.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            self.next_heartbeat = now + 1
        if now >= self.next_target_read and now - self.fc_target_at > .2:
            self.link._send(mav.MAV_CMD_REQUEST_MESSAGE, mav.MAVLINK_MSG_ID_ATTITUDE_TARGET, 0, 0, 0, 0, 0, 0)
            self.next_target_read = now + .2
        if not self.confirmed:
            if self.armed or throttle or arm or now - self.started > 4:
                raise ValueError("PX4 has not confirmed Stabilized mode with the FC disarmed.")
            self._stick(0, 0, 0)
            if not self.mode_sent and now - self.started > .35:
                self._mode(STABILIZED)
                self.mode_sent = True
            return
        if self.mode != STABILIZED:
            raise ValueError("PX4 left Stabilized mode.")
        if arm or self.armed:
            if self.armed and down_z < math.cos(math.radians(ABORT_TILT_DEG)):
                raise ValueError(f"Measured tilt exceeded the {ABORT_TILT_DEG}° jig cutoff.")
            if now - self.fc_target_at > .7:
                raise ValueError("PX4 attitude-target feedback is stale.")
            if (now - self.target_since > 1 and
                    any(abs(a - b) > .8 for a, b in zip(self.target, self.fc_target))):
                raise ValueError("PX4 target differs from the jig request.")
        if throttle and not self.armed:
            raise ValueError("PX4 has not confirmed the jig armed. Throttle blocked.")
        if arm:
            if (self.armed or self.arm_pending or throttle or roll or pitch or self.target != (0, 0)
                    or max(map(abs, self.fc_target)) > .4):
                raise ValueError("Arm the secured jig at zero throttle with level targets.")
            if down_z < START_DOWN_Z:
                self.arm_message = ""
                self.record_arm("blocked", f"Combined tilt exceeded the {TRIAL_TILT_DEG}° start limit. "
                                "The jig stayed in standby.")
                self._stick(0, 0, 0)
                return
            self.arm_pending, self.arm_time, self.arm_message = True, now, ""
            self.arm_authorized = True
            self._stick(0, 0, 0)
            self.link._send(mav.MAV_CMD_COMPONENT_ARM_DISARM, 1, 0, 0, 0, 0, 0, 0)
            self.record_arm("pending", "Arm request sent. Waiting for PX4.")
        if self.arm_pending and now - self.arm_time > 3:
            self.record_arm("unconfirmed", "PX4 did not confirm arming before the request timed out.")
            raise ValueError("Arming was not confirmed by PX4.")
        self._stick(x, y, throttle * 10)
        self.throttle = throttle
        if self.target != (roll, pitch):
            self.target, self.target_since = (roll, pitch), now

    def status(self):
        now = time.monotonic()
        notice = self.arm_message
        if notice and self.warning_at >= self.arm_time - 1 and now - self.warning_at < 8:
            notice += " " + self.last_warning
        return {"confirmed": self.confirmed, "armed": self.armed, "arming": self.arm_pending,
                "start_tilt": self.start_tilt(now),
                "stick_pct": self.throttle, "target": list(self.target), "notice": notice,
                "fc_target": self.fc_target if self.confirmed and now - self.fc_target_at < .7 else None}
