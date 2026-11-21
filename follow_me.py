#!/usr/bin/env python3
# follow_me.py — pan/tilt camera web app: manual control, register faces, lock-on follow.
#   /usr/bin/python3 follow_me.py [--port 8000] [--target owner] [--no-follow]
# Open http://<pi-ip>:8000/ . UI lives in follow_me_ui.html (re-read on every page load).
#
# Lock-follow design (measured 2026-11-15 with a camera step-response test):
#   * servo command -> image moves: ~70 ms to 50 %, ~120 ms to 90 %
#   * pan 2.0 px/us, tilt 2.1 px/us at 1280x720  (ch1 pan +us = left, ch0 tilt +us = up)
# Every frame carries its sensor timestamp. We look up where the servos actually were when
# it was exposed (dead time + first-order servo model), add the face's pixel offset, and
# get the face's ABSOLUTE pan/tilt. An alpha-beta filter tracks that position + velocity and
# we command the predicted position. Because the setpoint is absolute, camera/servo lag no
# longer causes the overshoot/oscillation that a relative "nudge toward centre" loop has.
#
# Pipeline: camera 60 fps; ISP gives a hardware-scaled 640x360 "lores" image for detection
# (YuNet), full-res 1280x720 "main" only for face crops (SFace). Faces are tracked between
# frames by IoU (motion-compensated for servo movement) so SFace runs only to (re)verify.
# JPEG encoding for the browser runs in its own thread, only while someone is watching.
import argparse, json, os, queue, re, shutil, signal, socketserver, threading, time
from collections import deque
from http import server
import cv2, numpy as np
import face_id as F
from servo_test import PCA9685

HERE = os.path.dirname(os.path.abspath(__file__))
UI_FILE = os.path.join(HERE, "follow_me_ui.html")

W, H = 1280, 720                      # main (recognition) resolution
LW, LH = 640, 360                     # lores (detection) resolution
FPS = 60
PAN_RANGE, TILT_RANGE = (1000, 2000), (1050, 1800)
PAN_PX_PER_US, TILT_PX_PER_US = 2.0, 2.1    # at W x H
# Servo+camera response to a command step (measured): ~35 ms dead time, then first-order
# with ~40 ms time constant (t10 ~40 ms, t50 ~70 ms, t90 ~120 ms).
SERVO_DEAD, SERVO_TAU = 0.035, 0.040
LEAD = 0.06              # s: aim this far ahead of the estimate — only when really moving
# v5 used ALPHA .55 / BETA .15 and always applied the lead: detector noise became a fake
# velocity (+-200 us/s while sitting still) and the servo twitched every frame.
ALPHA, BETA = 0.30, 0.04   # alpha-beta filter gains (position, velocity)
V_GATE = 150             # us/s: below this the target counts as still -> no prediction
V_MAX = 1500             # us/s: cap on the estimated target velocity used for prediction
MAX_SPEED = 3000         # us/s: command slew limit (roughly the servo's own top speed)
HOLD_DEADBAND = 8        # us per axis (~16 px): ignore smaller corrections -> servo rests
PREDICT_S = 0.25         # s: after losing the face, keep following the prediction this long
LOST_HOLD_S = 1.5        # s: then hold still this long before scanning (exit direction unknown)
LOST_HOLD_DIR_S = 0.4    # s: ...or this long if we saw which way you left: scan that way first
EXIT_EDGE = 0.25         # face centre beyond this fraction of W from centre = "left via that edge"
# Sticky identity: a track confirmed as someone keeps that label through weak re-checks (head
# turned, blur) unless it fails ID_MAX_MISSES in a row or confidently matches someone else.
ID_CONFIRM_HITS, ID_MAX_MISSES, ID_RETRY_S, ID_SWITCH_SCORE = 2, 6, 0.25, 0.45
SEARCH_SPEED = 400       # us/s while scanning
SEARCH_TILTS = (1300, 1180, 1450)
# --- compute budget (v7). Identity is carried by IoU tracking between frames, so SFace only
# needs to re-confirm occasionally; detection rate depends on what the mode actually needs.
VERIFY_TARGET_S, VERIFY_OTHER_S, VERIFY_UNKNOWN_S = 0.5, 2.0, 1.0   # SFace re-check intervals
MAX_SFACE_PER_FRAME = 2  # max new SFace jobs queued per frame (they run in a worker thread)
FULL_EVERY = 15          # while locked: full-frame detection every N frames, ROI-only otherwise
SEARCH_DET_HZ = 20       # full-frame detection rate while acquiring / scanning
MANUAL_DET_HZ = 8        # manual mode: only for on-screen labels, and only while someone watches
FPS_ACTIVE, FPS_IDLE = 60, 30   # camera frame rate when following/registering vs manual
ROI_SCALE, ROI_MIN, ROI_IN = 3.0, 200, 192   # ROI = 3x face size (>=200px), detected at 192x192
ROI_LORES_MIN = 300      # ROI >= this (main px) is cut from the 640x360 lores image: no main copy
CV_THREADS = 2           # 4 threads: ~same latency, +25% CPU (spin-waiting workers)
ENROLL_SAMPLES, ENROLL_TIMEOUT_S, ENROLL_MIN_W = 30, 40, 110
ENROLL_EVERY_S = 0.12    # min time between samples
ENROLL_DUP = 0.93        # skip a sample this similar to one we have -> forces varied angles
ENROLL_DUP_RELAX_S = 15  # ...but after this long, accept near-duplicates (user isn't moving)
STREAM_FPS = 12          # browser preview, encoded from lores (no resize)
NAME_RE = re.compile(r"^[A-Za-z0-9_-]{1,32}$")

cmds = queue.Queue()
sface_q, sface_res = queue.Queue(maxsize=4), queue.Queue()
DB_SNAP = {}                         # replaced (never mutated) by the main thread; read by worker
latest_jpg, jpg_cond = None, threading.Condition()
last_view = 0.0                      # last time a browser fetched a frame
shared = {"img": None, "labels": [], "seq": 0}
shared_lock = threading.Lock()
status = {}


def clamp(v, r):
    return max(r[0], min(r[1], v))


class RobustPCA(PCA9685):
    """Retries I2C writes; servo current spikes occasionally cause Errno 121 on the bus."""
    def set_pulse_us(self, ch, us):
        for i in range(5):
            try:
                return super().set_pulse_us(ch, us)
            except OSError as e:
                if i == 4:
                    print(f"I2C write failed 5x ({e}), skipping", flush=True)
                    return us
                time.sleep(0.01 * (i + 1))


class Servos:
    """Pan/tilt with command history, so we know where the camera pointed at any past time."""
    def __init__(self, pca, pan, tilt):
        self.pca, self.pan, self.tilt = pca, pan, tilt
        self.hist = deque(maxlen=400)                    # (t, pan, tilt)
        self._write(force=True)

    def _write(self, force=False):
        self.pca.set_pulse_us(F.PAN_CH, self.pan); self.pca.set_pulse_us(F.TILT_CH, self.tilt)
        self.hist.append((time.monotonic(), self.pan, self.tilt))

    def go(self, pan, tilt, dt=None, max_speed=None):
        pan, tilt = clamp(pan, PAN_RANGE), clamp(tilt, TILT_RANGE)
        if dt is not None and max_speed:
            lim = max_speed * dt
            pan = self.pan + np.clip(pan - self.pan, -lim, lim)
            tilt = self.tilt + np.clip(tilt - self.tilt, -lim, lim)
        # per-axis deadband: an axis only moves when its correction is worth it
        if abs(pan - self.pan) < HOLD_DEADBAND:
            pan = self.pan
        if abs(tilt - self.tilt) < HOLD_DEADBAND:
            tilt = self.tilt
        if pan == self.pan and tilt == self.tilt:
            return
        self.pan, self.tilt = pan, tilt
        self._write()

    def pose_at(self, t):
        """Physical pose at time t: command steps passed through dead time + first-order lag."""
        h = np.array(self.hist)                       # rows: t, pan, tilt
        te = t - SERVO_DEAD
        h = h[h[:, 0] <= te]
        if len(h) == 0:
            return self.hist[0][1], self.hist[0][2]
        steps = np.diff(h[:, 1:], axis=0, prepend=h[:1, 1:] * 0)   # first row = initial value
        resp = 1 - np.exp(-(te - h[:, 0]) / SERVO_TAU)
        resp[0] = 1.0                                  # initial position fully reached
        p = (steps * resp[:, None]).sum(axis=0)
        return float(p[0]), float(p[1])


class AlphaBeta:
    def __init__(self):
        self.x = None

    def reset(self, z, t):
        self.x, self.v, self.t = np.array(z, float), np.zeros(2), t

    def update(self, z, t):
        if self.x is None or t - self.t > 0.4:
            return self.reset(z, t)
        dt = max(1e-3, t - self.t)
        xp = self.x + self.v * dt
        r = np.array(z, float) - xp
        self.x = xp + ALPHA * r
        self.v = np.clip(self.v + BETA * r / dt, -V_MAX, V_MAX)
        self.t = t

    def predict(self, t):
        v = self.v * (np.abs(self.v) > V_GATE)        # ignore noise-level "velocity"
        return self.x + v * min(t - self.t, PREDICT_S + LEAD)


def apply_identity(tr, name, s, t):
    """Update a track's identity from an SFace result, with hysteresis (see ID_* constants)."""
    prev, hits, misses = tr.get("name"), tr.get("hits", 0), tr.get("misses", 0)
    confirmed = prev is not None and hits >= ID_CONFIRM_HITS
    if name is not None and (name == prev or not confirmed or s >= ID_SWITCH_SCORE):
        tr["hits"] = hits + 1 if name == prev else 1
        tr["name"], tr["misses"] = name, 0
    elif confirmed and misses + 1 < ID_MAX_MISSES:
        tr["misses"] = misses + 1                 # keep the label, re-check soon
    else:
        tr["name"], tr["hits"], tr["misses"] = name, (1 if name else 0), 0
    tr["score"], tr["ver"] = s, t


def exit_direction(ex, v_pan):
    """Which way the target left: +1 = toward higher pan (camera left), -1 = lower, 0 = unknown."""
    if ex is not None and abs(ex) > EXIT_EDGE * W:
        return -1 if ex > 0 else 1                # face was at the right edge -> turn right
    if abs(v_pan) > V_GATE:
        return 1 if v_pan > 0 else -1
    return 0


def iou(a, b):
    ax, ay, aw, ah = a; bx, by, bw, bh = b
    ix = max(0, min(ax + aw, bx + bw) - max(ax, bx)); iy = max(0, min(ay + ah, by + bh) - max(ay, by))
    inter = ix * iy
    return inter / (aw * ah + bw * bh - inter + 1e-6)


def search_waypoints():
    pts = []
    for i, t in enumerate(SEARCH_TILTS):
        ends = (PAN_RANGE[0], PAN_RANGE[1]) if i % 2 == 0 else (PAN_RANGE[1], PAN_RANGE[0])
        pts += [(ends[0], t), (ends[1], t)]
    return pts


def open_cam():
    from picamera2 import Picamera2
    cam = Picamera2()
    # "RGB888" is BGR byte order (what OpenCV wants). lores is scaled by the ISP for free.
    cfg = cam.create_video_configuration(main={"size": (W, H), "format": "RGB888"},
                                         lores={"size": (LW, LH), "format": "RGB888"},
                                         buffer_count=4, controls={"FrameRate": FPS})
    cam.configure(cfg); cam.start()
    try:   # short exposures = less motion blur on fast moves
        from libcamera import controls
        cam.set_controls({"AeExposureMode": controls.AeExposureModeEnum.Short})
    except Exception as e:
        print(f"short exposure not set: {e}", flush=True)
    time.sleep(0.8)
    return cam


# ---------------------------------------------------------------- SFace worker (own thread)
def sface_worker():
    rec = None  # Private to this worker; load on the first recognition job.
    while True:
        tid, img, f, n_sub = sface_q.get()
        try:
            if rec is None:
                rec = cv2.FaceRecognizerSF.create(F.REC_MODEL, "")
            e = rec.feature(rec.alignCrop(img, f)).flatten(); e /= np.linalg.norm(e)
            name, s = F.identify(e, DB_SNAP) if DB_SNAP else (None, 0.0)
        except Exception as ex:          # bad landmarks near the border etc.
            print(f"sface error: {ex}", flush=True); name, s = None, 0.0
        sface_res.put((tid, name, s, n_sub))


def detect_roi(det_roi, src, scale, cx, cy, size):
    """Detect faces in a square ROI (given in main coords) of `src`, which is the main image
    (scale 1) or lores (scale 2). Returns faces and the ROI in main coords."""
    s = int(clamp(size, (ROI_MIN, H)))
    x0 = clamp(cx - s / 2, (0, W - s)); y0 = clamp(cy - s / 2, (0, H - s))
    xs, ys, ss = int(x0 / scale), int(y0 / scale), int(s / scale)
    k = ROI_IN / ss
    _, fs = det_roi.detect(cv2.resize(src[ys:ys + ss, xs:xs + ss], (ROI_IN, ROI_IN),
                                      interpolation=cv2.INTER_AREA if k < 1 else cv2.INTER_LINEAR))
    out = []
    for f in ([] if fs is None else fs):
        f = f.copy(); f[:14] *= scale / k
        f[0] += xs * scale; f[1] += ys * scale; f[4:14:2] += xs * scale; f[5:14:2] += ys * scale
        out.append(f)
    return out, (xs * scale, ys * scale, ss * scale)


# ---------------------------------------------------------------- browser stream (own thread)
def encoder():
    global latest_jpg
    seen = -1
    while True:
        time.sleep(1 / STREAM_FPS)
        if time.monotonic() - last_view > 3:
            continue
        with shared_lock:
            img, labels, seq, hud = shared["img"], shared["labels"], shared["seq"], shared.get("hud")
        if img is None or seq == seen:
            continue
        seen = seq
        img = img.copy()                 # lores 640x360; label coords are main/2
        for x, y, w, h, txt, col in labels:
            x, y, w, h = x // 2, y // 2, w // 2, h // 2
            cv2.rectangle(img, (x, y), (x + w, y + h), col, 2)
            if txt:
                cv2.putText(img, txt, (x, max(14, y - 5)), 0, 0.5, col, 1, cv2.LINE_AA)
        if hud is not None:      # where the filter thinks the target is (absolute -> pixels)
            cv2.circle(img, (hud[0] // 2, hud[1] // 2), 6, (0, 255, 0), 2)
        cv2.drawMarker(img, (LW // 2, LH // 2), (255, 255, 0), cv2.MARKER_CROSS, 18, 1)
        ok, j = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 75])
        with jpg_cond:
            latest_jpg = j.tobytes(); jpg_cond.notify_all()


class Handler(server.BaseHTTPRequestHandler):
    def _send(self, body, ctype, code=200):
        self.send_response(code); self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", len(body)); self.send_header("Cache-Control", "no-cache")
        self.end_headers(); self.wfile.write(body)

    def do_GET(self):
        global last_view
        if self.path == "/":
            with open(UI_FILE, "rb") as f:
                return self._send(f.read(), "text/html; charset=utf-8")
        if self.path == "/api/status":
            return self._send(json.dumps(status).encode(), "application/json")
        if self.path.startswith("/frame.jpg"):
            last_view = time.monotonic()
            with jpg_cond:
                j = latest_jpg or (jpg_cond.wait(2) and latest_jpg)
            return self._send(j, "image/jpeg") if j else self.send_error(503)
        if self.path.startswith("/crop/"):
            name = self.path[6:].split("?")[0]
            p = os.path.join(F.FACES, name, "crops", "001.jpg")
            if NAME_RE.match(name) and os.path.exists(p):
                with open(p, "rb") as f:
                    return self._send(f.read(), "image/jpeg")
        self.send_error(404)

    def do_POST(self):
        if self.path != "/api/cmd":
            return self.send_error(404)
        try:
            body = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))) or b"{}")
            assert isinstance(body, dict) and "action" in body
        except Exception:
            return self._send(b'{"ok":false,"error":"bad json"}', "application/json", 400)
        cmds.put(body)
        self._send(b'{"ok":true}', "application/json")

    def log_message(self, *a):
        pass


class Server(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = daemon_threads = True


# ---------------------------------------------------------------- main loop
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", "--stream", type=int, default=8000)
    ap.add_argument("--target", "--name", default="owner")
    ap.add_argument("--no-follow", action="store_true", help="start in manual mode")
    a = ap.parse_args()

    cv2.setNumThreads(CV_THREADS)
    # kill (SIGTERM) only sets a flag; the main loop exits at the next frame and the finally
    # block centres + releases the servos. Raising from the handler could fire mid-setup or
    # mid-cleanup and skip the release.
    stop_evt = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop_evt.set())
    db = F.load_db()
    target_name = a.target if a.target in db else next(iter(sorted(db)), None)
    follow = not a.no_follow and target_name is not None
    threading.Thread(target=Server(("", a.port), Handler).serve_forever, daemon=True).start()
    threading.Thread(target=encoder, daemon=True).start()
    print(f"UI: http://<pi-ip>:{a.port}/", flush=True)

    global DB_SNAP
    DB_SNAP = dict(db)
    threading.Thread(target=sface_worker, daemon=True).start()
    pending = set()                      # track ids with an SFace job in flight
    cam = open_cam()
    det = cv2.FaceDetectorYN.create(F.DET_MODEL, "", (LW, LH), 0.75)
    det_roi = cv2.FaceDetectorYN.create(F.DET_MODEL, "", (ROI_IN, ROI_IN), 0.7)
    roi_frames = full_frames = 0
    sv = Servos(RobustPCA(), float(F.HOME[F.PAN_CH]), float(F.HOME[F.TILT_CH]))
    filt = AlphaBeta()

    tracks, next_id, lock_id = [], 0, None
    last_seen, last_frame_pose = 0.0, None
    wps, wp_i = search_waypoints(), None
    enroll, msg, state, logged = None, "", "manual", None
    n, fps, proc_ms, age_ms, t_last = 0, 0.0, 0.0, 0.0, time.monotonic()
    err_px = 0.0             # smoothed distance of the locked face from frame centre
    trace = deque(maxlen=600)   # per-frame tracking log, dumped with {"action":"dump"}
    last_full, cam_fps, req, dt_avg = 0.0, FPS_ACTIVE, None, 1 / FPS_ACTIVE
    last_ex, last_vpan, search_first = None, 0.0, None   # where/how the target left view
    main_cache = [None]
    lo = None

    def lores_img():
        nonlocal lo
        if lo is None:
            lo = req.make_array("lores")
        return lo

    def main_img():
        """Full-res frame, copied out of the camera buffer only when something needs it."""
        if main_cache[0] is None:
            main_cache[0] = req.make_array("main")
        return main_cache[0]

    try:
        while not stop_evt.is_set():
            if req is not None:          # held through the previous iteration for main_img()
                r, req = req, None       # clear first: an interrupt here must not double-release
                r.release()
            req = cam.capture_request(); main_cache[0] = None
            ts = req.get_metadata()["SensorTimestamp"] / 1e9     # CLOCK_MONOTONIC, s
            lo = None
            t0 = time.monotonic(); n += 1
            want_fps = FPS_ACTIVE if (follow or enroll is not None) else FPS_IDLE
            if want_fps != cam_fps:
                fd = int(1e6 / want_fps)
                cam.set_controls({"FrameDurationLimits": (fd, fd)}); cam_fps = want_fps
            dt = min(0.1, t0 - t_last); t_last = t0
            dt_avg = 0.95 * dt_avg + 0.05 * dt; fps = 1 / dt_avg if dt_avg > 0 else 0.0

            # ---- commands from the browser
            while not cmds.empty():
                c = cmds.get(); act = c.get("action")
                if act not in ("move", "set"):
                    print(f"{time.strftime('%H:%M:%S')} cmd {c}", flush=True)
                if act == "move":
                    follow = False
                    sv.go(sv.pan + float(c.get("dpan", 0)), sv.tilt + float(c.get("dtilt", 0)))
                elif act == "set":
                    follow = False
                    sv.go(float(c.get("pan", sv.pan)), float(c.get("tilt", sv.tilt)))
                elif act == "home":
                    follow = False
                    sv.go(F.HOME[F.PAN_CH], F.HOME[F.TILT_CH])
                elif act == "follow":
                    t = c.get("target") or target_name
                    if c.get("on") and t not in db:
                        msg = f"'{t}' is not registered"
                    else:
                        follow, target_name = bool(c.get("on")), t
                        lock_id, wp_i, filt.x, last_seen = None, None, None, t0
                        last_ex, last_vpan, search_first = None, 0.0, None
                        for tr in tracks:            # re-label against the new target soon
                            tr["ver"] = -999
                        msg = f"following {t}" if follow else "follow off"
                elif act == "enroll":
                    name = str(c.get("name", "")).strip()
                    if not NAME_RE.match(name):
                        msg = "name: letters, digits, - or _ (max 32)"
                    else:
                        follow = False
                        enroll = {"name": name, "embs": [], "crops": [], "t0": t0, "rejected": 0}
                        msg = f"registering {name}: look at the camera, turn your head slightly"
                elif act == "enroll_cancel" and enroll:
                    enroll, msg = None, "registration cancelled"
                elif act == "dump":         # debug: write the recent tracking trace to CSV
                    p = str(c.get("path", "/tmp/follow_trace.csv"))
                    with open(p, "w") as fh:
                        fh.write("ts,roi,ex,ey,z_pan,z_tilt,aim_pan,aim_tilt,v_pan,v_tilt,cmd_pan,cmd_tilt,face_w\n")
                        fh.writelines(",".join(map(str, r)) + "\n" for r in trace)
                    msg = f"trace: {len(trace)} rows -> {p}"
                elif act == "forget":
                    name = str(c.get("name", ""))
                    if NAME_RE.match(name):
                        shutil.rmtree(os.path.join(F.FACES, name), ignore_errors=True)
                        db.pop(name, None); DB_SNAP = dict(db)
                        for tr in tracks:
                            if tr["name"] == name:
                                tr["name"], tr["ver"] = None, -999
                        if target_name == name:
                            follow, target_name = False, next(iter(sorted(db)), None)
                        msg = f"deleted {name}"

            # ---- camera motion since last frame (to predict where tracked faces moved)
            pose = sv.pose_at(ts)
            if last_frame_pose is not None:
                sx = (pose[0] - last_frame_pose[0]) * PAN_PX_PER_US      # pan +us => scene moves +x
                sy = (pose[1] - last_frame_pose[1]) * TILT_PX_PER_US     # tilt +us => scene moves +y
            else:
                sx = sy = 0.0
            last_frame_pose = pose

            # ---- detect, at the rate the current mode needs:
            #   locked: ROI around the face every frame (+ full frame every FULL_EVERY)
            #   acquiring/searching/registering: full frame at SEARCH_DET_HZ
            #   manual: full frame at MANUAL_DET_HZ, only while the page is open
            viewing = t0 - last_view < 3
            locked_tr = next((tr for tr in tracks if tr["id"] == lock_id), None) \
                if follow and enroll is None else None
            dets, roi, do_full = None, None, False
            if locked_tr is not None and n % FULL_EVERY:
                px, py, pw, ph = locked_tr["box"]
                size = ROI_SCALE * max(pw, ph)
                src, scale = (lores_img(), 2) if size >= ROI_LORES_MIN else (main_img(), 1)
                dets, roi = detect_roi(det_roi, src, scale, px + pw / 2 + sx, py + ph / 2 + sy, size)
                if not dets:
                    dets, roi, do_full = None, None, True   # lost in ROI -> full frame right now
            elif locked_tr is not None:
                do_full = True
            elif follow or enroll is not None:
                do_full = t0 - last_full >= 1 / SEARCH_DET_HZ
            else:
                do_full = viewing and t0 - last_full >= 1 / MANUAL_DET_HZ
            if do_full:
                _, fs = det.detect(lores_img())
                dets = [] if fs is None else [f.copy() for f in fs]
                for f in dets:
                    f[:14] *= W / LW
                full_frames += 1; last_full = t0
            elif dets is not None:
                roi_frames += 1

            if dets is None:   # no detection this frame: carry tracks along with camera motion
                tracks = [dict(tr, box=(tr["box"][0] + sx, tr["box"][1] + sy) + tuple(tr["box"][2:]),
                               f=None) for tr in tracks] if (follow or viewing) else []
                dets = []
                skip_assoc = True
            else:
                skip_assoc = False

            # ---- associate with previous tracks
            new_tracks, used = ([], set()) if not skip_assoc else (tracks, set(range(len(tracks))))
            for f in sorted(dets, key=lambda r: -r[2] * r[3]):
                box = tuple(float(v) for v in f[:4])
                best, bi = 0.25, None
                for i, tr in enumerate(tracks):
                    if i in used:
                        continue
                    px, py, pw, ph = tr["box"]
                    o = iou(box, (px + sx, py + sy, pw, ph))
                    if o > best:
                        best, bi = o, i
                if bi is not None:
                    used.add(bi); tr = dict(tracks[bi]); tr["box"], tr["f"] = box, f
                else:
                    tr = {"id": next_id, "box": box, "f": f, "name": None, "score": 0.0, "ver": -999}
                    next_id += 1
                tr["seen"] = n                   # detected in THIS frame (fresh measurement)
                new_tracks.append(tr)
            if roi is not None:     # ROI frame: faces outside the ROI weren't looked for — keep them
                rx, ry, rs = roi
                for i, tr in enumerate(tracks):
                    px, py, pw, ph = tr["box"]
                    cx, cy = px + pw / 2 + sx, py + ph / 2 + sy
                    if i not in used and not (rx <= cx < rx + rs and ry <= cy < ry + rs):
                        new_tracks.append(dict(tr, box=(px + sx, py + sy, pw, ph), f=None))
            tracks = new_tracks

            # ---- identities: collect finished SFace jobs, queue new (re)checks (non-blocking)
            while not sface_res.empty():
                tid, name, s, t_sub = sface_res.get(); pending.discard(tid)
                for tr in tracks:
                    if tr["id"] == tid and t_sub >= tr["ver"]:
                        apply_identity(tr, name, s, t_sub)
            if enroll is None and db and (follow or viewing):
                def due(tr):            # seconds overdue for an identity re-check
                    gap = (ID_RETRY_S if tr.get("misses") else
                           VERIFY_TARGET_S if tr["name"] == target_name else
                           VERIFY_OTHER_S if tr["name"] else VERIFY_UNKNOWN_S)
                    return t0 - tr["ver"] - gap
                todo = sorted([tr for tr in tracks if due(tr) >= 0 and tr["id"] not in pending
                               and tr.get("seen") == n and tr.get("f") is not None],
                              key=lambda tr: -due(tr))
                for tr in todo[:MAX_SFACE_PER_FRAME]:
                    try:
                        sface_q.put_nowait((tr["id"], main_img(), tr["f"], t0)); pending.add(tr["id"])
                    except queue.Full:
                        break

            # ---- registration
            target = None
            if enroll is not None:
                state = "registering"
                big = [tr for tr in tracks if tr["box"][2] >= ENROLL_MIN_W * 2 and tr.get("seen") == n]
                if big and t0 - enroll.get("t_last", 0) >= ENROLL_EVERY_S:
                    enroll["t_last"] = t0
                    tr = max(big, key=lambda t: t["box"][2] * t["box"][3])
                    e, crop = F.embed(main_img(), tr["f"])
                    E = enroll["embs"]
                    anchor = None
                    if E:
                        m = np.mean(E, axis=0); anchor = m / np.linalg.norm(m)
                    dup = max((float(e @ x) for x in E), default=0.0)
                    relaxed = t0 - enroll["t0"] > ENROLL_DUP_RELAX_S
                    if anchor is not None and float(e @ anchor) < 0.55:
                        enroll["rejected"] += 1           # a different person became largest
                    elif dup < ENROLL_DUP or relaxed:
                        E.append(e); enroll["crops"].append(crop)
                got = len(enroll["embs"])
                if got < ENROLL_SAMPLES:
                    msg = (f"registering {enroll['name']}: {got}/{ENROLL_SAMPLES} — slowly turn your "
                           f"head left, right, up, down" if got else
                           f"registering {enroll['name']}: look at the camera")
                if got >= ENROLL_SAMPLES:
                    out = os.path.join(F.FACES, enroll["name"])
                    shutil.rmtree(out, ignore_errors=True); os.makedirs(os.path.join(out, "crops"))
                    Emat = np.array(enroll["embs"], dtype=np.float32)
                    np.save(os.path.join(out, "embeddings.npy"), Emat)
                    for i, cr in enumerate(enroll["crops"], 1):
                        cv2.imwrite(os.path.join(out, "crops", f"{i:03d}.jpg"), cr)
                    db[enroll["name"]] = Emat; DB_SNAP = dict(db)
                    for tr in tracks:
                        tr["ver"] = -999
                    msg = f"registered {enroll['name']} ({got} samples)"; print(msg, flush=True)
                    target_name = target_name or enroll["name"]
                    enroll = None
                elif t0 - enroll["t0"] > ENROLL_TIMEOUT_S:
                    msg = f"registration timed out ({got}/{ENROLL_SAMPLES}) — get closer / more light"
                    enroll = None

            # ---- lock-follow
            elif not follow:
                state = "manual"
            else:
                cands = [tr for tr in tracks if tr["name"] == target_name and tr.get("seen") == n]
                if cands:
                    # stay on the locked track unless it's in doubt and a clean match exists
                    locked = [tr for tr in cands if tr["id"] == lock_id]
                    clean = [tr for tr in cands if not tr.get("misses")]
                    if locked and (not locked[0].get("misses") or not clean):
                        target = locked[0]
                    else:
                        target = max(clean or cands, key=lambda t: t["box"][2] * t["box"][3])
                    lock_id = target["id"]
                    x, y, w, h = target["box"]
                    ex, ey = x + w / 2 - W / 2, y + h / 2 - H / 2       # +ex face right, +ey face low
                    # absolute direction of the face in servo units at exposure time
                    z = (pose[0] - ex / PAN_PX_PER_US, pose[1] - ey / TILT_PX_PER_US)
                    filt.update(z, ts)
                    err_px = 0.95 * err_px + 0.05 * float(np.hypot(ex, ey))
                    last_seen, wp_i, search_first = t0, None, None
                    last_ex, last_vpan = ex, float(filt.v[0])
                    aim = filt.predict(time.monotonic() + LEAD)
                    sv.go(aim[0], aim[1], dt, MAX_SPEED)
                    trace.append((round(ts, 4), int(roi is not None), round(ex, 1), round(ey, 1),
                                  round(z[0], 1), round(z[1], 1), round(aim[0], 1), round(aim[1], 1),
                                  round(float(filt.v[0])), round(float(filt.v[1])), round(sv.pan, 1),
                                  round(sv.tilt, 1), round(w)))
                    state = "tracking"
                else:
                    # keep the lock if the face is merely not re-detected yet (skipped frame)
                    if not any(tr["id"] == lock_id for tr in tracks):
                        lock_id = None
                    gone = t0 - last_seen
                    exit_dir = exit_direction(last_ex, last_vpan)
                    if filt.x is not None and gone < PREDICT_S:
                        aim = filt.predict(time.monotonic() + LEAD)    # brief coast on the filter
                        sv.go(aim[0], aim[1], dt, MAX_SPEED)
                        state = "tracking"
                    elif gone < (LOST_HOLD_DIR_S if exit_dir else LOST_HOLD_S):
                        state = "lost (holding)"
                    else:
                        state = "searching"; filt.x = None
                        if wp_i is None:
                            if exit_dir:      # first sweep to the end stop on the side you left
                                search_first = (PAN_RANGE[1] if exit_dir > 0 else PAN_RANGE[0], sv.tilt)
                                ref = search_first
                            else:
                                ref = (sv.pan, sv.tilt)
                            wp_i = int(np.argmin([abs(p - ref[0]) + abs(t - ref[1]) for p, t in wps]))
                            last_ex, last_vpan = None, 0.0   # use the hint once
                        gp, gt = search_first or wps[wp_i]
                        dp, dtl = gp - sv.pan, gt - sv.tilt
                        dist, step = np.hypot(dp, dtl), SEARCH_SPEED * dt
                        if dist <= step:
                            sv.go(gp, gt)
                            if search_first:
                                search_first = None
                            else:
                                wp_i = (wp_i + 1) % len(wps)
                        else:
                            sv.pan += dp / dist * step; sv.tilt += dtl / dist * step
                            sv._write()

            proc_ms = 0.9 * proc_ms + 0.1 * (time.monotonic() - t0) * 1000
            age_ms = 0.9 * age_ms + 0.1 * (time.monotonic() - ts) * 1000
            status.update(state=state, follow=follow, target=target_name, pan=int(sv.pan),
                          tilt=int(sv.tilt), fps=round(fps, 1), proc_ms=round(proc_ms, 1),
                          latency_ms=round(age_ms, 1), err_px=round(err_px),
                          score=round(float(target["score"]), 3) if target else None,
                          faces_in_view=len(tracks), registered=sorted(db), msg=msg,
                          enroll=None if enroll is None else
                          {"name": enroll["name"], "got": len(enroll["embs"]), "need": ENROLL_SAMPLES},
                          pan_range=PAN_RANGE, tilt_range=TILT_RANGE)
            if state != logged:
                print(f"{time.strftime('%H:%M:%S')} {state} pan={sv.pan:.0f} tilt={sv.tilt:.0f}", flush=True)
                logged = state
            if n % 300 == 0:
                print(f"{time.strftime('%H:%M:%S')} {fps:.1f} fps, proc {proc_ms:.1f} ms, "
                      f"frame age {age_ms:.1f} ms, roi {roi_frames}/full {full_frames}, "
                      f"err {err_px:.0f}px, {state}",
                      flush=True)
                roi_frames = full_frames = 0

            # ---- hand the frame to the encoder thread (only if someone is watching)
            if time.monotonic() - last_view < 3:
                labels = []
                for tr in tracks:
                    x, y, w, h = map(int, tr["box"])
                    if enroll is not None:
                        labels.append((x, y, w, h, "", (0, 180, 255)))
                        continue
                    col = ((0, 200, 0) if follow and tr is target else
                           (255, 180, 0) if tr["name"] else (0, 0, 255))
                    labels.append((x, y, w, h, f"{tr['name'] or 'unknown'} {tr['score']:.2f}", col))
                hud = None
                if follow and filt.x is not None:   # filter estimate projected into this frame
                    hx = W / 2 - (filt.x[0] - pose[0]) * PAN_PX_PER_US
                    hy = H / 2 - (filt.x[1] - pose[1]) * TILT_PX_PER_US
                    if 0 <= hx < W and 0 <= hy < H:
                        hud = (int(hx), int(hy))
                preview = lores_img()
                with shared_lock:
                    shared.update(img=preview, labels=labels, hud=hud); shared["seq"] += 1
    except KeyboardInterrupt:
        pass
    finally:
        # servos first: each step guarded so one failure can't leave them powered
        for step in (lambda: sv.go(F.HOME[F.PAN_CH], F.HOME[F.TILT_CH]),
                     lambda: time.sleep(0.5), lambda: sv.pca.close(),
                     lambda: req is not None and req.release(), cam.stop):
            try:
                step()
            except Exception as e:
                print(f"cleanup: {e}", flush=True)
        print("stopped, centred and released", flush=True)


if __name__ == "__main__":
    main()
