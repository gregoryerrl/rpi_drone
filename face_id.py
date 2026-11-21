#!/usr/bin/env python3
# face_id.py — enroll / recognize faces with the pan-tilt camera (all data stays local)
#   /usr/bin/python3 face_id.py enroll [--name owner] [--samples 40]
#   /usr/bin/python3 face_id.py recognize [--seconds 15]
#   /usr/bin/python3 face_id.py forget --name owner        # delete stored biometrics
# Models: OpenCV Zoo YuNet (detect) + SFace (128-d embedding), in ./models.
# Storage: faces/<name>/embeddings.npy (+ aligned 112x112 crops for review).
import argparse, os, shutil, time
from functools import cache
import cv2, numpy as np
from picamera2 import Picamera2
from servo_test import PCA9685

HERE = os.path.dirname(os.path.abspath(__file__))
DET_MODEL = os.path.join(HERE, "models/face_detection_yunet_2023mar.onnx")
REC_MODEL = os.path.join(HERE, "models/face_recognition_sface_2021dec.onnx")
FACES = os.path.join(HERE, "faces")
SIZE = (1280, 720)
MATCH = 0.363          # SFace cosine threshold recommended by OpenCV
TILT_CH, PAN_CH = 0, 1  # ch0 tilt (+us = up), ch1 pan (+us = left)
HOME = {TILT_CH: 1350, PAN_CH: 1500}

@cache
def detector():
    return cv2.FaceDetectorYN.create(DET_MODEL, "", SIZE, 0.85)


@cache
def recognizer():
    return cv2.FaceRecognizerSF.create(REC_MODEL, "")


def open_cam():
    cam = Picamera2()
    # "RGB888" in picamera2 is BGR byte order, i.e. what OpenCV expects
    cam.configure(cam.create_video_configuration(main={"size": SIZE, "format": "RGB888"}))
    cam.start(); time.sleep(1.0)
    return cam


def faces_in(img):
    _, f = detector().detect(img)
    return [] if f is None else sorted(f, key=lambda r: -r[2] * r[3])   # largest first


def embed(img, face):
    rec = recognizer()
    crop = rec.alignCrop(img, face)
    e = rec.feature(crop).flatten()
    return e / np.linalg.norm(e), crop


def load_db():
    db = {}
    if os.path.isdir(FACES):
        for name in os.listdir(FACES):
            p = os.path.join(FACES, name, "embeddings.npy")
            if os.path.exists(p):
                db[name] = np.load(p)
    return db


def enroll(name, n_samples, min_w):
    cam, pca = open_cam(), PCA9685()
    out = os.path.join(FACES, name); os.makedirs(os.path.join(out, "crops"), exist_ok=True)
    # small pan/tilt offsets give the model a few viewing angles
    poses = [(0, 0), (40, 0), (-40, 0), (0, 40), (0, -40)]
    embs, rejected = [], 0
    try:
        per_pose = max(1, n_samples // len(poses))
        for dt, dp in poses:
            pca.set_pulse_us(TILT_CH, HOME[TILT_CH] + dt); pca.set_pulse_us(PAN_CH, HOME[PAN_CH] + dp)
            time.sleep(0.8)
            got, tries = 0, 0
            while got < per_pose and tries < per_pose * 6:
                tries += 1
                img = cam.capture_array()
                fs = faces_in(img)
                if not fs or fs[0][2] < min_w:
                    continue
                e, crop = embed(img, fs[0])
                if embs:
                    anchor = np.mean(embs, axis=0); anchor /= np.linalg.norm(anchor)
                    if float(e @ anchor) < 0.55:      # different person became largest face
                        rejected += 1
                        continue
                embs.append(e); got += 1
                cv2.imwrite(os.path.join(out, "crops", f"{len(embs):03d}.jpg"), crop)
                time.sleep(0.25)
            print(f"pose tilt{dt:+d} pan{dp:+d}: {got} samples", flush=True)
    finally:
        for ch, us in HOME.items():
            pca.set_pulse_us(ch, us)
        time.sleep(0.4); pca.close(); cam.stop()
    if len(embs) < 5:
        raise SystemExit(f"only {len(embs)} usable samples — sit closer / face the camera and retry")
    E = np.array(embs, dtype=np.float32)
    np.save(os.path.join(out, "embeddings.npy"), E)
    c = E.mean(0); c /= np.linalg.norm(c)
    sims = E @ c
    print(f"enrolled '{name}': {len(E)} samples, rejected {rejected} non-matching, "
          f"self-similarity min {sims.min():.2f} mean {sims.mean():.2f}")


def identify(e, db):
    best, best_s = None, -1.0
    for name, E in db.items():
        s = float(np.max(E @ e))
        if s > best_s:
            best, best_s = name, s
    return (best, best_s) if best_s >= MATCH else (None, best_s)


def recognize(seconds, save):
    db = load_db()
    if not db:
        raise SystemExit("no enrolled faces — run 'enroll' first")
    cam, pca = open_cam(), PCA9685()
    for ch, us in HOME.items():
        pca.set_pulse_us(ch, us)
    t0, last = time.time(), None
    try:
        while time.time() - t0 < seconds:
            img = cam.capture_array()
            labels = []
            for f in faces_in(img):
                e, _ = embed(img, f)
                name, s = identify(e, db)
                labels.append(f"{name or 'unknown'}({s:.2f})")
                if save:
                    x, y, w, h = map(int, f[:4])
                    col = (0, 200, 0) if name else (0, 0, 255)
                    cv2.rectangle(img, (x, y), (x + w, y + h), col, 3)
                    cv2.putText(img, f"{name or 'unknown'} {s:.2f}", (x, y - 8), 0, 0.9, col, 2)
            line = ", ".join(labels) or "no face"
            if line != last:
                print(f"{time.time()-t0:5.1f}s  {line}", flush=True); last = line
            if save:
                cv2.imwrite(save, img)
            time.sleep(0.3)
    finally:
        pca.close(); cam.stop()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["enroll", "recognize", "forget"])
    ap.add_argument("--name", default="owner")
    ap.add_argument("--samples", type=int, default=40)
    ap.add_argument("--min-width", type=int, default=150, help="min face width px to enroll")
    ap.add_argument("--seconds", type=float, default=15)
    ap.add_argument("--save", help="write last annotated frame here (recognize)")
    a = ap.parse_args()
    if a.cmd == "enroll":
        enroll(a.name, a.samples, a.min_width)
    elif a.cmd == "recognize":
        recognize(a.seconds, a.save)
    else:
        shutil.rmtree(os.path.join(FACES, a.name), ignore_errors=True); print(f"deleted faces/{a.name}")
