# Checkpoint 1 (2026-11-27)

The Pi's CH8 lock-and-follow on the jig, after PX4's motor layout was fixed (front and back) and
before PX4's rate gains were changed. Holding the object still, the drone stayed put with the
object within 3% of the picture's centre; following a moving object was still slow.

- Code: the git tag `checkpoint-1`.
- Flight controller: `px4.params`, all 978 PX4 1.15.4 parameters. Among them:
  - motor layout: CA_ROTOR0-3_PX -1/+1/-1/+1, _PY -1/+1/+1/-1 and _KM +/+/-/- (a standard
    quad X: M2 front-right CCW, M4 front-left CW, M1 back-left CCW, M3 back-right CW);
  - SENS_BOARD_ROT 12, RC_MAP_OFFB_SW 8 and COM_FLTMODE1 8;
  - rate gains MC_YAWRATE_K 1.2, MC_ROLLRATE_K 0.35 and MC_PITCHRATE_K 0.4.
- Camera mount: `camera-mount.json`, the copy of `~/.config/rpi_drone/haras-amor/camera-mount.json`.

## Going back

1. Code: `git checkout checkpoint-1`, or `git switch -c <new-branch> checkpoint-1` to work from it.
2. Flight controller, with the drone disarmed and the service stopped
   (`systemctl --user stop ch8-takeover`):
   - `venv/bin/python px4_params.py diff checkpoints/checkpoint-1/px4.params` shows what has
     changed since;
   - `venv/bin/python px4_params.py restore checkpoints/checkpoint-1/px4.params` sets those back.

   QGroundControl can load the same file too: Parameters, Tools, Load from file.
3. Camera: `cp checkpoints/checkpoint-1/camera-mount.json ~/.config/rpi_drone/haras-amor/`, then
   `systemctl --user start ch8-takeover`.
