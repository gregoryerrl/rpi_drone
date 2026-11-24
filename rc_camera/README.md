# RC pan-tilt object tracking

Use the TX16S to aim the stand, then lock onto the visual detail under the center crosshair. The MicoAir flight controller supplies read-only RC inputs. The PCA9685 on the Pi drives the two camera servos. The camera continues processing and accepting radio input when the preview is closed.

| Radio input | Camera control |
| --- | --- |
| Right stick horizontal, CH1 | Pan. More deflection moves the camera faster. |
| Right stick vertical, CH2 | Tilt. A higher pulse points the camera up on the tested mount. |
| SG middle, about 1500 µs | Follow off. Aim manually and release the stick to hold position. |
| SG high, about 2000 µs | Select the object at the crosshair once and start following. |
| SG low, about 1000 µs | Follow off, with manual steering available. |

The manual stick holds the current pose within 30 µs of center. Beyond that, the speed increases gradually for small corrections and more steeply toward full stick. On a single axis, quarter, half, and full travel beyond the deadband command about 84, 315, and 1,800 µs/s on the tested stand. Full speed is reached at 1,050 or 1,950 µs. Releasing the stick holds the new aim.

Open `http://<pi-address>:8001/live` to see the crosshair, channel values and commanded servo positions. Center the sticks, aim at a textured feature with SG in the middle, then flip SG high. The preview shows **Acquiring** and **Following**. If the target is lost, lower SG, aim again and raise it for a fresh selection. A stale radio or camera feed stops movement. After a signal loss during follow, cycle SG to select again. If the servo board disconnects, the preview stays live and the Pi checks for its return once a second.

## Run

From the root of this repository on Raspberry Pi OS, initialize the pinned camera research code and set up the system-site-packages virtual environment described in the main README.

```sh
git submodule update --init --recursive
sudo apt install python3-picamera2 python3-opencv python3-flask python3-smbus2
venv/bin/python -m rc_camera.server --bind "$(hostname -I | awk '{print $1}')" --device /dev/ttyACM0
```

The two camera gains and starting pose default to the measured IMX296 setup on this stand. Use `--camera-gains PAN TILT` and `--start-pose PAN_US TILT_US` for another mount. The radio uses the camera project's calibrated pan and tilt ranges. The server starts on request and releases the devices when stopped.

If the RC keybinding monitor already owns the USB port, share its local input feed. Start the commands in separate terminals on the Pi. If it has several IP addresses, set `pi_addr` to the Wi-Fi address in both terminals. Both processes use the Pi's monotonic clock to check the age of each RC packet.

```sh
# Terminal 1
pi_addr=$(hostname -I | awk '{print $1}')
venv/bin/python rc_monitor/server.py --bind "$pi_addr" --port 8002 --device /dev/ttyACM0
# Terminal 2
pi_addr=$(hostname -I | awk '{print $1}')
venv/bin/python -m rc_camera.server --bind "$pi_addr" --rc-url "http://$pi_addr:8002/api/inputs"
```

The camera page and keybinding monitor are read-only LAN previews. The current camera state is available at `/api/state` and a crosshair-overlaid JPEG at `/frame.jpg` on the camera port.
