# Raspberry Pi drone project template

Use a TX16S right stick to aim a physical camera mount and SG to switch object tracking on or off. [RC pan-tilt camera setup](rc_camera/README.md)

Use **Use this template → Create a new repository** on GitHub, then clone your new repository onto the Pi. This template provides a camera preview, a pan/tilt application, and bench utilities. The tested platform is a Raspberry Pi 5 Model B running 64-bit Raspberry Pi OS Trixie with an IMX708 NoIR camera. The [Pi setup template](https://github.com/gregoryerrl/rpi_dev) provides optional SSH access and a fallback hotspot.

## Set up a fresh checkout

From the root of your clone, run:

```sh
sudo apt install python3-venv python3-picamera2 python3-opencv python3-smbus2
/usr/bin/python3 -m venv --system-site-packages venv
venv/bin/python -m pip install -r requirements.txt
venv/bin/python check_setup.py
```

The check runs both model files on synthetic images and lists camera, I²C, and serial availability. It does not capture images or send device commands. An absent flight controller is normal for camera-only work. The virtual environment uses the OS's Picamera2, libcamera, and OpenCV builds. Enrolled face data, the virtual environment, and machine-specific settings belong on the device.

## Camera preview

Run `venv/bin/python cam_server.py` on the Pi. From the same network, open `http://<pi-address>:8000/`. The preview is visible to other clients on that network while it runs. Viewers share a 1280×720 encoder. Closing the last viewer releases the camera. Press Ctrl-C to stop the server.

## Pan/tilt control

The control page listens on the Pi's loopback interface by default. From your laptop, use `ssh -L 8000:127.0.0.1:8000 <pi-user>@<pi-address>`. Open `http://127.0.0.1:8000/` on the laptop once the application is running. For an explicitly trusted LAN, `--bind` selects another listen address. The application has no separate web password.

Use `venv/bin/python follow_me.py --help` to inspect its options. Starting the application initializes the actuators, even in manual mode. Stop the standalone preview before using the pan/tilt application; both use the camera and port 8000 by default. A trace export creates a new private temporary CSV and reports its path in the UI.

## Adapt to your hardware

The supplied configuration assumes an IMX708 camera, I²C bus 1 for the PCA9685, and `/dev/ttyACM0` for the flight controller. Check the connected devices, user permissions and calibration on each Pi. Enable I²C in Raspberry Pi settings when using the controller. A working camera preview and successful software check do not validate actuator wiring or flight readiness.

`diag.py`, `spin.py`, `series.py`, and the calibration scripts are active bench utilities. Some issue actuator commands as soon as they run. Use `check_setup.py` for unattended verification. Validate hardware separately with the aircraft secured and propellers removed.

Run `venv/bin/python -m unittest -v` for regression checks that exercise the control page on loopback and confirm importing the bench modules leaves the hardware idle.
