# rpi_drone on Raspberry Pi 5

The camera programs use the system Picamera2/libcamera and OpenCV packages. The project virtual environment on this Pi can also import them, alongside its MAVLink and serial packages. To recreate an environment on Raspberry Pi OS Trixie with the same dependencies:

```sh
sudo apt install python3-venv python3-picamera2 python3-opencv python3-smbus2
python3 -m venv --system-site-packages venv
venv/bin/pip install pymavlink==2.4.50 pyserial==3.5
```

## Camera preview over SSH

From the project directory on the Pi, run `venv/bin/python cam_server.py`. Open `http://<pi-address>:8000/` from the same network. The preview captures 1280×720 video on demand. Viewers share one encoder, and closing the last stream releases the camera for other programs. Press Ctrl-C in the SSH session to stop the server.

The pan/tilt application has a separate web interface. Run `venv/bin/python follow_me.py --help` to see its options. Both servers use port 8000 by default. Starting the pan/tilt application initializes the actuators, including in manual mode.
