# Haras Amor

## Aircraft hardware

| Component | Qty | Model |
| --- | ---: | --- |
| Companion computer | 1 | Raspberry Pi 5 Model B Rev 1.1, 4 GB |
| Flight controller | 1 | MicoAir743v2 |
| Radio receiver | 1 | RP4TD ExpressLRS 2.4 GHz True Diversity Receiver |
| Propulsion motors | 4 | Avenger 2507 V2 |

The ground transmitter is a **RadioMaster TX16S**. Its stick and switch inputs reach the RP4TD over ExpressLRS. The MicoAir reports the received channels to the Pi over USB using MAVLink.

The separate **CV research stand** uses an INNO-MAKER IMX296 camera, two MG996R pan-tilt servos and a PCA9685 servo driver. The RC camera experiment uses that stand to test the radio controls. [RC camera bench setup](rc_camera/README.md)

## Setup on the Pi

Run these commands from the root of the `rpi_drone` checkout on Raspberry Pi OS. The camera tests use the pinned version of the [CV research project](https://github.com/gregoryerrl/cv-research).

```sh
git submodule update --init --recursive -- haras-amor/research/cv-research
cd haras-amor
sudo apt install python3-venv python3-opencv python3-flask python3-smbus2
/usr/bin/python3 -m venv --system-site-packages venv
venv/bin/python -m pip install -r requirements.txt
venv/bin/python check_setup.py
venv/bin/python -m unittest -v
```

The setup check reports Python driver availability and whether the current user can access `/dev/ttyACM0`. The tests use simulated inputs and run with the flight controller disconnected.

## Bench experiments

### Live camera and motor slider

With the propellers removed, secure the frame and keep the motor-power disconnect within reach. The operator preview has one spring-return slider for all four motors, from 0 to 50% in 1% steps. Hold and drag with a mouse or touch, or focus the slider and hold an arrow key. Releasing returns it to zero and releases actuator control. Closing or hiding the page, losing focus, or losing the connection ends the hold. Each positive command also expires in PX4 after 0.6 seconds without a fresh update.

Run the following from `haras-amor/` on the Pi to start the preview on demand.

```sh
sudo apt install python3-picamera2
venv/bin/python -m motor_preview.server
```

Open `http://<pi-address>:8000/` for the LAN camera view. To control the motors, run this on the laptop with your Pi login and address, then open `http://127.0.0.1:8080/`. The operator tab uses the FC USB port while connected. Close it to release the port for another experiment.

```sh
ssh -N -L 127.0.0.1:8080:127.0.0.1:8000 <pi-user>@<pi-address>
```

The controller requires a disarmed PX4 quadrotor. It uses the MicoAir USB device by ID. Pass `--device` for another serial path. The read-only camera view works independently of the flight controller.

### Other experiments

- **RC input monitor.** See which channels change as you move a stick or flip a switch. Run these commands from `haras-amor/` and open `http://<pi-address>:8002/` from the same network. If the Pi has several addresses, use its Wi-Fi or hotspot address for `pi_addr`. Set `--device` to match the controller’s serial port. [RC monitor source](rc_monitor/server.py)

  ```sh
  pi_addr=$(hostname -I | awk '{print $1}')
  venv/bin/python rc_monitor/server.py --bind "$pi_addr" --device /dev/ttyACM0
  ```

- **RC camera stand.** The TX16S right stick aims the stand. SG on CH8 selects manual or follow mode. The camera can share the RC monitor’s input feed. [Setup and channel mapping](rc_camera/README.md)
- **Motor bench utilities.** `diag.py`, `spin.py` and `series.py` can run the propulsion motors. Secure the aircraft and remove the propellers before running them.
