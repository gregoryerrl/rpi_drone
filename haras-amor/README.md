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

### Live camera, radio switches and flight-jig attitude

Start the jig page on the Pi from `haras-amor/` with `venv/bin/python -m jig_view.server`. Open `http://<pi-address>:8003/` on the same network. This command uses the aircraft camera in its mounted orientation and opens the MicoAir USB port read-only. Close any other experiment that owns the camera or FC serial port first. It runs on demand and continues listening to the TX16S when the browser is closed.

**Aircraft battery** shows pack voltage, current, power, and remaining charge from the flight controller. Watts use voltage and current from the same sample. Separate battery reports appear individually. Dashes mark unavailable measurements, and **Not reported** means the FC is online without fresh pack data. The MicoAir743v2 senses pack voltage onboard and accepts a current-sensor signal at the ESC connector. [MicoAir battery monitoring](https://micoair.com/en/docs/flight-controller/micoair743/micoair743v2-flight-controller-manual#battery-voltage-and-current-monitoring) · [MAVLink battery protocol](https://mavlink.io/en/services/battery.html)

The camera shows the full frame in its mounted orientation. The page also shows measured FC pitch and roll, with two ±45° sliders for comparing reference angles while moving the jig by hand. The radio panel shows CH1 roll, CH2 pitch, CH3 stick throttle, CH5 arm input, CH6 kill input, and CH8 switch-test input.

Open **Try the on-screen switches** on the jig page to use the switch simulator. Raise virtual CH5 to start at 1% manual throttle. Raise virtual CH8 to display 25% on all four outputs for up to 20 seconds. At timeout all four simulated outputs go to zero and stay there until a fresh switch action. Virtual CH6 cancels the timer and sets all four outputs to zero immediately. The simulator runs entirely in the browser.

To test with the actual TX16S, open the **TX16S switch test** at `http://<pi-address>:8003/#radio-test`. You can leave CH5 low and motor power disconnected. With CH6 clear, move CH8 from low or middle to high. A browser meter shows a simulated 15% for M1–M4 for up to 20 seconds. Moving CH8 back down or raising CH6 clears the meter. A CH6 tap cancels the interval even if the switch returns low between browser updates. Cycle CH8 for a new interval. The Pi reads the receiver pulses independently of the camera, and a break in the radio signal clears the switch history. [MAVLink RC channel telemetry](https://mavlink.io/en/messages/common.html#RC_CHANNELS)

On Haras Amor, PX4 maps the radio kill switch to CH6. With the TX16S as the active input, CH6 high cuts motor outputs immediately. Keep it high until PX4 reports disarmed. The configured delay is five seconds and releasing the switch earlier can restart the motors. [PX4 kill switch](https://docs.px4.io/v1.15/en/config/safety#kill-switch)

#### CH8 motor test with CH5 enabled

Remove the propellers, secure the frame and keep CH6 clear. CH5 enables the test while PX4 reports **Disarmed**. The Pi confirms the FC's actual arm state from a heartbeat after CH5 goes high. PX4 also checks its arm state when each motor command arrives. [PX4 actuator test](https://github.com/PX4/PX4-Autopilot/blob/v1.15.4/src/modules/commander/Commander.cpp#L1422-L1469)

Start the jig server in motor-control mode from `haras-amor/` with CH8 low or middle. Stop the read-only jig server first if it already owns the serial port.

```sh
venv/bin/python -m jig_view.server --ch5-override
```

With CH5 high, move CH8 from low or middle to high. The Pi sends motor-test commands directly to the FC for **15% on each of M1–M4 for up to 20 seconds**. Lowering CH5 or CH8, raising CH6, arming PX4, losing fresh receiver data or engaging the Pi motor stop ends the test early. With CH5 still high and CH6 clear, cycle CH8 for another run. A CH8 switch already high at startup or after an interruption must be cycled. The radio test runs independently of the camera and browser. If USB or the Pi drops out, the last command expires in PX4 within 0.6 seconds.

The jig page shows **Pi control** after PX4 accepts all four commands. The percentage is the requested motor output. Measuring propeller RPM requires separate feedback.

### Live camera and motor slider

With the propellers removed, secure the frame and keep the motor-power disconnect within reach. The motor bench slider selects a level for all four motors, from 0 to 50% in 1% steps. **Hold to run bench** applies that selection while pressed with a mouse, touch, Space or Enter. Releasing the button ends the command and leaves the slider where you set it. Closing or hiding the page, losing focus, or losing the connection ends the hold. A hold that reaches 15% starts a 20-second cutoff at the motor driver. Repeated updates share that deadline. At timeout the Pi releases all four outputs and ends the hold. Each positive test command also expires in PX4 after at most 0.6 seconds, shortened to the time remaining.

Run the following from `haras-amor/` on the Pi to start the preview on demand.

```sh
sudo apt install python3-picamera2
venv/bin/python -m motor_preview.server
```

Open `http://<pi-address>:8000/` for the LAN camera view. To control the motors, run this on the laptop with your Pi login and address, then open `http://127.0.0.1:8080/`. The operator tab uses the FC USB port while connected. Close it to release the port for another experiment.

```sh
ssh -N -L 127.0.0.1:8080:127.0.0.1:8000 <pi-user>@<pi-address>
```

The manual motor test requires a disarmed PX4 quadrotor. The server uses the MicoAir USB device by ID. Pass `--device` for another serial path. The read-only camera view works independently of the flight controller.

In the SSH operator preview, click **Stop motors** or press **Esc** to end a bench motor test. The Pi blocks further output immediately and sends a release for each motor it commanded, including when it is still waiting for the FC to acknowledge one. The button also works from a second operator tab when the first tab owns the controls. The camera and leveling telemetry stay available. [MAVLink actuator test and release](https://mavlink.io/en/messages/common.html#MAV_CMD_ACTUATOR_TEST)

The stop stays latched across page reloads and service restarts. **Reset motor stop** allows a fresh hold. Both selections start at zero after a stop or reconnection. The on-screen confirmation means the Pi has blocked its own bench commands. If the browser cannot confirm the stop, or if the aircraft is under independent FC or radio control, use the independent kill switch or motor-power disconnect.

### Pi-only jig stabilization

On the SSH operator page, selecting **Auto stabilize** starts a disarmed angle check. The Pi supplies joystick input over USB. **Standby** means PX4 confirmed Stabilized mode and is disarmed. Move a slider and **FC target** should follow it. With motor power disconnected and **Level (0° / 0°)** selected, gently tilting the jig should change **Measured angle** while **FC target** stays level. The per-axis motor preview uses a **45% reference**. Up and down arrows predict which motors would increase or decrease from that reference. The actual throttle command in standby is zero. An armed session shows the throttle stick percentage sent by the Pi. This page has no motor speed feedback. [PX4 Stabilized mode for this firmware](https://docs.px4.io/v1.15/en/flight_modes_mc/manual_stabilized)

The target limit is **5° of combined tilt**. The angle sliders share this limit. The display beside **Arm secured jig…** shows measured combined tilt and whether it meets the start condition. A blocked tilt check leaves the jig in standby with zero throttle. Arming requires a fresh two-step request, level targets, and PX4’s normal arming checks. With the TX16S off, keep the independent motor-power disconnect within reach.

The ground jig has its own throttle preset. You can set it while the operator page is connected and disarmed. The Pi sends zero jig throttle in standby. **Hold to apply jig throttle** sends the selected percentage as collective joystick input during an armed jig session. The arrow keys adjust the two angle targets during a jig hold. PX4 balances the four motors. Arming can spin motors at zero stick. An individual motor can exceed the 50% stick setting. Releasing the jig button, switching off, **Stop motors** or **Esc**, or loss of the operator session sends neutral input and requests an ordinary disarm. The service also ends the run on stale attitude, loss of the on-ground indication, or measured tilt above 10°. Keep this page out of the flight control path. If PX4 declines disarm or the browser cannot confirm it, use the independent power disconnect. The operator page retains the last arm request and its outcome across tab changes until the service restarts. Closing or hiding the operator tab ends the session.

Haras Amor runs PX4 1.15.4. For this Pi-only trial, its saved `COM_RC_IN_MODE` changed from `0` (radio only) to `2` (first valid radio or joystick, with fallback). The TX16S stays off while selecting the Pi. Radio switches including CH6 are ignored while the MAVLink joystick is the selected input on this firmware. The configured 35° manual tilt scale is read from PX4. The 5° trial limit is applied in the Pi service. [PX4 input parameters](https://docs.px4.io/v1.15/en/advanced_config/parameter_reference#COM_RC_IN_MODE) · [PX4 1.15.4 switch handling](https://github.com/PX4/PX4-Autopilot/blob/v1.15.4/src/modules/manual_control/ManualControl.cpp#L154-L162)

#### Sensor display and motor geometry

The correction column estimates how to move from the measured tilt toward the selected angles. It uses fused attitude and body rates from the same MAVLink `ATTITUDE` sample, with damping proportional to angular rate. The display model has a natural frequency of 1/s, critical damping, and a combined angular-acceleration bound of 15°/s². PX4's native controller supplies the actual stabilization. [Display model](motor_preview/leveling.py) · [MAVLink attitude fields](https://mavlink.io/en/messages/common.html#ATTITUDE)

For each connection, the preview reads the rotor count, positions and thrust axes from PX4. On Haras Amor, airframe 4001 reported the following logical motor positions as viewed from above with the nose forward. Physical wiring needs a separate identification check. [PX4 motor geometry](https://docs.px4.io/main/en/config/actuators#motor-geometry-multicopter)

| M1 | M2 | M3 | M4 |
| --- | --- | --- | --- |
| Front-right | Rear-left | Front-left | Rear-right |

At rest, a right-side drop calls for more lift on M1 and M4. A nose drop calls for more lift on M1 and M3. The panel shows each axis independently and suppresses motor hints smaller than 0.5°/s². It clears corrections when samples stop advancing for 350 ms, contain invalid values, or the frame leaves the upright hemisphere. Incomplete or unsupported rotor geometry hides the motor numbers.

The read-only JSON feed is at `/api/leveling`. Both views share the operator tab’s USB connection. The regression tests include a simulated jig recovering from an angular disturbance and a simulated FC with a different motor order.

### Other experiments

- **RC input monitor.** The monitor opens the FC USB port read-only. It shows the current armed state and flight mode from the FC heartbeat, measured roll and pitch, and changes to the receiver channels. For a radio-controlled session, close the browser motor controller. The standalone camera preview and RC monitor can run together. Run these commands from `haras-amor/` and open `http://<pi-address>:8002/` from the same network. If the Pi has several addresses, use its Wi-Fi or hotspot address for `pi_addr`. Set `--device` to match the controller’s serial port. [RC monitor source](rc_monitor/server.py)

  ```sh
  pi_addr=$(hostname -I | awk '{print $1}')
  venv/bin/python rc_monitor/server.py --bind "$pi_addr" --device /dev/ttyACM0
  ```

- **RC camera stand.** The TX16S right stick aims the stand. SG on CH8 selects manual or follow mode. The camera can share the RC monitor’s input feed. [Setup and channel mapping](rc_camera/README.md)
- **Motor bench utilities.** `diag.py`, `spin.py` and `series.py` can run the propulsion motors. Secure the aircraft and remove the propellers before running them.
- **Flight-controller checks and checkpoints.** `check_layout.py` spins PX4's front, back, left and right motor pairs on the jig (disarmed, CH6 low) and reports whether each tips the drone the way PX4's motor layout expects. `px4_params.py` saves every PX4 parameter to a QGroundControl `.params` file, shows what has changed since one, and restores it. `checkpoints/` holds saved states of the code, the PX4 parameters and the camera mount, each with how to go back.
- **CH8 handover service and camera website.** The `ch8-takeover` user service runs `ch8_takeover.py` from boot and holds the FC USB port and the aircraft camera. Open `http://<pi-address>:8004/` for the live camera with a box in the middle and the lock, CH8, camera and drone status. The service uses the stand setting, with the frame strapped down: 20% thrust with the locked object straight above, rising smoothly to 50% the further the drone has to lean to centre it, or the further it still is from that lean (`--thrust` and `--max-thrust` set the range; equal values give a fixed thrust). Moving CH8 from low to the middle locks the object in the box, and CH8 low unlocks it. Arm with CH5 while CH6 and CH8 are low and you control the motors. Raising CH8 hands them to the Pi in PX4 Offboard mode. It keeps the heading it had at the handover and tilts the drone (roll and pitch, up to 10° combined by default; `--max-tilt` raises it, up to 90° on the stand, which caps nothing, or 45° with `--flight`) to keep the locked object in the middle of the picture, easing back to level when nothing is locked. The camera runs at 60 fps and looks up the drone's attitude (streamed at 100 Hz) at the middle of each frame's exposure, and exact rotation geometry turns the object's place in the picture into the lean that centres it, at any tilt. A brief uncertain match holds the lean, and the lean follows at up to 180°/s. The camera looks straight up with the nose at the top of the picture. `calibrate_camera.py` measures how the picture slides as the drone rolls on the jig (props on, service stopped) and saves the result to `~/.config/rpi_drone/haras-amor/camera-mount.json` for the camera to use. It refuses to save if PX4's roll still responds backwards to the motors. If the drone tilts past 30°, or 15° past a larger tilt limit, the Pi withdraws. Lowering CH8 gives them back. For flight, stop the service and run `venv/bin/python ch8_takeover.py --flight`. The Pi then holds the height the drone had at the handover, starting from your thrust, and damps the follow so the drone settles under the object. It only takes over while you fly in Stabilized or Altitude, PX4 reports the drone in the air and it is below 3 m. It gives control back if the height drifts 1 m or PX4's height data stops. Keep the throttle stick at hover while the Pi flies, because taking control back returns your stick's thrust at once. PX4 uses CH8 as its Offboard switch (`RC_MAP_OFFB_SW=8`). PX4 keeps stabilizing, so on a props-off bench individual motors spread well above and below 20%. Stop the service with `systemctl --user stop ch8-takeover` before another experiment uses the FC port or the camera, and start it again afterwards. [Source and safety rules](ch8_takeover.py) · [Camera](follow_camera/camera.py) · [Service unit](ch8-takeover.service)
