# Build, connect and test your robot

Your team wires/checks the hardware, builds the supplied ROS baseline and student program together in the robot workspace and verifies operation. The Jetson already has the commissioned OS/firmware, ROS 2 Jazzy, Python environments, device permissions, robot identity and standalone OLED/camera runtime. Do not reflash it or repeat router commissioning. Ask staff if those prerequisites fail.

## Before starting

- Identify your robot/router number. Use that router throughout; different routers reuse the same IP subnet.
- Gather the Jetson, its commissioned power bank/lead, RVR with charged battery, OLED and jumper lead, USB camera, F710/receiver and your PC. A USB keyboard is useful for the OLED menu; a monitor is optional.
- Secure the Jetson, display, camera and power bank to the robot with suitable mounts. Keep wires away from treads, avoid exposed contacts and strain on the header, and keep the RVR power control accessible.
- Your PC needs an SSH client and a compatible ROS 2 Jazzy desktop environment for RViz. See [PC viewer setup](../viewer/README.md). Building/downloading dependencies requires internet access; robot-local operation does not require an upstream internet connection.

## Hardware connections and pinout

### Wiring references

| Hardware | Documentation |
| --- | --- |
| Jetson Orin Nano | NVIDIA [carrier-board specification: 40-pin expansion header pin assignments](https://developer.nvidia.com/downloads/assets/embedded/secure/jetson/orin_nano/docs/jetson_orin_nano_devkit_carrier_board_specification_sp.pdf) and [hardware layout: power, USB, DisplayPort and header locations](https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/latest/hardware_layout.html). |
| Sphero RVR | Sphero [UART wiring diagram and TX/RX connections](https://sdk.sphero.com/raspberry-pi-setup/python-sdk-setup) and [UART/USB power and signal levels](https://help.sphero.com/sphero-support/rvr-and-rvr-usb-and-uart-power-information). The wiring example uses a Raspberry Pi; use the Jetson pin table below for this lab, with RVR 5 V unconnected. |
| Adafruit 4440 OLED | [Exact 0.91-inch 128×32 I2C module: board photos, electrical details and address](https://www.adafruit.com/product/4440), [OLED I2C wiring guide](https://learn.adafruit.com/monochrome-oled-breakouts/wiring-128x32-i2c-display) and [STEMMA QT connector reference](https://cdn-learn.adafruit.com/downloads/pdf/introducing-adafruit-stemma-qt.pdf). The tutorial also covers older boards/Arduino examples; use the four-wire, 3.3 V lab mapping below. |
| Logitech F710 | Logitech [quick-start guide: USB receiver, X/D switch and MODE button](https://www.logitech.com/assets/48300/f710-wireless-gamepad-quick-start-guide.pdf). |
| TP-Link Archer AX21 | TP-Link [installation/user guides](https://www.tp-link.com/us/support/download/archer-ax21/): select the hardware revision printed on your router for LAN/WAN and power connection diagrams. |

### Pin numbers and connections

This is the tested lab wiring for the Jetson Orin Nano, Sphero RVR and Adafruit 4440 OLED. **Power off the Jetson and RVR before changing wiring.** Header numbers below are **physical positions on the Jetson's 40-pin header**, not GPIO/BCM numbers. Locate pin 1 using the board's pin-1 marking before counting; do not infer orientation from a rotated photo.

#### RVR UART and OLED

| Jetson physical pin | Signal | Connect to |
| --- | --- | --- |
| **1** | 3.3 V | OLED **VIN** |
| **3** | I2C SDA | OLED **SDA** |
| **5** | I2C SCL | OLED **SCL** |
| **6** | Ground | RVR UART **GND** |
| **8** | UART TX | RVR UART **RX** |
| **9** | Ground | OLED **GND** |
| **10** | UART RX | RVR UART **TX** |

UART crosses **TX → RX** and **RX ← TX**, with a common ground. Connect these three wires to the RVR's UART expansion connector, following its signal labels. **Leave the RVR UART 5 V pin unconnected.** Do not connect a power pin to TX, RX, SDA or SCL, and do not use the RVR UART header to power the Jetson.

As a general wiring convention, use **red for VCC/power** and **black for ground**. Signal-wire colors are your choice; identify every connection by its pin/signal label rather than assuming a color. The fitted OLED is I2C, not SPI. The commissioned image uses UART `/dev/ttyTHS1` and OLED `/dev/i2c-7`, address `0x3C`; another image may need its header interfaces configured. Ask staff if these commissioned device interfaces are missing; base installation does not configure or flash the OS.

### Power and peripherals

| Item | Connection / setup |
| --- | --- |
| Jetson power | Fitted Belkin **20K UltraCharge** power bank → fitted USB-to-barrel lead → Jetson barrel power input. The lead is specified as **12 V / 5 A**; this is its rating, not a measured guarantee of power-bank output. Use the commissioned power assembly. |
| RVR power | RVR's own battery; turn the base on separately for UART/sensor/driving tests. |
| OLED power | Jetson header pin **1 (3.3 V)** and pin **9 (GND)**, as above; no separate supply. |
| Camera | Arducam **UC684 Rev F3** → Jetson USB host port. |
| Joystick | Logitech **F710 USB receiver** → Jetson USB host port. Set the controller switch to **X** and leave the **MODE light off**. |
| Keyboard | USB keyboard or receiver → Jetson USB host port; **F8** (or **Fn+F8**) opens the OLED menu. |
| PC maintenance cable | PC USB → Jetson **USB-C data port**. The commissioned USB network uses Jetson `192.168.55.1`; keep the Jetson's separate barrel power connected. |
| Optional monitor | Monitor → Jetson **DisplayPort**. Normal SSH/OLED operation does not require a monitor. |
| Router | Power its supplied router, connect the PC to a **LAN** port or its Wi-Fi, and connect the Jetson to that same router's Wi-Fi. Use the **WAN** port for an upstream network when available. See [connection details](#connect). |

After powering up, use the OLED's IP for SSH. The standalone OLED and camera services are `rvr-oled.service` and `rvr-camera.service`; ROS starts separately. Stop the browser camera service before ROS opens the camera, and run only one RVR controller at a time. Check sensors with `motion_enabled:=false` before the controlled movement test described below.

## Get the source

Clone this repository **on your PC**.

```bash
cd ~
git clone https://github.com/plancherb1/cosc-281-f26-project.git a2r-student
```

The repository is public, so no GitHub sign-in is needed. Do not overwrite an existing `~/a2r-student` folder.

```text
~/a2r-student/
├── robot/src/        # Baseline + student program; build on Jetson
├── viewer/src/       # Display tools; build on PC
└── docs/             # Instructions for both machines
```

## Connect

Use only the router supplied with your robot. Join its Wi-Fi (or connect your PC to a router LAN port):

| Connection | Name / username | Password |
| --- | --- | --- |
| 2.4 GHz Wi-Fi | Robot 1: `a2r-projects`; robots 2–5: `a2r-projects-N` (replace `N` with the robot number) | See Canvas |
| 5 GHz Wi-Fi | Same assigned SSID with `_5G` appended | See Canvas |
| SSH, all robots | `a2r-lab-jetson` | See Canvas |

The passwords are posted on Canvas, not in this public repository; keep them out of your commits. Each Jetson has an address reservation on its assigned router. Read its current IP from the OLED and replace `ROBOT_IP` below:

```bash
ssh a2r-lab-jetson@ROBOT_IP
```

Enter the SSH password from Canvas when prompted; typed characters are not displayed. Commands entered after connecting run on the Jetson. RViz runs in a separate terminal on your own computer, outside SSH. No SSH alias is required. No monitor or USB maintenance/flashing connection is needed for operation.

## Check the commissioned prerequisites

On the Jetson, with RVR drive power off during installation:

```bash
hostname
cat /etc/a2r-robot/robot.env
test -r /opt/ros/jazzy/setup.bash && echo 'ROS setup present'
test -x ~/ros-venv/bin/python && echo 'Python environment present'
source /opt/ros/jazzy/setup.bash
~/ros-venv/bin/python -c 'import rclpy, evdev, serial, serial_asyncio, smbus2, cv2; print("Python imports OK")'
ls -l /dev/ttyTHS1 /dev/i2c-7 /dev/input/by-id/
ls -l /dev/v4l/by-id/
systemctl --user status --no-pager rvr-oled.service rvr-camera.service
```

Expect your assigned hostname/namespace/domain, accessible device nodes, successful imports and both services active with the camera attached. Open `http://ROBOT_IP:8080/` on your PC and confirm that the camera image changes live; check that the OLED shows the correct identity/IP. Missing runtime, permission errors or a failing NVIDIA/firmware boot are staff prerequisites to repair, not something fixed by rebuilding student code.

## Build the robot workspace on the Jetson

### 1. Log into the Jetson

From your **PC**, connect to the robot's Wi-Fi and open an SSH terminal. Replace `ROBOT_IP` with the address on the display:

```bash
ssh a2r-lab-jetson@ROBOT_IP
```

Enter the SSH password from Canvas. **The following steps run on the Jetson in this SSH terminal.** Keep RVR drive power off during the build and stop any running robot-control programs.

### 2. Clone the repository on the Jetson

```bash
cd ~
git clone https://github.com/plancherb1/cosc-281-f26-project.git a2r-student
```

If the Jetson cannot access GitHub but you already cloned the folder locally you can zip and move the data with something like (replace `ROBOT_IP`):
```bash
ssh a2r-lab-jetson@ROBOT_IP
mkdir a2r-student
scp a2r-student.zip a2r-lab-jetson@ROBOT_IP:a2r-student
ssh a2r-lab-jetson@ROBOT_IP
cd a2r-student
unzip a2r-student.zip
mv a2r-student/* .
rm -r -f a2r-student
```
Note that this bulk copy will move over the hidden `.git` folder and be tied to your credentials etc. so you should be careful there. You can do more fancy zips or modifications locally or just download the zip from github first to get the raw code etc.

Use this copy method before building, so you do not transfer generated build files from one machine to another. Do not copy personal keys or tokens to the Jetson. Both methods produce the same repository layout. Build directly inside `robot/` on the Jetson and `viewer/` on your PC; no package copying or separate base workspace is needed.

Check the source folders:

```bash
ls ~/a2r-student/robot/src
```

**Check:** it should list `a2r_robot`, `a2r_robot_interfaces`, `course_robot_interfaces` and `team_navigation`. These are the four ROS packages you will build together. The first three are the baseline; `team_navigation` is the supplied student starting point. You can modify the baseline as part of your project.

### 3. Load the tools and build

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros-venv/bin/activate
cd ~/a2r-student/robot
python -m colcon build --symlink-install
```

The first command loads ROS. The second selects the supplied Python environment containing the device dependencies. The build runs in `robot/`, which is already a ROS workspace because it contains `src/`. It creates `build/`, `install/` and `log/` there; do not create another workspace or copy packages elsewhere.

**Check:** the summary should report `4 packages finished`, with no failed packages. Stop and resolve any build error before launching the robot. This builds both the baseline and student program; there is no second student overlay to build.

To rebuild after editing any robot package, stop its running program, open a fresh Jetson terminal, load ROS and the Python environment as above, and repeat the build in `~/a2r-student/robot`. Do not source the old workspace install before rebuilding it.

## Prepare each Jetson terminal

Run this in **every fresh Jetson runtime/check terminal**:

```bash
source /opt/ros/jazzy/setup.bash
source ~/a2r-student/robot/install/setup.bash
source ~/ros-venv/bin/activate
source /etc/a2r-robot/robot.sh
printf 'robot=%s domain=%s\n' "$ROBOT_NAMESPACE" "$ROS_DOMAIN_ID"
ros2 pkg prefix a2r_robot
```

Robot N uses namespace `jetsonN` and ROS domain N. Confirm the printed values match the physical robot. Do not copy another robot's identity/DDS configuration. The package prefix should resolve inside `~/a2r-student/robot/install`.

## First launch: motors disabled

1. **Turn on the RVR base** with its own power button (it has a separate battery from the Jetson). The Jetson needs the base powered to talk to it over UART and read its sensors; the motors stay disabled in this step.
2. **Set up the F710 gamepad:** slide the switch on its front edge (between the bumpers) to **X**, not D. If the green light next to the **MODE** button is lit, press MODE once to turn it off.
3. **Leave the controls neutral:** do not hold LB or RB, and let both sticks rest at center.

Then, in prepared terminal A:

```bash
systemctl --user stop rvr-camera.service
ros2 launch a2r_robot robot.launch.py namespace:="$ROBOT_NAMESPACE" motion_enabled:=false
```

Keep it running. The OLED can remain active. Only one process may own the RVR UART or camera at a time. **If this fails check that you didn't swap the RX and TX pins!**

## Run the student program

The previous build already included `team_navigation`. Keep the disabled robot launch running in terminal A. **In a fresh Jetson terminal B, run the [terminal setup commands](#prepare-each-jetson-terminal)**, then:

```bash
ros2 launch team_navigation team.launch.py namespace:="$ROBOT_NAMESPACE"
```

The supplied navigation node now detects yellow road pixels and publishes a low-speed follow request when the current camera image contains a valid yellow path. It requests zero when the image is missing or stale, yellow is not found, or enough blue pixels appear in the forward region. This is a starter vision heuristic, not a safety-rated water detector or a validated navigation solution.

**In another prepared Jetson terminal C**, run each check separately (Ctrl+C ends continuous commands) and make sure you see reasonable outputs. **Talk to your team about what "reasonable outputs" would mean for each.**

```bash
ros2 node list
ros2 topic list -t
ros2 topic echo "/$ROBOT_NAMESPACE/hardware/status" --qos-reliability best_effort --once
ros2 topic echo "/$ROBOT_NAMESPACE/joystick/state" --qos-reliability best_effort
ros2 topic echo "/$ROBOT_NAMESPACE/encoders" --qos-reliability best_effort --once
ros2 topic hz "/$ROBOT_NAMESPACE/imu/data"
ros2 topic hz "/$ROBOT_NAMESPACE/camera/image_raw/compressed"
ros2 topic echo "/$ROBOT_NAMESPACE/navigation/request" --qos-reliability best_effort
ros2 topic echo "/$ROBOT_NAMESPACE/safety/status" --once
```

**Below you will find some notes on what you may see and what it means.**
| Check | Pass condition |
| --- | --- |
| Discovery | Hardware, safety, visualization and launched team node appear under your namespace; avoid duplicate launches. |
| Hardware status | After startup, `uart_ok`, `imu_fresh`, `encoders_fresh`, `joystick_connected` and `camera_fresh` are true; `demo_locked` false. With this disabled launch, `drive_enabled` false and applied tread speeds zero. |
| Joystick | Connected state and input messages remain fresh; moving sticks/pressing buttons changes the reported input. No physical movement with motor output disabled. |
| IMU | Repeated messages with advancing stamps and plausible values; gently changing the stationary robot's orientation changes orientation readings. A topic name alone is insufficient. |
| Camera | Repeated frames; PC RViz image changes when you move an object in view. |
| Encoders | Fresh stamped messages. Counts may remain constant at rest; verify changes during the later controlled drive. `calibrated: false` and NaN distances/speeds are expected before calibration. |
| Navigation | `navigation/request` has advancing timestamps. On this 2x2 synthetic/invalid view or when no yellow road is detected, both requested velocities are zero. |
| PC communication | With matching domain on the same router, the PC sees the robot's live topics and RViz telemetry/image. |

Startup gaps can briefly disarm the controller. Persistent stale data or faults must be resolved before enabling motors. `ros2 topic hz` reports the subscriber's observed rate; require continuing data, not an assumed exact frequency.

## Controlled movement and stopping check

Use a clear floor area with someone able to reach RVR power. Stop the disabled base with Ctrl+C. Keep the team launch stopped during this manual-only acceptance check. In prepared terminal A:

```bash
ros2 launch a2r_robot robot.launch.py namespace:="$ROBOT_NAMESPACE" motion_enabled:=true
```

1. Start in **Low**. With sticks centered, hold **LB** and apply a small stick input. Verify both treads, forward/reverse response and steering at low speed.
2. Release **LB**: verify the robot stops. Center sticks, re-arm, drive briefly and press **A**: verify it stops again. Both bumpers together must also stop/disarm. Neither bumper permits motion.
3. Verify encoder counts change during movement and applied tread speeds return to zero after stopping. Here `drive_enabled` indicates launch permission and can remain true while stopped; use physical observation and applied speed fields to verify stopping.
4. Release bumpers, center sticks and stop ROS. Record the result for this robot. Do not proceed with navigation if any stopping check fails.

**RB** enables autonomy, **LB** manual tank driving. **Y** changes Low/Medium/High only while stopped and neutral; Low/Medium/High tread-target limits are 0.15/0.60/1.40 m/s. Use Low for acceptance. Software controls are not a safety-rated emergency stop; use accessible RVR power if required.

Autonomy needs implemented student code and measured `track_width_m`. Its default zero blocks nonzero autonomous conversion. Encoder calibration is separately required for meaningful distance/speed feedback; it is not a substitute for geometry or a guarantee that autonomy is safe.

## Encoder calibration and geometry

Calibrate **this robot**, on the intended surface, with its current tread setup. Never copy another robot's scales. Leave calibration unset until measured.

1. Mark a straight forward path (for example 1 metre). With the ROS base running, record `left_relative_ticks` and `right_relative_ticks` from `/jetsonN/encoders` while stationary.
2. If the base is still in the disabled-check launch, stop it and relaunch with `motion_enabled:=true` using the controlled movement procedure above. In Low manual mode, drive straight forward without reversing/turning. Stop, measure actual forward distance and record ending relative counts. Record the `hardware_session` alongside both samples. Keep the **same hardware session** throughout; discard the run if the node restarts or the run curves/slips appreciably.
3. For each tread compute `delta_ticks = end - start` and `signed_metres_per_tick = measured_distance_m / delta_ticks`. Preserve the sign: forward motion must produce positive calibrated travel. Both deltas must be nonzero; repeat for consistency.
4. Stop ROS before installing the file. Review an existing calibration with staff before replacing it. Generate numeric JSON on the Jetson using your measurements:

```bash
cat > /tmp/a2r-encoder-calibration.py <<'PY_CAL'
import json, math
from pathlib import Path

distance = float(input('Measured forward distance (metres): '))
left = int(input('Left signed relative tick difference: '))
right = int(input('Right signed relative tick difference: '))
if not math.isfinite(distance) or distance < 0.1 or min(abs(left), abs(right)) < 10:
    raise SystemExit('Invalid/insufficient measured run; repeat it.')
scales = {'left': distance / left, 'right': distance / right}
if not all(0 < abs(v) < 1 for v in scales.values()):
    raise SystemExit('Implausible scale; check measurements.')
p = Path.home() / 'encoder_calibration.json'
p.write_text(json.dumps({'version': 1,
    'method': 'externally measured straight forward ground run',
    'measured_distance_m': distance,
    'run_ticks': {'left': left, 'right': right},
    'signed_metres_per_tick': scales}, indent=2) + '\n')
print(p)
PY_CAL
python /tmp/a2r-encoder-calibration.py && \
sudo install -m 0644 ~/encoder_calibration.json /etc/a2r-robot/encoder_calibration.json
```

Installing under `/etc` requires authorized sudo access; if the robot account cannot run `sudo install`, ask staff to install your measured file without changing account permissions. Restart the base with motors disabled and confirm `calibrated: true` and finite distance/speed fields. Then stop the disabled launch and use the controlled movement procedure to validate on a separate measured Low-speed run, comparing **changes** in calibrated tread distances to ground travel. Record errors, surface and measurements; redo calibration if unsuitable.

Effective tread spacing is a separate investigation: use measured left/right tread travel and an independently measured turn angle (radians) to estimate `track_width_m = (right_travel - left_travel) / yaw_change`, following the base's forward/left-turn sign convention. Measure repeatable turns in both directions, account for tread slip and validate the resulting positive width; do not assume the physical outside width is the effective value. Manual mode works while width remains unset.

After validating your measured value, add `track_width_m:=YOUR_MEASURED_VALUE` to the base launch (replace the placeholder with a positive numeric value in metres). Begin with motor output disabled to inspect student requests, then perform supervised Low-speed autonomy testing. Tread distance is not a global position estimate; no odometry/navigation solution is supplied.

## View on your PC

Follow [PC viewer setup](../viewer/README.md), outside SSH. Set `ROS_DOMAIN_ID` to your robot's number and use `rmw_cyclonedds_cpp`, matching the commissioned robots, in every PC ROS terminal. Match the RViz namespace/topics/frames to `jetsonN`. Do not copy the robot's interface-specific DDS file onto your PC.

## Troubleshooting

| Symptom | Check / action |
| --- | --- |
| SSH timeout / wrong robot | PC and Jetson must use the assigned router. Read OLED IP again; verify power and router number. Overlapping router subnets do not identify a robot. |
| SSH password rejected | Username `a2r-lab-jetson`, password from Canvas. If rejected, ask staff to verify/reset that robot's account. Do not guess router-admin credentials. |
| SSH host-key warning | Ask staff to verify the robot's host-key fingerprint before changing saved trust; do not disable host-key checking. |
| Package not found / `evdev` missing | Use a fresh terminal, source Jazzy, the robot workspace and the Python environment as documented; rebuild using `python -m colcon`. Verify source version and workspace paths. |
| UART timeout | RVR must be powered on; power down before checking TX↔RX/common-ground wiring. Stop other ROS/legacy UART owners. Ask staff if `/dev/ttyTHS1` or device permission is wrong. |
| Camera busy / no frames | Stop `rvr-camera.service` before ROS; stop other camera owners. Check the USB camera and `/dev/v4l/by-id/`. Close competing launches before retrying. |
| Missing joystick | Plug F710 receiver into Jetson, check batteries, X mode and MODE light off. If mode was switched, stop/reconnect/relaunch; verify `/dev/input/by-id/`. |
| Blank OLED | Check service status/logs and I2C device presence. Power off before checking VIN/GND/SDA/SCL. Ask staff about bus/address/device permissions. |
| PC sees no ROS data | Same router, Jazzy, namespace/domain and compatible middleware/QoS. Check PC VPN/firewall/multicast and whether ROS chose another active network interface. Do not broadly disable security settings; use staff-approved network rules. |
| Controller stays disarmed | Inspect hardware/safety status; repair stale input/UART/sensors, then release bumpers and center sticks before re-arming. |
| NaN encoder distances | Expected until this robot has a valid measured calibration. Raw counts should still publish. |
| Nonzero autonomy blocked | Check measured positive `track_width_m`, fresh continuously stamped requests, healthy hardware and RB arming. The unchanged starter sends zero. |

Useful Jetson diagnostics (Ctrl+C ends logs):

```bash
systemctl --user status --no-pager rvr-camera.service rvr-oled.service
journalctl --user -b -u rvr-camera.service -u rvr-oled.service -n 60 --no-pager
```

Keep launch terminal output when reporting a fault. If user journal access is unavailable, ask staff for service logs.

## Stop, restore and record

Release both bumpers. Stop team and base launches with Ctrl+C and wait for exit. Then restore the browser camera:

```bash
systemctl --user start rvr-camera.service
systemctl --user status --no-pager rvr-camera.service rvr-oled.service
```

Confirm the camera page works again. Normally leave the OLED on. Do not enable automatic driving services.

Record robot/router number, repository commit (`git -C ~/a2r-student rev-parse HEAD` on the PC clone), build result, service/sensor/PC checks, physical movement and each stopping result, calibration/geometry measurements (or explicitly uncalibrated/unset), and unresolved faults. Passing the starter check does not validate your later navigation implementation.

## Getting started with autonomy

Everything above ran the robot with the supplied starter, which only ever asks for zero motion. This section shows how to change what the robot is asked to do, rebuild, and test it in autonomy mode by holding **RB**.

### How the two launches fit together

Both launches come from the same workspace and use the same `robot/install/setup.bash`:

| Launch | What it runs | Do you edit it? |
| --- | --- | --- |
| `ros2 launch a2r_robot robot.launch.py ...` (terminal A) | The base: hardware, safety and visualization. It owns the motors, sensors and gamepad. | Not to get started. |
| `ros2 launch team_navigation team.launch.py ...` (terminal B) | Your navigation node. 20 times a second it publishes a request (forward speed in m/s, left-turn yaw rate in rad/s) on `navigation/request`. | Yes, this is your code. |

The continuous zero command came from the team launch, not the base launch. The safety node in the base passes your requests on to the treads only while **RB** is held, and stops the robot as soon as RB is released or requests stop arriving (they expire after 0.25 seconds).

### Before you start

- Pass the [controlled movement and stopping check](#controlled-movement-and-stopping-check) in manual mode first.
- Measure this robot's `track_width_m` as described in [encoder calibration and geometry](#encoder-calibration-and-geometry). It defaults to zero, and while it is zero the safety node refuses **any** nonzero autonomous request, including driving straight.

### 1. Edit your navigation code

On the Jetson, open:

```text
~/a2r-student/robot/src/team_navigation/team_navigation/navigation.py
```

The function that decides what the robot is asked to do is `compute_motion`. It returns `(forward m/s, left-turn rad/s)` and the starter returns `0., 0.`. For a first experiment, ask for a slow, straight crawl using the supplied 0.05 m/s setting:

```python
    def compute_motion(self, action):
        return self.target_speed, 0.
```

The other hooks (`process_image`, `update_estimate`, `choose_action`) feed `compute_motion` and are where your perception and decision logic will go later.

### 2. Rebuild

Stop the team launch with Ctrl+C in terminal B (stop the base in terminal A too if you edited a baseline package). Then open a **fresh** Jetson terminal and repeat the build from [step 3](#3-load-the-tools-and-build):

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros-venv/bin/activate
cd ~/a2r-student/robot
python -m colcon build --symlink-install
```

**Check:** `4 packages finished`, no failed packages. Do not reuse a terminal that already sourced `robot/install/setup.bash` for the build.

Because the build uses `--symlink-install`, an edit to an existing Python file takes effect the next time you launch, even without a rebuild. You must rebuild when you add a file or a new node, change `setup.py` or `package.xml`, or change a message definition. When in doubt, rebuild; it takes a few seconds.

### 3. Dry run with motors disabled

Check what your code is asking for before the treads can move. In [prepared terminals](#prepare-each-jetson-terminal), replacing `YOUR_MEASURED_VALUE` with your track width in metres:

```bash
# Terminal A: base, motors disabled
systemctl --user stop rvr-camera.service
ros2 launch a2r_robot robot.launch.py namespace:="$ROBOT_NAMESPACE" motion_enabled:=false track_width_m:=YOUR_MEASURED_VALUE
```

```bash
# Terminal B: your code
ros2 launch team_navigation team.launch.py namespace:="$ROBOT_NAMESPACE"
```

```bash
# Terminal C: watch your requests, then the safety node (Ctrl+C between them)
ros2 topic echo "/$ROBOT_NAMESPACE/navigation/request" --qos-reliability best_effort
ros2 topic echo "/$ROBOT_NAMESPACE/safety/status"
```

**Check:** `navigation/request` shows the values you expect with advancing timestamps. Then, while watching `safety/status`, release both bumpers, center the sticks and **hold RB**. The status should read `autonomous`, and `commanded_treads` shows the tread speeds that would be sent. The robot does not move in this launch.

### 4. Drive in autonomy mode

Use a clear floor area with someone able to reach RVR power. Stop the base with Ctrl+C in terminal A and relaunch it with motors enabled; leave your team launch running in terminal B:

```bash
ros2 launch a2r_robot robot.launch.py namespace:="$ROBOT_NAMESPACE" motion_enabled:=true track_width_m:=YOUR_MEASURED_VALUE
```

1. Stay in the **Low** speed profile (the startup default, 0.15 m/s limit).
2. Release both bumpers and center the sticks. `safety/status` reads `ready for deliberate selection`.
3. **Press and hold RB.** The robot follows your code's requests for as long as RB stays held.
4. **Release RB to stop.** Pressing **A** or holding both bumpers also stops it.
5. To go again, release both bumpers, center the sticks, then hold RB again. Holding RB through a stop or fault does not re-arm.

To drive the robot back by hand, release RB first, then hold **LB** and use the sticks. You cannot switch between RB and LB without releasing both in between.

### If holding RB does nothing

Read the first words of `safety/status`:

| Status begins with | Meaning / action |
| --- | --- |
| `ready for deliberate selection` | Neutral and waiting. Hold RB. |
| `autonomous` | Your requests are being passed to the treads. |
| `autonomous (speed limited)` | Your request exceeded the current speed profile; both treads were scaled down together. |
| `center sticks and release bumpers` | Let go of everything, then hold RB again. |
| `autonomous conversion not configured` | You sent a nonzero request but `track_width_m` is still zero. Add it to the base launch. |
| `no navigation request after selection` or `navigation request expired` | The team launch is not running, crashed, or stopped publishing. Check terminal B. |
| `hardware or joystick stale` | The base is not healthy. Check terminal A and `hardware/status`. |

If the robot holds still with status `autonomous`, first check whether the camera image passes the configured yellow/blue thresholds and whether `navigation/request` is nonzero. Missing/stale images, no yellow detection and detected blue regions deliberately request zero. Check the terminal B log for a `Navigation failed; requesting zero motion` error as well.

### The edit-test loop

Edit `navigation.py`, stop the team launch, [rebuild](#2-rebuild), relaunch the team launch, dry run, then drive. When you finish, follow [stop, restore and record](#stop-restore-and-record).

## End of Session Shutdown

At the end of the session stop motion/ROS, turn off RVR power, save work, verify the intended Jetson, then shut it down:

```bash
hostname
sudo shutdown -h now
```

Wait for shutdown completion before removing Jetson power. RVR has an independent battery; removing Jetson power does not remove tread power.
