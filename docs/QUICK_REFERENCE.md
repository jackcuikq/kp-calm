# Quick reference

The everyday commands on one page. Use this after you have worked through [setup and testing](SETUP.md) once; that guide explains each step and what to check.

## Connect (your PC)

Join your robot's router (`a2r-projects-N`, or `a2r-projects` for robot 1), read the robot's IP from its OLED, then:

```bash
ssh a2r-lab-jetson@ROBOT_IP
```

Passwords are on Canvas. Everything below runs on the Jetson over SSH unless it says otherwise.

## Prepare every Jetson terminal

```bash
source /opt/ros/jazzy/setup.bash
source ~/a2r-student/robot/install/setup.bash
source ~/ros-venv/bin/activate
source /etc/a2r-robot/robot.sh
```

## Rebuild after changes

Stop the launch you edited, then in a **fresh** terminal (not one prepared as above):

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros-venv/bin/activate
cd ~/a2r-student/robot
python -m colcon build --symlink-install
```

Expect `4 packages finished`. An edit to an existing Python file takes effect on the next launch even without a rebuild; new files, new nodes and message changes need one.

## Launch

Turn on the RVR base first. Run only one base launch at a time.

```bash
# Terminal A: the base. Choose one.
systemctl --user stop rvr-camera.service
ros2 launch a2r_robot robot.launch.py namespace:="$ROBOT_NAMESPACE" motion_enabled:=false
ros2 launch a2r_robot robot.launch.py namespace:="$ROBOT_NAMESPACE" motion_enabled:=true track_width_m:=YOUR_MEASURED_VALUE
```

```bash
# Terminal B: your navigation code.
ros2 launch team_navigation team.launch.py namespace:="$ROBOT_NAMESPACE"
```

Leave `track_width_m` off for manual driving only. Autonomy with nonzero requests needs your measured value.

## Gamepad

Switch on the front edge set to **X**, **MODE** light off.

| Control | Effect |
| --- | --- |
| Hold **LB** + sticks | Manual tank driving |
| Hold **RB** | Autonomy: the robot follows your code's requests |
| Release the bumper | Stop |
| **A**, or both bumpers | Stop and disarm |
| **Y** (while stopped, nothing held) | Cycle speed limit: Low 0.15, Medium 0.60, High 1.40 m/s |

To arm, or to re-arm after any stop or fault: release both bumpers, center the sticks, then hold one bumper. Software controls are not an emergency stop; keep the RVR power button within reach.

## Check what is happening

```bash
ros2 node list
ros2 topic echo "/$ROBOT_NAMESPACE/safety/status"
ros2 topic echo "/$ROBOT_NAMESPACE/hardware/status" --qos-reliability best_effort --once
ros2 topic echo "/$ROBOT_NAMESPACE/navigation/request" --qos-reliability best_effort
ros2 topic echo "/$ROBOT_NAMESPACE/encoders" --qos-reliability best_effort --once
ros2 topic hz "/$ROBOT_NAMESPACE/imu/data"
ros2 topic hz "/$ROBOT_NAMESPACE/camera/image_raw/compressed"
```

`safety/status` begins with the reason the robot is or is not moving. The [status table](SETUP.md#if-holding-rb-does-nothing) explains each message.

To cut motor output until the base is next restarted:

```bash
ros2 service call "/$ROBOT_NAMESPACE/motors/disable" std_srvs/srv/Trigger '{}'
```

## Camera and OLED services

The browser camera (`http://ROBOT_IP:8080/` on your PC) and the ROS base cannot use the camera at the same time. Stop the camera service before a ROS launch and start it again afterwards. Normally leave the OLED running.

```bash
systemctl --user stop rvr-camera.service
systemctl --user start rvr-camera.service
systemctl --user restart rvr-camera.service rvr-oled.service
systemctl --user status --no-pager rvr-camera.service rvr-oled.service
journalctl --user -b -u rvr-camera.service -u rvr-oled.service -n 60 --no-pager
```

## Copy files to your PC

Run on your PC, not in the SSH terminal:

```bash
scp a2r-lab-jetson@ROBOT_IP:~/a2r-student/PATH/TO/FILE .
```

## Common errors

| What you see | Cause |
| --- | --- |
| `BlockingIOError: [Errno 11] Resource temporarily unavailable` at launch | Another base launch is still running. Stop it with Ctrl+C and wait for it to exit. |
| `TimeoutError` from the hardware node at launch | The RVR base is off, or its TX and RX wires are swapped. |
| `F710 not found` | Plug in the receiver and set the gamepad to **X**. |
| Holding RB does nothing | Read `safety/status`; see the [status table](SETUP.md#if-holding-rb-does-nothing). |

More in [troubleshooting](SETUP.md#troubleshooting).

## Finish

1. Release both bumpers. Ctrl+C the team launch, then the base launch, and wait for both to exit.
2. Restore the browser camera: `systemctl --user start rvr-camera.service`
3. Turn off the RVR base with its own power button.
4. Shut down the Jetson, checking the hostname first:

```bash
hostname
sudo shutdown -h now
```

Wait for shutdown to finish before removing Jetson power.

## Viewer (your PC)

RViz setup and launch are in the [viewer instructions](../viewer/README.md). Only one team member needs it.
