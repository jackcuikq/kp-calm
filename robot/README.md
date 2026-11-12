# Robot baseline and student program — Jetson

This is the ROS 2 workspace you build and run **on the Jetson**. The baseline is supplied scaffold code: students may extend or modify it. The viewer runs on your PC; see [viewer instructions](../viewer/README.md).

## Packages

| Package in `src/` | Purpose |
| --- | --- |
| `a2r_robot` | Hardware access, controller input, motor control, stopping rules and visualization data |
| `a2r_robot_interfaces` | Robot telemetry/control message definitions |
| `course_robot_interfaces` | Movement requests shared with the student program |
| `team_navigation` | Yellow-road camera follower; tune and validate before supervised use |

## Build

Clone the repository at `~/a2r-student` on the Jetson as described in [setup](../docs/SETUP.md#build-the-robot-workspace-on-the-jetson). In a fresh Jetson terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros-venv/bin/activate
cd ~/a2r-student/robot
python -m colcon build --symlink-install
```

Expect four packages to finish. Rebuild here after edits; no package copying or separate base build is needed. Do not source this workspace's old install in the build terminal.

## Run

In every fresh Jetson runtime terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/a2r-student/robot/install/setup.bash
source ~/ros-venv/bin/activate
source /etc/a2r-robot/robot.sh
```

In terminal A, after powering the RVR on for sensors:

```bash
systemctl --user stop rvr-camera.service
ros2 launch a2r_robot robot.launch.py namespace:="$ROBOT_NAMESPACE" motion_enabled:=false
```

In terminal B, prepared with the same commands:

```bash
ros2 launch team_navigation team.launch.py namespace:="$ROBOT_NAMESPACE"
```

Follow [sensor checks and physical acceptance](../docs/SETUP.md#run-the-student-program) before enabling movement. Keep namespace/domain consistent in all terminals.

For this team's calibrated robot, `scripts/start_robot_motion.sh` prepares the
runtime environment and launches the base with motion enabled and the initial
robot-specific `0.22 m` track-width estimate. It does not launch navigation or
bypass the controller/safety gates:

```bash
cd ~/a2r-student/robot
./scripts/start_robot_motion.sh
```

After a refined physical calibration, either update the script or override it
for one launch with `ROBOT_TRACK_WIDTH_M=VALUE ./scripts/start_robot_motion.sh`.

## Public request

Topic: `navigation/request`, relative to the assigned robot namespace.
Type: `course_robot_interfaces/msg/MotionRequest`.

```text
std_msgs/Header header
float64 forward_speed_m_s
float64 yaw_rate_rad_s
```

Generate a fresh `header.stamp` for each request. Positive speed is forward; positive yaw rate is a left turn. Request expiry is 0.25 seconds. Publish continuously with best-effort delivery supported by the base. Request timestamps are not image timestamps: students must determine whether the observations used by their algorithm are recent enough.

LB enables manual tank driving; RB permits the navigation requests. A or both bumpers stop/disarm. To select or change modes, release A/both bumpers and center sticks. Y changes the global speed profile only while stopped and neutral. Low/Medium/High tread-target limits are 0.15/0.60/1.40 m/s. No autonomy is permitted from a held bumper after a fault without re-arming.

Students determine and validate the effective tread spacing, then supply `track_width_m` to the base launch. It defaults to zero; no robot 1 geometry or encoder calibration is copied to other robots. Manual driving does not require the body-to-tread conversion.

## Navigation behavior and tuning

`team_navigation/team_navigation/navigation.py` provides a basic HSV yellow-road follower. It thresholds blue in the lower forward image and requests a stop when blue occupies enough of that region. A bottom-image yellow centroid gives the road offset; each fresh detection is smoothed, larger offsets reduce speed, and the PD controller steers toward the road. When both sides of the lookahead image contain separated yellow regions, it selects the side with more yellow pixels as the fork branch. These color thresholds and fork heuristics depend on your camera, lighting and course; inspect and tune them on recorded or stationary images before driving. A camera color threshold is not a safety-rated obstacle detector.

The parameters and initial conservative speeds are in `team_navigation/config/navigation.yaml`. Tune steering first in the Low profile, on a clear course, with a spotter and an accessible power switch. The positive road offset means the road is right of image center; the controller requests negative yaw to steer right. Verify that sign with the robot lifted or under another staff-approved safe check before any floor test.

**RB is the autonomous enable button.** The navigation node publishes requests continuously, but the base safety node only applies them in its armed autonomous mode after bumpers are released and sticks centered. Release and re-center before selecting the mode. **LB is manual drive; A or both bumpers stop/disarm.** Do not use RB for motion until `motion_enabled:=true` has been deliberately selected after acceptance checks and this robot has a measured positive `track_width_m`; the default width of zero rejects nonzero autonomous commands. Start in Low, and have a person ready to cut robot power. The button and software stops are not safety-rated.

The node subscribes to camera images and IMU only; the IMU is not currently used for steering. Encoder counts, calibrated tread distances and speeds are available if you decide to extend estimation. No onboard odometry or supplied sensor-fusion solution is exposed. Exceptions, invalid images, stale frames, no yellow detection and blue detections produce fresh zero requests rather than reusing a prior motion command. OpenCV images are BGR.

### Record a short camera clip

With the base launch running, a second prepared Jetson terminal can record the
compressed ROS camera stream without enabling or commanding motion:

```bash
ros2 run team_navigation record_camera --seconds 5 --namespace "$ROBOT_NAMESPACE"
```

The command waits up to ten seconds for the first frame and then writes an
MJPEG AVI under `camera_clips/` in the current directory. Use `--output
/path/to/name.avi` to choose a different location. Record clips with the camera
mounted on the robot in the real course lighting; do not commit videos to Git.

## Run the software tests

These tests need no robot and no ROS. Run them on your PC or the Jetson, from the repository root:

```bash
cd ~/a2r-student
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements-test.txt
python -m pytest tests -q
```

They check the stopping rules, speed limits, message layout and the navigation adapter's fail-closed behavior. They are most useful if you modify the baseline: a failure in the guard or speed-profile tests means you have changed how the robot stops.

Two further checks in `tests/ros/` exercise real ROS messages with synthetic sensors and no motors. Run them on the Jetson after building, with no launch running, in a terminal you close afterwards:

```bash
source /opt/ros/jazzy/setup.bash
source ~/a2r-student/robot/install/setup.bash
source ~/ros-venv/bin/activate
unset CYCLONEDDS_URI RMW_IMPLEMENTATION ROS_LOCALHOST_ONLY
export ROS_DOMAIN_ID=92
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
python3 ~/a2r-student/tests/ros/ros_smoke.py
python3 ~/a2r-student/tests/ros/starter_smoke.py
```

Domain 92 keeps the synthetic inputs away from the live robot. `starter_smoke.py` verifies that the node publishes fresh requests and stops on invalid synthetic image data.

## Your changes

Keep your project changes in this repository, which is released under the [MIT license](../LICENSE), and retain the [course attribution](NOTICE.md). Keep the lab passwords and your personal credentials out of commits; the lab connection details are in [setup](../docs/SETUP.md#connect). Do not commit `build/`, `install/`, `log/`, videos or recordings.
