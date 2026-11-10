# RViz on your computer

Run RViz on your own computer, using ROS 2 Jazzy to match the robot. Connect both devices to the router supplied with the robot. The robot ROS launch must be running and publishing. The browser camera service is separate and stays stopped during ROS camera capture.

In this repository cloned as `~/a2r-student`, this folder is `~/a2r-student/viewer` on your own PC. Run `cd ~/a2r-student/viewer` before building. Do not run these viewer commands in the Jetson SSH terminal.

This folder is an independent viewer workspace. It contains the custom read-only Robot telemetry panel and the exact telemetry message definitions used by the base. The duplicate interface copy here is for building the viewer on a different computer; do not build this folder inside the robot base/team workspace.

Use a PC environment compatible with **ROS 2 Jazzy desktop** (native Ubuntu 24.04 is the direct path). If ROS is missing, install it using the [official Jazzy Ubuntu instructions](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html); do not use a different ROS distribution. On macOS/Windows or another OS, arrange a supported Linux environment with the teaching team before class. The optional Dockerfile still needs working GUI and robot-LAN networking; Docker Desktop/VM discovery is not assumed to work automatically.

After configuring the official ROS apt repository, on the Ubuntu PC:

```bash
sudo apt update
sudo apt install ros-jazzy-desktop python3-colcon-common-extensions build-essential ros-jazzy-compressed-image-transport ros-jazzy-rmw-cyclonedds-cpp qtbase5-dev
source /opt/ros/jazzy/setup.bash
cd ~/a2r-student/viewer
colcon build --symlink-install --packages-up-to a2r_rviz_panels
```

In every fresh **PC** runtime/check terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/a2r-student/viewer/install/setup.bash
export ROS_DOMAIN_ID=1  # Replace 1 with your assigned robot number.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# Use LAN discovery on the PC, not a DDS file tied to a Jetson interface.
unset CYCLONEDDS_URI ROS_LOCALHOST_ONLY ROS_AUTOMATIC_DISCOVERY_RANGE
ros2 daemon stop
ros2 node list
ros2 topic list
```

Expect the running robot's namespaced nodes/topics. Stopping the ROS CLI daemon above clears cached discovery from a different domain/middleware; it does not stop robot nodes. If absent, resolve router/domain/multicast/firewall issues before starting RViz; see [troubleshooting](../docs/SETUP.md#troubleshooting). Do not source Jetson paths or copy its interface-specific DDS XML onto the PC. In this prepared PC terminal:

```bash
ros2 run rviz2 rviz2
```

Choose File > Open Config and open `config/jetson1.rviz`. For another robot, make a local copy and replace all `jetson1` topic/frame and Robot Namespace entries with its assigned name. Use its matching domain (2–5 for the other commissioned robots).

The view includes the camera, stationary robot attitude model and telemetry dock. Its Status, Encoders and IMU tabs show values alongside the scene. If the dock is hidden, choose Panels > Add New Panel > a2r_rviz_panels/Telemetry. Encoder distances are tread travel, not a supplied global odometry solution.

The image display uses `/jetson1/camera/image_raw/compressed` with Best Effort delivery. Only JPEG crosses the network for that display; a raw subscriber such as the team node makes the robot publish raw images too. Fixed Frame is `jetson1/visualization_world`.

No X11-forwarded Jetson desktop or monitor is required. Docker is optional; ask the course teaching team for help if you choose it. The included Dockerfile uses this viewer directory as its build context. No static peer IP or network XML is distributed for students to edit.

Acceptance: confirm the image changes live, IMU/encoder telemetry advances, and the Robot Namespace and Fixed Frame match your robot. A stationary encoder count is normal at rest; calibrated distances can be NaN until measured calibration is installed. Close RViz with Ctrl+C when finished.
