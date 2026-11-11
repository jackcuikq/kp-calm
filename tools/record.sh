#!/usr/bin/env bash
# Record a run on the robot into a ROS 2 bag, then convert it on the laptop with tools/bag2video.py.
#
#   source your prepared terminal first (ROBOT_NAMESPACE must be set), base + team launch running, then:
#   bash ~/Projects/kp-calm/tools/record.sh            # -> ~/bags/<timestamp>
#   bash ~/Projects/kp-calm/tools/record.sh lap_low    # -> ~/bags/<timestamp>_lap_low
#   Ctrl+C to stop.  Copy to the laptop:  scp -r a2r-lab-jetson@ROBOT_IP:bags/<name> .
#
# Topics: camera (JPEG, ~30 fps), IMU, encoders, what navigation requested, what safety commanded,
# safety/hardware status, joystick, and navigation/telemetry (JSON per tick: offset, estimate,
# action, image age, pipeline time). Roughly 5 MB per second at 720p.
set -euo pipefail
ns="${ROBOT_NAMESPACE:?source /etc/a2r-robot/robot.sh first}"
name="$(date +%Y%m%d_%H%M%S)${1:+_$1}"
mkdir -p "$HOME/bags"
echo "recording /$ns/* -> $HOME/bags/$name   (Ctrl+C to stop)"
exec ros2 bag record -o "$HOME/bags/$name" \
  "/$ns/camera/image_raw/compressed" \
  "/$ns/navigation/debug/compressed" \
  "/$ns/navigation/telemetry" \
  "/$ns/navigation/request" \
  "/$ns/imu/data" \
  "/$ns/encoders" \
  "/$ns/motors/command" \
  "/$ns/safety/status" \
  "/$ns/hardware/status" \
  "/$ns/joystick/state"
