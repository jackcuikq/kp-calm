#!/usr/bin/env bash
# ROS-generated setup files may read optional unset variables, so enable
# errexit/pipefail here and turn nounset on only after sourcing them.
set -eo pipefail

# Robot-specific initial calibration estimated from the physical tread spacing.
# Override for a future refined measurement with, for example:
#   ROBOT_TRACK_WIDTH_M=0.215 ./scripts/start_robot_motion.sh
ROBOT_TRACK_WIDTH_M="${ROBOT_TRACK_WIDTH_M:-0.22}"

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_WORKSPACE="$(cd -- "$SCRIPT_DIR/.." && pwd)"

source /opt/ros/jazzy/setup.bash
source "$ROBOT_WORKSPACE/install/setup.bash"
source "$HOME/ros-venv/bin/activate"
source /etc/a2r-robot/robot.sh

set -u

if [[ -z "${ROBOT_NAMESPACE:-}" ]]; then
    echo "ROBOT_NAMESPACE is empty; check /etc/a2r-robot/robot.sh" >&2
    exit 1
fi

echo "Robot namespace: $ROBOT_NAMESPACE"
echo "Track width: $ROBOT_TRACK_WIDTH_M m"
echo "MOTION IS ENABLED. The base still requires healthy hardware and deliberate controller arming."

systemctl --user stop rvr-camera.service

exec ros2 launch a2r_robot robot.launch.py \
    namespace:="$ROBOT_NAMESPACE" \
    motion_enabled:=true \
    track_width_m:="$ROBOT_TRACK_WIDTH_M"
