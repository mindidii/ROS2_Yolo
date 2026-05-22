#!/usr/bin/env bash
set -eo pipefail

export ROS_DISTRO="${ROS_DISTRO:-jazzy}"
export ROS_WS="${ROS_WS:-/ros2_ws}"

cd "${ROS_WS}"

if [ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    echo "ROS 2 setup file not found: /opt/ros/${ROS_DISTRO}/setup.bash" >&2
    exit 1
fi

if [ ! -f "${ROS_WS}/install/setup.bash" ]; then
    echo "Workspace setup file not found: ${ROS_WS}/install/setup.bash" >&2
    echo "Build the workspace first: cd ${ROS_WS} && colcon build --symlink-install" >&2
    exit 1
fi

source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${ROS_WS}/install/setup.bash"

TRACK_SELECTOR_EXEC="${ROS_WS}/install/yolo_detector_pkg/lib/yolo_detector_pkg/track_selector_node"
TRACK_SELECTOR_CONFIG="${ROS_WS}/install/sentinel_bringup/share/sentinel_bringup/config/track_selector.yaml"
if [ ! -x "${TRACK_SELECTOR_EXEC}" ] || [ ! -f "${TRACK_SELECTOR_CONFIG}" ]; then
    echo "track_selector install artifacts are missing; launching with enable_track_selector:=false" >&2
    set -- "enable_track_selector:=false" "$@"
fi

exec ros2 launch sentinel_bringup video_and_yolo.launch.py "$@"
