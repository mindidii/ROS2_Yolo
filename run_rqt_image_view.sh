#!/usr/bin/env bash
set -e

source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash

export PATH=/opt/ros/jazzy/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
export PYTHONPATH=/usr/lib/python3/dist-packages:${PYTHONPATH:-}
export QT_BINDING=${QT_BINDING:-pyqt}
export XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR:-/tmp/runtime-root}
export QT_X11_NO_MITSHM=1

mkdir -p "$XDG_RUNTIME_DIR"

exec ros2 run rqt_image_view rqt_image_view "$@"
