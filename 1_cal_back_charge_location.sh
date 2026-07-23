#!/usr/bin/env bash

set -euo pipefail

workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

set +u
source /userdata/2_slam/setup_env.sh
set +u
source "${workspace_dir}/install/setup.bash"
set -u

# setup_env.sh activates a virtualenv that does not contain OpenCV. Keep its
# ROS tools on PATH, but make /usr/bin/python3 the interpreter selected by
# installed scripts whose shebang is /usr/bin/env python3.
export PATH="/usr/bin:/bin:${PATH}"

exec ros2 launch aruco_pose_estimation aruco_pose_estimation.launch.py "$@"
