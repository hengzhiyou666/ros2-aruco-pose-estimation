#!/usr/bin/env bash

set -euo pipefail

workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "${workspace_dir}"

# Build against the complete base ROS installation. dog3's ros_sdk_overlay
# contains incomplete geometry_msgs interface metadata and cannot generate
# this workspace's custom message package.
set +u
source /app/opt/ros/humble/setup.bash
set -u
export PATH="/userdata/2_slam/.deps/venv/bin:${PATH}"

exec colcon build --symlink-install --cmake-clean-cache "$@"
