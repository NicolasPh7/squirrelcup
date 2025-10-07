#!/usr/bin/env bash
set -euo pipefail

ARTIFACT_DIR=/tmp/ci_artifacts
mkdir -p "$ARTIFACT_DIR"
LOGFILE="$ARTIFACT_DIR/test_run.log"

# Ensure ROS environment is available
source /opt/ros/$ROS_DISTRO/setup.bash

# Move workspace into place and build (assumes source mounted as read-only)
cd /home/rosdev/eurobot_2026_ws || exit 1

# Run tests headless: run pytest for the integration test only
# We run with xvfb-run to allow headless GUI if Gazebo needs X
if command -v xvfb-run >/dev/null 2>&1; then
  XVFB="xvfb-run -s '-screen 0 1280x720x24'"
else
  XVFB=""
fi

echo "Running integration test (headless)" | tee "$LOGFILE"
# Run the single pytest with verbose output and write to logfile
$XVFB python3 -m pytest src/mam_eurobot_2026/test/test_integration_odometry.py -q --maxfail=1 2>&1 | tee -a "$LOGFILE"

# copy any generated ros2 test output (colcon pytest style)
if [ -d build/mam_eurobot_2026/test_results ]; then
  cp -r build/mam_eurobot_2026/test_results "$ARTIFACT_DIR/" || true
fi

# If Gazebo or test produced a bag or video, copy them (not typical)
# For now, just exit with the pytest exit status
