#!/usr/bin/env bash
set -eo pipefail


ARTIFACT_DIR=/tmp/ci_artifacts
mkdir -p "$ARTIFACT_DIR"
LOGFILE="$ARTIFACT_DIR/test_run.log"

sudo chown -R rosdev /tmp/ci_artifacts

# Ensure ROS environment is available
source /opt/ros/$ROS_DISTRO/setup.bash

# Move workspace into place and build (assumes source mounted as read-only)
cd /home/rosdev/eurobot_2026_ws || exit 1

echo "Running integration test (headless)" | tee "$LOGFILE"

xvfb-run -a colcon test --event-handlers console_direct+ | tee -a "$LOGFILE"

# copy any generated ros2 test output (colcon pytest style)
if [ -d build/mam_eurobot_2026/test_results ]; then
  cp -r build/mam_eurobot_2026/test_results "$ARTIFACT_DIR/" || true
fi

exit 0

