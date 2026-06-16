#!/bin/bash
set -e

source /opt/ros/humble/setup.bash

if [ -f /workspace/piperx_ws/install/setup.bash ]; then
    source /workspace/piperx_ws/install/setup.bash
fi

exec "$@"
