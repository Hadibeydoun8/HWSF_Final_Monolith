#!/bin/bash
export LD_LIBRARY_PATH=/opt/ros/humble/lib:$LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
source /home/ws/ros2_ws/install/setup.bash
exec cmake "$@"
