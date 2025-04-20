#!/bin/bash
export LD_LIBRARY_PATH=/opt/ros/humble/lib:$LD_LIBRARY_PATH
source /opt/ros/jazzy/setup.bash
source "/home/parallels/Desktop/Parallels Shared Folders/Home/Documents/Monolith/ros2_ws/install/local_setup.bash"
exec cmake "$@"
