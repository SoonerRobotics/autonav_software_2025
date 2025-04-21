#!/bin/bash
# shellcheck disable=SC2164
cd ..; pwd
echo "Sourcing ROS"

source /opt/ros/jazzy/setup.bash

cd autonav_software_2025/autonav_ws; pwd

# colcon build
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1


source install/setup.bash

ros2 launch autonav_launch test.xml

exec "$SHELL"