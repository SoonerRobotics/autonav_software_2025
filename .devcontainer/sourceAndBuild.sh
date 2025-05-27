#!/bin/bash
# shellcheck disable=SC2164
echo "Sourcing ROS"

source /opt/ros/jazzy/setup.bash

cd /autonav_software_2025/autonav_ws; pwd

# colcon build
#colcon build --packages-skip autonav_hardware --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1 #original build, one below ignores unused params
colcon build --packages-skip autonav_hardware --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DCMAKE_CXX_FLAGS="-Wno-unused-parameter"

source install/setup.bash

ros2 launch autonav_launch test.xml

exec "$SHELL"
