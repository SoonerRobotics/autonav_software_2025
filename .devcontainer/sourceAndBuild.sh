#!/bin/bash
# shellcheck disable=SC2164

echo "Installing deps"
cd setup
echo "machine files.dylanzeml.in login <user> password <password>" > vectorsecrets.txt
./setup.sh
cd ..

echo "Sourcing ROS"
source /opt/ros/jazzy/setup.bash

cd autonav_ws
echo "Building project"
colcon build

echo "Sourcing the built packages"
source install/setup.bash


ros2 launch autonav_launch competition.xml

exec "$SHELL"
