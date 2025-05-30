# Get local user
LOCAL_USER=$(whoami)

# Setup ROS2
export DISPLAY=:0
source /opt/ros/jazzy/setup.bash

# Check if build is needed, if so, build
cd /home/$LOCAL_USER/autonav_software_2025/autonav_ws
colcon build

# Launch
source /home/$LOCAL_USER/autonav_software_2025/autonav_ws/install/setup.bash
ros2 launch autonav_launch competition.xml