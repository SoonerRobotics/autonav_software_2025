FROM osrf/ros:jazzy-desktop-full
RUN apt-get update && apt-get install -y \
    python3-opencv \
    ros-jazzy-cv-bridge \
    ros-jazzy-rclpy \
    python3-aiohttp \
    python3-pip
# RUN apt-get update && apt-get install -y \ #Optional add in if running from a volume instead..

############## Base Image setup above ^^
############# ROS setup below VV
ENTRYPOINT [".devcontainer/sourceAndBuild.sh"]

#RUN git clone https://github.com/SoonerRobotics/autonav_software_2025.git #optional if wanting to run by cloning instead

COPY . /autonav_software_2025/

WORKDIR /autonav_software_2025