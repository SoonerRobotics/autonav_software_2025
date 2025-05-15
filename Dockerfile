FROM osrf/ros:jazzy-desktop-full
RUN apt-get update && apt-get install -y \
    python3-opencv \
    ros-jazzy-cv-bridge \
    ros-jazzy-rclpy \
    python3-aiohttp \
    python3-pip
# RUN apt-get update && apt-get install -y \ #Optional add in if running from a volume instead..

RUN pip install sty just_playback evdev PySoundSphere --break-system-packages \
    && apt-get install -y ros-jazzy-cv-bridge ros-jazzy-image-transport

############## Base Image setup above

############# ROS setup below
ENTRYPOINT [".devcontainer/sourceAndBuild.sh"]

#RUN git clone https://github.com/SoonerRobotics/autonav_software_2025.git #optional if wanting to run by cloning instead else copy all current dir contents to container

COPY . /autonav_software_2025/

WORKDIR /autonav_software_2025

EXPOSE 8023
EXPOSE 8080
# Pretty sure this is needed since running the websocket