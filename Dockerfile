FROM osrf/ros:jazzy-desktop-full

RUN mkdir -p /autonav/autonav_ws/src
RUN mkdir -p /autonav/setup
# RUN mkdir -p /autonav/deps

COPY autonav_ws/src /autonav/autonav_ws/src
COPY setup /autonav/setup
COPY vectorsecrets.txt /autonav/setup/vectorsecrets.txt
# COPY deps /autonav/deps

WORKDIR /autonav/setup
RUN /bin/bash -c "sudo chmod +x ./setup.sh"
RUN /bin/bash -c "./setup.sh"

WORKDIR /autonav/autonav_ws
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash; colcon build"
