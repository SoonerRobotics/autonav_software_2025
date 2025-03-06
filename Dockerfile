FROM osrf/ros:jazzy-desktop-full

RUN mkdir -p /autonav_software_2025
COPY . /autonav_software_2025

WORKDIR /autonav_software_2025
# RUN apt-get update  \
#     && \
#     apt-get install -y python3-pip


#RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash; colcon build"