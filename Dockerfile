FROM osrf/ros:jazzy-desktop-full
# RUN apt-get update && apt-get install -y \ #Optional add in if running from a volume instead..
ENTRYPOINT [".devcontainer/sourceAndBuild.sh"]

#RUN git clone https://github.com/SoonerRobotics/autonav_software_2025.git #optional if wanting to run by cloning instead

COPY . /autonav_software_2025/

WORKDIR /autonav_software_2025