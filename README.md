# autonav_software_2025

![Github Workflow Status](https://img.shields.io/github/actions/workflow/status/SoonerRobotics/autonav_software_2025/compile_run.yml)


Software for our 2025 [Intelligent Ground Vehicle Competition](http://www.igvc.org/) AutoNav and Self Drive challenge entry, **Twistopher**.  

We are using [ROS2 Jazzy](https://docs.ros.org/en/jazzy/index.html) on [Ubuntu 24.04](https://releases.ubuntu.com/24.04/).

## Dependencies

To setup all dependencies, run the following two commands. It is **CRITICAL** you do **NOT** run these commmands as **sudo**. Contact Dylan for the password username and password!
```bash
cd setup
echo "machine files.dylanzeml.in login <user> password <password>" > vectorsecrets.txt
./setup.sh
```

## Building

```bash
source /opt/ros/jazzy/setup.bash
cd autonav_ws
colcon build
source install/setup.bash
```

## Autonomous/Manual

Follow the steps in [building](#building) and then run the following command
```bash
ros2 launch autonav_launch competition.xml
```
or
```bash
ros2 launch autonav_launch manual.xml
```

![](./img/Autonav25%20Controller%20Input%20Documentation.jpg)

## Simulation

Follow the steps in [building](#building) and then run the following command
```bash
ros2 launch autonav_launch simulation.xml
```

## VSCode

To edit the software with Visual Studio Code, please install the ros extension and open VSCode through the command line via `code` after running all steps under [Building](#building). To get proper intellisense for C++, create the following file: `.vscode/c_cpp_properties.json`
```json
{
    "configurations": [
        {
            "name": "Linux",
            "includePath": [
                "${workspaceFolder}/**",
                "/opt/ros/jazzy/include/**"
            ],
            "defines": [],
            "compilerPath": "/usr/bin/gcc",
            "cStandard": "c17",
            "cppStandard": "gnu++17",
            "intelliSenseMode": "linux-gcc-x64"
        }
    ],
    "version": 4
}
```
