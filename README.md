# autonav_software_2025

![Github Workflow Status](https://img.shields.io/github/actions/workflow/status/SoonerRobotics/autonav_software_2024/compile_run.yml)

Software for our 2025 [Intelligent Ground Vehicle Competition](http://www.igvc.org/) AutoNav challenge entry, **NAME TBD**.  
We are using [ROS2 Jazzy](https://docs.ros.org/en/jazzy/index.html) on [Ubuntu 24.04](https://releases.ubuntu.com/24.04/).

## Dependencies

To setup all dependencies, run the following two commands. It is **CRITICAL** you do **NOT** run these commmands as **sudo**
```bash
cd setup
echo "machine files.dylanzeml.in login <user> password <password>" > vectorsecrets.txt
./setup.sh
```

## Building

```bash
source /opt/ros/humble/setup.bash
cd autonav_ws
colcon build
source /install/setup.bash
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

# Shared Node Compatability

Listed below is a table that compares compability for the C++ and Python version of the shared node. This should be updated **any** time either version is changed.

| Feature | C++ | Python |
| ------- | --- | ------ |
| Console Logging | ✅ | ✅ |
| File Logging | ❌ | ❌ |
| Topic Logging | ❌ | ❌ |
| Configuration | ❌ | ❌ |
| Conbus | ❌ | ❌ |
| Performance Logging | ✅ | ✅ |
| System State | ✅ | ✅ |
| Device State | ✅ | ✅ |

# Shared Node API

Listed below is a table that shows the API for the shared node. This should be updated **any** time either version is changed.

| Feature | C++ | Python |
| ------- | --- | ------ |
| log(message, level) | ✅ | ✅ |
| set_system_state(state) | ✅ | ✅ |
| set_system_state(state, has_mobility) | ✅ | ❌ |
| set_mobility(has_mobility) | ✅ | ✅ |
| set_device_state(state) | ✅ | ✅ |
| set_device_state(device, state) | ✅ | ❌ |
| get_device_state() | ✅ | ✅ |
| get_device_state(device) | ✅ | ✅ |
| get_system_state() | ✅ | ✅ |
| is_mobility() | ✅ | ✅ |
| perf_start(name) | ✅ | ✅ |
| perf_stop(name, print_to_console) | ✅ | ✅ |

## Style
All folder names including packages should be in `snake_case`

Avoid most abbreviations

Tab length is 4 spaces (default)

<!-- CMakeLists.txt and package.xml should follow the patterns found in [format_packages](https://github.com/SoonerRobotics/autonav_software_2024/tree/feat/particle_filter/scripts/format_package) -->

### Python
Python files should be written in the [PEP-8 style](https://peps.python.org/pep-0008/)

### C++
file names should be in `snake_case`

class names should be in `UpperCamelCase` (to agree with [rclcpp](https://docs.ros2.org/foxy/api/rclcpp/index.html))

function names should be in `camelCase`

