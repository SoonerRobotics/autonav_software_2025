cmake_minimum_required(VERSION 3.8)
project(autonav_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# generate messages
set(msg_files
    "msg/AngularPIDStatistics.msg"
    "msg/AudibleFeedback.msg"
    "msg/CanStats.msg"
    "msg/Conbus.msg"
    "msg/ConfigurationBroadcast.msg"
    "msg/ConfigurationUpdate.msg"
    "msg/ControllerInput.msg"
    "msg/DeviceState.msg"
    "msg/GPSFeedback.msg"
    "msg/HardwarePerformance.msg"
    "msg/IMUData.msg"
    "msg/LinearPIDStatistics.msg"
    "msg/Log.msg"
    "msg/MotorFeedback.msg"
    "msg/MotorInput.msg"
    "msg/MotorStatistics.msg"
    "msg/NUCStatistics.msg"
    "msg/Performance.msg"
    "msg/Position.msg"
    "msg/SafetyLights.msg"
    "msg/SystemState.msg"
    "msg/Ultrasonic.msg"
    "msg/ZeroEncoders.msg"
    "msg/SwerveAbsoluteFeedback.msg"
    "msg/Performance.msg"
    "msg/ConfigurationBroadcast.msg"

    "msg/SwerveFeedback.msg"
    "msg/PathingDebug.msg"
    "msg/Command.msg"
)

set(srv_files
    "srv/SetDeviceState.srv"
    "srv/SetSystemState.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
    ${srv_files}
    DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()