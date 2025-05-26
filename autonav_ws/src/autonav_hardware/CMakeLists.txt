cmake_minimum_required(VERSION 3.8)
project(autonav_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(autonav_shared REQUIRED)
find_package(autonav_msgs REQUIRED)
# C++

# Inlcude Cpp "include" directory
include_directories(include)

# Create Cpp executables
#add_executable(executable_name path_to_executable/executable.cpp)
#ament_target_dependencies(executable_name rclcpp other_dependencies)

# Install Cpp executables
install(TARGETS
  # install executables by name
  # executable_name
  DESTINATION lib/${PROJECT_NAME}
 )
 
# Python

# Use only if not using rosidl_generate_interfaces
# Install Python modules
#ament_python_install_package(${PROJECT_NAME})
 
 # Install Python programs
install(PROGRAMS
  src/swerve/can_spark_max.py
  src/swerve/rev_messages.py
  src/swerve/swerve_config.py
  src/swerve/swerve_drive.py
  src/swerve/swerve_module.py
  DESTINATION lib/${PROJECT_NAME}/swerve
)

install(PROGRAMS
  # add programs in format:
  src/can_node.py
  src/sparkmax_node.py
  src/audible_feedback.py
  src/simple_audible_feedback_publisher.py
  src/can_node.py
  src/camera_node.py
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install Vectornav Dependencies
set(VECTORNAV_DIR "/usr/local/vectornav/cpp")
add_library(vectornav SHARED
	${VECTORNAV_DIR}/src/Implementation/AsciiPacketDispatcher.cpp
	${VECTORNAV_DIR}/src/Implementation/AsciiPacketProtocol.cpp
	${VECTORNAV_DIR}/src/Implementation/BinaryHeader.cpp
	${VECTORNAV_DIR}/src/Implementation/CommandProcessor.cpp
	${VECTORNAV_DIR}/src/Implementation/FaPacketDispatcher.cpp
	${VECTORNAV_DIR}/src/Implementation/FaPacketProtocol.cpp
	${VECTORNAV_DIR}/src/Implementation/FbPacketDispatcher.cpp
	${VECTORNAV_DIR}/src/Implementation/FbPacketProtocol.cpp
	${VECTORNAV_DIR}/src/Implementation/PacketSynchronizer.cpp
	${VECTORNAV_DIR}/src/Interface/Command.cpp
	${VECTORNAV_DIR}/src/Interface/Registers.cpp
	${VECTORNAV_DIR}/src/Interface/Sensor.cpp
)

target_include_directories(vectornav PUBLIC
	${VECTORNAV_DIR}/include
)

add_executable(vectornav_node
	src/vectornav_node.cpp
)
ament_target_dependencies(vectornav_node
  rclcpp
	autonav_shared
	autonav_msgs
)

target_link_libraries(vectornav_node
	vectornav
)

ament_export_include_directories(include)
ament_export_libraries(vectornav)

install(TARGETS
	vectornav_node
	DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS vectornav EXPORT export_vectornav
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
)

ament_package()