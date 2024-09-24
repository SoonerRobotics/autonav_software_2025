cmake_minimum_required(VERSION 3.8)
project(autonav_feelers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(autonav_msgs REQUIRED)
# find_package(scr REQUIRED) #TODO
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
 
# Install Python programs
install(PROGRAMS
  src/feelers.py
  src/feeler_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# C++
add_executable(feeler_node src/feeler_node.cpp)
ament_target_dependencies(expandify rclcpp autonav_msgs cv_bridge image_transport scr) #TODO

install(TARGETS
  feeler_node
  # feeler
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()