cmake_minimum_required(VERSION 3.8)
project(vectornav_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(autonav_shared REQUIRED)
find_package(autonav_mgs REQUIRED)
 
# C++
add_executable(vectornav_node src/vectornav_node.cpp)
ament_target_dependencies(vectornav_node rclcpp autonav_shared autonav_mgs)

install(TARGETS
  vectornav_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
