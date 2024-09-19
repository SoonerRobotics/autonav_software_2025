cmake_minimum_required(VERSION 3.8)
project(autonav_example_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(autonav_shared REQUIRED)
 
# C++
add_executable(autonav_example_cpp src/example.cpp)
ament_target_dependencies(autonav_example_cpp rclcpp autonav_shared)

install(TARGETS
  autonav_example_cpp
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
