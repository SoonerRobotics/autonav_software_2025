cmake_minimum_required(VERSION 3.8)
project(example_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(autonav_shared REQUIRED)
 
# C++
add_executable(example_cpp src/main.cpp)
ament_target_dependencies(example_cpp rclcpp autonav_shared)

install(TARGETS
  example_cpp
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()