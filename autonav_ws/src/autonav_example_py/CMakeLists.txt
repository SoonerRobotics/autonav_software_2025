cmake_minimum_required(VERSION 3.8)
project(autonav_example_py)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(autonav_shared REQUIRED)
 
# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python programs
install(PROGRAMS
  src/example.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
