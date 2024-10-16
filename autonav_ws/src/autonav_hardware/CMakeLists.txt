cmake_minimum_required(VERSION 3.8)
project(autonav_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(autonav_shared REQUIRED)
find_package(autonav_msgs REQUIRED)
 
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