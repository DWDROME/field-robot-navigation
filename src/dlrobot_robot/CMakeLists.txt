cmake_minimum_required(VERSION 3.8)
project(dlrobot_robot)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(std_msgs REQUIRED)

add_library(dlrobot_robot_core STATIC
  src/orientation_filter.cpp
  src/protocol.cpp
)
target_include_directories(dlrobot_robot_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(dlrobot_robot_node src/dlrobot_robot_node.cpp)
target_link_libraries(dlrobot_robot_node dlrobot_robot_core)
ament_target_dependencies(dlrobot_robot_node
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  serial
  std_msgs
)

install(TARGETS dlrobot_robot_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_protocol test/test_protocol.cpp)
  target_link_libraries(test_protocol dlrobot_robot_core)
  add_test(NAME protocol COMMAND test_protocol)
endif()

ament_package()
