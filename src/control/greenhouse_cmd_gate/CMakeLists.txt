cmake_minimum_required(VERSION 3.16)
project(greenhouse_cmd_gate)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_library(greenhouse_command_gate_core
  src/command_gate_core.cpp
)
target_include_directories(greenhouse_command_gate_core
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
target_compile_options(greenhouse_command_gate_core
  PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)

add_executable(greenhouse_cmd_gate
  src/command_gate_node.cpp
)
target_link_libraries(greenhouse_cmd_gate
  greenhouse_command_gate_core
)
ament_target_dependencies(greenhouse_cmd_gate
  diagnostic_msgs
  geometry_msgs
  rclcpp
)
target_compile_options(greenhouse_cmd_gate
  PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)

add_executable(greenhouse_cmd_sink
  src/command_sink_node.cpp
)
ament_target_dependencies(greenhouse_cmd_sink
  geometry_msgs
  rclcpp
)
target_compile_options(greenhouse_cmd_sink
  PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)

install(
  TARGETS
    greenhouse_command_gate_core
    greenhouse_cmd_gate
    greenhouse_cmd_sink
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  enable_testing()
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_executable(command_gate_core_test
    test/command_gate_core_test.cpp
  )
  target_link_libraries(command_gate_core_test
    greenhouse_command_gate_core
  )
  target_compile_options(command_gate_core_test
    PRIVATE
      -Wall
      -Wextra
      -Wpedantic
  )
  add_test(
    NAME command_gate_core_test
    COMMAND command_gate_core_test
  )
  add_test(
    NAME command_gate_shutdown_test
    COMMAND
      "${Python3_EXECUTABLE}"
      "${CMAKE_CURRENT_SOURCE_DIR}/test/command_gate_shutdown_test.py"
      "$<TARGET_FILE:greenhouse_cmd_gate>"
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(greenhouse_command_gate_core)
ament_package()
