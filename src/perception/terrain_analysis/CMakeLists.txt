cmake_minimum_required(VERSION 3.16)
project(terrain_analysis)

# ROS Jazzy's ortools_vendor ships an Eigen3 config that exports
# EIGEN_MPL2_ONLY without the -D prefix, which the compiler then reads as an
# input file name. Prefer the system Eigen3 configuration when it exists.
if(NOT DEFINED Eigen3_DIR AND EXISTS "/usr/share/eigen3/cmake/Eigen3Config.cmake")
  set(Eigen3_DIR "/usr/share/eigen3/cmake" CACHE PATH "System Eigen3, avoiding the ortools_vendor export")
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)

add_executable(terrainAnalysis src/terrainAnalysis.cpp)
target_include_directories(terrainAnalysis PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(terrainAnalysis ${PCL_LIBRARIES})
ament_target_dependencies(
  terrainAnalysis
  builtin_interfaces
  nav_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  std_msgs
  tf2
)

install(
  TARGETS terrainAnalysis
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
