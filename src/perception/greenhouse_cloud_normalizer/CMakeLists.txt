cmake_minimum_required(VERSION 3.16)
project(greenhouse_cloud_normalizer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

add_executable(greenhouse_cloud_normalizer src/cloud_normalizer_node.cpp)
target_include_directories(
  greenhouse_cloud_normalizer
  PRIVATE include
)
ament_target_dependencies(
  greenhouse_cloud_normalizer
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
)

install(TARGETS greenhouse_cloud_normalizer DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(
    cloud_transformer_test
    test/test_cloud_transformer.cpp
  )
  if(TARGET cloud_transformer_test)
    target_include_directories(cloud_transformer_test PRIVATE include)
    ament_target_dependencies(
      cloud_transformer_test
      rclcpp
      sensor_msgs
      tf2
      tf2_ros
      tf2_sensor_msgs
    )
  endif()
endif()

ament_export_include_directories(include)
ament_export_dependencies(sensor_msgs tf2 tf2_ros tf2_sensor_msgs)
ament_package()
