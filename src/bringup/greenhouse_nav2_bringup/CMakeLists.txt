cmake_minimum_required(VERSION 3.16)
project(greenhouse_nav2_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    scripts/greenhouse_bringup_health
    scripts/greenhouse_validate_bringup
    scripts/run_matrix.py
    scripts/run_simulation_run.py
    scripts/capture_simulation_image.py
    scripts/capture_simulation_rviz.py
    scripts/simulation_lidar_filter.py
    scripts/simulation_ground_startup.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    config
    launch
    worlds
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(tf2_msgs REQUIRED)
  add_executable(check_tf_owners test/check_tf_owners.cpp)
  target_compile_features(check_tf_owners PRIVATE cxx_std_17)
  ament_target_dependencies(check_tf_owners rclcpp tf2_msgs)

  ament_add_pytest_test(
    test_simulation_process_cleanup
    test/test_simulation_process_cleanup.py
    TIMEOUT 30
  )

  ament_add_pytest_test(
    test_simulation_px4_startup
    test/test_simulation_px4_startup.py
    TIMEOUT 30
  )

  ament_add_pytest_test(
    test_simulation_lidar_filter
    test/test_simulation_lidar_filter.py
    TIMEOUT 30
  )
  ament_add_pytest_test(
    test_contracts
    test/test_contracts.py
    TIMEOUT 30
  )
  ament_add_pytest_test(
    test_health_core
    test/test_health_core.py
    TIMEOUT 30
  )
  ament_add_pytest_test(
    test_health_monitor_shutdown
    test/test_health_monitor_shutdown.py
    TIMEOUT 30
  )
  ament_add_pytest_test(
    test_launch_contract
    test/test_launch_contract.py
    TIMEOUT 30
  )
endif()

ament_package()
