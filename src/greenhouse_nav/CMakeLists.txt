cmake_minimum_required(VERSION 3.0.2)
project(greenhouse_nav)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  std_msgs
  tf
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/cmd_vel_stamped_bridge.py
  scripts/global_to_local_adapter.py
  scripts/localization_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
