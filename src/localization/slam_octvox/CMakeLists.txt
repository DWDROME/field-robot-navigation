cmake_minimum_required(VERSION 3.10)
project(slam_octvox)

# slam_octvox is a header-only library. It vendors Super-LIO's OctVoxMap
# (GPLv3) and the MIT-licensed tsl robin_map. See LICENSE + include/*/*.hpp
# for original attribution.
#
# Consumers only need to add this package to their catkin COMPONENTS; the
# include directories propagate automatically via catkin_package().

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS include
)

# Install headers if the workspace is installed. Not required for source
# builds (devel space picks up include/ automatically).
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/..
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)
