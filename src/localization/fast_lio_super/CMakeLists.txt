cmake_minimum_required(VERSION 3.10)
project(fast_lio_super)

SET(CMAKE_BUILD_TYPE "Release")

# Phase B：全包升级到 C++17，与 slam_octvox（vendored Super-LIO）一致。
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -pthread -fexceptions")
set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -fexceptions")

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")
  if(N GREATER 4)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3)
    message("core for MP: 3")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
    message("core for MP: 2")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

find_package(PythonLibs REQUIRED)
find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  pcl_ros
  tf
  livox_ros_driver2
  message_generation
  eigen_conversions
  message_filters
  diagnostic_msgs
  slam_octvox          # Phase B-1：header-only GPL vendored OctVoxMap
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(GTSAM 4.2 REQUIRED)
find_package(Threads REQUIRED)

message(Eigen: ${EIGEN3_INCLUDE_DIR})

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  include)

add_message_files(
  FILES
  Pose6D.msg
)

generate_messages(
 DEPENDENCIES
 geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime slam_octvox message_filters diagnostic_msgs
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS include
)

# ------------------------------------------------------------
# slam_core: 可插拔 MapManager / NeighborSearch / Relocalizer 层
# 见 docs/ARCHITECTURE.md 与 docs/ITERATION_PLAN.md
# Phase A：ikd-tree 后端（IkdTreeMap）
# Phase B-1：新增 OctVoxMap 后端，来自 slam_octvox 包的 vendored Super-LIO
# ------------------------------------------------------------
add_library(fast_lio_super_slam STATIC
  src/slam/map/ikd_tree_map.cpp
  src/slam/map/oct_vox_map.cpp
  include/ikd-Tree/ikd_Tree.cpp
)
add_dependencies(fast_lio_super_slam ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_include_directories(fast_lio_super_slam PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  include
)
target_link_libraries(fast_lio_super_slam ${PCL_LIBRARIES})
set_target_properties(fast_lio_super_slam PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(fastlio_super_mapping src/laserMapping.cpp src/preprocess.cpp)
add_dependencies(fastlio_super_mapping ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(fastlio_super_mapping fast_lio_super_slam ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES})
target_include_directories(fastlio_super_mapping PRIVATE ${PYTHON_INCLUDE_DIRS})

add_library(fast_lio_super_graph STATIC src/slam/graph/scan_context.cpp src/slam/graph/gicp.cpp
  src/slam/graph/pose_graph.cpp src/slam/graph/backend.cpp)
target_include_directories(fast_lio_super_graph PUBLIC include ${GTSAM_INCLUDE_DIR})
target_link_libraries(fast_lio_super_graph PUBLIC gtsam ${PCL_LIBRARIES} Threads::Threads)
add_executable(fast_lio_super_backend src/slam/backend_ros1.cpp)
target_link_libraries(fast_lio_super_backend fast_lio_super_graph ${catkin_LIBRARIES})
add_dependencies(fast_lio_super_backend ${catkin_EXPORTED_TARGETS})
install(TARGETS fastlio_super_mapping fast_lio_super_backend
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
