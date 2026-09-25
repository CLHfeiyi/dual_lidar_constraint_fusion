cmake_minimum_required(VERSION 2.8.3)
project(lio_sam)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  tf
  tf2
  tf2_ros
  tf2_sensor_msgs
  roscpp
  rospy
  cv_bridge
  pcl_conversions
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_generation
  message_filters
  visualization_msgs
)

find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
find_package(GTSAM REQUIRED QUIET)
find_package(Boost REQUIRED COMPONENTS timer)

add_message_files(
  DIRECTORY msg
  FILES
  cloud_info.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  save_map.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL GTSAM
  CATKIN_DEPENDS
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  message_runtime
  message_generation
  visualization_msgs
)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${GTSAM_LIBRARY_DIRS}
)

# Paper SLAM Algorithm (IEKF + VGICP Loop Detection)
add_executable(${PROJECT_NAME}_paper src/paper_slam.cpp)
add_dependencies(${PROJECT_NAME}_paper ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_compile_options(${PROJECT_NAME}_paper PRIVATE ${OpenMP_CXX_FLAGS} -include ${CMAKE_CURRENT_SOURCE_DIR}/flann_fix.h)
target_link_libraries(${PROJECT_NAME}_paper Boost::timer ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS} gtsam)

# Dual-LiDAR point cloud merger
add_executable(dual_lidar_merger src/dual_lidar_merger.cpp)
add_dependencies(dual_lidar_merger ${catkin_EXPORTED_TARGETS})
target_link_libraries(dual_lidar_merger ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# Python scripts
catkin_install_python(PROGRAMS
  scripts/trajectory_saver.py
  scripts/run_hkmems.py
  scripts/plot_comparison.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
