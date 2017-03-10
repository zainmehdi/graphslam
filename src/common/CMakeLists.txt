cmake_minimum_required(VERSION 2.8.3)
project(common)

find_package(catkin REQUIRED COMPONENTS
  tf
  pcl_ros
  pcl_conversions
  laser_geometry
  cmake_modules
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
  visualization_msgs
  )

find_package(
  Eigen3 REQUIRED
  )

add_message_files(
  FILES
  Factor.msg
  Keyframe.msg
  Keyframes.msg
  Odometry.msg
  Registration.msg
  Pose2DWithCovariance.msg
  )

add_service_files(
  FILES
  LastKeyframe.srv
  ClosestKeyframe.srv
  OdometryBuffer.srv
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  )

catkin_package(
  DEPENDS message_runtime
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  )

add_executable(markers src/markers.cpp)
target_link_libraries(markers ${catkin_LIBRARIES})
add_dependencies(markers common_gencpp)

add_executable(gicp src/gicp.cpp)
target_link_libraries(gicp ${catkin_LIBRARIES})
add_dependencies(gicp common_gencpp)


