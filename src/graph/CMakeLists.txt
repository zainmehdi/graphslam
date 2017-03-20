cmake_minimum_required(VERSION 2.8.3)
project(graph)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  common
  )

find_package(GTSAM REQUIRED)

find_package(
  Eigen3 REQUIRED
  )


catkin_package()

include_directories(
  include
  ../common/include
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS})

add_executable(graph src/graph.cpp)
target_link_libraries(graph ${catkin_LIBRARIES} gtsam)
add_dependencies(graph common_gencpp)

