cmake_minimum_required(VERSION 3.0.2)
project(algorithm_2)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  mixed_reality_library
)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES algorithm_2
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(sim_algorithm_2 src/sim_algorithm_2.cpp)
target_link_libraries(sim_algorithm_2 ${catkin_LIBRARIES})


