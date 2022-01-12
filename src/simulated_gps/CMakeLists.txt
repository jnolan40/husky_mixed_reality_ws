cmake_minimum_required(VERSION 3.0.2)
project(simulated_gps)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  mixed_reality_library
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES simulated_gps
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(simulated_gps src/simulated_gps.cpp)
target_link_libraries(simulated_gps ${catkin_LIBRARIES})

add_executable(sim_gps_processor src/sim_gps_processor.cpp)
target_link_libraries(sim_gps_processor ${catkin_LIBRARIES})
