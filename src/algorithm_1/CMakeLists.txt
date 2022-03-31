cmake_minimum_required(VERSION 3.0.2)
project(algorithm_1)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  mixed_reality_library
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES algorithm_1
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(sim_algorithm_1 src/sim_algorithm_1.cpp)
target_link_libraries(sim_algorithm_1 ${catkin_LIBRARIES})

add_executable(real_algorithm_1 src/real_algorithm_1.cpp)
target_link_libraries(real_algorithm_1 ${catkin_LIBRARIES})

add_executable(test_include_library src/test_include_library.cpp)
target_link_libraries(test_include_library ${catkin_LIBRARIES})
