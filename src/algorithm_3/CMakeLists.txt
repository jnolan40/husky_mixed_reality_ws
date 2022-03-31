cmake_minimum_required(VERSION 3.0.2)
project(algorithm_3)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  mixed_reality_library
  ubxtranslator
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(sim_algorithm_3 src/sim_algorithm_3.cpp)
target_link_libraries(sim_algorithm_3 ${catkin_LIBRARIES})

add_executable(mr_algorithm_3 src/mr_algorithm_3.cpp)
target_link_libraries(mr_algorithm_3 ${catkin_LIBRARIES})

add_executable(real_algorithm_3 src/real_algorithm_3.cpp)
target_link_libraries(real_algorithm_3 ${catkin_LIBRARIES})

add_executable(a_star src/a_star.cpp)
target_link_libraries(a_star ${catkin_LIBRARIES})

add_executable(expt_astar src/expt_astar.cpp)
target_link_libraries(expt_astar ${catkin_LIBRARIES})

add_executable(testr3 src/testr3.cpp)
target_link_libraries(testr3 ${catkin_LIBRARIES})

add_executable(data_harvester src/data_harvester.cpp)
target_link_libraries(data_harvester ${catkin_LIBRARIES})
