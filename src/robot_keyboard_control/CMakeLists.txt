cmake_minimum_required(VERSION 3.0.2)
project(robot_keyboard_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  teleop_twist_keyboard
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(robot_keyboard_control src/robot_keyboard_control.cpp)
target_link_libraries(robot_keyboard_control ${catkin_LIBRARIES})
