cmake_minimum_required(VERSION 3.0.2)
project(husky_vicon_link)

## Use C++11
add_definitions (--std=c++11)

## Find catkin macros and libraries
find_package (catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs tf)

## Generate added messages and services with any dependencies listed here
generate_messages (DEPENDENCIES std_msgs geometry_msgs)

## Declare a catkin package
catkin_package (
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs 
)

## Specify additional locations of header files
include_directories (include ${catkin_INCLUDE_DIRS})

add_executable(husky_vicon_link src/husky_vicon_link.cpp)
target_link_libraries(husky_vicon_link ${catkin_LIBRARIES})

add_dependencies (husky_vicon_link ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${Boost_EXPORTED_TARGETS} husky_vicon_link_generate_messages_cpp)

## Mark executables and/or libraries for installation
install(TARGETS husky_vicon_link
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)
