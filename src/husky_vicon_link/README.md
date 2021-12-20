Launch with:
rosrun husky_vicon_link husky_vicon_link

This package contains files to take robot pose data from the VICON motion capture system and use it to control the pose of a simulated robot in a Unity Robotics environment. The vicon_bridge package must be run with this package to provide the pose data. Adjust robot starting position, distance scale, height, etc. in the husky_vicon_link.cpp file.
