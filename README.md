This workspace contains code for using mixed reality to study GPS waypoint/path following algorithms on the Clearpath Husky robot. 

-----------------------------------------------
----------WARNING-----------DANGER-------------
-----------------------------------------------
The navigation algorithms in this repository don't have any obstacle avoidance features. Therefore EXTREME CAUTION must be used when testing these algorithms with a physical Husky robot in a real-world environment. Testing should only be done where the environment is free of people and obstacles and someone must always be ready to intervene and stop the robot in case it goes haywire.
-----------------------------------------------

This package contains 3 autonomous navigation algorithms intended to be used with the Clearpath Husky robot. The first is an open loop GPS waypoint finding algorithm. The second is a closed loop docking algorithm. The third is an implementation of the Pure Pursuit path following algorithm, and works with an A* search algorithm node that plans the path around user-defined obstacles in the code.

Below are instructions for testing algorithms 1, 2, and 3 with three different testing methods: simulation, outdoor, and mixed reality.

1. Testing in Simulation
sim_algorithm_x.cpp files should be run with the Unity Robotics simulation open, and one must also run simulated_gps.cpp and sim_gps_processor.cpp in the simulated_gps package to get simulated GPS readings from the simulated Husky robot.

2. Outdoor Testing
real_algorithm_x.cpp files should be run on the Husky Robot in an outdoor setting with the ubxtranslator package used to get data from the simpleRTK2B Budget GPS receiver and gps_message_combiner.py to combine the messages.

3. Mixed Reality Testing
mr_algorithm_x.cpp files should be run with the Unity Robotics simulation open, the Husky robot in the Starlab, and the VICON motion tracking system operating with the Shogun program open and recognizing the Husky on a Windows machine. All computers need to be connected to the same network and the vicon_bridge package needs to be configured and run to publish the Husky's position to a ROS message that can be seen on the computer running Unity. It is also necessary to run the husky_vicon_link.cpp file in the husky_vicon_link package to allow the position of the physical Husky to control the position of the virtual Husky. Once the husky_vicon_link is successfully controlling the simulated robot based on the physical robot, run simulated_gps.cpp and sim_gps_processor.cpp in the simulated_gps package to start getting simulated GPS readings. At this point the mr_algorithm_x.cpp algorithms can be tested in mixed reality.

Brief description of all packages:
- algorithms 1, 2, and 3 are described above
- husky_vicon_link must be used in conjunction with the vicon_bridge package, and contains code to allow the location of a physical robot in the Starlab to control the location of a simulated robot in real time.
- mixed_reality_library includes references used by other codes in the workspace.
- robot_keyboard_control contains a simple keyboard controller for the Husky.
- simulated_gps contains code to simulate a GPS reading from the Husky robot in the TAMU RELLIS Campus environment in Unity.
- ubxtranslator reads data from the simpleRTK2B Budget GPS receiver and broadcasts the data as ROS messages. Usegps_message_combiner.py in the simulated GPS package to combine lat, long, and heading readings into 1 message.
- vicon_bridge takes data from a VICON system on the same network and broadcasts the position of tracked objects on ROS messages.