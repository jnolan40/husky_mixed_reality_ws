// This code takes in /cmd_vel messages from the teleop_twist_keyboard package and 
// converts them into the desired format for the robot being controlled.
#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
using namespace std;
// Publisher and Subscribers
ros::Publisher robot_twist_control; 
ros::Subscriber cmd_sub; 
void write_vel(const geometry_msgs::Twist &msg)
{
  double vx = msg.linear.x;
  double wz = msg.angular.z;
  geometry_msgs::Twist command;
  command.linear.x = vx;
  command.angular.z = wz;
  robot_twist_control.publish(command);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_keyboard_control");
  ros::NodeHandle n;
  cmd_sub = n.subscribe("/cmd_vel", 500, write_vel);
  // Publish velocity commands to the robot
  robot_twist_control = n.advertise<geometry_msgs::Twist>("/husky/husky_velocity_controller/cmd_vel", 1000);
  ros::spin();
}
