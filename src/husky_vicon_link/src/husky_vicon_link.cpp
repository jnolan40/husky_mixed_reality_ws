// This code takes Husky robot pose data from the VICON motion capture system 
// and feeds it into the Unity robotics simulation to control the position
// of a simulated husky.

#include "husky_vicon_link/husky_vicon_link.h"

geometry_msgs::Point init_pose;// intial robot state capture
bool vicon_init = true; 

ros::Publisher simcontrol_pub;

void vicon_pose_cb(const geometry_msgs::TransformStamped &msg) {
  
  // Define variables
  tf::Quaternion cur_quat;
  geometry_msgs::Point husPos;
  double roll, pitch, yaw;
  geometry_msgs::Point vicon_pose_husframe;
  
  // vicon message convert the input quaternion to RPY
  tf::quaternionMsgToTF(msg.transform.rotation, cur_quat);
  tf::Matrix3x3(cur_quat).getRPY(roll, pitch, yaw);  


  // save the inital position of the robot
  if (vicon_init) {
    init_pose.x = msg.transform.translation.x;
    init_pose.y = msg.transform.translation.y;
    init_pose.z = yaw;
    vicon_init = false;
  } // end of initialization block

  // husky frame from starting point --> coordinate transform
  vicon_pose_husframe.x = -((init_pose.x - msg.transform.translation.x)
   * cos(init_pose.z) + (init_pose.y - msg.transform.translation.y)
   * sin(init_pose.z));
  vicon_pose_husframe.y = -(-(init_pose.x - msg.transform.translation.x)
   * sin(init_pose.z) + (init_pose.y - msg.transform.translation.y)
   * cos(init_pose.z));
  vicon_pose_husframe.z = yaw - init_pose.z;
 
  //Creating the message to control simulated Husky location in Phoenix/Unity
  //worldframe coordinates.

  double xscale = 1; double yscale = 1; double thetascale = 1;
  double x0 = 90.5; double y0 = -10; 	//position in Unity map to start

  double x = x0 + xscale*(vicon_pose_husframe.x);	
  double y = y0 + yscale*(vicon_pose_husframe.y);	
  double z = 0.15;
  double u = 0;
  double v = 0;
  //note theta is reversed due to differences in sign convention in unity
  double w = -thetascale*(vicon_pose_husframe.z * 180/M_PI); 

  std_msgs::String message;
  std::stringstream ss;
  ss << "husky pose " << x << " " << y << " " <<  z << " " <<  u << " "
   <<  v << " " <<  w;
  message.data = ss.str();

  ROS_INFO("%s", message.data.c_str());
  simcontrol_pub.publish(message);
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "husky_vicon");
  ros::NodeHandle hv;
  ros::Subscriber vicon_tf_sub;
  simcontrol_pub = hv.advertise<std_msgs::String>("/unity_command/command_topic",
   10);
  vicon_tf_sub = hv.subscribe("/vicon/Husky2/root", 100, vicon_pose_cb);
  ros::spin();
}
