#include "mixed_reality_library/mixed_reality_library.h"

// Publisher and Subscribers
ros::Publisher compiled_gps; 
ros::Subscriber gps_sub; 
ros::Subscriber heading_sub; 
// Declare global variables
double heading_rad;

void gps_define(const sensor_msgs::NavSatFix &msg)
{
  double latitude = msg.latitude;
  double longitude = msg.longitude;
  
  geometry_msgs::Pose2D pose;
  pose.x = latitude;
  pose.y = longitude;
  pose.theta = heading_rad;
  ROS_INFO("Publishing compiled GPS data");
  compiled_gps.publish(pose); 
}

void heading_define(const geometry_msgs::Point &msg)
{
  double heading_deg = msg.z;
  heading_rad = heading_deg*M_PI/180;
  //ROS_INFO("debugging");
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "sim_compiled_gps");
  ros::NodeHandle n;

  // Get estimates for current lat, long, and heading, and call functions
  gps_sub = n.subscribe("/sim_gps", 500, gps_define);
  heading_sub = n.subscribe("/sim_heading", 500, heading_define);

  // Publish compiled gps information
  compiled_gps = n.advertise<geometry_msgs::Pose2D>("/sim_compiled_gps", 1000);

  ros::spin();

}
