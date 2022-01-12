// Demos basics of how to create and publish a nav_msgs/Path message

#include "mixed_reality_library/mixed_reality_library.h"

ros::Subscriber path_sub;
ros::Publisher test_pub;

void received(const sensor_msgs::NavSatFix &path)
{
  double lat = path.latitude;
  double lon = path.longitude;
  ROS_INFO("got path! %f  %f", lat, lon);
  nav_msgs::Path tpath;
  geometry_msgs::PoseStamped pose;
  for (int i = 0; i < 100; i++)
  {
    pose.pose.position.x = i;
    pose.pose.position.y = 2*i;
    tpath.poses.push_back(pose);
  }
  test_pub.publish(tpath);
  
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "testr2");
  ros::NodeHandle n;
  ROS_INFO("hello");
  path_sub = n.subscribe("/sim_gps", 100, received);
  test_pub = n.advertise<nav_msgs::Path>("/gps_path", 1000);
  ros::spin();
  return 0;
}
