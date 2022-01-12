// Demos basics of how to subscribe to and decompose a nav_msgs/Path message
#include "mixed_reality_library/mixed_reality_library.h"
using namespace std;
ros::Subscriber path_sub;

void received(const nav_msgs::Path &msg)
{
  int L = msg.poses.size();
  std::vector<double> lat_vector(L);
  std::vector<double> long_vector(L);
  for (int i = 0; i < L; i++)
  {
  geometry_msgs::PoseStamped pose1;
  pose1 = msg.poses.at(i);
  lat_vector.at(i) = pose1.pose.position.x;
  long_vector.at(i) = pose1.pose.position.y;
  cout << lat_vector.begin() << endl
  //ROS_INFO("lat %d = %f, long %d = %f", i, lat_vector.at(i), i, 
  //  long_vector.at(i));
  }
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "testr");
  ros::NodeHandle n;
  path_sub = n.subscribe("/gps_path", 10000, received);
  ros::spin();
  return 0;
}
