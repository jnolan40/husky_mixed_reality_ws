// code that subscribes to data from emulated simulation data (chatter1) found in testr file
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
ros::Subscriber new_gps_subscriber;
void gps_define(const std_msgs::String &msg)
{
  ROS_INFO("Received Chatter1 Message"); 
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dual");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Rate loop_rate(10);
  int count = 0;
  new_gps_subscriber = n.subscribe("/chatter1", 500, gps_define);
  while (ros::ok())
  {
    std_msgs::String msg;
    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();
    ROS_INFO("%s", msg.data.c_str());
    chatter_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}
