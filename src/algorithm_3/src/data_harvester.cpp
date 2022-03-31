// This code reads in recorded gps data and writes it in a text file for 
// further plotting and analysis
#include "mixed_reality_library/mixed_reality_library.h"
using namespace std;
int init = 0;

// Publisher and Subscribers
ros::Publisher husky_twist_control; 
ros::Subscriber gps_sub; 
ros::Subscriber path_sub;
double t_start;
void read_data(const geometry_msgs::Twist &msg)
{
  if (init == 0){
    t_start = ros::Time::now().toSec();
    init = 1;
  }
  double lat = msg.linear.x;
  double lon = msg.linear.y;
  double ht = msg.linear.z;
  double lat_err = msg.angular.x;
  double lon_err = msg.angular.y;
  double ht_err = msg.angular.z;
  // Record current time
  double t_now = ros::Time::now().toSec();
  double t_elapsed = t_now - t_start;
  // Write data to file
  fstream gps_data;
  gps_data.open("gps_data_t4_3.15.2022.txt", ios_base::app);
  ROS_INFO("Writing Data %f, %f", lat, lon);
  gps_data << t_elapsed << ", " << lat << ", " << lon << ", " << ht << ", " << lat_err << ", " << lon_err << ", " << ht_err << endl;
  gps_data.close();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "data_harvester");
  ros::NodeHandle n;
  // Subscribe to relevant GPS readings and error data
  path_sub = n.subscribe("UBX/Combined", 500, read_data);

  ros::spin();
}
