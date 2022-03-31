#include <mixed_reality_library/mixed_reality_library.h>
#include <iterator>
#include <random>

// Variables
double heading_deg;
double heading_rad;

// Define GPS coordinates of origin on map (Current numbers for Rellis Starlab environment)
double origin_lat = 30.632760;
double origin_long = -96.476100;
    
ros::Publisher sim_gps_pub; // publish simulated GPS data from robot
ros::Publisher sim_heading_pub; //publish simulated heading data from robot
ros::Subscriber gnd_truth_sub; // subscribe to lat and lon from GPS receiver



void gps_publish(const geometry_msgs::PoseStamped &msg)
{
  //Convert the quaternion into euler angles.
  tf::Quaternion cur_quat;
  double roll, pitch, yaw;
  tf::quaternionMsgToTF(msg.pose.orientation, cur_quat);
  tf::Matrix3x3(cur_quat).getRPY(roll, pitch, yaw); 
  // Note: 0 heading is east, north is pi/2.
  heading_rad = yaw;
  heading_deg = heading_rad*180/M_PI;
  
  // Calculate simulated GPS coordinates of robot.
  double x_meters = msg.pose.position.x;
  double y_meters = msg.pose.position.y;
  const double mean = 0.0;
  const double stddev = 0.02;
  std::mt19937 generator(std::random_device{}()); 
  //std::default_random_engine generator;
  std::normal_distribution<double> dist(mean, stddev);
  double noise = dist(generator);
  double x_meters_noisy = x_meters + noise;
  double y_meters_noisy = y_meters + noise;
  double gps_lat = origin_lat + y_meters_noisy/111320;
  double gps_long = origin_long + x_meters_noisy/(40075000*cos(gps_lat*M_PI/180)/360);
  
  //Publish simulated GPS coordinates.
  sensor_msgs::NavSatFix gps_msg;
  gps_msg.latitude = gps_lat;
  gps_msg.longitude = gps_long;
  sim_gps_pub.publish(gps_msg);

  
  //Publish simulated heading data to a point message
  geometry_msgs::Point head_msg;
  head_msg.z = heading_deg;
  sim_heading_pub.publish(head_msg);

  ROS_INFO("Heading: %f, %f, %f,", gps_lat, gps_long, heading_deg);
ROS_INFO(" %f, %f, %f,", noise, x_meters_noisy, y_meters_noisy);

}
// edited version

int main (int argc, char** argv) {
  ros::init(argc, argv, "simulated_gps");
  ros::NodeHandle n;
  while (ros::ok())
  {
     // get estimates for current pose and heading and write global variables
     gnd_truth_sub = n.subscribe("/unity_command/ground_truth/husky/pose", 10, gps_publish);
    ros::Rate loop_rate(100);
    ros::spinOnce();
    loop_rate.sleep();}
  
  sim_gps_pub = n.advertise<sensor_msgs::NavSatFix>("/sim_gps", 10);
  sim_heading_pub = n.advertise<geometry_msgs::Point>("/sim_heading", 10);
  
  ros::spin(); // this line is not in original example code.. might be an issue
return 0;
}

