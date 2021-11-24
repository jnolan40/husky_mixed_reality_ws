// This code is an open loop GPS waypoint navigation code. It works on the principle
// of pure pursuit and does not have closed loop feedback. 

#include "mixed_reality_library/mixed_reality_library.h"

// Publisher and Subscribers
ros::Publisher husky_twist_control; 
ros::Subscriber gps_sub; 
ros::Subscriber heading_sub; 
// Declare global variables
double heading_rad;
double vxi = 0;
double wzi;
double t_end;
double final_pos_error = 0;
int init = 1;
double xgi;
double ygi;
// Goal coordinates 
double lat_goal = 30.632500;
double long_goal = -96.47525;

void gps_define(const sensor_msgs::NavSatFix &msg)
{
  double latitude = msg.latitude;
  double longitude = msg.longitude;
  
  double ygn = (lat_goal - latitude)*111320; // meters north of robot
  double xgn = (long_goal - longitude)*40075000*cos(latitude*M_PI/180)/360; //meters east of robot
  double theta = -(heading_rad - M_PI/2);
  double xg = xgn*cos(theta) - ygn*sin(theta); // meters to right of robot
  double yg = xgn*sin(theta) + ygn*cos(theta); // meters in front of robot

  double l = sqrt(pow(xg,2) + pow(yg,2));
  double rho = abs(pow(l,2)/(2*xg));
  double d = rho - abs(xg);
  double angle = abs(atan2(yg,d));
  double s = rho*angle;

  double vx;
  if(yg >= 0)
  {
    vx = 0.75;
  }
  else
  {
    vx = -0.75;
  }
  double wz;
  if(xg>=0)
  {
    wz = -vx/rho;
  }
  else
  {
    wz = vx/rho;
  }
  double t_needed = abs(s/vx);
  double ref_time;
  if(init == 5)
  {
    vxi = vx;
    wzi = wz;
    xgi = xg;
    ygi = yg;
    ref_time = t_needed;
    double t_start = ros::Time::now().toSec();
    t_end = t_start + t_needed;
  }

  double t_now = ros::Time::now().toSec(); 
  double t_remaining = t_end - t_now;

  geometry_msgs::Twist command;
  if(t_remaining > 0)
  {
  command.linear.x = vxi;
  command.angular.z = wzi;
  }
  else
  {
  command.linear.x = 0;
  command.angular.z = 0;
  final_pos_error = l;
  }
  
  husky_twist_control.publish(command);
  init = init + 1;
  ROS_INFO(" %f, %f, %f, %f", ref_time, t_remaining, theta, l); // Debug tool---------------------------------
}

void heading_define(const geometry_msgs::Point &msg)
{
  double heading_deg = msg.z;
  heading_rad = heading_deg*M_PI/180;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "sim_algorithm_1");
  ros::NodeHandle n;

  // Get estimates for current lat, long, and heading, and call functions
  gps_sub = n.subscribe("/sim_gps", 500, gps_define);
  heading_sub = n.subscribe("/sim_heading", 500, heading_define);

  // Publish velocity commands to the robot
  husky_twist_control = n.advertise<geometry_msgs::Twist>("/husky/husky_velocity_controller/cmd_vel", 1000);

  ros::spin();

}
