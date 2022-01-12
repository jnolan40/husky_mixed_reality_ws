// This code is an closed-loop GPS waypoint navigation code. It tends to dock the robot facing east.

#include "mixed_reality_library/mixed_reality_library.h"

// Publisher and Subscribers
ros::Publisher husky_twist_control; 
ros::Subscriber gps_sub; 
// Declare global variables
double xgi;
double ygi;
// Declare controller gains
double kp = 0.2;
double ka = 0.3;
double kb = -0.2;
// Declare Goal coordinates 
double lat_goal = 30.632500;
double long_goal = -96.47525;
double tolerance = 1;
int init = 1;
int second_init = 1;
double t_now;
double t_start;
double t_end;
double t_taken;
int status;

double modpi(double x){
    x = fmod(x + 3.14159/1,6.28318/1);
    if (x < 0)
        x += 6.28318/1;
    return x - 3.14159/1;
}

void nav_control(const geometry_msgs::Pose2D &msg)
{
  double latitude = msg.x;
  double longitude = msg.y;
  double heading_rad = msg.theta;
  
  double ygn = (lat_goal - latitude)*111320; // meters north of robot
  double xgn = (long_goal - longitude)*40075000*
    cos(latitude*M_PI/180)/360; //meters east of robot
  double theta = -(heading_rad - M_PI/2);
  double xg = xgn*sin(theta) + ygn*cos(theta); // meters in front of robot
  double yg = -xgn*cos(theta) + ygn*sin(theta); // meters to left of robot

  t_now = ros::Time::now().toSec();
  if(init == 1)
  {
    t_start = t_now;
    status = 0;
    init = init + 1;
  }

  double DY = ygn; //meters north of robot
  double DX = xgn; //meters east of robot
  double p = sqrt(pow(DX,2) + pow(DY,2));
  double alpha = modpi(-heading_rad + atan2(DY, DX));
  double beta = -atan2(DY,DX);
  double v;
  double w;

  if (alpha > -M_PI/2 && alpha < M_PI/2)
  {
  v = kp*p; //forward velocity command
  w = ka*alpha + kb*beta; //angular velocity command
  }
  else
  {
  v = -kp*p; //forward velocity command
  w = ka*modpi(alpha - M_PI) + kb*beta; //angular velocity command
  }
  
  if (p < tolerance)
  {
  v = 0;
  w = 0;
  if (second_init == 1)
  {
  t_end = t_now;
  t_taken = t_end - t_start;
  status = 1;
  second_init = second_init + 1;
  }
  }
  
  geometry_msgs::Twist command;
  command.linear.x = v;
  command.angular.z = w;
  
  husky_twist_control.publish(command); 
  if (status == 0)
  {
  ROS_INFO("status: navigating, time taken = %f", t_taken);
  }
  else
  {
  ROS_INFO("status: finished, time taken = %f", t_taken);
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "sim_algorithm_2");
  ros::NodeHandle n;

  // Get estimates for current lat, long, and heading, and call functions
  gps_sub = n.subscribe("/sim_compiled_gps", 500, nav_control);

  // Publish velocity commands to the robot
  husky_twist_control = n.advertise<geometry_msgs::Twist>("/husky/husky_velocity_controller/cmd_vel", 1000);

  ros::spin();

}
