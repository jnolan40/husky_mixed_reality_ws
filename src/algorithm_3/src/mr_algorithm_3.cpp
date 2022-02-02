// This code is an closed-loop GPS waypoint navigation code that uses the Pure Pursuit algorithm to track 
#include "mixed_reality_library/mixed_reality_library.h"
using namespace std;

// Publisher and Subscribers
ros::Publisher husky_twist_control; 
ros::Subscriber gps_sub; 
ros::Subscriber path_sub;

int path_init = 0;
double t_start;
vector<double> path_lat(1); 
vector<double> path_long(1); 

void cross_track_error(int P, int P1, double lP, double lP1)
{
  double d = sqrt(pow(((path_lat.at(P) - path_lat.at(P1))*111320), 2) +
    pow(((path_long.at(P) - path_long.at(P1))*40075000*cos(path_lat.at(P) 
    *M_PI/180)/360), 2));
  double c = acos((pow(lP, 2) + pow(d, 2) - pow(lP1, 2))/(2*lP*d));
  if (isnan(c))
  {
    c = 0;
  }
  double l = lP*sin(c);
  // Output the minimum distance to a path point, as a means to measure error
  double t_now = ros::Time::now().toSec();
  double t_elapsed = t_now - t_start;
  fstream cross_track_error;
  cross_track_error.open("cross_track_error.txt", ios_base::app);
  ROS_INFO("l = %f", l);
  cross_track_error << t_elapsed << ", " << l << endl;
  cross_track_error.close();
}

int find_goal_point(double a, double b)
{
  
  //find distance of each point away from current position
  vector<double> dist(path_lat.size());
  for (int i = 0; i < path_lat.size(); i++)
  {
    dist.at(i) = sqrt(pow(((a - path_lat.at(i))*111320), 2) +
      pow(((b - path_long.at(i))*40075000*cos(a*M_PI/180)/360), 2));
  }
  //find index of point P that is closest to current position
  ///aka find minimum position in dist vector
  int P = std::min_element(dist.begin(),dist.end()) - dist.begin();
  // Find index of point P1 that is second-closest to the current position
  int P1;
  int second_smallest = INT_MAX;
  for (int i = 0; i < dist.size(); i++)
  {
    if (dist.at(i) < second_smallest && i != P)
    {
      second_smallest = dist.at(i);
      P1 = i;
    }
  }
  // Find distance between closest path point and second closest path point
  double lP = dist.at(P);
  double lP1 = dist.at(P1);
  cross_track_error(P, P1, lP, lP1);
  //find goal point
  int j = 0;
  int i = P;
  int G; //current goal point
  while (j == 0) //70
  {
    if(i == dist.size() || i == dist.size() - 1)
    {
      G = dist.size() - 1;
      j = 1;
    }
    int LD = 1; //lookahead distance in meters
    if(dist[i+1] > LD)
    {
      G = i + 1;
      j = 1;
    }
   i = i+1;
  }
  return(G);
}

double goal_coordinate_transform(double latitude, double longitude, double heading_rad, double G, double* xg, double* yg)
{
  double ygn = (path_lat.at(G) - latitude)*111320; // meters north of robot
  double xgn = (path_long.at(G) - longitude)*40075000*
    cos(latitude*M_PI/180)/360; //meters east of robot
  double theta = -(heading_rad - M_PI/2);
  *xg = xgn*sin(theta) + ygn*cos(theta); // meters in front of robot
  *yg = -xgn*cos(theta) + ygn*sin(theta); // meters to left of robot
}

void get_path(const nav_msgs::Path &msg2)
{
  if (path_init == 0)
  {
  int L = msg2.poses.size();
  for (int i = 0; i < L; i++)
  {
  geometry_msgs::PoseStamped pose1;
  pose1 = msg2.poses.at(i);
  if (i == 0)
  {
    path_lat.at(i) = pose1.pose.position.x;
    path_long.at(i) = pose1.pose.position.y;
  }
  else
  {
  path_lat.push_back(pose1.pose.position.x);
  path_long.push_back(pose1.pose.position.y);
  }
  }
  t_start = ros::Time::now().toSec();
  }
  path_init = 1;
}

void follow_path(const geometry_msgs::Pose2D &msg)
{
  if (path_init == 1)
  {
  double latitude = msg.x;
  double longitude = msg.y;
  double heading_rad = msg.theta;
  int G = find_goal_point(latitude, longitude);
  double xg, yg;
  goal_coordinate_transform(latitude, longitude, heading_rad, G, &xg, &yg);
  double l = sqrt(pow(xg,2) + pow(yg,2));

  double rho = abs(pow(l,2)/(2*-yg));
  double vx;
  if(xg >= 0)
  {
    vx = 0.75;
  }
  else
  {
    vx = -0.75;
  }
  double wz;
  if(-yg>=0)
  {
    wz = -vx/rho;
  }
  else
  {
    wz = vx/rho;
  }
  geometry_msgs::Twist command;
  if(G == path_lat.size() - 1 && l < 1)
  {
  command.linear.x = 0;
  command.angular.z = 0;
  }
  else
  {
  command.linear.x = vx;
  command.angular.z = wz;
  }
  husky_twist_control.publish(command);
  int size = path_lat.size() - 1;
  ROS_INFO("G = %d, size = %d", G, size);
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "mr_algorithm_3");
  ros::NodeHandle n;
  // Get path gps path information
  path_sub = n.subscribe("gps_path", 500, get_path);
  // Get estimates for current lat, long, and heading, and call function
  gps_sub = n.subscribe("/sim_compiled_gps", 500, follow_path);
  // Publish velocity commands to the robot
  husky_twist_control = n.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 1000);
  ros::spin();
}
