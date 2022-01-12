#ifndef MIXED_REALITY_LIBRARY_H_
#define MIXED_REALITY_LIBRARY_H_

// Libraries to include
#include <math.h>
#include <ros/ros.h>
#include <time.h>
#include <string>
#include <vector>
#include <algorithm>
#include <fstream>
#include "tf/transform_datatypes.h"

// Messages to include
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"

void sayHello();

#endif
