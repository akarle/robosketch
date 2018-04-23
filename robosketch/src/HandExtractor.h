#include <vector>
#include <iostream>
#include <iterator>
#include <algorithm>
#include <cmath>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <limits>
#include <cstdlib>

#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Point.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>

#include "structs.h"

extern ros::Publisher left_pub;
extern ros::Publisher right_pub;

Hands getHandsFromHumanCloud(const HumanCloud& hc);
