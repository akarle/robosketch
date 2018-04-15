#ifndef D2PC
#define D2PC 1
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

#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include "structs.h"

using std::vector;
using geometry_msgs::Point;
using geometry_msgs::Point32;
using sensor_msgs::PointCloud2;
using sensor_msgs::PointCloud;
using sensor_msgs::convertPointCloud2ToPointCloud;
using Eigen::Vector3f;
using Eigen::Matrix;
using ros::Publisher;
using ros::Subscriber;
using std::fabs;
using std::rand;

void setPoints(const PointCloud2 &pc);

void Calibrate(PointCloud &pc, HumanCloud &human);
#endif
