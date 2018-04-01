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

using std::vector;
using sensor_msgs::Image;
using geometry_msgs::Point;
using sensor_msgs::PointCloud;
using Eigen::Vector3f;
using Eigen::Matrix;

Point getPointFromDisparity(double disparity, double x, double y);
void imageToPointCloud(Image &image);
PointCloud RANSAC(const PointCloud);
