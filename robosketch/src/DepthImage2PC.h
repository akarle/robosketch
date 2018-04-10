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

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <sensor_msgs/point_cloud_conversion.h>

using std::vector;
using sensor_msgs::Image;
using sensor_msgs::ImageConstPtr;
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

extern Publisher pointPublisher;

void filterPoints(const PointCloud2 &pc);

PointCloud RANSAC(const vector<Point32> pc);

vector<Point32> findInliers(Vector3f p0, const vector<Point32> points, double epsilon);

#endif
