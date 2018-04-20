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

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include<visualization_msgs/Marker.h>
#include<visualization_msgs/MarkerArray.h>

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

extern ros::Publisher vis_pub;
extern ros::Publisher point_pub;
extern ros::Publisher human_pub;

void setPoints(const PointCloud2 &pc);
void Calibrate(PointCloud &pc, HumanCloud &human);
void FilterHuman(PointCloud &pc, HumanCloud &human);
Point32 AvgPoint(vector<Point32> points);
vector<Point32> RANSAC(const vector<Point32> pc);
#endif
