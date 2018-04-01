#include <algorithm>
#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>



#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Point.h>

using Eigen::Vector2f;
using geometry_msgs::Point;
using std::fabs;
using std::max;
using std::atan2;
using std::cout;
using std::vector;

int main(int argc, char **argv) {
    ros::init(argc, argv, "xy2vw");



    ROS_INFO("hello");
    return 0;
}