#include <algorithm>
#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>

#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>

#include "structs.h"

using Eigen::Vector2f;
using geometry_msgs::Point;
using std::fabs;
using std::max;
using std::atan2;
using std::cout;
using std::vector;
using geometry_msgs::Twist;

ros::Publisher movement_publisher_;

const float R = 0.18;
const float MAX_V = 0.5;
const float MAX_W = 1.5;
const float CMD_HW = 0.05;

float CURR_V = 0;
float CURR_W = 0;

float ymax;
float ymin;
float ymiddle;

// LEFT HAND UP => DRIVE FORWARD
// LEFT HAND DOWN => DRIVE BACKWARD
// RIGHT HAND UP => TURN LEFT
// RIGHT HAND UP => TURN RIGHT
MovementPair convertXY(HandCoordinate L, HandCoordinate R){
    MovementPair m;

    float v_scale = (L.y - ymiddle) / (ymax - ymiddle);
    float w_scale = (R.y - ymiddle) / (ymax - ymiddle);

    m.v = MAX_V * v_scale;
    m.w = MAX_W * w_scale;

    return m;
}


void Hands2Movement(Hands h){
    Twist twist;

    MovementPair m = convertXY(h.L, h.R);

    CURR_V = m.v;
    CURR_W = m.w;

    twist.linear.x = m.v;
    twist.angular.z = m.w;

    movement_publisher_.publish(twist);
} 

int main(int argc, char **argv) {
    ros::init(argc, argv, "xy2vw");
    ros::NodeHandle n;

    movement_publisher_ = n.advertise<geometry_msgs::Twist>("robosketch/commands", 10);
    

    ROS_INFO("hello");
    ros::spin();
    return 0;
}
