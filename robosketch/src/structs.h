#ifndef STRUCTS
#define STRUCTS
#include <sensor_msgs/PointCloud.h>

struct MovementPair {
    float v;
    float w;
};

struct Hands {
    float L_y;
    float R_y;
};

// arm_baseline --> y coord from calibration
// nose_x --> midpoint (x) of body
// pc --> filtered point cloud
struct HumanCloud {
    float arm_baseline;
    float nose_x;
    sensor_msgs::PointCloud pc;
};

struct HandClouds {
    std::vector<geometry_msgs::Point32> r_points;
    std::vector<geometry_msgs::Point32> l_points;
};
#endif
