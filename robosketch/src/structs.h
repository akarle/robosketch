#include <sensor_msgs/PointCloud.h>

struct MovementPair {
    float v;
    float w;
};

struct HandCoordinate {
    bool isRight;
    float x;
    float y;
};

struct Hands {
    HandCoordinate L;
    HandCoordinate R;
};

// arm_baseline --> y coord from calibration
// nose_x --> midpoint (x) of body
// pc --> filtered point cloud
struct HumanCloud {
    float arm_baseline;
    float nose_x;
    sensor_msgs::PointCloud pc;
};
