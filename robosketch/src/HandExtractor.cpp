#include "HandExtractor.h"

using sensor_msgs::PointCloud;
using geometry_msgs::Point32;

// Globals:
float BODY_TOL = .2; // 20 cm on either side of center ignored

// Helper function to filter PC for the body
std::vector<Point32> FilterBodyFromPC(const PointCloud& pc, float center)
{
    // Go through the PC and eliminate all that are outside
    // BODY_TOL of center
    std::vector<Point32> fpc;
    for(unsigned int i = 0; i < pc.points.size(); i++){
        if(fabs(pc.points[i].x - center) > BODY_TOL){
            fpc.push_back(pc.points[i]);
        }
    }
    return fpc;
}
    

Hands getHandsFromHumanCloud(const HumanCloud& hc){
    Hands h;
    return h;
}
