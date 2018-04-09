#include "HandExtractor.h"

using sensor_msgs::PointCloud;
using geometry_msgs::Point32;

// Globals:
float BODY_TOL = .2; // 20 cm on either side of center ignored

// Helper function to filter PC for the body
HandClouds FilterBodyFromPC(const PointCloud& pc, float center)
{
    // Go through the PC and eliminate all that are outside
    // BODY_TOL of center
    HandClouds hc;
    for(unsigned int i = 0; i < pc.points.size(); i++){
        if(fabs(pc.points[i].x - center) > BODY_TOL){
            // Additionally, determine which side of body its on
            if(pc.points[i].x < center){
                hc.l_points.push_back(pc.points[i]);
            }
            else{
                hc.r_points.push_back(pc.points[i]);
            }
        }
    }
    return hc;
}
    

Hands getHandsFromHumanCloud(const HumanCloud& hc){
    Hands h;
    return h;
}
