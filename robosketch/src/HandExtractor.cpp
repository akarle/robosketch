#include "HandExtractor.h"

using sensor_msgs::PointCloud;
using geometry_msgs::Point32;

// Globals:

// Helper function to filter PC for the body
HandClouds FilterBodyFromPC(const HumanCloud& human, float center)
{
    PointCloud pc = human.pc;
    float BODY_TOL = fabs(human.min_x - human.max_x)/6.0;
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

    PointCloud left;
    PointCloud right;

    left.header = pc.header;
    right.header = pc.header;

    left.points = hc.l_points;
    right.points = hc.r_points;

    left_pub.publish(left);
    right_pub.publish(right);

    return hc;
}

// Helper from vector of Points -> Hand Coordinate (really just y)
float YFromCloud(const std::vector<Point32>& pc, float baseline)
{
    // Find max & min of pc
    // y is the greatest mag from baseline
    float y_max = pc[0].y;
    float y_min = pc[0].y;
    for(unsigned int i=1; i < pc.size(); i++){
        if(pc[i].y > y_max){ y_max = pc[i].y; }
        if(pc[i].y < y_min){ y_min = pc[i].y; }
    }
    // even tho might all be on one side... this still checks out
    float dist_above = fabs(baseline - y_max);
    float dist_below = fabs(baseline - y_min);
    if(dist_above > dist_below){ return y_max; }  
    else{ return y_min; } }



// Helper to go from HandClouds -> Hands
Hands HandsFromHandClouds(const HandClouds& hc, float baseline)
{
    Hands h;
    if(hc.l_points.size() == 0){
        h.L_y = baseline;
    }
    else{
        h.L_y = YFromCloud(hc.l_points, baseline);
    }
    if(hc.r_points.size() == 0){
        h.R_y = baseline;
    }
    else{
        h.R_y = YFromCloud(hc.r_points, baseline);
    }
    return h;
}
    

Hands getHandsFromHumanCloud(const HumanCloud& hc){
    float nose_x = (hc.min_x + hc.max_x) / 2;
    HandClouds clouds = FilterBodyFromPC(hc, nose_x);
    return HandsFromHandClouds(clouds, hc.arm_baseline);
}
