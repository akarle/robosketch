/*
 * The job of this node is to spin and listen for bag files and publish
 * both the kinect pointclouds and also markers for the hands
 */

#include "ros/ros.h"
#include "HandExtractor.h"
#include "FilterHuman.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Twist.h>

using geometry_msgs::Twist;

HumanCloud base;

ros::Publisher vis_pub;
ros::Publisher point_pub;
ros::Publisher human_pub;
ros::Publisher left_pub;
ros::Publisher right_pub;
ros::Publisher movement_publisher_;

const float R = 0.18;
const float MAX_V = 0.5;
const float MAX_W = 1.5;
const float CMD_HZ = 0.05;

float CURR_V = 0;
float CURR_W = 0;

void publishPoints(float arm_baseline, float nose_x, float R_y, float L_y){
    // Prepare markers...
    visualization_msgs::MarkerArray arr;

    // instantiate base marker to keep editing and pushing
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/camera_depth_optical_frame";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.b = 0.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;


    // Publish markers for hands!
    marker.type = visualization_msgs::Marker::CUBE;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    marker.color.b = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;

    // Left hand
    marker.id = 100000;
    marker.pose.position.x = -1; // some random number...
    marker.pose.position.y = L_y;
    marker.pose.position.z = 1.5;
    arr.markers.push_back(marker);
    // Right hand
    marker.id = 100001;
    marker.pose.position.x = 1; // some random number...
    marker.pose.position.y = R_y;
    marker.pose.position.z = 1.5;
    arr.markers.push_back(marker);


    // Nose x
    marker.id = 100002;
    marker.pose.position.x = nose_x; // some random number...
    marker.pose.position.y = arm_baseline;
    marker.pose.position.z = 1.5;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    arr.markers.push_back(marker);

    // Publish them!
    vis_pub.publish(arr);

}

// LEFT HAND UP => DRIVE FORWARD
// LEFT HAND DOWN => DRIVE BACKWARD
// RIGHT HAND UP => TURN LEFT
// RIGHT HAND UP => TURN RIGHT
MovementPair convertXY(float L_y, float R_y, float arm_baseline, float ymax){
    MovementPair m;

    float scale_factor = fabs(ymax - arm_baseline);
    float v_scale = (L_y - arm_baseline) / scale_factor;
    float w_scale = (R_y - arm_baseline) / scale_factor;

    // cap scales at 1 in case user jumps or somehow gets scale > 1
    if(v_scale > 1) v_scale = 1.0;
    if(w_scale > 1) w_scale = 1.0;
    
    if(v_scale < .1) v_scale = 0;
    if(w_scale < .1) w_scale = 0;

    // TODO: "move in direction" if change is bigger than feasible!
    // (use CMD_HZ, MAX_DV, MAX_DW, CURR_V, CURR_W)
    m.v = MAX_V * v_scale;
    m.w = MAX_W * w_scale;

    return m;
}


void Hands2Movement(Hands h, float arm_baseline, float ymax){
    Twist twist;

    MovementPair m = convertXY(h.L_y, h.R_y, arm_baseline, ymax);

    CURR_V = m.v;
    CURR_W = m.w;

    twist.linear.x = m.v;
    twist.angular.z = m.w;

    ROS_INFO("Publishing command (v, w): (%f, %f)", m.v, m.w);
    movement_publisher_.publish(twist);
}


void CloudCallBack(const sensor_msgs::PointCloud2& cloud)
{
    // convert the received pc2 -> pc
    sensor_msgs::PointCloud pc;
    sensor_msgs::convertPointCloud2ToPointCloud(cloud, pc);

    // if base arm baseline unset (-1) calibrate!
    if(base.arm_baseline == -1){
        Calibrate(pc, base);
    }

    // else u calibrated --> get those commands!
    else{
        // Filter Human
        HumanCloud hc;
        FilterHuman(pc, hc);

        // publish human points for testing
        human_pub.publish(hc.pc);

        hc.arm_baseline = base.arm_baseline;
        hc.min_x = base.min_x;
        hc.max_x = base.max_x;
        float nose_x = (hc.max_x + hc.min_x)/2;

        // Get Hands from hc
        Hands h = getHandsFromHumanCloud(hc);

        // publish hands and baseline/center for testing
        publishPoints(base.arm_baseline, nose_x, h.R_y,h.L_y);

        // Get and Send Movement Commands
        // TODO: do a better job getting ymax!
        float ymax = base.arm_baseline + 0.5;
        Hands2Movement(h, base.arm_baseline, ymax);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "viznode");
    ros::NodeHandle n;

    base.arm_baseline = -1;

    // Subscribe to the bag point clouds
    ros::Subscriber sub = n.subscribe("/camera/depth/points", 1000, CloudCallBack);
    vis_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1000);
    point_pub = n.advertise<PointCloud>("arm_points", 1000);
    human_pub = n.advertise<PointCloud>("human_points", 1000);
    left_pub = n.advertise<PointCloud>("left_points", 1000);
    right_pub = n.advertise<PointCloud>("right_points", 1000);
    movement_publisher_ = n.advertise<geometry_msgs::Twist>("robosketch/commands", 10);

    ros::Rate loop(30.0);
    while(ros::ok()){
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}
