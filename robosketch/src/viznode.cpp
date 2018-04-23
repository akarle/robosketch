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

HumanCloud base;

ros::Publisher vis_pub;
ros::Publisher point_pub;
ros::Publisher human_pub;
ros::Publisher left_pub;
ros::Publisher right_pub;

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

void CloudCallBack(const sensor_msgs::PointCloud2& cloud)
{
    sensor_msgs::PointCloud pc;
    sensor_msgs::convertPointCloud2ToPointCloud(cloud, pc);

    if(base.arm_baseline == -1){

        int num_cal = 0;
        int num_fail = 0;

        HumanCloud temp;
        while(num_cal < 5){
            Calibrate(pc, temp);
            if(temp.arm_baseline != -1){
                base.arm_baseline += temp.arm_baseline;
                ++num_cal;
            }else {
                ++num_fail;
                if(num_fail > 5) return;
            }
        }
        base.arm_baseline /= num_cal;
        base.max_x = temp.max_x;
        base.min_x = temp.min_x;

    }

    else{
        // Filter Human
        HumanCloud hc;
        FilterHuman(pc, hc);
        human_pub.publish(hc.pc);

        hc.arm_baseline = base.arm_baseline;
        hc.min_x = base.min_x;
        hc.max_x = base.max_x;

        float nose_x = (hc.max_x + hc.min_x)/2;

        // Get Hands from hc
        Hands h = getHandsFromHumanCloud(hc);
        
        publishPoints(base.arm_baseline, nose_x, h.R_y,h.L_y);
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


    ros::Rate loop(30.0);
    while(ros::ok()){
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}
