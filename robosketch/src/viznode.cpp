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

ros::Publisher vis_pub;

void bagcb(const sensor_msgs::PointCloud2& cloud)
{
    sensor_msgs::PointCloud pc;
    sensor_msgs::convertPointCloud2ToPointCloud(cloud, pc);
    // TODO: get Alex G's filtering...
    HumanCloud hc;
    Calibrate(pc, hc);

    // Prepare markers...
    visualization_msgs::MarkerArray arr;

    // instantiate base marker to keep editing and pushing
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
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

    // Go through HC and add all to marker arr
    for(unsigned int i = 0; i < hc.pc.points.size(); i++){
        // give marker a unique id and the proper coords / color
        marker.id = 10 + i;
        marker.pose.position.x = hc.pc.points[i].x;
        marker.pose.position.y = hc.pc.points[i].y;

        // push back into arr
        arr.markers.push_back(marker);
    }


    // Get Hands from hc
    Hands h = getHandsFromHumanCloud(hc);

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
    marker.pose.position.y = h.L_y;
    arr.markers.push_back(marker);
    marker.id = 100001;
    marker.pose.position.x = 1; // some random number...
    marker.pose.position.y = h.R_y;
    arr.markers.push_back(marker);

    // Publish them!
    vis_pub.publish(arr);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "viznode");
    ros::NodeHandle n;

    // Subscribe to the bag point clouds
    ros::Subscriber sub = n.subscribe("/camera/depth/points", 1000, bagcb);
    ros::spin();

    return 0;
}
