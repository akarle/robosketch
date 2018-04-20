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

//extern ros::Publisher vis_pub;
//extern ros::Publisher point_pub;

HumanCloud base;
ros::Publisher vis_pub;
ros::Publisher point_pub;
ros::Publisher human_pub;

void bagcb(const sensor_msgs::PointCloud2& cloud)
{
    sensor_msgs::PointCloud pc;
    sensor_msgs::convertPointCloud2ToPointCloud(cloud, pc);

    // TODO: get Alex G's filtering...
    if(base.nose_x == -1 || base.arm_baseline == -1){
        int num_cal = 0;
        HumanCloud temp;
        while(num_cal < 5){
          Calibrate(pc, temp);
          if(temp.nose_x != -1 && temp.arm_baseline != -1){
            base.nose_x += temp.nose_x;
            base.arm_baseline += temp.arm_baseline;
            ++num_cal;
          }
        }
        base.nose_x /= num_cal;
        base.arm_baseline /= num_cal;
    }

    else{
        // Filter Human
        HumanCloud hc;
        //ROS_INFO("Pre Filter");
        FilterHuman(pc, hc);
        human_pub.publish(hc.pc);
        hc.nose_x = base.nose_x;
        hc.arm_baseline = base.arm_baseline;

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

        // Go through HC and add all to marker arr
        /*
        for(unsigned int i = 0; i < hc.pc.points.size(); i++){
            // give marker a unique id and the proper coords / color
            marker.id = 11 + i;
            marker.pose.position.x = hc.pc.points[i].x;
            marker.pose.position.y = hc.pc.points[i].y;

            // push back into arr
            arr.markers.push_back(marker);
        }*/


        // Get Hands from hc
        //ROS_INFO("Pre Get Hands");
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
        marker.pose.position.z = 1.5;
        arr.markers.push_back(marker);
        marker.id = 100001;
        marker.pose.position.x = 1; // some random number...
        marker.pose.position.y = h.R_y;
        marker.pose.position.z = 1.5;
        arr.markers.push_back(marker);

        // Publish them!
        vis_pub.publish(arr);
        //ROS_INFO("Post Publish");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "viznode");
    ros::NodeHandle n;

    base.nose_x = -1;
    base.arm_baseline = -1;

    // Subscribe to the bag point clouds
    ros::Subscriber sub = n.subscribe("/camera/depth/points", 1000, bagcb);
    vis_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1000);
    point_pub = n.advertise<PointCloud>("arm_points", 1000);
    human_pub = n.advertise<PointCloud>("human_points", 1000);

    ros::Rate loop(30.0);
    while(ros::ok){
      ros::spinOnce();
      loop.sleep();
    }

    return 0;
}
