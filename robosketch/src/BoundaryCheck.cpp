#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Twist.h>

ros::Publisher movementPublisher;
bool isNearEdge = false;

void boundaryCheckCallback(const sensor_msgs::ImageConstPtr& msg) {
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("Couldn't Convert Image, error: %s",e.what());
		return;
	}
	cv::Mat colorImage = cv_ptr->image;
	cv::Mat grayscaleImage;
	cv::cvtColor(colorImage,grayscaleImage,CV_RGB2GRAY);
	cv::Mat threshImage;
	bool updatedIsNearEdge = false;
	threshold(grayscaleImage,threshImage,255,5,1);
	for(int i=0;i<threshImage.rows;i++) {
		for(int j=0;i<threshImage.cols;j++) {
			if((uint)threshImage.at<char>(i,j)==5) {
				//ROS_INFO("GrayImage: %u",(uint)threshImage.at<char>(i,j));
				isNearEdge=true;
				return;
			}
		}
	}
	//ROS_INFO("Boundary updated to: %d", updatedIsNearEdge);
	isNearEdge=updatedIsNearEdge;
}

void velocityCallback(const geometry_msgs::Twist msg) {
	if(!isNearEdge) {
		movementPublisher.publish(msg);
	} else {
		geometry_msgs::Twist updatedVelocity;
		updatedVelocity.angular.z=msg.angular.z;
		if(msg.linear.x>0) {
			updatedVelocity.linear.x=0;
		} else {
			updatedVelocity.linear.x=msg.linear.x;
		}
		movementPublisher.publish(updatedVelocity);
	}
}


int main(int argc, char **argv) {
	ros::init(argc,argv,"BoundaryCheck");

	ros::NodeHandle n;
	image_transport::ImageTransport it(n);
	image_transport::Subscriber sub = it.subscribe("/usb_cam/image_raw",1,boundaryCheckCallback);
	ros::Subscriber velSub = n.subscribe("/robosketch/commands",1000,velocityCallback);
	movementPublisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi",1000);
	ros::spin();
	return 0;
}

