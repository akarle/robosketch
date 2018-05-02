#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv) {
	ros::init(argc,argv,"VelPublisher");

	ros::NodeHandle n;

	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/robosketch/commands",1000);
	geometry_msgs::Twist test;
	test.linear.x=5;
	test.angular.z=2;
	ros::Rate rate(30);
	while(true) {
		pub.publish(test);
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}
