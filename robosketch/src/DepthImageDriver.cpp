#include "DepthImage2PC.h"

Publisher pointPublisher;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;


  pointPublisher = n.advertise<PointCloud>("points", 10);

  Subscriber pointSub = n.subscribe("/camera/depth/points",10,filterPoints);


  ros::Rate loop_rate(30);

  while (ros::ok())
  {

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}

