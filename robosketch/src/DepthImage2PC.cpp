#include "DepthImage2PC.h"

Point32 AvgPoint(vector<Point32> points);

void filterPoints(const PointCloud2 &msg){
  
  PointCloud pc;
  
  convertPointCloud2ToPointCloud(msg, pc);

  size_t size = pc.points.size();
  vector<Point32> filtered_points;

	for(size_t i = 0; i < size; i++){
    double depth = pc.points[i].z;
		if(depth < 1.7 && depth > 1.5){
			filtered_points.push_back(pc.points[i]);
		}
	}
  PointCloud filtered = RANSAC(filtered_points);
  if(filtered.points.size() > 0)
    AvgPoint(filtered.points);

  filtered.header = pc.header;

  pointPublisher.publish(filtered);	
}

Point32 AvgPoint(vector<Point32> points){
  double sumx = 0;
  double sumy = 0;
  double sumz = 0;
  double size = points.size();

  for(size_t i = 0 ; i < size ; i++){
    sumx += points[i].x;
    sumy += points[i].y;
    sumz += points[i].z;
  }

  Point32 ret;
  ret.x = sumx/size;
  ret.y = sumy/size;
  ret.z = sumz/size;

  ROS_INFO("(%f %f %f)", ret.x, ret.y, ret.z);

  return ret;
}

PointCloud RANSAC(const vector<Point32> points){

  const double threshold = .35;
  double percentage_inliers = 0;
  
  vector<Point32> ilyers;

  int size = points.size();
  if(size > 0){
    int iter = 0;
    while(percentage_inliers < threshold && iter < 300){

      Point32 p0 = points[rand() % size];

      Vector3f P0(p0.x, p0.y, p0.z);

      ilyers = findInliers(P0, points, .0700);

      percentage_inliers = (double) ilyers.size() / (double) size;
      ++iter;
    }
    if(iter < 100) 
      ROS_INFO("Converged in %d iterations", iter);
    else{
      ROS_INFO("Could not fit line");
      ilyers.clear();
    }
  }else{
    ROS_INFO("no points");
  }

  PointCloud filtered;
  filtered.points = ilyers;

  return filtered;
  
}

vector<Point32> findInliers(Vector3f p0, const vector<Point32> points, double epsilon)
{

  Vector3f temp;
  vector<Point32> inliers;

  for(size_t i = 0; i < points.size(); i++){
    Point32 p = points[i];
    temp.x() = p.x;
    temp.y() = p.y;
    temp.z() = p.z;

    if(fabs(temp.y() - p0.y()) < epsilon) 
      inliers.push_back(p);
  }

  return inliers;
}

