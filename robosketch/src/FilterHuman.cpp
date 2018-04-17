#include "FilterHuman.h"

Point32 AvgPoint(vector<Point32> points);
static vector<Point32> RANSAC(const vector<Point32> pc);
static vector<Point32> findInliers(Vector3f p0, const vector<Point32> points, double epsilon);

/************************************************************
 * For use only in tester node.                             *
 ***********************************************************/

void setPoints(const PointCloud2 &pc){
  PointCloud temp;
  convertPointCloud2ToPointCloud(pc, temp);
  HumanCloud human;
  Calibrate(temp, human);
}


/************************************************************ 
 * Calibrates as to where the human is within the pointcloud*
 * given by the kinect. It is assumed that the human will be*
 * 1.5 - 1.8 meters away from the kinect with no other      *
 * obsticals within that range                              *
 * **********************************************************
 * @param PointCloud &pc    -> PointCloud given by the      *
 *                             kinect                       *
 * @param HumanCloud &human -> Return struct passed by      *
 *                             reference                    *
 ************************************************************
 */
 


void Calibrate(PointCloud &pc, HumanCloud &human){
  
  size_t size = pc.points.size();
  vector<Point32> filtered_points;

	for(size_t i = 0; i < size; i++){
    double depth = pc.points[i].z;
		if(depth < 1.8 && depth > 1.5){
			filtered_points.push_back(pc.points[i]);
		}
	}

  vector<Point32> filtered = RANSAC(filtered_points);
  Point32 avg;
  if(filtered.size() > 0)
    avg = AvgPoint(filtered);

  human.arm_baseline = avg.y;
  human.nose_x = avg.x;

  human.pc.header = pc.header;
  human.pc.points = filtered_points;

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

static vector<Point32> RANSAC(const vector<Point32> points){

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

  return ilyers;
  
}

static vector<Point32> findInliers(Vector3f p0, const vector<Point32> points, double epsilon)
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

