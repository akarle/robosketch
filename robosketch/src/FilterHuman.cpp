#include "FilterHuman.h"


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

void FilterHuman(PointCloud &pc, HumanCloud &human){

  human.pc.header = pc.header;
  float max_x = FLT_MIN; 
  float min_x = FLT_MAX;

  for(size_t i = 0; i < pc.points.size(); i++){
    double depth = pc.points[i].z;
    double x = pc.points[i].x;

    if(depth < 1.8 && depth > 1.5){
      human.pc.points.push_back(pc.points[i]);
      if(x < min_x) min_x = x;
      if(x > max_x) max_x = x;

    }
  }

  human.max_x = max_x;
  human.min_x = min_x;

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

 FilterHuman(pc, human);
  
  vector<Point32> filtered = RANSAC(human.pc.points); //Should be arm
  Point32 avg;
  if(filtered.size() > 0){

    PointCloud arms;
    arms.header = pc.header;
    arms.points = filtered;

    avg = AvgPoint(filtered);

    human.arm_baseline = avg.y;
  }
  else {
    human.arm_baseline = -1;
  }
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

vector<Point32> RANSAC(const vector<Point32> points){

  const double threshold = .42;
  double percentage_inliers = 0;

  vector<Point32> ilyers;

  int size = points.size();
  if(size > 0){
    int iter = 0;
    Point32 p0;
    while(percentage_inliers < threshold && iter < 300){

      ilyers.clear();
      p0 = points[rand() % size]; // get random point

      Vector3f P0(p0.x, p0.y, p0.z); 

      ilyers = findInliers(P0, points, .0600);

      percentage_inliers = (double) ilyers.size() / (double) size;
      ++iter;
    }
    if(iter < 300) {
      ROS_INFO("Converged in %d iterations", iter);
    }
    else{
      ROS_INFO("Could not fit line");
      ilyers.clear();
    }

    return ilyers;
  }else{
    ROS_INFO("no points");
  }

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

