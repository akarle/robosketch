#include "FilterHuman.h"


static vector<Point32> findInliers(Vector3f p0, const vector<Point32> points, double epsilon);


static void publishPoint(Point32 point){
  
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
  marker.color.r = 1.0;
  marker.color.g = 0.0;

  marker.id = 10;

  marker.pose.position.x = point.x;
  marker.pose.position.y = point.y;
  marker.pose.position.z = point.z;

  arr.markers.push_back(marker);
  
  vis_pub.publish(arr);
}
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
  for(size_t i = 0; i < pc.points.size(); i++){
    double depth = pc.points[i].z;
    if(depth < 1.8 && depth > 1.5){
      human.pc.points.push_back(pc.points[i]);
    }
  }
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
      filtered_points.push_back(pc.points[i]); //Should be human
    }
  }
  PointCloud h;
  h.header = pc.header;
  h.points = filtered_points;
  human_pub.publish(h);

  vector<Point32> filtered = RANSAC(filtered_points); //Should be arm
  Point32 avg;
  if(filtered.size() > 0){

    PointCloud arms;
    arms.header = pc.header;
    arms.points = filtered;

    avg = AvgPoint(filtered);
    publishPoint(avg);

    //publishPoint(avg);

    human.arm_baseline = avg.y;
    human.nose_x = avg.x;

    human.pc.header = pc.header;
    human.pc.points = filtered_points;

    point_pub.publish(arms);
  }
  else {
    human.arm_baseline = -1;
    human.nose_x = -1;
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

