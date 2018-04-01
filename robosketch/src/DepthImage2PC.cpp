#include "DepthImage2PC.h"

static const double a  = 3.008;
static const double b  = -0.002745;
static const double fx = 588.446;
static const double fy = -564.227;
static const double px = 320.0;
static const double py = 240.0;

Point getPointFromDisparity(double disparity, double x, double y){
  Point p;
  
  double Z = 1/(a + (disparity*b));
  p.x = (Z * (x - px))/fx;
  p.y = (Z * (y - py))/fy;
  p.z = Z;

  return p;
}

void imageToPointCloud(Image &image){
  sensor_msgs::PointCloud pointCloud;
  pointCloud.header = image.header;

  pointCloud.points.resize(image.width * image.height);

  const int image_width = image.width;
  const int image_height = image.height;
  
  int index = 0;
  
  for (int y = 0; y < image_height; ++y) {
    for (int x = 0; x < image_width; ++x) {
      uint16_t byte0 = image.data[2 * (x + y * image_width) + 0];
      uint16_t byte1 = image.data[2 * (x + y * image_width) + 1];
      if (!image.is_bigendian) {
        std::swap(byte0, byte1);
      }
      // Combine the two bytes to form a 16 bit value, and disregard the
      // most significant 4 bits to extract the lowest 12 bits.
      const uint16_t disparity = ((byte0 << 8) | byte1) & 0x7FF;

     Point p = getPointFromDisparity(disparity,
                                     static_cast<double>(x),
                                     static_cast<double>(y));

      pointCloud.points[index].x = p.x;
      pointCloud.points[index].y = p.y;
      pointCloud.points[index].z = p.z;

      index ++;
    }
  }

}

PointCloud RANSAC(const PointCloud pc){
/*
  // TODO 
  // implement RANSAC

  const double threshold = .80;
  double percentage_inliers = 0;
  
  Vector3f p0;
  Vector3f n;

  Point P0;
  Point N;

  vector<Point32> ilyers;


  while(percentage_inliers < threshold){

    Point p1 = pc.points[rand() % pc.points.size()];
    Point p2 = pc.points[rand() % pc.points.size()];
    Point p3 = pc.points[rand() % pc.points.size()];

    Matrix<double,3,2> minPlane = fitMinPlane(p1,p2,p3);

    n(0)  = minPlane(0,0); 
    n(1)  = minPlane(1,0); 
    n(2)  = minPlane(2,0); 

    p0(0) = minPlane(0,1);
    p0(1) = minPlane(1,1);
    p0(2) = minPlane(2,1);

    P0.x = p0.x();
    P0.y = p0.y();
    P0.z = p0.z();

    N.x = n.x();
    N.y = n.y();
    N.z = n.z();

    ilyers = finliers(P0, N, pc.points, .0089);

    percentage_inliers = (double) ilyers.size() / (double) pc.points.size();

  }

  Eigen::Matrix<double,3,2> pn = fitPlane(ilyers);

  n = pn.col(0);
  p0 = pn.col(1);
  
  P0.x = p0.x();
  P0.y = p0.y();
  P0.z = p0.z();

  N.x = n.x();
  N.y = n.y();
  N.z = n.z();

  PointCloud filtered;

  filtered.header = pc.header;
  filtered.points = ilyers;

  return filtered;
*/
  return pc;
}



