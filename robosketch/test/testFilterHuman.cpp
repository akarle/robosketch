#include <gtest/gtest.h>

#include "../src/FilterHuman.h"

ros::Publisher vis_pub;
ros::Publisher point_pub;
ros::Publisher human_pub;

TEST(FilterHuman, AvgPoints){
  Point32 a;
  a.x = 1.0;
  a.y = 2.0;
  a.z = 3.0;

  Point32 b;
  b.x = 2.0;
  b.y = 3.0;
  b.z = 4.0;

  Point32 c;
  c.x = 3.0;
  c.y = 4.0;
  c.z = 5.0;

  vector<Point32> points;
  points.push_back(a);
  points.push_back(b);
  points.push_back(c);
  
  Point32 avg = AvgPoint(points);
  ASSERT_EQ(avg.x, 2.0);
  ASSERT_EQ(avg.y, 3.0);
  ASSERT_EQ(avg.z, 4.0);
}

TEST(FitlerHuman, RANSAC){
  Point32 a;
  a.x = 1.99;
  a.y = 2.99;
  a.z = 3.99;

  Point32 b;
  b.x = 2.0;
  b.y = 3.0;
  b.z = 4.0;

  Point32 c;
  c.x = 2.01;
  c.y = 3.01;
  c.z = 4.01;

  Point32 d;
  d.x = 50.0;
  d.y = 50.0;
  d.z = 50.0;

  vector<Point32> points;
  points.push_back(a);
  points.push_back(b);
  points.push_back(c);
  points.push_back(d);
  
  vector<Point32> newPoints = RANSAC(points);
  Point32 avg = AvgPoint(newPoints);
  EXPECT_EQ(2.0, avg.x);
  EXPECT_EQ(3.0, avg.y);
  EXPECT_EQ(4.0, avg.z);

}

int main(int argc, char * argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
