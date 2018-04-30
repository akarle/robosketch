PROJECT(xy2vw)
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
ROSBUILD_INIT()

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

ROSBUILD_GENMSG()
ROSBUILD_GENSRV()

SET(libs roslib roscpp)
ROSBUILD_ADD_LIBRARY(Depth2PC STATIC src/DepthImage2PC.cpp)
ROSBUILD_ADD_LIBRARY(HandExtractor STATIC src/HandExtractor.cpp)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})
#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

ROSBUILD_ADD_EXECUTABLE(xy2vw src/xy2vw.cpp)
TARGET_LINK_LIBRARIES(xy2vw ${libs})

ROSBUILD_ADD_EXECUTABLE(BoundaryCheck src/BoundaryCheck.cpp)
TARGET_LINK_LIBRARIES(BoundaryCheck ${libs})
TARGET_LINK_LIBRARIES(BoundaryCheck ${OpenCV_LIBRARIES})

ROSBUILD_ADD_EXECUTABLE(VelPublisher src/VelPublisher.cpp)
TARGET_LINK_LIBRARIES(VelPublisher ${libs})