PROJECT(xy2vw)
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
ROSBUILD_INIT()

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

ROSBUILD_GENMSG()
ROSBUILD_GENSRV()

SET(libs roslib roscpp)

ROSBUILD_ADD_LIBRARY(FilterHuman STATIC src/FilterHuman.cpp)
ROSBUILD_ADD_LIBRARY(HandExtractor STATIC src/HandExtractor.cpp)
ROSBUILD_ADD_EXECUTABLE(Depth2PCDriver src/DepthImageDriver.cpp)

TARGET_LINK_LIBRARIES(Depth2PCDriver FilterHuman)
#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

ROSBUILD_ADD_EXECUTABLE(xy2vw src/xy2vw.cpp)
ROSBUILD_ADD_EXECUTABLE(viznode src/viznode.cpp)
TARGET_LINK_LIBRARIES(xy2vw ${libs})
TARGET_LINK_LIBRARIES(viznode ${libs} FilterHuman HandExtractor)
