## Dependencies
ROS Kinetic, OpenNI, OpenCV, RVIZ (optional) 

Note that this is intented to run on Ubuntu 16.04. Running on a different OS may have unexpected results.

To install ROS see http://wiki.ros.org/kinetic/Installation/Ubuntu (OpenCV should be installed with ROS and installing the bare bones version will not have RVIZ included)

To install OpenNI run the following command
```sh
sudo apt-get install ros-kinetic-openni-launch
```
## Running on the Turtlebot
1) Connect Turtlebot and master computer to same wifi network
2) On Turtlebot 
```sh
export ROS_MASTER_URI=http://(Master Ip Address)
```
3) On master, with kinect pluged in
```sh
roscd robosketch
roscore &
make
roslaunch openni_launch openni.launch &
```
4) Once person is standing between 1.5 and 1.8 meters away from the kinect and is ready to calibrate. Note that if it cannot calibrate the program will exit and you will have to rerun it until it calibrates.

```sh
rosrun robosketch viznode 
```
5) Optionally run the visualization
```sh
rosrun rviz rviz -d visual.rviz
```

