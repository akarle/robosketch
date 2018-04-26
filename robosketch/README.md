## Dependencies
Ros Kinetic, OpenNI, OpenCV, RVIZ (optional) 
## Running on the Turtlebot
1) Connect Turtlebot and master computer to same wifi network
2) On Turtlebot 
```sh
foo@bar:~$ export ROS_MASTER_URI=http://(Master Ip Address)
```
3) On master, with kinect pluged in
```sh
foo@bar:~$ roscore
foo@bar:~$ roslaunch openni_launch openni.launch &
```
4) Once person is standing between 1.5 and 1.8 meters away from the kinect and is ready to calibrate
```sh
foo@bar:~$ rosrun robosketch viznode 
```
5) Optionally run the visualization
```sh
foo@bar:~$ rosrun rviz rviz -d visual.rviz
```

