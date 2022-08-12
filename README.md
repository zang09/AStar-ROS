# A* Path Planner

### C++ implementation of A* Path planning for ROS
![Demo](https://user-images.githubusercontent.com/31432135/184293660-5bb091d2-831e-45a5-bc91-10eb029970c0.gif)

### Prerequisition
Install package
```
$ sudo apt install ros-${ROS_DISTRO}-map-server
```
### How to use
1. Clone Package
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ git clone https://github.com/zang09/A-Star.git
```
2. Build Package
```
$ cd ..
$ catkin_make
$ source ~/catkin_ws/devel/setup.bash
```
3. Launch packages 
```
$ roslaunch path_generator run.launch
```
2. Send goal pose by rviz tool `2D Nav Goal`
