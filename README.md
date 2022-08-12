# A* Path Planner

### Astar demo in ROS Rviz
![Demo](https://user-images.githubusercontent.com/31432135/184294267-b0fe5840-1d76-44a7-80a5-d5ba0d35c083.gif)
<br>

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
4. Send goal pose by rviz tool `2D Nav Goal`
