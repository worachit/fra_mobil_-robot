# Simple differential drive robot
this is FRA502 "Mobile Robot" project at KMUTT, FIBO
## Prerequisites
- Ubuntu 20.04 LTS
- ROS noetic
- Gazebo 11
- rviz 1.14
### prerequisite ROS Packages
- movebase
- gmapping
- teleop_twist_keyboard

to install all of ROS Package dependencies run
```sh
rosdep install --from-paths src --ignore-src -r -y
```
![alt text](/doc/robot.png?raw=true)
![alt text](/doc/gazebo_env.png?raw=true)

## Getting Start
To run basic mobile robot navigation 
```sh
roslaunch rb_gazebo gazebo_navigation.launch 
```
Basically, this will launch gazebo, rviz and movebase node. Then target goal can be set using "2D Nav Goal" in rviz.

To lauch waypoint follower node using command 
```sh
rosrun rb_navigation waypoint_follower.py 
```
Using "2D pose estimate" to add new waypoint to queue

### create new map
To create new map launch 
```sh
roslaunch rb_gazebo gazebo_create_map.launch 
```
and run teleop node using
```sh
rosrun rb_teleop teleop_twist_keyboard.py
```
the map can be saved via mapserver
```sh
rosrun map_server map_saver -f mymap
```
## Graph
![alt text](/doc/rosgraph.png?raw=true)
