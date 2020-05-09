# Udacity-Home-Delivery-Robot
This project is part of the Udacity Robotics Engineer Software program.  The goal of the Home Delivery Robot is to pick up objects from one location and to drop it to another specified location in a Gazezo environment.

This project includes:
- The creation of a world using Gazebo building editor
- The ROS Navigation stack using SLAM techniques to localize and navigate the robot in a mapped environment
- C++ codes add_markers and pick_objects that programmatically display objects and move the Turtlebot to designated locations
- The Home Delivery script which simulates the robot in both Gazebo and rviz
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace
$ cd ~/catkin_ws
$ catkin_make
```
