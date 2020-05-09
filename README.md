# Udacity-Home-Delivery-Robot
This project is part of the Udacity Robotics Engineer Software program.  The goal of the Home Delivery Robot is to pick up an object from one location and to drop it to another specified location in a Gazezo environment.

This project includes:
- A world created using Gazebo building editor
- The ROS Navigation stack which uses SLAM techniques to localize and navigate the robot inside a mapped environment
- The C++ codes add_markers and pick_objects that programmatically display objects and move the Turtlebot to designated locations
- The Home Delivery script which simulates the robot in both Gazebo and rviz

### Clone the repository
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace
$ cd ~/catkin_ws
$ catkin_make

cd ~/catkin_ws/src
$ git clone https://github.com/rtshilombo/Udacity-Home-Delivery-Robot.git
$ catkin_make
```
### Source and run the project
```
$ source devel/setup.bash
$ cd src/ShellScripts
$./Home_service.sh
