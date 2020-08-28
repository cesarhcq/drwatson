#!/usr/bin/env bash
cd ~/guntherBot_ws/
source devel/setup.bash
roslaunch robot_gazebo second.launch

# if you want to run the robot in another world
# roslaunch robot_gazebo first.launch
