#!/usr/bin/env bash
cd

mkdir -p ~/guntherBot_ws/src && cd ~/guntherBot_ws

catkin init

cd ~/guntherBot_ws/src/ 

git clone git@github.com:ActaVisio/GuntherBot.git

git clone git@github.com:ros-drivers/joystick_drivers.git

cd ~/guntherBot_ws/

catkin_make
