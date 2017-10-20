#!/bin/sh
# Create Catkin Environment for ROS

# create a folder for Catkin Workspace
mkdir -p catkin_ws/src
cd catkin_ws/src
catkin_init_workspace

# compile the environment
cd /home/ubuntu/catkin_ws
catkin_make

echo "" >> ~/.bashrc
echo "# Load Catkin Workspace" >> ~/.bashrc
echo "source /home/ubuntu/catkin_ws/devel/setup.bash" >> ~/.bashrc
