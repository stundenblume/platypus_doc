#!/bin/sh
# Install Robot Operating System (ROS) in Jetson

# Add ros repository to sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -

# Install ROS and Point Cloud Library
sudo apt-get update
sudo apt-get install ros-indigo-desktop
sudo apt-get install ros-indigo-pcl-conversions

echo "" >> ~/.bashrc
echo "# Load ROS environment" >> ~/.bashrc
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc

source ~/.bashrc

