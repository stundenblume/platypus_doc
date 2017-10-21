#!/bin/sh
# Install ZED Camera in Jetson

# download and run the ZED SDK file
wget https://www.stereolabs.com/developers/downloads/archives/ZED_SDK_Linux_JTK1_v1.2.0.run
chmod +x ZED_SDK_Linux_JTK1_v1.2.0.run
./ZED_SDK_Linux_JTK1_v1.2.0.run

# download the ZED Wrapper for ROS
cd ~/catkin_ws/src
git clone https://github.com/stereolabs/zed-ros-wrapper.git
cd zed-ros-wrapper
git checkout f2a62b0
cd ~/catkin_ws/
catkin_make
source ./devel/setup.bash
