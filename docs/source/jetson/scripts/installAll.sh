#!/bin/bash

# This script runs the whole installation process, downloading packages and configuring the
# Jetson board. The process consists of:
# - Installing Grinch kernel in TK1
# - Configuring USB to USB 3.0
# - Installing important packages 
# - Setting TK1 as Access Point
# - Installing ROS
# - Creating Catkin workspace to run ROS
# - Installing ZED Camera
# - Installing ZED ROS Wrapper
# - Installing IMU Support
# - Installing GPS Support
#

###################################################################################################################
# Install GRINCH Kernel                                                                                           #
###################################################################################################################

# Download Grinch kernel
wget http://www.jarzebski.pl/files/jetsontk1/grinch-21.3.4/zImage
wget http://www.jarzebski.pl/files/jetsontk1/grinch-21.3.4/jetson-tk1-grinch-21.3.4-modules.tar.bz2
wget http://www.jarzebski.pl/files/jetsontk1/grinch-21.3.4/jetson-tk1-grinch-21.3.4-firmware.tar.bz2

# compare the md5sum checksum for the downloaded files to make sure that they are not corrupt
# check zImage
CSUM="a4a4ea10f2fe74fbb6b10eb2a3ad5409"
MD5=$(md5sum zImage | cut -d ' ' -f 1)

if [ "$MD5" != "$CSUM" ] 
then
  /bin/echo -e "\e[0;31mThe checksum does not match for the file 'zImage'.\e[0m" 
  echo "Correct checksum: "$CSUM
  echo "Checksum of downloaded file: "$MD5
  /bin/echo -e "\e[0;31mInstallation Aborted. Please try downloading file again and doing a manual installation.\e[0m"
  exit 1
fi

# check modules download
CSUM="3f84d425a13930af681cc463ad4cf3e6"
MD5=$(md5sum jetson-tk1-grinch-21.3.4-modules.tar.bz2 | cut -d ' ' -f 1)

if [ "$MD5" != "$CSUM" ] 
then
  /bin/echo -e "\e[0;31mThe checksum does not match for the file 'jetson-tk1-grinch-21.3.4-modules.tar.bz2'.\e[0m" 
  echo "Correct checksum: "$CSUM
  echo "Checksum of downloaded file: "$MD5
  /bin/echo -e "\e[0;31mInstallation Aborted. Please try downloading file again and doing a manual installation.\e[0m"
  exit 1
fi

# check firmware downloads
CSUM="f80d37ca6ae31d03e86707ce0943eb7f"
MD5=$(md5sum jetson-tk1-grinch-21.3.4-firmware.tar.bz2 | cut -d ' ' -f 1)

if [ "$MD5" != "$CSUM" ] 
then
  /bin/echo -e "\e[0;31mThe checksum does not match for the file 'jetson-tk1-grinch-21.3.4-modules.tar.bz2'.\e[0m" 
  echo "Correct checksum: "$CSUM
  echo "Checksum of downloaded file: "$MD5
  /bin/echo -e "\e[0;31mInstallation Aborted. Please try downloading file again and doing a manual installation.\e[0m"
  exit 1
fi

/bin/echo -e "\e[0;32mChecksum matches for downloaded files. Installation will now start.\e[0m"
sudo tar -C /lib/modules -vxjf jetson-tk1-grinch-21.3.4-modules.tar.bz2
sudo tar -C /lib -vxjf jetson-tk1-grinch-21.3.4-firmware.tar.bz2
sudo cp zImage /boot/zImage
/bin/echo -e "\e[0;32mGrinch Kernel Installed! Please Reboot.\e[0m"

###################################################################################################################
# Enable USB 3.0                                                                                                  #
###################################################################################################################

wget --no-check-certificate --content-disposition https://raw.githubusercontent.com/lsa-pucrs/platypus_doc/master/docs/source/jetson/scripts/extlinux.conf
sudo mv extlinux.conf /boot/extlinux/

###################################################################################################################
# Install Important Packages                                                                                      #
###################################################################################################################

# add universe to the repository and update it all
sudo apt-add-repository universe
sudo apt-get update

# packages for compilation
sudo apt-get install build-essential make cmake cmake-curses-gui g++
# packages to access i2c (GPIO)
sudo apt-get install i2c-tools libi2c-dev
# packages to facilitate development
sudo apt-get install git terminator screen

###################################################################################################################
# Setting TK1 as Access Point                                                                                     #
###################################################################################################################

sudo apt-get install hostapd udhcpd dnsmasq

# download the udhcpd.conf file containing the configuration of the network
wget --no-check-certificate --content-disposition https://raw.githubusercontent.com/lsa-pucrs/platypus_doc/master/docs/source/jetson/scripts/udhcpd.conf
# update the original udhcpd.conf file with the new configuration
sudo mv udhcpd.conf /etc/udhcpd.conf

# download the udhcpd file containing the configuration of DHCP
wget --no-check-certificate --content-disposition https://raw.githubusercontent.com/lsa-pucrs/platypus_doc/master/docs/source/jetson/scripts/udhcpd
# update the original udhcpd file with the new configuration
sudo mv udhcpd /etc/default/udhcpd

# download the hostapd.conf file containing the configuration to connect the network
wget --no-check-certificate --content-disposition https://raw.githubusercontent.com/lsa-pucrs/platypus_doc/master/docs/source/jetson/scripts/hostapd.conf
# creates the hostapd.conf file
sudo mv hostapd.conf /etc/hostapd/hostapd.conf

# download the interfaces file containing the configuration to set up the network
wget --no-check-certificate --content-disposition https://raw.githubusercontent.com/lsa-pucrs/platypus_doc/master/docs/source/jetson/scripts/interfaces
# update the content of the interfaces file
sudo mv interfaces /etc/network/interfaces

# download the dnsmasq.conf file containing the configuration of the DNS
wget --no-check-certificate --content-disposition https://raw.githubusercontent.com/lsa-pucrs/platypus_doc/master/docs/source/jetson/scripts/dnsmasq.conf
# update the content of the dnsmasq.conf file
sudo mv dnsmasq.conf /etc/dnsmasq.conf

# download the sysctl.conf file containing the configuration for setting variables
wget --no-check-certificate --content-disposition https://raw.githubusercontent.com/lsa-pucrs/platypus_doc/master/docs/source/jetson/scripts/sysctl.conf
# update the content of the dnsmasq.conf file
sudo mv sysctl.conf /etc/sysctl.conf

# download the accesspoint.sh file containing commands to start up the network
wget --no-check-certificate --content-disposition https://raw.githubusercontent.com/lsa-pucrs/platypus_doc/master/docs/source/jetson/scripts/accesspoint.sh
# update the content of the dnsmasq.conf file
chmod +x accesspoint.sh
mv accesspoint.sh /home/ubuntu/.accesspoint.sh

# download the rc.local file containing the call to .accesspoint.sh
wget --no-check-certificate --content-disposition https://raw.githubusercontent.com/lsa-pucrs/platypus_doc/master/docs/source/jetson/scripts/rc.local
# update the content of the dnsmasq.conf file
sudo mv rc.local /etc/rc.local

###################################################################################################################
# Installing ROS                                                                                                  #
###################################################################################################################

# add ros repository to sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -

# install ROS and Point Cloud Library
sudo apt-get update
sudo apt-get install ros-indigo-desktop
sudo apt-get install ros-indigo-pcl-conversions

echo "" >> ~/.bashrc
echo "# Load ROS environment" >> ~/.bashrc
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc

source ~/.bashrc

###################################################################################################################
# Creating Catkin workspace to run ROS                                                                            #
###################################################################################################################

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

###################################################################################################################
# Installing ZED Camera                                                                                           #
###################################################################################################################

cd /home/ubuntu
# download ZED SDK
wget https://www.stereolabs.com/developers/downloads/archives/ZED_SDK_Linux_JTK1_v1.2.0.run
chmod +x ZED_SDK_Linux_JTK1_v1.2.0.run
# install SDK
/bin/bash ZED_SDK_Linux_JTK1_v1.2.0.run

###################################################################################################################
# Installing ZED ROS Wrapper                                                                                      #
###################################################################################################################

cd ~/catkin_ws/src
# download ZED ROS Wrapper
git clone https://github.com/stereolabs/zed-ros-wrapper.git
cd ~/catkin_ws/src/zed-ros-wrapper
# checkout to the wrapper compatible with the SDK version 1.2
git checkout f2a62b0
cd ~/catkin_ws/
catkin_make
source ./devel/setup.bash

###################################################################################################################
# Installing IMU Support                                                                                          #
###################################################################################################################


###################################################################################################################
# Installing GPS Support                                                                                          #
###################################################################################################################


