#!/bin/sh
# Additional packages to Jetson TK1

# Add universe to the repository and update it all
sudo apt-add-repository universe
sudo apt-get update

# packages for compilation
sudo apt-get install build-essential make cmake cmake-curses-gui g++
# packages to access i2c (GPIO)
sudo apt-get install i2c-tools libi2c-dev
# packages to facilitate development
sudo apt-get install git terminator screen
