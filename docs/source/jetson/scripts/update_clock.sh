#!/bin/bash

# This script updates the clock of the Jetson by checking the server
# located in ``IP`` machine. The server must be broadcasting NTP in 
# the connected network. In order to configure NTP server and client, 
# please check
# http://platypus-boats.readthedocs.io/en/latest/source/jetson/peripheral/network.html#syncronizing-clocks-in-jetson-tk1
#

if [[ $# -ne 1 ]]; then
    echo "ERROR: update clock by passing the IP of the server"
    echo "./update_clock.sh 192.168.2.100"
elif [[ $1 =~ ^[0-9]+\.[0-9]+\.+[0-9]+\.[0-9]+$ ]]; then
    sudo service ntp stop
    sudo ntpdate -t 3 -s $1
    sudo service ntp start
else
    echo "ERROR: $1 is not a valid IP address"
fi
