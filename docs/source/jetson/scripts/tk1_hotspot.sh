#!/bin/sh
# Allows the Jetson to work as access point

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
