============================
Setting TK1 as Access Point
============================

This configuration allows the Jetson board to work as access point to connect to other devices. We configure the Jetson to work as access point specifically using TP Link (`TL-WN722N V1 <http://www.tp-link.com/us/download/TL-WN722N.html>`_) - chipset Atheros Communications, Inc. AR9271 802.11n. Other access points may have a different configuration. 

.. image:: ../images/tplink.jpg
    :align: center
    :width: 500pt


According with `TL-WN722N Datasheet <http://static.tp-link.com/TL-WN722N(UN)(US)_V2_Datasheet.pdf>`_, this wireless USB adapter connects your notebook or desktop computer to a wireless network. It has the following features:

+-------------------------------------------------------------------------------------------+
| - Wireless Standards: IEEE 802.11n, IEEE 802.11g, IEEE 802.11b                            |
| - Supports WPA/WPA2 data security, IEEE802.1x authentication, TKIP/AES and WEP encryption |
| - Frequency: 2.4~2.4835GHz                                                                |
| - Signal Rate: 150Mbps at 2.4GHz                                                          |
| - Provides USB 2.0 interface                                                              |
| - Wireless Modes: Ad-Hoc and infrastructure mode                                          |
| - Wireless Security: WEP, WPA-PSK/WPA2-PSK Encryptions                                    |
+-------------------------------------------------------------------------------------------+


Configuring TL-WN722N as Access Point
--------------------------------------

By default, Grinch kernel has compatible drivers to the TP Link chipset and allows Wifi network connection. Thus, we have to set the device as hotspot. In order to do so, download and run the ``tk1_hotspot.sh`` file as:

.. code-block:: bash

    $ wget --no-check-certificate --content-disposition https://raw.githubusercontent.com/lsa-pucrs/platypus_doc/master/docs/source/jetson/scripts/tk1_hotspot.sh
    $ chmod +x tk1_hotspot.sh
    $ ./tk1_hotspot.sh

This script first download the `hostapd <https://packages.ubuntu.com/trusty/hostapd>`_, `udhcpd <https://packages.ubuntu.com/trusty/udhcpd>`_ and `dnsmasq <https://packages.ubuntu.com/search?keywords=dnsmasq&searchon=names>`_ packages. Next step, the script subscribes the ``/etc/udhcpd.conf`` file, adding the configuration to the new network, setting the range of IPs from ``192.168.2.100`` to ``192.168.2.200``. Thus, any device connected to the Jetson will have an IP between these values. The values changed in ``udhcpd.conf`` are:

.. code-block:: bash
    start       192.168.2.100
    end         192.168.2.200

    interface   wlan0

    remaining   yes

    opt dns     8.8.8.8 4.2.2.2
    option subnet  255.255.255.0
    opt router  192.168.2.1

Then, the script subscribes the ``/etc/default/udhcpd``, by commenting the line: 

.. code-block:: bash

    # DHCPD_ENABLED="no"

The script also download and insert the file ``/etc/hostapd/hostapd.conf`` containing the configuration to connect the network. In order to connect in Jetson's network we configure the file as:

.. code-block:: bash
    
    interface=wlan0
    ssid=Tegra-WLAN
    hw_mode=g
    channel=3
    wpa=2
    wpa_passphrase=1234567890
    wpa_key_mgmt=WPA-PSK

Where the ``interface`` refers to the access point, ``ssid`` is the name of the network and ``wpa_passphrase`` is the password to connect in the Jetson. Next, the script downloads and updates the ``/etc/network/interfaces`` file. This file sets up the wireless interface by adding the following configuration:

.. code-block:: bash

    auto lo
    iface lo inet loopback

    auto wlan0
    iface wlan0 inet static
    hostapd /etc/hostapd/hostapd.conf
    address 192.168.2.1
    netmask 255.255.255.0

Next, download and update the ``/etc/dnsmasq.conf`` file, adding the following configuration:

.. code-block:: bash

    interface=lo,wlan0

    no-dhcp-interface=lo

    dhcp-range=192.168.2.100,192.168.2.200,255.255.255.0,12h

Next, download and update the ``sysctl.conf`` file, which only uncomment the line ``net.ipv4.ip_forward=1``. A file containing the configuration of the access point to start when the Jetson is turned on is downloaded and moved to ``/home/ubuntu/.accesspoint.sh`` and the command to call this file is added to the ``/etc/rc.local`` file, thus, allowing the access point to run when the Jetson start up. If everything is OK, when rebooting the Jetson, the access point network should be available to connect.
