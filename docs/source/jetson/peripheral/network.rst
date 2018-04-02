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


Syncronizing clocks in Jetson TK1
----------------------------------

As Jetson does not have an internal battery, every time it is turned off, the clock returns to ``Wed, Dec 31 1969``. As we record bags using timestamp, it is important to keep the clock updated. Here, we detail how to configure the server machine (laptop) and the client machine (Jetson) in order to syncronize both clocks. It is expected that the server clock is correct/updated. We separate the explanation in two parts: the server machine and the client machine.

**Server machine (Laptop)**

The first step in the server machine is to install the Network Time Protocol (`NTP <https://help.ubuntu.com/lts/serverguide/NTP.html>`_). NTP is a protocol designed for accurately synchronizing local time clocks with networked time servers. The NTP network of time servers is set up as a hierarchical manner, such that any user can enter the system as a server at some level. In order to install the NTP package, run:

.. code-block:: bash

   $ sudo apt-get update
   $ sudo apt-get install ntp

After installing the NTP protocol, we have to configure the daemon in order to broadcast the correct time to the network. The configuration file for NTP is located at ``/etc/ntp.conf``. In servers block, you should add a few extra lines to the bottom of your servers list to provide your current local time as a default should you temporarily lose Internet connectivity:

.. code-block:: bash
    # Use Ubuntu's ntp server as a fallback.
    server ntp.ubuntu.com
    server 127.127.1.0
    fudge 127.127.1.0 stratum 10

Below in this file you should add the address of the network to which you want to broadcast the correct time. These lines are as follows:

.. code-block:: bash

    # Local users may interrogate the ntp server more closely.
    restrict 127.0.0.1
    restrict ::1
    restrict 192.168.2.1

where ``192.168.2.1`` is the IP address of the client machine. In order to allow computers from the network to request the time and broadcast the current time, add the following lines:

.. code-block:: bash

    restrict 192.168.2.0 mask 255.255.255.0 nomodify notrap

    # If you want to provide time to your local subnet, change the next line.
    # (Again, the address is an example only.)
    broadcast 192.168.2.255

An example of the ``ntp.conf`` file in the server can be seen in the `Github page <https://raw.githubusercontent.com/lsa-pucrs/platypus_doc/master/docs/source/jetson/scripts/ntp.server.conf>`_.
Having configurated the server properly, you have to start the daemon by running:

.. code-block:: bash

    $ sudo /etc/init.d/ntp restart

In order to monitor the system and see if the time server is syncronized, run:

.. code-block:: bash

    $ tail -f /var/log/syslog

You can also verify if the machine is broadcasting the time clock, by running:


This command will generate a output as below, where ``192.168.2.255  .BCST.`` indicates that the current machine is broadcasting time to the 192.168.2.0 network.

.. code-block:: bash

         remote           refid      st t when poll reach   delay   offset  jitter
    ==============================================================================
     b.ntp.br        200.160.7.186    2 u   33   64    3   25.261   -0.686   0.188
     c.ntp.br        200.160.7.186    2 u   35   64    3   89.748  -10.390   0.678
     gps.ntp.br      200.160.7.197    2 u   31   64    3   24.908    2.942   4.804
     santuario.pads. .GPS.            1 u   32   64    3   28.793    0.454   0.543
     chilipepper.can 17.253.34.125    2 u   32   64    3  231.398   -7.629   2.121
     192.168.2.255   .BCST.          16 u    -   64    0    0.000    0.000   0.000


**Client machine (Jetson)**

As occurred in the server machine, the first step is to install the Network Time Protocol (`NTP <https://help.ubuntu.com/lts/serverguide/NTP.html>`_). In order to install the NTP package, run:

.. code-block:: bash

    $ sudo apt-get update
    $ sudo apt-get install ntp

Next step we have to configure the daemon in order to receive the correct time from the server machine. Hence, edit the file ``/etc/ntp.conf``, adding the IP of the server machine and localhost as fudge as:
 
.. code-block:: bash

    # IP of the NTP server machine 
    server 192.168.2.185

    # Use Ubuntu's ntp server as a fallback.
    server ntp.ubuntu.com
    server 127.127.1.0
    fudge 127.127.1.0 stratum 10

Finally, to listen to time broadcasts on the local network you should de-comment the last two lines:

.. code-block:: bash

    disable auth
    broadcastclient

An example of the configuration used in ``ntp.conf`` in the client machine, access the `Github page <https://raw.githubusercontent.com/lsa-pucrs/platypus_doc/master/docs/source/jetson/scripts/ntp.client.conf>`_. Having configurated the client, you have to restart the NTP daemon and wait few seconds to update the clock. Restart the daemon by running:

.. code-block:: bash

    $ sudo /etc/init.d/ntp restart

In order check whether the system is syncronized or not, run:

.. code-block:: bash

    $ ntpq -c lpeer

This command will generate an output as bellow, where delay, offset and jitter different than zero indicates that it is receiving :

.. code-block:: bash

         remote           refid      st t when poll reach   delay   offset  jitter
    ==============================================================================
     *192.168.2.185  146.164.48.5     2 u   89 1024  337    1.603   -0.745   0.469
      192.168.2.1    .STEP.          16 u    - 1024    0    0.000    0.000   0.000

To check if the date is updated, run:

.. code-block:: bash

    $ date


In case the date is not automatically updated, you can force the update by stopping NTP server and manually checking time by the server IP. Finally, start the NTP client again, as follows:

.. code-block:: bash

    $ sudo service ntp stop
    $ sudo ntpd -s 192.168.2.185
    $ sudo service ntp start

Finally, check if the date is updated:

.. code-block:: bash

    $ date
