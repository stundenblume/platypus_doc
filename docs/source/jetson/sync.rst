=================================
Syncronizing clocks in Jetson TK1
=================================

As Jetson does not have an internal battery, every time it is turned off, the clock returns to `Wed, Dec 31 1969`. As we record bags using timestamp, it is important to keep the clock updated. Here, we detail how to configure the server machine (laptop) and the client machine (Jetson) in order to syncronize both clocks. It is expected that the server clock is correct/updated. We separate the explanation in two parts: the server machine and the client machine.

Server machine (Laptop)
------------------------

The first step in the server machine is to install the Network Time Protocol (`NTP <https://help.ubuntu.com/lts/serverguide/NTP.html>`_). NTP is a protocol designed for accurately synchronizing local time clocks with networked time servers. The NTP network of time servers is set up as a hierarchical manner, such that any user can enter the system as a server at some level. In order to install the NTP package, run:

.. code-block:: bash

   $ sudo apt-get update
   $ sudo apt-get install ntp

After installing the NTP protocol, we have to configure the daemon in order to broadcast the correct time to the network. The configuration file for NTP is located at `/etc/ntp.conf`. In servers block, you should add a few extra lines to the bottom of your servers list to provide your current local time as a default should you temporarily lose Internet connectivity:

.. code-block bash
    # Use Ubuntu's ntp server as a fallback.
    server ntp.ubuntu.com
    server 127.127.1.0
    fudge 127.127.1.0 stratum 10

Below in this file you should add the address of the network to which you want to broadcast the correct time. These lines are as follows:

.. code-block bash

    # Local users may interrogate the ntp server more closely.
    restrict 127.0.0.1
    restrict ::1
    restrict 192.168.2.1

where 192.168.2.1 is the IP address of the client. In order to allow computers from the network to request the time and broadcast the current time, add the following lines:

.. code-block bash

    restrict 192.168.2.0 mask 255.255.255.0 nomodify notrap

    # If you want to provide time to your local subnet, change the next line.
    # (Again, the address is an example only.)
    broadcast 192.168.2.255

An example of the `ntp.conf` file in the server can be seen in the `Github page <https://raw.githubusercontent.com/lsa-pucrs/platypus_doc/master/docs/source/jetson/scripts/ntp.server.conf>`_.
Having configurated the server properly, you have to start the daemon by running:

.. code-block bash

    $ sudo /etc/init.d/ntp restart

In order to monitor the system and see if the time server is syncronized, run:

.. code-block bash

    $ tail -f /var/log/syslog

You can also verify if the machine is broadcasting the time clock, by running:


This command will generate a output as below, where `192.168.2.255  .BCST.` indicates that the current machine is broadcasting time to the 192.168.2.0 network.

.. code-block bash

         remote           refid      st t when poll reach   delay   offset  jitter
    ==============================================================================
     b.ntp.br        200.160.7.186    2 u   33   64    3   25.261   -0.686   0.188
     c.ntp.br        200.160.7.186    2 u   35   64    3   89.748  -10.390   0.678
     gps.ntp.br      200.160.7.197    2 u   31   64    3   24.908    2.942   4.804
     santuario.pads. .GPS.            1 u   32   64    3   28.793    0.454   0.543
     chilipepper.can 17.253.34.125    2 u   32   64    3  231.398   -7.629   2.121
     192.168.2.255   .BCST.          16 u    -   64    0    0.000    0.000   0.000




Client machine (Jetson)
------------------------

As occurred in the server machine, the first step is to install the Network Time Protocol (`NTP <https://help.ubuntu.com/lts/serverguide/NTP.html>`_). In order to install the NTP package, run:

.. code-block:: bash

    $ sudo apt-get update
    $ sudo apt-get install ntp

Next step we have to configure the daemon in order to receive the correct time from the server machine. Hence, edit the file `/etc/ntp.conf`, adding the IP of the server machine and localhost as fudge as:
 
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

An example of the configuration used in `ntp.conf` in the client machine, access the `Github page <https://raw.githubusercontent.com/lsa-pucrs/platypus_doc/master/docs/source/jetson/scripts/ntp.client.conf>`_. Having configurated the client, you have to restart the NTP daemon and wait few seconds to update the clock. Restart the daemon by running:

.. code-block bash

    $ sudo /etc/init.d/ntp restart

In order check whether the system is syncronized or not, run:

.. code-block bash

    $ ntpq -c lpeer

This command will generate an output as bellow, where delay, offset and jitter different than zero indicates that it is receiving :

.. code-block bash

         remote           refid      st t when poll reach   delay   offset  jitter
    ==============================================================================
     *192.168.2.185  146.164.48.5     2 u   89 1024  337    1.603   -0.745   0.469
      192.168.2.1    .STEP.          16 u    - 1024    0    0.000    0.000   0.000

To check if the date is updated, run:

.. code-block bash

    $ date





