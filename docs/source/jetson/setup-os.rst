=============================================
Setting Up the OS
=============================================

NVIDIA Jetson TK1 exhibits a lot of promise with lots of raw performance for its form factor and intended use, with low power consumption to boot. But as is typical with most of these types of products, the "out of the box" experience needs some help. A missing part in TK1 is a support out of the box for WiFi or Bluetooth. For people coming from commodity PCs, tablets, phones and such this is a little confusing. Usually one just installs a driver and the device starts to work. In the case of the Jetson, the actual signals on the board need to be played with a little, as well as having the driver issue [1]_. In order to overcome such issues, a new Linux kernel named **Grinch** includes a lot of the features to which most desktop users are accustomed.


Installing Grinch kernel
--------------------------

The Grinch Kernel for L4T provides over 60 changes and additions to the kernel, including fixes, configuration, module and firmware support to the stock kernel. The kernel is written and supported by Jetson Forum user Santyago. For further information about the Grinch Kernel, please see the `NVidia Jetson Forum <https://devtalk.nvidia.com/forums/board/162/>`.

In order to install the Grinch kernel, download the ``installGrinch.sh`` file and run as:

.. code-block:: bash

    $ wget --no-check-certificate --content-disposition https://raw.githubusercontent.com/lsa-pucrs/platypus_doc/master/docs/source/jetson/scripts/installGrinch.sh
    $ chmod +x installGrinch.sh
    $ ./installGrinch.sh

Running these command will download the kernel image and install it in Jetson board. After the install script completes, reboot the Jetson TK1.


Enabling USB 3.0
-----------------

By default, the USB port in Jetson TK1 is not compatible to 3.0 devices. In order to enable the USB 3.0 support, you have to configure the ``extlinux.conf`` file, changing the parameter ``usb_port_owner_info=0`` to ``usb_port_owner_info=2``. In order to do so, run:

.. code-block:: bash

    $ sudo gedit /boot/extlinux/extlinux.conf

The ``extlinux.conf`` file looks like:

.. code-block:: bash

    TIMEOUT 30
    DEFAULT primary

    MENU TITLE Jetson-TK1 eMMC boot options

    LABEL primary
        MENU LABEL primary kernel
        LINUX /boot/zImage
        FDT /boot/tegra124-jetson_tk1-pm375-000-c00-00.dtb
        APPEND console=ttyS0,115200n8 console=tty1 no_console_suspend=1 lp0_vec=2064@0xf46ff000 mem=2015M@2048M memtype=255 ddr_die=2048M@2048M section=256M pmuboard=0x0177:0x0000:0x02:0x43:0x00 tsec=32M@3913M otf_key=c75e5bb91eb3bd947560357b64422f85 usbcore.old_scheme_first=1 core_edp_mv=1150 core_edp_ma=4000 tegraid=40.1.1.0.0 debug_uartport=lsport,3 power_supply=Adapter audio_codec=rt5640 modem_id=0 android.kerneltype=normal fbcon=map:1 commchip_id=0 usb_port_owner_info=2 lane_owner_info=6 emc_max_dvfs=0 touch_id=0@0 board_info=0x0177:0x0000:0x02:0x43:0x00 net.ifnames=0 root=/dev/mmcblk0p1 rw rootwait tegraboot=sdmmc gpt

Look for `usb_port_owner_info=0` in the file and replace it by `usb_port_owner_info=2`. Finally, save the file and exit.


Important Packages
-------------------

Here we add several packages that should be installed to work in Jetson. All packages are installed via ``apt-get``. In order to easily install all packages a script was created and can be downloaded by running:

.. code-block:: bash

    $ wget --no-check-certificate --content-disposition https://raw.githubusercontent.com/lsa-pucrs/platypus_doc/master/docs/source/jetson/scripts/additionalPackages.sh
    $ chmod +x additionalPackages.sh
    $ ./additionalPackages.sh

The script installs the following packages:

- `Informational list of build-essential packages (build-essential) <https://packages.ubuntu.com/trusty/build-essential>`_
- `Cross-platform, open-source make system (CMake) <https://packages.ubuntu.com/trusty/cmake>`_
- `Curses based user interface for CMake (cmake-curses-gui) <https://packages.ubuntu.com/trusty/cmake-curses-gui>`_
- `GNU C++ compiler (G++) <https://packages.ubuntu.com/trusty/g++>`_
- `Set of I2C tools for Linux <https://packages.ubuntu.com/trusty/i2c-tools>`_
- `Userspace I2C programming library development files <https://packages.ubuntu.com/trusty/libi2c-dev>`_
- `Distributed revision control system (Git) <https://packages.ubuntu.com/trusty/git>`_
- `Multiple GNOME terminals in one window <https://packages.ubuntu.com/trusty/terminator>`_
- `Terminal multiplexer with VT100/ANSI terminal emulation (Screen) <https://packages.ubuntu.com/trusty/screen>`_


Setting TK1 as Access Point
----------------------------

This configuration allows the Jetson board to work as access point to connect to other devices. The configuration below refers specifically to TP Link (`TL-WN722N V1 <http://www.tp-link.com/us/download/TL-WN722N.html>`_) - chipset Atheros Communications, Inc. AR9271 802.11n. Other access points may have a different configuration. By default, Grinch kernel has compatible drivers to the TP Link chipset and allows Wifi network connection. Thus, we have to set the device as hotspot. In order to do so, download and run the ``tk1_hotspot.sh`` file as:

.. code-block:: bash

    $ wget --no-check-certificate --content-disposition https://raw.githubusercontent.com/lsa-pucrs/platypus_doc/master/docs/source/jetson/scripts/tk1_hotspot.sh
    $ chmod +x tk1_hotspot.sh
    $ ./tk1_hotspot.sh

This script first download the `hostapd <https://packages.ubuntu.com/trusty/hostapd>`_, `udhcpd <https://packages.ubuntu.com/trusty/udhcpd>`_ and `dnsmasq <https://packages.ubuntu.com/search?keywords=dnsmasq&searchon=names>`_ packages. Next step, the script subscribes the `/etc/udhcpd.conf` file, adding the configuration to the new network, setting the range of IPs from ``192.168.2.100`` to ``192.168.2.200``. Thus, any device connected to the Jetson will have an IP between these values. The values changed in ``udhcpd.conf`` are:

.. code-block:: bash
    start       192.168.2.100
    end         192.168.2.200

    interface   wlan0

    remaining   yes

    opt dns     8.8.8.8 4.2.2.2
    option subnet  255.255.255.0
    opt router  192.168.2.1

Then, the script subscribes the `/etc/default/udhcpd`, by commenting the line: 

.. code-block:: bash

    # DHCPD_ENABLED="no"

The script also download and insert the file `/etc/hostapd/hostapd.conf` containing the configuration to connect the network. In order to connect in Jetson's network we configure the file as:

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


References
-----------

.. [1] `Installing Grinch L4T <http://www.jetsonhacks.com/2014/10/12/installing-grinch-linuxfortegra-l4t-nvidia-jetson-tk1/>`_



