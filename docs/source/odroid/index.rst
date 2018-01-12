
======================
Setting UP ODROID XU4
======================



.. WARNING::

  @ To be done by Roger and Amory


Download Image 
-----------------------------

Oficial Ubuntu MATE 16.04 for Odroid XU4
 * ``https://wiki.odroid.com/odroid-xu4/os_images/linux/start``
 * ``https://odroid.in/ubuntu_16.04lts/``

Unzip the Image file and go on to the next part.

Load Image to the SDCard
-----------------------------

The procedure is the same compared to procedure for Raspberry Pi

 * :doc:`builds`
 * :ref:`our steps for webhook creation <webhook-creation>`
 * Burn the Image using Linux
 * Burn the Image using Windows


Load Image to the eMMC memory
-----------------------------

One of the nice features of Odroid XU4 is that it has the eMMC memory module.
http://www.hardkernel.com/main/products/prdt_info.php?g_code=G145628174287
According to them, the eMMC 5.0 storage is ~7x faster than the MicroSD Class-10 card in read tests.
By using it, one can realise that the boot is clearly faster than SD cards. 

There are two ways to load eMMC memory:
- using the `eMMC Module Reader <http://www.hardkernel.com/main/products/prdt_info.php?g_code=G135415955758>`_

.. image:: ./source/odroid/images/emmc-reader.png
    :align: center
  
Then the procedure is the same for SDCards, but it requires the Reader.
  
- Or, the the Reader is not available, one can follow these steps

  * Load the Image on a SDCard as described above
  * Set the memory Switch to select SDCard
  * Plug the eMMC module
  * Turn the system on and boot the system
  * Plug in the USB3 interface a external drive with the Image file
  * Find out the mounting point for the eMMC module, which is most probably ``/dev/mmcblk1``
  * Go to the directory with the Image file and 
  * Execute: ``sudo dcfldd of=/dev/mmcblk1 if=./MyImage.img``

Done, the eMMC module was loaded without using the Reader. 
Now, to test boot with the eMMC, follow these steps:

  * Shutdown the computer
  * Remove the SDCard and make sure the eMMC is connected
  * Set the memory Switch to select eMMC
  * Turn the system on and boot the system

You will see that the time for the boot is shorter and the computer is faster.
It will boot twice when you boot it for the first time. The reason is that it automatically expand the file system in the 1st boot.
However, it is transparant to the user.


Backing up Image from the eMMC memory
-----------------------------

A similar procedure as described in the previous section can be used for backing up the Image in the eMMC module.
Just plug and external hard drive (USB3) to receive the .img file. Plug a Image 
Dont 

  * Load the Image on a SDCard as described above
  * Set the memory Switch to select SDCard
  * Turn the system on and boot the system
  * Plug in the USB3 interface a external drive with the Image file
  * Find out the mounting point for the eMMC module, which is most probably ``/dev/mmcblk1``
  * Go to the directory where the Image file will be saved and
  * Execute: ``sudo dcfldd if=/dev/mmcblk1 of=./MyImage.img``

Done! Now it is recommended to shrink the Image file as described in :ref:`our steps for webhook creation <webhook-creation>`.


Setting Up ROS
-----------------------------

There is no special procedure to install ROS dor Odroid. 
Just follow the same procedure for :ref:`Raspberry Pi 3 <webhook-creation>`., using Ubuntu Mate 16.04.

Setting Up GAMS/Madara
-----------------------------

There is no special procedure to install GAMS/Madara dor Odroid. 
Just follow the same procedure for :ref:`Raspberry Pi 3 <webhook-creation>`., using Ubuntu Mate 16.04.


Setting Up Peripherals
-----------------------------

GPS
~~~~~~~~~~~

http://www.hardkernel.com/main/products/prdt_info.php?g_code=G142502154078


oCam-1MGN-U : Global Shutter
~~~~~~~~~~~

http://www.hardkernel.com/main/products/prdt_info.php?g_code=G147245683619

Wifi
~~~~~~~~~~~

The recommended wifi dongle is called Wifi module 3 because it has deattachable antenna

http://www.hardkernel.com/main/products/prdt_info.php?g_code=G137447734369

It works out of the box for Ubuntu Mate. No further installation or configuration is required.


USB IO Board
~~~~~~~~~~~

http://www.hardkernel.com/main/products/prdt_info.php?g_code=G135390529643

