
=============================================
Image Handling
=============================================


  This section presents how to burn a Linux image into a SDcard and also to backup the SDCard. 
  It has foucus on RPi, but it should work for any Linux based embedded computer.


Download OS
------------

Oficial Ubuntu MATE 16.04 for Raspberry Pi 3
https://ubuntu-mate.org/raspberry-pi/

Ubuntu MATE 16.04 with ROS Kinetic for Raspberry Pi 3
http://www.german-robot.com/2016/05/26/raspberry-pi-sd-card-image/

Oficial Ubuntu MATE 16.04 for Odroid XU4
https://wiki.odroid.com/odroid-xu4/os_images/linux/start
https://odroid.in/ubuntu_16.04lts/

Unzip the Image file and go on to the next part.

Write the Image using Windows
------------

 Use `Win32DiskImager<https://sourceforge.net/projects/win32diskimager/>` for writing and reading Image files.


Write the Image using Linux
------------

Execute `lsblk` or `df -l` to find out the mouting palce for the SDCard. It should be `/dev/sdX`, most probably `/dev/sdb` if your computer has only one disk.


There are several programs to burn the SDcard. dd is most well known but there are newer options such as `ddrescue` or `dcfldd`.


   $ sudo ddrescue -D --force ubuntu-mate-16.04.2-desktop-armhf-raspberry-pi.img /dev/sdx



   $ sudo dcfldd of=/dev/sdb if=~/MyImage.img

Force a synchronise of any outstanding input or output, then the card will be safe to remove. 

   $ sudo sync

That's it.



Expand the Image Size to Match the SDCard Size
------------

For Raspberry Pi, execute 

   $ sudo raspi-config

and select 'Expand Filesystem'. 

or, in the command line

   $ sudo raspi-config --expand-rootfs
   $ sudo reboot

For other embedded computers (e.g. ODroid), execute: 


TO BE DONE


Backing Up an Image File
------------

Once your embedded computer is fully configured, it is a good ideia to:

- save a script with all the procedure to build the current image (packages installed, files configured, etc)
- backup the SDCard using minimal size, i.e. shrinking the Image file


Reading the Image file
~~~~~~~~~~~~~~~

Shutdown the embedded computer, take the SDCard to a Linux PC computer and proceed with the following steps. 

Open a Terminal instance and enter the following Linux command where the SDCard is mounted;

   $ df -h


Example:

   $ df -h
   Filesystem                  Size  Used Avail Use% Mounted on
   udev                        7,8G     0  7,8G   0% /dev
   tmpfs                       1,6G   50M  1,6G   4% /run
   /dev/sda1                    50G   20G   28G  42% /
   tmpfs                       7,8G  3,4M  7,8G   1% /dev/shm
   tmpfs                       5,0M  4,0K  5,0M   1% /run/lock
   tmpfs                       7,8G     0  7,8G   0% /sys/fs/cgroup
   /dev/sda4                   813G  132G  640G  18% /home
   tmpfs                       1,6G  124K  1,6G   1% /run/user/1000
   /dev/sdb2                    58G  6,7G   51G  12% /media/ale/PI_ROOT
   /dev/sdb1                    63M   21M   43M  34% /media/ale/PI_BOOT


The last two are important: ``/dev/sdb1`` and ``/dev/sdb2``.


Next we 'unmount', the Raspberry Pi SDcard:

   $ sudo umount /dev/sdb1 /dev/sdb2

Now we make a backup copy of the Raspberry Pi image.     

   $ sudo dcfldd if=/dev/sdb of=~/MyImage.img

If ``dcfldd`` is not installed, then install it and reexecute the last command.

   $ sudo apt-get update
   $ sudo apt-get install dcfldd

Next we use the sync command to force a synchronise of any outstanding input or output

   $ sudo sync

Let's take a look at the backed up image file. The file size  should match the SDCard size.

   $ ls -lsah ~/MyImage.img

That's all !


Shrinking the Image file
~~~~~~~~~~~~~~~

Let us assume the you used a 64GB SDcard to build your system. When you back it up, it will result in a 64GB image file, redardless the actual amount of space used in the SDcard. 
It will not be possible to directly use this image file in a, for example, 16GB SDCard. Before it, you need to shirink the image file. 

This process is not exactly simple. It involves several steps as described `here<http://www.aoakley.com/articles/2015-10-09-resizing-sd-images.php>`.
Fortunalty, there are some scripts that perform these steps automatically. I personaly suggest this `script <https://github.com/qrti/shrink>`. To run it you need:

- A Linux PC computer or a Linux VM for windows users
- Take the SDcard from the embedded computer and mount it on the Linux PC computer
- download the `PiShrink script <https://github.com/Drewsif/PiShrink>` and follow the instructions  

   $ wget https://raw.githubusercontent.com/Drewsif/PiShrink/master/pishrink.sh


   $ chmod +x ../pishrink.sh 
   ale@gaphl40:~/img$ sudo ../pishrink.sh image.img 
   [sudo] password for ale: 
   Creating new /etc/rc.local
   e2fsck 1.42.13 (17-May-2015)
   Pass 1: Checking inodes, blocks, and sizes
   Pass 2: Checking directory structure
   Pass 3: Checking directory connectivity
   Pass 4: Checking reference counts
   Pass 5: Checking group summary information
   PI_ROOT: 289654/3795104 files (0.2% non-contiguous), 2014409/15251456 blocks
   resize2fs 1.42.13 (17-May-2015)
   resize2fs 1.42.13 (17-May-2015)
   Resizing the filesystem on /dev/loop1 to 2226004 (4k) blocks.
   Begin pass 2 (max = 369264)
   Relocating blocks             XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
   Begin pass 3 (max = 466)
   Scanning inode table          XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
   Begin pass 4 (max = 22681)
   Updating inode references     XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
   The filesystem on /dev/loop1 is now 2226004 (4k) blocks long.
   
   Shrunk image.img from 59G to 8,6G

Generating checksum
~~~~~~~~~~~~~~~

Finally, it is recommended to generate a checksum file for the image file.
This is usefull to check whether the file was correcpted during some data transfer. 

   $ md5sum image.img > image.md5

It results in a text file like this one. Save this file with the image file.

   $ cat image.md5 
   75e87507e672de53241df4d724a0aac4  image.img
