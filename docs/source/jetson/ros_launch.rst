====================================
Creating a ROS launch to peripheral
====================================

Instead of running each launch command for each periferal, we create a ROS launch that loads all the packages in a single shot. The launch is created inside our ``launch`` folder of our package in the Catkin workspace. In order to create it, we first create a new package named ``jetson_launchers`` with the folder ``launch`` inside. Then we download the ``jetson.launch`` file to our folder and perform the ``catkin_make`` to compile our new package.

.. code-block:: bash

   $ cd ~/catkin_ws/src/
   $ catkin_create_pkg jetson_launchers roscpp rospy
   $ mkdir -p ~/catkin_ws/src/jetson_launchers/launch/
   $ wget --no-check-certificate --content-disposition https://raw.githubusercontent.com/lsa-pucrs/platypus_doc/master/docs/source/jetson/scripts/jetson_launchers/launch/jetson.launch
   $ mv jetson.launch ~/catkin_ws/src/jetson_launchers/launch/
   $ cd ~/catkin_ws/
   $ catkin_make
   $ catkin_make install

These commands create a package named ``jetson_launchers`` with support to ``rospy`` and ``roscpp``. In our package, our launch file loads GPS, IMU and ZED packages into memory and then you can access their ROS topics. If you do not want to download the ``jetson.launch`` file, go to ``~/catkin_ws/src/jetson_launchers/launch/`` folder and create a file named ``jetson.launch``. In this file type:

.. code-block:: xml

   <launch>
	  <!-- GPS -->
	  <node name="adafruit_gps" pkg="nmea_navsat_driver" type="nmea_serial_driver" args="_port:=/dev/ttyUSB0 fix:=/gps/fix" />

	  <!-- IMU -->
      <include file="$(find i2c_imu)/launch/i2c_imu_auto.launch" />

	  <!-- ZED -->
	  <include file="$(find zed_wrapper)/launch/zed.launch" />
   </launch>

You may edit the ``package.xml`` file, adding:

.. code-block:: xml
   
   <build_export_depend>roscpp</build_export_depend>
   <build_export_depend>rospy</build_export_depend>
   <exec_depend>roscpp</exec_depend>
   <exec_depend>rospy</exec_depend>


Before running the ROS launch, you should set the default language for output (in case it is not set yet). In order to do that, add to the ``.bashrc`` file the line:

.. code-block:: bash

   export LC_ALL="en_US.UTF-8"

Finally, in order to test, you should call the ROS launch by typing:

.. code-block:: bash

   $ roslaunch jetson_launchers jetson.launch

To record the content of the topics, run the command:

.. code-block:: bash

   $ rosbag record /gps/fix /zed/left/image_raw_color /zed/right/image_raw_color /zed/odom /zed/depth/depth_registered /gps/fix /imu/data  -o /media/ubuntu/Card/<filename>.bag
