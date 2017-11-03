====================================
Creating a ROS launch to peripheral
====================================

Instead of running each launch for each periferal, we create a ROS launch that loads all the packages in a single shot. The launch is created inside our `launch` folder of the Catkin workspace. In order to do so, we first create a new package named `jetson_launchers` with the folder `launch` inside. Then we download `package.xml`, `CMakeLists.txt` and `jetson.launch` to our folders and perform the `catkin_make` to compile our new package.

.. code-block:: bash

   $ mkdir -p ~/catkin_ws/src/jetson_launchers/launch/
   $ wget --no-check-certificate --content-disposition https://raw.githubusercontent.com/lsa-pucrs/platypus_doc/master/docs/source/jetson/scripts/jetson_launchers/package.xml
   $ wget --no-check-certificate --content-disposition https://raw.githubusercontent.com/lsa-pucrs/platypus_doc/master/docs/source/jetson/scripts/jetson_launchers/CMakeLists.txt
   $ wget --no-check-certificate --content-disposition https://raw.githubusercontent.com/lsa-pucrs/platypus_doc/master/docs/source/jetson/scripts/jetson_launchers/launch/jetson.launch
   $ mv package.xml CMakeLists.txt ~/catkin_ws/src/jetson_launchers/
   $ mv jetson.launch ~/catkin_ws/src/jetson_launchers/launch/
   $ cd ~/catkin_ws/
   $ catkin_make
   $ catkin_make install

If you want to create the package from scratch, go to the source folder of Catkin workspace and run the `catkin_create_pkg` command as:

.. code-block:: bash

   $ cd ~/catkin_ws/src/
   $ catkin_create_pkg jetson_launchers
   
Edit the `package.xml` and `CMakeLists.txt` adding:

- package.xml

.. code-block:: xml
   
   <buildtool_depend>catkin</buildtool_depend>
   <build_depend>roscpp</build_depend>
   <build_depend>rospy</build_depend>
   <build_export_depend>roscpp</build_export_depend>
   <build_export_depend>rospy</build_export_depend>
   <exec_depend>roscpp</exec_depend>
   <exec_depend>rospy</exec_depend>


- CMakeLists.txt

.. code-block:: bash

   find_package(catkin REQUIRED COMPONENTS
      roscpp
      rospy
   )


Finally, create a file `jetson.launch` inside `launch` folder with the content:

.. code-block:: xml

   <launch>
	  <!-- GPS -->
	  <node name="adafruit_gps" pkg="nmea_navsat_driver" type="nmea_serial_driver" args="_port:=/dev/ttyUSB0 fix:=/gps/fix" />

	  <!-- IMU -->
      <include file="$(find i2c_imu)/launch/i2c_imu_auto.launch" />

	  <!-- ZED -->
	  <include file="$(find zed_wrapper)/launch/zed.launch" />
   </launch>

In order to test, you should call the ROS launch by typing:

.. code-block:: bash

   $ roslaunch jetson_launchers jetson.launch

To record the content of the topics, run the command:

.. code-block:: bash

   $ rosbag record /gps/fix /zed/left/image_raw_color /zed/right/image_raw_color /zed/odom /zed/depth/depth_registered /gps/fix /imu/data  -o /media/ubuntu/Card/<filename>.bag
