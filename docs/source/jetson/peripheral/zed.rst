======================
Stereolabs ZED Camera
======================

The ZED is a camera that reproduces the way human vision works. Using its two "eyes" and through triangulation, the ZED provides a three-dimensional understanding of the scene it observes, allowing your application to become space and motion aware. ZED perceives the world in three dimensions by using dual lenses. Using binocular vision and high-resolution sensors, the camera can tell how far objects are around you from 0.5 to 20m at 100FPS, indoors and outdoors. It captures high-definition 3D video with a wide field of view and outputs two synchronized left and right video streams in side-by-side format on USB 3.0 [1]_. 

.. image:: ../images/zed.jpg
   :align: center
   :width: 500pt

The ZED camera is a stereoscopic imaging camera which contains two high definition imagers. One imager is mounted on the left and the other on the right side of the camera enclosure. The camera provides a video stream, each frame of which consists of a composite of an image from each camera, side by side. The images are time synchronized. The video stream is sent over USB 3.0 to a host. On the host, the frame in the stream is then converted to a depth map using the host GPU. The Stereolabs SDK on the host uses the geometry of the fixed distance between the imaging elements, and using the known field of view of the imagers calculates an accurate depth map. The ZED can sense depth between 1 and 20 meters [2]_.


Installing ZED SDK
-------------------

To install the ZED SDK in Jetson TK1, we need to access the *Archive* section in *Developer* menu of the Stereolabs web site, and download the **ZED SDK 1.2** file. Although ZED SDK has newer versions, they are not compatible with Jetson TK1 board or with Ubuntu 14.04 installed in Jetson TK1. To download the ZED SDK directly without the need of navigate in the site and install it in the Jetson, run:

.. code-block:: bash

   $ wget https://www.stereolabs.com/developers/downloads/archives/ZED_SDK_Linux_JTK1_v1.2.0.run
   $ chmod +x ZED_SDK_Linux_JTK1_v1.2.0.run
   $ ./ZED_SDK_Linux_JTK1_v1.2.0.run

Running these commands will start to install the SDK. After finishing the installation process, reboot the Jetson TK1 board to apply the modifications.  


Installing ZED ROS Wrapper
---------------------------

The ZED ROS wrapper lets you use the ZED stereo camera with ROS [3]_. It provides access to the following data:
- Left and right rectified/unrectified images
- Depth map
- Colored 3D point cloud
- Visual odometry (position and orientation of the camera)

In order to Download the version 1.2 of ZED ROS Wrapper from Github, copy the folder to Catkin environment anc compile it as:

.. code-block:: bash

   $ cd ~/catkin_ws/src
   $ git clone https://github.com/stereolabs/zed-ros-wrapper.git
   $ cd ~/catkin_ws/src/zed-ros-wrapper
   $ git checkout f2a62b0
   $ cd ~/catkin_ws/
   $ catkin_make
   $ source ./devel/setup.bash

These commands will download and install the ZED Wrapper to ROS and compile the Catkin Workspace. It is important to note that the git checkout is performed on ``f2a62b0``, which means the version v1.2.0 of the ROS Wrapper -- this version is compatible with the ZED SDK to Jetson TK1. 


Install ZED and ROS Wrapper from script
----------------------------------------

Instead of running all the previous commands in sequence, we wrote a script to perform the whole process in a single file. You can download and run the script using:

.. code-block:: bash

   $ wget --no-check-certificate --content-disposition https://raw.githubusercontent.com/lsa-pucrs/platypus_doc/master/docs/source/jetson/scripts/installZED.sh
   $ chmod +x installZED.sh
   $ ./installZED.sh


Testing ZED Camera through ROS Wrapper
---------------------------------------

To check if the installation was concluded without errors and the SDK is running OK, open a terminal and run ROS core by using the command:

.. code-block:: bash

   $ roscore

Open another terminal and launch the ZED wrapper with the command:

.. code-block:: bash

   $ roslaunch zed_wrapper zed.launch

Open a third terminal and check if the topics of ZED are available by using the command:

.. code-block:: bash

   $ rostopic list

In case everything is looking fine, ZED will publish the following topics in ROS:

.. code-block:: bash

    /zed/depth/camera_info
    /zed/depth/depth_registered
    /zed/joint_states
    /zed/left/camera_info
    /zed/left/image_raw_color
    /zed/left/image_rect_color
    /zed/odom
    /zed/point_cloud/cloud_registered
    /zed/rgb/camera_info
    /zed/rgb/image_raw_color
    /zed/rgb/image_rect_color
    /zed/right/camera_info
    /zed/right/image_raw_color
    /zed/right/image_rect_color


References
-----------

.. [1] `Stereolabs Introduction <https://www.stereolabs.com/documentation/overview/getting-started/introduction.html>`_
.. [2] `Jetsonhacks ZED Camera <http://www.jetsonhacks.com/2016/02/03/stereolabs-zed-camera/>`_
.. [3] `ZED Integrations: ROS <https://www.stereolabs.com/documentation/integrations/ros/getting-started.html>`_
