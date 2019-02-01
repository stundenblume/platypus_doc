===========================================
Robot Operating System (ROS) on Jetson TX2
===========================================

Robot Operating System (ROS) is an open-source, meta-operating system for your robot. It provides the services you would expect from an operating system, including hardware abstraction, low-level device control, implementation of commonly-used functionality, message-passing between processes, and package management. It also provides tools and libraries for obtaining, building, writing, and running code across multiple computers. The ROS runtime "graph" is a peer-to-peer network of processes (potentially distributed across machines) that are loosely coupled using the ROS communication infrastructure. ROS implements several different styles of communication, including synchronous RPC-style communication over services, asynchronous streaming of data over topics, and storage of data on a Parameter Server [1]_. 

The primary goal of ROS is to support code reuse in robotics research and development so you can find a built-in package system. It is interesting to note that although ROS contains Operation System in the name, keep in mind that ROS is not an OS, a library, or an RTOS, but a framework using the concept of an OS. A good introduction is given in the freely available book named `A Gentle Introduction to ROS <https://www.cse.sc.edu/~jokane/agitr/agitr-letter.pdf>`_ by Jason O'Kane. The `ROS Wiki <http://wiki.ros.org/ROS/Tutorials>`_ also contains lots of tutorials to introduce you to its main concepts.


Installing ROS in Jetson TX2
-----------------------------

JetsonPack for Jetson TX2 is based on Ubuntu 16.04 and thus, the ROS version to install is named *Kinetic*. In order to install the ROS package, you can run the ``installROS.sh`` file as:

.. code-block:: bash

   $ wget --no-check-certificate --content-disposition https://raw.githubusercontent.com/lsa-pucrs/platypus_doc/master/docs/source/jetson/TX2/scripts/installROS.sh
   $ chmod +x installROS.sh
   $ ./installROS.sh

This script installs ROS Kinetic Desktop and the Point Cloud Library.


Creating Catkin workspace to run ROS
-------------------------------------

Catkin packages can be built as a standalone project, in the same way that normal cmake projects can be built, but catkin also provides the concept of workspaces, where you can build multiple, interdependent packages together all at once. A catkin workspace is a folder where you modify, build, and install catkin packages [2]_. In order to create a Catkin workspace, you should download and run the file ``createCatkin.sh`` as:

.. code-block:: bash

   $ wget --no-check-certificate --content-disposition https://raw.githubusercontent.com/lsa-pucrs/platypus_doc/master/docs/source/jetson/scripts/createCatkin.sh
   $ chmod +x createCatkin.sh
   $ ./createCatkin.sh

This script creates a structure in the home folder with the root workspace set in ``/home/ubuntu/catkin_ws``. 


Installing vision_opencv in ROS
--------------------------------

By default, ROS has support to native OpenCV. However, when trying to compile a C++ code with a call to ``cv_bridge``, the following error 

.. ERROR::
   make[2]: *** No rule to make target `/usr/lib/arm-linux-gnueabihf/libopencv_videostab.so.2.4.8'.  Stop.

In order to get rid of the error, we have to compile `vision_opencv <http://wiki.ros.org/vision_opencv>`_ in our Catkin workspace. Hence, the first step is to clone the Indigo version of ``vision_opencv`` into the ``catkin_ws/src`` folder. Then, we have to compile the workspace, by running:

.. code-block:: bash

   $ cd ~/catkin_ws/src
   $ git clone https://github.com/ros-perception/vision_opencv.git
   $ cd vision_opencv/
   $ git checkout kinetic
   $ cd ../..
   $ catkin_make

Next time you compile a C++ code that contains a call to ``cv_bridge``, no errors should appear.


Testing ROS installation
-------------------------

To check if the ROS is installed correctly, run:

.. code-block:: bash

    $ roscore

And see it starts running correctly. In case of problem, you can check the log files by running:

.. code-block:: bash

    $ roscd log


References
-----------

.. [1] `ROS Introduction <http://wiki.ros.org/ROS/Introduction>`_
.. [2] `Catkin Workspaces <http://wiki.ros.org/catkin/workspaces#Catkin_Workspaces>`_

