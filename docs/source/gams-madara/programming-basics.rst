
=========================
GAMS/MADARA Programming Basics
=========================
In this page, we will provide you a quick start with GAMS/MADARA. 


There are three parts of the ROS Quick Start tutorials. In the first part, you will get an understanding about ROS main concepts.
Then, you will learn how to develop your first program with ROS, namely programming a publisher and subscriber using both C++ and Python.
Finally, some more advanced concepts will be introduced through the development of a cleaning application with the Turtlesim simulator. 
At the end of ROS Quick Start tutorials, you will get a full understanding of ROS that allows you to dive deeper and develop more advanced robotics applications. 

Project creation
----------------

To create a new GAMS project, you should use a linux script called gpc.pl. So, you should run the following in a terminal window:

.. code-block:: bash

  $GAMS_ROOT/scripts/projects/gpc.pl --path $PROJECT_ROOT/tutorial1
  
With this command, you create a project named tutorial1, which has the following folder structure:
 
.. code-block:: bash

   .
   ├── bin  
   ├── sim  
   src  
   action.bat  
   action.sh  
   README.txt  
 
