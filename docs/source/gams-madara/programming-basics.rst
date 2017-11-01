
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

To create a new GAMS project, you should use a linux script called ``gpc.pl``. So, you should run the following in a terminal window:

.. code-block:: bash

  $GAMS_ROOT/scripts/projects/gpc.pl --path $PROJECT_HOME/tutorial1
  
With this command, you create a project named ``tutorial1``, which has the following folder structure:
 
.. code-block:: bash

   .
   ├── bin                  # store binary files
   ├── sim                  # store simulation files
   |    ├── agent_0.mf          #
   |    ├── common.mf           #
   |    ├── env.mf              #
   |    ├── run.pl              #
   ├── src                  # store your source code files
   |    ├── algorithms          # store algorithms (non blocking piece of code)
   |    ├── filters             #
   |    ├── platforms           #
   |    ├── threads             # store algorithms (blocking piece of code)
   |    ├── transports          #
   ├── action.bat           # script to compile and run the project in Windows
   ├── action.sh            # script to compile and run the project in Linux
   └── README.txt           # some how-to file with compile and run commands
 
 
 PRINTING INTO AGENT TERMINAL WINDOW
 -----------------------------------
 
 To print some message into Agent terminal window you can use the following command:
 
  madara_logger_ptr_log (gams::loggers::global_logger.get (), gams::loggers::LOG_MAJOR, " My message is hello folks!");
  
 
 
