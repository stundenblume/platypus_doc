
=========================
GAMS/MADARA Programming Basics
=========================
In this page, we will provide you a quick start with GAMS/MADARA. 


There are three parts of the ROS Quick Start tutorials. In the first part, you will get an understanding about ROS main concepts.
Then, you will learn how to develop your first program with ROS, namely programming a publisher and subscriber using both C++ and Python.
Finally, some more advanced concepts will be introduced through the development of a cleaning application with the Turtlesim simulator. 
At the end of ROS Quick Start tutorials, you will get a full understanding of ROS that allows you to dive deeper and develop more advanced robotics applications. 



Printing into agent terminal window
-----------------------------------
 
To print some message into Agent terminal window you can use the following command:
 
  madara_logger_ptr_log (gams::loggers::global_logger.get (), gams::loggers::LOG_MAJOR, " My message is hello folks!");
  
 


Creating an consumer/producer application
-----------------------------------------

As your first application, we going to create an application that one agent generate values and other agent consume them. To do that, we start creating our application by running the following command:

.. code-block:: bash

  $GAMS_ROOT/scripts/projects/gpc.pl --path $PROJECT_HOME/tutorial1
  
After the project creation, we should create two algorithms ``producer`` and ``consumer``. This is done by running:

.. code-block:: bash

  $GAMS_ROOT/scripts/projects/gpc.pl --new-algorithm producer --path $PROJECT_HOME/tutorial1
  $GAMS_ROOT/scripts/projects/gpc.pl --new-algorithm consumer --path $PROJECT_HOME/tutorial1
  
After running those commands, your filesystem looks like:

.. code-block:: bash

   .
   ├── bin                  
   ├── sim                  
   |    ├── agent_0.mf          
   |    ├── common.mf           
   |    ├── env.mf              
   |    ├── run.pl              
   ├── src                  
   |    ├── controller.cpp          
   |    ├── algorithms          
   |    |    ├── consumer.h          
   |    |    ├── consumer.cpp
   |    |    ├── producer.h
   |    |    ├── producer.cpp
   |    ├── filters             
   |    ├── platforms           
   |    ├── threads             
   |    ├── transports          
   ├── action.bat           
   ├── action.sh            
   ├── using_gams.mpb            
   ├── using_vrep.mpb
   ├── using_ace.mpb
   ├── using_madara.mpb
   ├── workspace.mwc
   └── README.txt           
   
Now, we need add a second agent to our simulation, so we run the following command:

.. code-block:: bash

 $GAMS_ROOT/scripts/projects/gpc.pl --agents 2 --randomize
 
 With that, the folder ``sim`` will get updated by the addition of file ``agent_1.mf``.
 
 Finally, we have to configure the algorithm that each agent should run. Edit file ``agent_0.mf`` so the algorithm name be ``producer``:
 
 .. code-block::
 
  agent.0.algorithm = "producer";

Edit file ``agent_1.mf`` so the algorithm name be ``consumer``:

 .. code-block::
 
  agent.0.algorithm = "consumer";
