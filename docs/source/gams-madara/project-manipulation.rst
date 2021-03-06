=========================
GAMS/MADARA Project manipulation
=========================
In this page, we will provide you a quick start with GAMS/MADARA project creation. 

Creating a project
------------------

To create a new GAMS project, you should use a linux script called ``gpc.pl``. So, you should run the following in a terminal window:

.. code-block:: bash

  $GAMS_ROOT/scripts/projects/gpc.pl --path $PROJECT_HOME/tutorial1
  
With this command, you create a project named ``tutorial1``, which has the following folder structure:
 
.. code-block:: bash

   .
   ├── bin                  # store binary files
   ├── sim                  # store simulation files
   |    ├── agent_0.mf          # stores vrep port number, starting location(lat,long, alt) and algorithm name
   |    ├── common.mf           # stores vrep configuration (ip adress, max distance of VREPBase::move)
   |    ├── env.mf              # stores vrep environment configuration (size, surface texture/type, etc)
   |    ├── run.pl              # stores running configurations (number of agents, hosts, domains, etc)
   ├── src                  # store your source code files
   |    ├── algorithms          # store algorithms (non blocking piece of code)
   |    ├── filters             #
   |    ├── platforms           #
   |    |    ├── threads        # store threads related to platforms
   |    ├── threads             # store algorithms (blocking piece of code)
   |    ├── transports          #
   ├── action.bat           # script to compile and run the project in Windows
   ├── action.sh            # script to compile and run the project in Linux
   └── README.txt           # some how-to file with compile and run commands
 
Creating algorithms
-------------------

You can create an algorithm (named as ``talker``) into your project (called ``tutorial3``) by running the following code:

.. code-block:: bash

  $GAMS_ROOT/scripts/projects/gpc.pl --new-algorithm talker --path $PROJECT_HOME/tutorial1
  
Creating threads (algorithms)
-----------------------------

You can create a thread (named as ``sense``) into your project (called ``tutorial3``) by running the following command inside your project's folder:
 
.. code-block:: bash

  $GAMS_ROOT/scripts/projects/gpc.pl --new-thread sense
  
  
Creating threads (platforms)
-----------------------------

You can create a thread (named as ``pid``) into your project (called ``tutorial3``) by running the following command inside your project's folder:
 
.. code-block:: bash

  $GAMS_ROOT/scripts/projects/gpc.pl --new-platform-thread pid
 
Adding more agents into simulation
----------------------------------

You can add more agents into simulation by running the following command into terminal:

.. code-block:: bash

  $GAMS_ROOT/scripts/projects/gpc.pl --agents 2 --randomize

With this command, your simulation will have 2 agents, which will generate randomized coordinates (lat, long). These values is stored into ``agent_0.mf`` and ``agent_1.mf``,whose are located into ``sim`` folder.


Killing broken V-REP
--------------------

Sometimes when you try to close your simulation, V-REP stops working. To fast kill it, just run the following command:

.. code-block:: bash

  kill $(pgrep vrep)


Creating plataforms
-------------------

To create plataforms in your projects, you should run the following command:

.. code-block:: bash

  $GAMS_ROOT/scripts/projects/gpc.pl --new-platform my_platform --path $PROJECT_HOME/tutorial1
  
  
DEBUG LOG LEVELS
----------------

You can define the log level that will be printed into gams terminal by configuring the file ``sim/run.pl``:

.. code-block:: bash

  $madara_debug = 3;
  $gams_debug = 3;
  
You can define the following values to log level:

     * LOG_EMERGENCY = 0,
     * LOG_ALWAYS = 0,
     * LOG_ERROR = 1,
     * LOG_WARNING = 2,
     * LOG_MAJOR = 3,
     * LOG_MINOR = 4,
     * LOG_TRACE = 5,
     * LOG_DETAILED = 6,
     * LOG_MAX = 6
      
      
