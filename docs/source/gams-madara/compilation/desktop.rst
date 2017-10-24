
=========================
GAMS/MADARA Compilation
=========================
In this page, we will provide you a quick start to compile GAMS/MADARA for Linux Desktop, more specifically all scripts was tested with Linux Ubuntu 16.04.


It is esperado that all dependencies be resolved automatically by scripts. You have to be aware and look for errors of download failures, which the script can't handle.

To download GAMS/MADARA, just run:

.. code-block:: bash

	$ export GAMS_ROOT=$HOME/gams
	$ export CORES=4
	$ git clone -b master --single-branch https://github.com/jredmondson/gams $GAMS_ROOT

After that, you should compile the following one of the following commands.

Build C++ with Tests

.. code-block:: bash

	$ $GAMS_ROOT/scripts/linux/base_build.sh prereqs ace madara gams vrep tests

Build C++ with Tests and Java support

.. code-block:: bash

	$ $GAMS_ROOT/scripts/linux/base_build.sh prereqs ace madara gams vrep tests java

Build C++ with Tests and Android support

.. code-block:: bash

	$ $GAMS_ROOT/scripts/linux/base_build.sh prereqs ace madara gams vrep tests android

Build C++ with ROS support

.. code-block:: bash

	$ $GAMS_ROOT/scripts/linux/base_build.sh prereqs ace madara gams ros


When the scripts finishes, you should see in folder you called the script the following the following folder structure:
  
 - ACE  
 - GAMS  
 - MADARA  
 - VREP  


