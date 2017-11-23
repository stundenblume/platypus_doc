
=========================
GAMS/MADARA Programming Basics
=========================
In this page, we will provide you a quick start with GAMS/MADARA programming. You could follow the several tutorials that will introduce in a pratical way the main aspects of GAMS/MADARA. 




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

  cd $PROJECT_HOME/tutorial1
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
 
Finally, we have to configure the algorithm that each agent should run. Edit file ``agent_0.mf`` so the algorithm name be ``producer``. Inside of file ``agent_0.mf`` will look like:
 
.. code-block:: bash

  agent.0.algorithm = "producer";

Edit file ``agent_1.mf`` so the algorithm name be ``consumer``:

 .. code-block:: bash
 
  agent.0.algorithm = "consumer";


Now, we have to declare a variable ``counter`` (of type ``madara::knowledge::containers::Integer``) in ``producer.h`` and ``consumer.h``. 

So, your file ``producer.h`` will looks like:

.. code-block:: bash

   class producer : public gams::algorithms::BaseAlgorithm
   {

     protected:
	    madara::knowledge::containers::Integer counter;
	
     ....

     

So, your file ``consumer.h`` will looks like:

.. code-block::

   class consumer : public gams::algorithms::BaseAlgorithm
   {

     protected:
	    madara::knowledge::containers::Integer counter;
	
     ....
     

In your ``producer.cpp`` we should configure the counter variable to be handled by madara::knowledge. So the file will looks like:

.. code-block::

	algorithms::producer::producer (
	  madara::knowledge::KnowledgeBase * knowledge,
	  gams::platforms::BasePlatform * platform,
	  gams::variables::Sensors * sensors,
	  gams::variables::Self * self,
	  gams::variables::Agents * agents)
	  : gams::algorithms::BaseAlgorithm (knowledge, platform, sensors, self, agents)
	{
	  status_.init_vars (*knowledge, "producer", self->agent.prefix);
	  status_.init_variable_values ();
	  counter.set_name("counter", *knowledge);
	}
     
	int algorithms::producer::plan (void)
	{
		counter += 1;
		madara_logger_ptr_log (gams::loggers::global_logger.get (), gams::loggers::LOG_MAJOR, "\n ----Incrementing the to counter: %d \n\n", counter.to_integer());

	  return 0;
	}


In your ``consumer.cpp``, we should relate the counter variable with madara::knowledge, so consumer will have access to updates made by producer. 

.. code-block::

	algorithms::consumer::consumer (
	  madara::knowledge::KnowledgeBase * knowledge,
	  gams::platforms::BasePlatform * platform,
	  gams::variables::Sensors * sensors,
	  gams::variables::Self * self,
	  gams::variables::Agents * agents)
	  : gams::algorithms::BaseAlgorithm (knowledge, platform, sensors, self, agents)
	{
	  status_.init_vars (*knowledge, "consumer", self->agent.prefix);
	  status_.init_variable_values ();
	  counter.set_name("counter", *knowledge);
	}

	int algorithms::consumer::plan (void)
	{
		madara_logger_ptr_log (gams::loggers::global_logger.get (), gams::loggers::LOG_MAJOR, "\n ----Now the counter is: %d \n\n", counter.to_integer());
	  return 0;
	}
	
After that, we should configure ``controller.cpp`` to share knowledge between the agents. You have to write the following commands:

.. code-block::

	// perform main logic of program
	int main (int argc, char ** argv)
	{
	  settings.type = madara::transport::MULTICAST;
	  settings.hosts.push_back("127.0.0.1:19906");
	  settings.hosts.push_back("127.0.0.1:19907");
	  
	  ...
	  
	  
in above lines, you are telling to MADARA that knowledge should be shared into two hosts (agents): the host located into IP 127.0.0.1 and at port 19906; and the host located into 127.0.0.1:19907. You can get  port number from agent file configuration located into folder ``sim`` (i.e. ``sim/agent_0.mf`` and ``sim/agent_1.mf``).


We should compile and run the project by executing the following commands into bash:

.. code-block:: bash

	$ ./action compile-vrep
	$ ./action vrep run



You can get full source of this project by running the following command:

.. code-block:: bash

	git clone https://github.com/marceloparavisi/gams_tutorials.git
	
