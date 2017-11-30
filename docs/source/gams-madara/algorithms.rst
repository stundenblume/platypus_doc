
Algorithms
----------


Waypoints
---------

To use this kind of algorithm, you should configure some variables into knowledge system. To do that, you can write into agents configuration file the following lines:

.. code::block: bash

  agent.0.algorithm = "waypoints";
  agent.0.algorithm.args.locations.size=4;
  agent.0.algorithm.args.locations.0=[0, 0, 0];
  agent.0.algorithm.args.locations.1=[0, 2, 0];
  agent.0.algorithm.args.locations.2=[2, 2, 0];
  agent.0.algorithm.args.locations.3=[2, 0, 0];
  agent.0.algorithm.args.repeat=3;

