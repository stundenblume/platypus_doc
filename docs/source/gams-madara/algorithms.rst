==========
Algorithms
==========



There is many algorithms to be used in GAMS/MADARA. 

   * Formation coverage
   * Prioritized Region Coverage
   * Minimum Time Coverage
   * Serpentine Coverage
   * Waypoints
   * Formation Follow
   * Synchronized Formations
   * Convoy Shielding
   * Line Defense
   * Arc Defense
   * Onion Defense
   * Executor


Waypoints
---------

To use this kind of algorithm, you should configure some variables into knowledge system. To do that into agent 0 (zero), you can write into agent configuration file ``agent_0.mf`` the following lines:

.. code-block:: bash

  agent.0.algorithm = "waypoints";
  agent.0.algorithm.args.locations.size=4;
  agent.0.algorithm.args.locations.0=[0, 0, 0];
  agent.0.algorithm.args.locations.1=[0, 2, 0];
  agent.0.algorithm.args.locations.2=[2, 2, 0];
  agent.0.algorithm.args.locations.3=[2, 0, 0];
  agent.0.algorithm.args.repeat=3;

So, you can set the number of waypoints with ``.algorithm.args.locations.size``, number of repetitions by writing ``.algorithm.args.repeat`` and the amount of time (in seconds) that robot should wait after reaching one waypoint by writing ``.algorithm.args.wait_time``.


Random Area Coverage
--------------------
This algorithm generate random waypoints inside the region to coverage by your robot. 

You can specify areas in env.mf to be coverage by agents. So, in env.mf you write: 

.. code-block:: bash

  region.0.object_type = 1;
  region.0.type = 0;
  region.0.priority = 0;
  region.0.size = 4;
  region.0.0 = [0, 0];
  region.0.1 = [0, 5];
  region.0.2 = [5, 5];
  region.0.3 = [5, 5];

while, in ``agent_0.mf`` you specify the area to covered by the agent:

.. code-block:: bash

  agent.0.algorithm="urac";
  agent.0.algorithm.args.area="region.0";

The ``object_type`` can assume the following values:

  1 region
  2 prioritized region
  4 search area (set of regions)
  



Formation coverage
Prioritized Region Coverage
Minimum Time Coverage
Serpentine Coverage
Formation Follow
Synchronized Formations
Convoy Shielding
Line Defense
Arc Defense
Onion Defense
Executor
