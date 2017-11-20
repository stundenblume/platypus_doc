
=========================
GAMS/MADARA Background
=========================

In this page, we will provide you a quick start with GAMS/MADARA.

The main loop of GAMS can be resumed with following sequence diagram:

.. image:: images/GamsRunLoop.png
   :align: center
   :width: 500pt
   


where the MAPE process is mapped as:

  * Monitor phase: platform sense;
  * Analyze phase: platform analyze, algorithm analyze;
  * Plan phase: algorithm plan;
  * Execute phase: algorithm execution.
  
The possible values of PlatformAnalyzeStatus is:
* UNKNOWN = 0,
* OK  = 1,
* WAITING = 2,
* DEADLOCKED = 4,
* FAILED = 8,
* MOVING = 16,
* REDUCED_SENSING_AVAILABLE = 128,
* REDUCED_MOVEMENT_AVAILABLE = 256,
* COMMUNICATION_AVAILABLE = 512,
* SENSORS_AVAILABLE = 1024,
* MOVEMENT_AVAILABLE = 2048

