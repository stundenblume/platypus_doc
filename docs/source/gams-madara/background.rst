
=========================
GAMS/MADARA Background
=========================

In this page, we will provide you a quick start with GAMS/MADARA.

The main loop of GAMS can be resumed with following sequence diagram:

.. image:: images/GamsRunLoop.png
   :align: center
   :width: 500pt
   


where the MAPE process is mapped as:

  Monitor phase: platform sense;
  Analyze phase: platform analyze, algorithm analyze;
  Plan phase: algorithm plan;
  Execute phase: algorithm execution.
  
  
