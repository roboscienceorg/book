.. _`Chap:AdvFiltering`:

*****************************
Advanced Filtering Techniques
*****************************

Observation is not the whole story. The robot will be out observing the
world, even though the observations often include random noise, you normally
have additional information that allows you to remove significant noise.
For example, if we are
observing motion, we know that not all types of motion will be possible.
The basic laws of physics clearly play a role. In addition the design
and details of the robot also play a role. These constrain the
possibilities and in doing so, help improve our estimates of the robot
state.

.. toctree::
   :maxdepth: 1

   ModelsandDynamicSystems
   KalmanFilters
   ExtendedKalmanFilter
   Particle
   Kalman_Problems
