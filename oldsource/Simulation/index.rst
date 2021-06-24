.. _`Chap:Simulation`:

**********
Simulation
**********

In this chapter we introduce the simulation tool, Veranda, and cover
the basic elements to simulate robot motion.   Veranda is a two
dimensional simulator.  It was designed to have a low barrier to
entry and overall ease of use.  Veranda uses Box2D, a 2D physics
engine for more realistic interactions between objects.   It is not
intended for very high precision work (and does not support 3D).
For those applications, we suggest Gazebo (see appendix).

Following the overview and examples with Veranda, we will cover some
standard techniques in simulation.  It is very helpful to understand
what is going on inside the simulator.  It also is very useful to know
how to create your own simulation when none of the current simulation
tools will suffice.  Before we get going on the simulation tools,
an overview of SciPy is done to get everyone up to speed.



.. toctree::
   :maxdepth: 1

   SimulatingMotion
   ScipyMath
   TwoLinkSim
   BuildDriveTutorial
   Startsim
   MovingDifferential
   Noise
   ObstacleSensorTutorial
   GroundRobotWorld
   MatPlotLib
   Simulation_Problems
