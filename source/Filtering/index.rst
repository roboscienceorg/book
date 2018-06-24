.. _`Chap:Filtering`:

******************************
Filtering and State Estimation
******************************

Consider the two robots given in
:numref:`fig:robotsandhumans`. How should we
approach designing robots to function in conjunction with humans or
instead of humans? For example:

-  Robotic system to move goods through a distribution center.

-  Robotic system to care for the elderly.

-  Robotic system to perform tasks in hostile environments (deep sea,
   reactors, space, underwater caves, etc).

-  Robotic systems for assistive technology.

To do this we need to have perception of a changing environment, we need
to make decisions about how to respond based on the robot function
(goals) and we need to control effectors to carry out the intended
functions. For robot perception alone, we scan the environment, segment
out objects and then recognize the objects. There many types of sensors
used to just understand the surrounding environment:


-  Contact sensors

-  Internal Sensors

-  Accelerometers

-  Gyroscopes

-  Compasses

-  Proximity Sensors

-  Sonar

-  Radar

-  Laser range finders

-  Infrared

-  Cameras

-  GPS

One of the basic functions, localization, is completely dependent on the
sensors. Decisions about future actions are made based on the sensors.
Feedback from the actuators is also based on the sensors. Many aspects
of the system ride on the sensors. The Sensors Chapter presented a number
and variety of sensing systems. There is a vast array of sensors which can sense or
measure physical quantities. Accuracy on sensors varies greatly with some very
accurate and others having considerable errors.

.. _`fig:robotsandhumans`:
.. figure:: FilteringFigures/PR2.png
   :width: 40%
   :align: center

   Examples of two robotics systems that interact with
   humans.

.. not checked  -  add this to the above
.. figure:: FilteringFigures/CartBot.png
   :width: 40%
   :align: center


**Problem**: How can we design a system which can function without human
input but has random elements in the environment? How can the system
determine its location accurately enough to navigate? How can the robot
use manipulators around humans safely if manipulator location, feedback
and control are uncertain. In manufacturing systems, we are able to
instrument objects of interest and highly constrain the environment. The
robot might not be local and the object to be manipulated may have a
known location. Clearly variation and noise occur, but not to the degree
found in robots that are intended to go into the world and operate
outside the confines of a factory.

.. toctree::
   :maxdepth: 1

   StateEstimation
   SensorFusion
   Filtering_Problems
