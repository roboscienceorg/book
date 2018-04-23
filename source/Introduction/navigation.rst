Navigation and Localization
---------------------------

Navigation is the process of routing the robot through the environment.
Localization is the process of determining where the robot is in the
environment. Most of the robots we imagine can move around. So, we
expect that a mobile robot can navigate its environment. This really
seems pretty simple. After all, worms and insects can do it, so machines
should have no problem. Right? Navigation in three dimensions requires
that the robot have a full understanding of the obstacles in the
environment as well as the size and shape of the robot. Determining a
path through the environment may also come with constraints on the path
or robot pose. Typically to route a robot to some location, the current
location is needed. Clearly just moving and avoiding obstacles does not
require any knowledge of location, but there are plenty of times where
the routing and localization problem are intertwined.


.. Image by Roboscience.
.. _`basic-navigation`:
.. figure:: IntroductionFigures/navigation.svg
   :width: 90%
   :align: center

   Navigation approaches.  a)  A very simple approach to navigating the robot.  The programmer
   codes in the times and velocities to run the motors for linear travel and
   turns.  b) By instrumenting the environment, for example placing lines or
   grooves on the floor, the robot can successfully navigate.


Navigation requires sensory information. The availability and type of
information is critical to how effectively the robot can navigate or
localize. Having only sensors that measure wheel location makes
localization difficult and path planning impossible. Dead Reckoning is
the method of determining the speed and run times for the motors. Then
repeating this in different combinations in order to navigate the
course. Essentially this is the game that you memorize your steps and
turns and then try to retrace them with a blindfold. Modifying the
environment allows for much better control of the robot but with the
added costs of environment modification, see Figure [environmentmods].
Dead reckoning normally has very poor results due to normal variations
in motors. Environmental instrumentation can be very successful if
available.

.. Owned by RoboScience
.. figure:: IntroductionFigures/localization.svg
   :width: 85%
   :align: center

   Localization can be very difficult. In this example, a LIDAR scan is
   compared to a known map to deduce the location of the robot.

The approaches and algorithms are based on the underlying
representations of space. We can represent space as a grid, or a
continuum or an abstract system, Figure [fig:maptypes]. Each method will
determine the way we index the object (integers or floating point
values), the resolution on location and the algorithm for accessing the
object. We could also represent space in a discrete manner. This makes
grid based approaches available. Space could also have a graph
structure. The algorithms to navigate then will use or exploit these
different ways space is represented. The differences give rise to
different performance, accuracy, and results.

0.3 |An example of different map types.[fig:maptypes]|

0.3 |An example of different map types.[fig:maptypes]|

0.35 |An example of different map types.[fig:maptypes]|

Although challenging, navigation is a core skill in mobile robotics.
Autonomous navigation is a focus for many industries. Farming is looking
at conversion to autonomous machines as well as autopilot systems for
automobiles. Of great current interest is a vision based autopilot
system, Figure [fig:visionautopilot]. This is an active area of research
and we touch on it in the next section.

.. figure:: vision/bosch.jpg
   :alt: Vision based driver assist system (Bosch).
   [fig:visionautopilot]

   Vision based driver assist system (Bosch). [fig:visionautopilot]
