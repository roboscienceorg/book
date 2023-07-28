.. _`Chap:ROS`:

********************************
ROS - The Robot Operating System
********************************

| The official ROS website defines ROS as follows:
| *ROS (Robot Operating System) provides libraries and tools to help
  software developers create robot applications. It provides hardware
  abstraction, device drivers, libraries, visualizers, message-passing,
  package management, and more.*

We present a brief summary of ROS, specifically ROS2.
It should be noted that this introduction is in no way complete, as it
does not
convey the sheer power and complexity of ROS/ROS2, and
the ways in which it goes about doing so. The reader is strongly
encouraged to look at the
`ROS2 tutorials <https://github.com/ros2/ros2/wiki/Tutorials>`_ .
ROS2 was in beta for most of 2017 until Ardent was released in Dec 2017.
Currently, there is not a significant collection of ROS2 information.
ROS (ROS1) does have some good references and there is some overlap in
concepts.  Because of the size of the install base, ROS1 is not leaving
anytime soon.  A couple of very good recent texts on
ROS are :cite:`okane:2014:GIR` and :cite:`quigley:2015:PRR`.  For the
remainder of this text we will only write "ROS" but unless explicitly stated
we will mean ROS2.


It is slightly misleading that ROS includes the phrase *operating
system* in the title. ROS itself is not an operating system in the
traditional sense, but it is much more than just a piece of software.
The many components combine to form an ecosystem of software which
warrants its title but is best thought of as middleware. While on the
“not” topic, ROS is not a programming language, not an IDE (integrated
development environment) and not just a library of robotics codes.

As mentioned, package management and hardware abstraction are just a
couple of features under the ROS umbrella, which support the
communication framework around which ROS is based. The intent is to
create a universal system which promotes code reuse and sharing among
multiple robotic platforms, operating systems, and applications as well
as small program footprints and efficient scaling. These pillars form
the core goals of ROS as a whole.

Next, we introduce the simulation tool, Veranda, and cover
the basic elements to simulate robot motion.   Veranda is a two
dimensional simulator.  It was designed to have a low barrier to
entry and overall ease of use.  Veranda uses Box2D, a 2D physics
engine for more realistic interactions between objects.   It is not
intended for very high precision work (and does not support 3D).
For those applications, we suggest Gazebo (see appendix).



.. toctree::
   :maxdepth: 1

   ROS
   ROSCommunication
   BuildDriveTutorial
   Startsim
   ObstacleSensorTutorial
   ROS_Problems
