ROS - The Robot Operating System
================================

| The official ROS website defines ROS as follows:
| *ROS (Robot Operating System) provides libraries and tools to help
  software developers create robot applications. It provides hardware
  abstraction, device drivers, libraries, visualizers, message-passing,
  package management, and more.*

We present a brief summary of ROS. It should be noted that it fails to
convey the sheer power and complexity of the tasks that it performs, and
the ways in which it goes about doing so. The reader is strongly
encouraged to look at some of the very good recent texts on
ROS :cite:`okane:2014:GIR`, :cite:`quigley:2015:PRR`.

It is slightly misleading that ROS includes the phrase “operating
system” in the title. ROS itself is not an operating system in the
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



Origins
-------

Player
~~~~~~
.. image:: ROSFigures/player_button_v3.png
   :width: 15%

The beginnings of ROS date back to the Player project, which was founded
in 2000 by Brian Gerkey. This model included a hardware-abstracted
robotic system known as the player, which interfaced with its simulated
environment, known as the stage.


Switchyard
~~~~~~~~~~
.. image:: ROSFigures/willow_garage.jpg
   :width: 15%

The common API used by player was a major part of the next step on the
road to ROS, the Stanford project known as “Switchyard.” :index:`Switchyard` was
developed by Morgan Quigley in 2007 under the Stanford Artificial
Intelligence Robot (STAIR) project. Development of the system was
shifted to a Stanford robotics start-up known as :index:`Willow Garage` in 2008.
The platform matured for about 2 years, and in 2010, Willow Garage
released the first version of ROS.


OSRF
~~~~
.. image:: ROSFigures/osrf_masthead.png
   :width: 15%

In 2012, development of ROS began to shift from Willow Garage to the
newly formed, Open Source Robotics Foundation, :index:`OSRF` also oversees development of the Gazebo robot
simulator, as well as the annual ROSCon, where ROS developers meet and
discuss various ROS-related topics. Development using ROS still
continues at Willow Garage, but the framework as a whole is developed at
OSRF.


.. toctree::
   :maxdepth: 2

   ROSInstallation
   FundamentalROS
   ROSCommunication
