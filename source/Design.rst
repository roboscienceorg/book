
Robotics Design Elements 
======================================

Robotics Frameworks
-------------------

A *Robotics Framework* is currently a “catch-all” term. To most
roboticists it means a collection of tools to support robotics software
development. Typically a framework will provide some form of
interprocess communication and a collection of hardware drivers.
Interprocess communication is either shared memory and semaphore
wrappers or TCP/IP socket support.  There are many simulation
systems available. These range from fairly simplistic 2D single robot
with a few obstacles to very sophisticated 3D full physics engine
support systems. It is similar to what is seen in computer gaming. We
will discuss a few of the more popular approaches below.

MS Robotics Developer Studio
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Microsoft Robotics Developers Studio, MSRDS, is a full featured robotics
development environment. It provides support tools for developing
applications, supporting communications, visual authoring and
simulation. MSRDS is a commercial application. The tool includes an
asynchronous runtime environment which supports threading and
interprocess communication. VPL is the Visual Programming Language which
is in the spirit of Visual Studio and Visual Basic. This tool provides a
drag and drop GUI for application development as well as export to C#.
DSSME is a configuration editor to support application configuration and
distribution. VSE, Visual Simulation Environment provides 3D simulation
with physics. Robotics control software may be developed, simulated and
tested without hardware. MSRDS is an active project. It can found at
http://www.microsoft.com/robotics/.

Webots
~~~~~~

Like MSRDS, is a full featured robotics development and simulation
environment as well. It is a commercial application and is more oriented
to instruction/simulation than the others described here. This tool
provides a large choice of simulated sensors and hardware. Robotic
control code can be prototyped in simulation and then ported to hardware
for tuning. The goal is to provide a realistic simulation to reduce
development time using their Model, Program, Simulate, Transfer
approach. Unlike MSRDS, Player and ROS; Webots is more of a real physics
engine, with collision detection and dynamics simulation and less of a
robot OS/communications framework. It can be found at
http://www.cyberbotics.com.

Player-Stage
~~~~~~~~~~~~

Player is a robotics framework. It provides communications and robot
control interfacing. This is open source freely available software.
Player is one of the leaders in the distributed approach to robotic
control software. It provides a network interface to a variety of
hardware devices and systems. Using a client-server approach, it gives
the ability to control any device from any location. This allows
multiple languages and multiple platforms to be used as a single robot
control system; as long as they support sockets (TCP). It is especially
useful in research when the low level software is in C, the sensor
package is in Java and the behavior system is written in Python,
allowing the best tool for the job to be used. Player is still
maintained, but development ceased in 2010 (mostly due to ROS).

Stage is the simulation system that is loosely coupled with Player. They
are separate but have been extensively used together. Stage is consider
a 2.5D (more than 2D but less than 3D) simulation environment. Stage is
oriented towards a world which is described by a two dimensional map of
objects with some height. Fine details in the :math:`z` direction are
not modeled so the tool is not designed for simulation of grasping or
manipulation. Stage is a very popular tool for modeling ground robots
(and multiple ground robots). It has options to be compiled with control
code or communicate with Player via the network interface. In this case,
your control code would talk to Player which interfaces with Stage. The
concept is that you develop your control software interacting with
Stage. Then when ready to deploy, you disconnect Player-Stage and
connect Player to the real hardware.

Player-Stage is a great idea. Getting it to compile and run is rather
difficult. Since development has slowed and many developers have moved
on, finding the right combination of Player version, Stage version, OS
version and library collection can be frustrating. When it compiles and
runs, it is a great tool. It can be found at
http://playerstage.sourceforge.net.

ROS-Gazebo
~~~~~~~~~~

ROS, the Robot Operating System, is an open source robotics framework.
This project grew out of Player and many of the lessons learned with
Player are found in ROS. ROS provides the communication system, a
filesystem, distribution system and several thousand packages for device
support, robot control, machine vision, sensor fusion, mapping,
localization, etc. ROS was supported by Willow Garage (robotics company
out of Stanford), but now maintained by the Open Source Robotics
Foundation. It can be found at http://www.ros.org.

ROS is able to connect to Stage however the current focus is on Gazebo.
Gazebo is an open source 3D simulation envirnoment for robotics (which
began life with Player). It includes full physics simulation with the
Open Dynamics Engine. Gazebo was packaged with ROS through 2012, but as
of 2013, Gazebo has moved to be independent of ROS. Gazebo is capable of
simulating the 2D movement of ground robots, the arms of the PR2
including grasping, flight of a UAV and a great deal more. It can be
found at http://gazebosim.org.

ROS and Gazebo are extensions in some sense to Player-Stage. The idea of
developing code in simulation then redirecting to real hardware is
essentially the same outside the differences in interface syntax.

Robotics Programming Languages
------------------------------

There is not a “best” robotics programming language just as there is not
a best programming language in general. Arguments about a best language
are left to novices attempting to justify the language they most
recently learned. Programming languages are tools like pliers,
screwdrivers and hammers. It depends on what you want to do, what
resources you have available and your personal skill set. Some languages
are more popular however, like C and C++. The C family is used heavily
since it has a small footprint (C fits on microcontrollers) and is very
efficient. C++ provides the object oriented approach to a code base and
is widely adopted in industry. Recent languages like Java and C# are
popular when the robot has a full computer available as a controller.
One can even find older languages like BASIC and FORTH as well. In this
text we will focus on two: C/C++ and Python.

Why C/C++ and Python? C is the major language for embedded systems. It
can compile down to very compact code to run on a variety of
microprocessors. C++ is the object oriented extension to C and both
remain in the most popular programming language lists even though they
have been around for some time. Python is an object oriented scripting
language and is very popular as well, especially with regards to
programming education. Another simple reason is that these are the two
languages supported by ROS, the Robot Operating System. The bulk of the
examples in this text are written in Python.

Also, for this text, we will assume that you are running a relatively
current version of Ubuntu (possibly in a virtual machine). Python, C and
C++ are part of the standard Ubuntu distribution. Normally it is not
critical which version of Python is used (Python 2 versus 3).
