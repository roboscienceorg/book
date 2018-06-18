
Getting Started
----------------

:index:`Veranda` is the simulation application we will use to introduce basic
concepts in robotics simulation.   The ROS community has used
Stage and Gazebo.  Stage is no longer supported and one must use either STDR or Veranda.
a ROS based two dimensional physics simulator. Gazebo will be discusssed
later in this text.  Veranda uses the physics
engine, Box2D, to determine both motion and interactions (collisions).  Essentially
this is a 2D game engine and the game players are robots.

Veranda is very general in scope.  Robots are a collection of masses which are
connected by joints and are subject to forces.   Forces are contolled by
the user through external programs.   All of the communication between the
collection of programs is done using ROS messages.

To install Veranda, goto to Roboscience.org.  Under software, click on *read more*
and you will see the link for Veranda.  Follow the link and then follow the
instructions on the page.   You will download the installer and it will
download the application for you.  It will then setup the paths and the
environment variables.

.. _`fig:veranda0`:
.. figure:: SimulationFigures/Veranda0.png
   :width: 80%
   :align: center

   Veranda at launch

To load a prebuilt robot, click on the folder symbol in the panel under simulator
tools and select one of the Differential Drive robots in the Veranda/Robots subdirectory.
Click on the plus symbol under simulator tools to place this in the simulation
world.   You can zoom in and out using the "q" and "e" keys.  You can start
the simulation by clicking on the run icon in the simulation panel.

.. _`fig:veranda1`:
.. figure:: SimulationFigures/Veranda1.png
   :width: 80%
   :align: center

   Differential Robot loaded.

In the Veranda/Scripts directory, you will find some example programs to
drive the robot.   The first step is to source the setup file:

::

   cd <veranda directory>
   source setup.bash
   python3 Scripts/fig8_differential.py

This should drive the robot in a figure 8 shaped path.   You will see other
examples in the directory.  First we will run the commands in the interpreter
*by hand*.   

::

   import rclpy
   from rclpy.node import Node
   from veranda.SimTimer import SimTimer
   from std_msgs.msg import Float32
   import math

   rclpy.init()
   node = Node("talker")
   publeft = node.create_publisher(Float32, 'robot0/left_wheel')
   pubright = node.create_publisher(Float32, 'robot0/right_wheel')

   msg = Float32()
   msg.data = 5.0
   publeft.publish(msg)
   pubright.publish(msg)

This will move the robot.  Note that the behavior of the simulator is to
keep the motors running on the last received wheel commands.   So in the
code above, the robot will continue to drive.  You will need to set msg.data
to zero to stop the bot.

Joystick

To drive a predetermined path, a precise sequence of commands must be sent.


Simple routing example  [more in navigation chapter]
