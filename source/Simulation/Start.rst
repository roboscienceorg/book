
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

To install Veranda, goto to Roboscience.org, follow the software links.

Load robot

Run a couple of commands - we can fire up the python interpreter and run
commands.

Joystick

Predetermined path program

Simple routing example  [more in navigation chapter]
