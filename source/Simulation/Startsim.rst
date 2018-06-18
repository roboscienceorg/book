
Simulation of a Differential Drive Robot
-----------------------------------------

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
to zero to stop the bot.  This is one common way that motor control systems
will operate.  This is a design decision which will affect the way a
vehicle operates when communications are disrupted.  Having a motor control
system that automatically slows the vehicle down after some time interval
if no communciations have been received is another way to design  the system.

Joystick (fill in details)

To drive a predetermined path, a precise sequence of commands must be sent.
Relative to the motion and timescale of a robot (real or correctly simulated)
the Python commands arrive very quicky.  Delays need to be added to insure
that certain movements have time to complete.  One can code the logic
directly as:

::

   msgl.data = left_speed ; msgr.data = right_speed
   publeft.publish(msgl); pubright.publish(msgr)
   sleep(val)

   msgl.data = left_speed ; msgr.data = right_speed
   publeft.publish(msgl); pubright.publish(msgr)
   sleep(val)

   ...

   msgl.data = left_speed ; msgr.data = right_speed
   publeft.publish(msgl); pubright.publish(msgr)
   sleep(val)

The left, right wheel speeds and time delay values can be made into arrays.
Then the preceeding commands can be called in a simple loop:

::

   for i in range(n):
      msg.data = left_speed[i]
      publeft.publish(msg[i])
      msg.data = right_speed[i]
      pubright.publish(msg[i])
      sleep(val[i])

To use the code above to drive a simple shape like a square that is
composed of line segments, you need to practice with
the timing to get the distances and angles set correctly.
To travel straight, you set the two wheel speeds equal.  One can figure
the delay time out from the differential drive formulas, but for here we
will assume it is done experimentally.   The harder part is the turns.
To turn in place (differential drive robots can pivot in place),
you will set the wheel speeds with opposite sign (same magnitude).

In a real robot, the variations in the hardware and environment will cause
the robot to drift over time.  This accumulates and at some point the error
can get large enough to render the system useless.  This will be addressed
when we discuss using sensor feedback to correct motion.

We will end the section with the figure 8 example code.  This example contains
many of the ideas discussed in the text so far.
::

   import rclpy
   from rclpy.node import Node
   from veranda.SimTimer import SimTimer
   from std_msgs.msg import Float32
   import math


   # Robot parameters
   R = 0.75
   L = 1.5

   # Location Functions to form a figure 8 of the necessary size
   def x_t(t):
       return 12.0*math.sin(t)

   def y_t(t):
       return 6.0*math.sin(2*t)

   # Differential drive inverse kinematics
   def DD_IK(x_t, y_t, t):
       # Calculate xdot and xdotdot at current time
       x_dot_t = 12*math.cos(t)
       x_dotdot_t = -12*math.sin(t)

       # Calculate ydot and ydot dot at current time
       y_dot_t = 12*math.cos(2*t)
       y_dotdot_t = -24*math.sin(2*t)

       #Calculate phi1, phi2
       v = math.sqrt(x_dot_t * x_dot_t + y_dot_t * y_dot_t)
       k = (x_dot_t * y_dotdot_t - y_dot_t * x_dotdot_t)/(v*v*v)

       phi1 = v/R*(k*L+1)
       phi2 = v/R*(-k*L + 1)

       return (phi1, phi2)


   # Publishes a set of wheel velocities
   # in the format required by the STDR
   def publishWheelVelocity(publeft, pubright, phi1, phi2):
       msg = Float32()
       msg.data = phi1
       publeft.publish(msg)
       msg.data = phi2
       pubright.publish(msg)


   def main():
       rclpy.init()
       node = Node("talker")
       publeft = node.create_publisher(Float32, 'robot0/left_wheel')
       pubright = node.create_publisher(Float32, 'robot0/right_wheel')
       simTime = SimTimer(True, "veranda/timestamp", node)

       # Factor to scale down speed by
       speedScale = 1

       # Tick time at 10 hz
       dt = 0.1

       def cb():
           # Calculate wheel velocities for current time
           phi1, phi2 = DD_IK(x_t, y_t, simTime.global_time() + 2*math.pi)
           print(phi1, phi2)
           # Publish velocities
           publishWheelVelocity(publeft, pubright, phi1, phi2)

       simTime.create_timer(dt, cb)
       rclpy.spin(node)
       node.destroy_node()
       rclpy.shutdown()

   if __name__ == '__main__':
       main()

Before we do more complicated motion planning, it is important to get a feel
for how the simulations are done and to do a few computations directly.  This
helps the roboticist understand the errors and limitations of the simulations.
