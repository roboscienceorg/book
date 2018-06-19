
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


Running the Demos
------------------

The Veranda project comes with a couple of demo robots and control scripts pre-made. The robots are specifically configured to work with the control scripts; you should be able to load the demo robots right into the simulation, start the script, press play on the simulation, and watch them go. Robots and Scripts can be found in the ``Demo`` folder of the Veranda repository.

The Demo Robots
~~~~~~~~~~~~~~~~~~

There are three robots included with the project:
    * Differential-w-GPS

        This is the classic turtle robot. It is a circular body with two wheels, which can be controlled as a differential drive system. The robot has a GPS attached to it, and will publish its absolute location in the simulation.

        Input Topics

        ==================  ========  ==========================================================
        Topic               Datatype  Purpose
        ==================  ========  ==========================================================
        robot0/left_wheel   Float32   Sets the velocity of the left wheel in radians per second
        robot0/right_wheel  Float32   Sets the velocity of the right wheel in radians per second
        ==================  ========  ==========================================================

        Output Topics

        =============  ========  ========================================================
        Topic          Datatype  Purpose
        =============  ========  ========================================================
        robot0/pose2d  Pose2D    Reports the current X, Y, Theta positioning of the robot
        =============  ========  ========================================================

    * Differential-w-Lidar-Touch

        This robot is the classic turtle robot with a twist; attached to the robot are a 360-degree Lidar sensor and a Bump sensor which will detect contact with any part of the robot body.

        Input Topics

        ==================  ========  ==========================================================
        Topic               Datatype  Purpose
        ==================  ========  ==========================================================
        robot0/left_wheel   Float32   Sets the velocity of the left wheel in radians per second
        robot0/right_wheel  Float32   Sets the velocity of the right wheel in radians per second
        ==================  ========  ==========================================================

        Output Topics

        ==============  ==============  ============================================================
        Topic           Datatype        Purpose
        ==============  ==============  ============================================================
        robot0/laser    LaserScan       Reports what is seen by the robot's Lidar sensor
        robot0/touches  ByteMultiArray  Reports the state of all the buttons spaced around the robot
        ==============  ==============  ============================================================

    * Ackermann-w-Lidar

        This robot is like a car! It has a rectangular base with 4 wheels; the back wheels are fixed and produce thrust, the front wheels cannot produce thrust, but are used to steer. The front wheels turn following the Ackermann constraint.
        This robot also has a lidar on it, but it only covers a 90-degree area in front of the robot.

        Input Topics

        ==================  ========  ===============================================================
        Topic               Datatype  Purpose
        ==================  ========  ===============================================================
        robot1/left_wheel   Float32   Sets the velocity of the left rear wheel in radians per second
        robot1/right_wheel  Float32   Sets the velocity of the right rear wheel in radians per second
        robot1/steer        Float32   Sets the angle in radians for the robot to steer towards
        ==================  ========  ===============================================================

        Output Topics

        ==============  ==============  ============================================================
        Topic           Datatype        Purpose
        ==============  ==============  ============================================================
        robot1/laser    LaserScan       Reports what is seen by the robot's Lidar sensor
        ==============  ==============  ============================================================

The Demo Scripts
~~~~~~~~~~~~~~~~~~~

The Demo/Scripts folder contains 6 different python scripts which can be run with the default robots. All of them can be run with the command ``python3 [scriptname]``, but some of them require the command line arguments described below. Remember to :ref:`source <sec-sourcing>` ROS2 and the Veranda workspace before running these scripts!

    * ``fig8_differential.py``

        Usage: ``python3 fig8_differential.py``

        Publishes left and right wheel velocities which will drive a robot in a figure-8 shape. It uses the topics ``robot0/left_wheel`` and ``robot0/right_wheel``. As you might expect, it can be used to control either of the demo differential drive robots. If multiple of them are in the simulation, they will all be controlled.

    * ``joystick_differential.py``

        Usage: ``python3 joystick_differential.py {input-topic} [output-topic]``

        Listens for messages from a 2-axis joystick and publishes left/right wheel commands on the topics ``[output-topic]/left_wheel`` and ``[output-topic]/right_wheel`` to drive a differential robot. The topic listened to for the joystick is ``[input-topic]/joystick``. If no output topic is given, it will be the same as the input topic. (For example, if the ``[input-topic]`` were 'banana', then the topics used would ``banana/joystick``, ``banana/left_wheel``, and ``banana/right_wheel``.

    * ``joystick_ackermann.py``

        Usage: ``python3 joystick_ackermann.py {input-topic} [output-topic]``

        Similarly to ``joystick_differential.py``, this script listens for messages from a 2-axis joystick; but it uses them to produce messages on three topics, in order to drive the Ackermann-w-lidar demo robot. The rules for determining the topic names are the same as for the differential joystick script; but there is an extra topic: ``[output-topic]/steer`` to control the steering wheels.

    * ``joystick_listener.py``

        Usage: ``python3 joystick_listener.py {topic}``

        Listens to the specified topic for joystick messages and writes the joystick information to stdout. It is similar to using the ``ros2 topic echo`` command, and is a good example of how to listen for joystick messages in a script.

    * ``lidar_listener.py``

        Usage: ``python3 lidar_listener.py {topic}``

        Listens to the specified topic for LaserScan messages and writes the lidar information to stdout. It is similar to using the ``ros2 topic echo`` command, and is a good example of how to listen for lidar messages in a script.

    * ``linux_joy_reader.py``

        Usage: ``python3 linux_joy_reader.py [device]``

        This script is only usable on Linux systems! It uses the plug-n-play joystick drivers available in the OS to listen for input from hardware joysticks. It has only been tested with Playstation controllers plugged into a USB port.

        If the script is run with no arguments, it will look through the available devices and print a list of all of the ones which are joysticks (they start with 'js'). When the script is run with one of these devices as its argument, it will listen to the input from that device and publish joystick messages to the topic ``[device]/joystick``.

Enough Talking, Lets Do Some Demos!
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Here we've outlined a number of demos that you can run right after installing Veranda. Each one will require that you have multiple command line terminals open, and will number them 1-n. The first time you encounter a terminal number, you should ``cd`` into the Veranda workspace and :ref:`source <sec-sourcing>` both ROS2 and the Veranda workspace immediately before continuing the demo. All commands are given in terms of Linux, so you may need to make adjustments!

Demo 1: Driving GPS Turtle in a Figure-8
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    * Terminal 1: ``ros2 run veranda veranda``
    * Load the Differential-w-GPS robot into the toolbox
    * Add that robot to the simulation
    * Start the simulation
    * Terminal 2: ``python3 ./src/veranda/veranda/Demo/Scripts/fig8_differential.py``
    * Bask in the glory of your achievement

Demo 2: Driving Lidar Turtle with the Virtual Joystick
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    * Terminal 1: ``ros2 run veranda veranda``
    * Load the Differential-w-Lidar-Touch robot into the toolbox
    * Add that robot to the simulation
    * Start the simulation
    * Create a Virtual Joystick
    * Set the joystick topic to ``demo/joystick``
    * Terminal 2: ``python3 ./src/veranda/veranda/Demo/Scripts/joystick_differential.py demo robot0``
    * Use the virtual joystick to drive the robot
    * Take over the world


Demo 3: Driving Ackermann Bot with the Virtual Joystick
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    * Terminal 1: ``ros2 run veranda veranda``
    * Load the Ackermann-w-Lidar robot into the toolbox
    * Add that robot to the simulation
    * Start the simulation
    * Create a Virtual Joystick
    * Set the joystick topic to ``demo/joystick``
    * Terminal 2: ``python3 ./src/veranda/veranda/Demo/Scripts/joystick_ackermann.py demo robot1``
    * Use the virtual joystick to drive the robot
    * Steal the moon

Bonus Demo! Driving Ackermann Bot with a Real, Live Joystick
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    * Acquire a USB joystick controller
    * Plug that joystick into your Linux machine
    * Terminal 1: ``python3 ./src/veranda/veranda/Demo/Scripts/linux_joy_reader.py``
    * Terminal 1: ``python3 ./src/veranda/veranda/Demo/Scripts/linux_joy_reader.py [insert device here]``
    * Try devices until one works, and the terminal prints stuff when you move the joystick
    * Terminal 2: ``python3 ./src/veranda/veranda/Demo/Scripts/joystick_ackermann.py [device] robot1``
    * Terminal 3: ``ros2 run veranda veranda``
    * Load the Ackermann-w-Lidar robot into the toolbox
    * Add that robot to the simulation
    * Start the simulation
    * Use the joystick to drive the robot
    * Step 3: Profit


Before we do more complicated motion planning, it is important to get a feel
for how the simulations are done and to do a few computations directly.  This
helps the roboticist understand the errors and limitations of the simulations.
