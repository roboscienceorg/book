.. role:: math(raw)
   :format: html latex
..

.. role:: raw-latex(raw)
   :format: latex
..

Terms and Basic Concepts of Robotics[Chap:Terms]
================================================

Getting the language down is the first step. Robotics is like any other
engineering field with lots of jargon and specialized terms. The terms
do convey important concepts which we will introduce here.

Terminology
-----------

In the Introduction, several terms were introduced such as end effector
or actuator. We will round out the common robotics terminology in the
section.

-  *Manipulator*: the movable part of the robot, often this is the
   robotic arm.

-  *Degrees of Freedom*: the number of independently adjustable or
   controllable elements in the robot. It is also the number of
   parameters that are needed to describe the physical state of the
   robot such as positions, angles and velocities.

-  *End Effector*: the end of the manipulator.

-  *Payload*: the amount the robot can manipulate or lift.

-  *Actuator*: the motor, servo or other device which translates
   commands into motion.

-  *Speed*: the linear or angular speed that a robot can achieve.

-  *Accuracy*: how closely a robot can achieve its desired position.

-  *Resolution*: the numerical precision of the device, usually with
   respect to the end-effector. This can also be measured in terms of
   repeatability. Related to accuracy vs precision in general
   measurements.

-  *Sensor*: any device that takes in environmental information and
   translates it to a signal for the computer such as cameras, switches,
   ultrasonic ranges, etc.

-  *Controller*: can refer to the hardware or software system that
   provides low level control of a physical device (mostly meaning
   positioning control), but may also refer to the robot control
   overall.

-  *Processor*: the cpu that controls the system. There may be multiple
   cpus and controllers or just one unit overall.

-  *Software*: all of the code required to make the system operate.

-  *Open Loop control*: preprogrammed actions performed without
   feedback.

-  *Closed Loop control*: using sensor feedback to improve the control
   accuracy.

Motion is achieved by some device that converts some energy source into
motion. Most often these are electric motors (even non-electric systems
often have electrically controlled components like valves.) However, it
is useful to not focus on the type of equipment, but just the type
motion induced. For simplicity anything that induces motion will be
called actuators for most of the text. Actuators apply forces to the
various robotic components in the system which in turn generates motion.
The connections between actuators are known as *links*. For this work we
will assume they are rigid and fixed in size. Connecting links are
joints. This allows the links to move with respect to each other. There
are two types of common joints, *rotary* and *linear* joints. The name
essentially indicates what it does. A rotary actuator allows the
relative angle between the links to change. A linear actuator changes
the length of a link. Examples of rotary joints are revolute,
cylindrical, helical, universal and spherical joints [fig:robotjoints].
Linear joints are also referred to as prismatic joints.
