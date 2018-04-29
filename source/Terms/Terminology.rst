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

-

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
cylindrical, helical, universal and spherical
joints \ `[fig:robotjoints] <#fig:robotjoints>`__. Linear joints are
also referred to as prismatic joints.


.. figure:: TermsFigures/robotjoints.svg
   :width: 70%
   :align: center

   Some common robot joints.[fig:robotjoints]

All of the machines we will study have moving components. The complexity
of the system depends on the number of components and the
interconnections therein. For example, a robotic arm may have three or
four joints that can be moved or varied. A vehicle can have
independently rotated wheels. The number of independently moving
components is referred to as the *degrees of freedom*; the number of
actuators that can induce unique configurations in the system. This
mathematical concept comes from the number of independent variables in
the system. It gives a measure of complexity. Higher degrees of freedom,
just as higher dimensions in an equation, indicate a system of higher
complexity. This concept of degrees of freedom is best understood from
examples.


Consider a computer-controlled router that can move the tool head freely
in the :math:`x` and :math:`y` directions. This device has *two degrees
of freedom*. It is like a point in the plane which has two parameters to
describe it. Going one step further, consider a 3D printer. These
devices can move the extruder head back and forth in the plane like the
router, but can also move up and down (in :math:`z`). With this we see
three degrees of motion or freedom. While it may seem from these two
examples that the degrees of freedom come from the physical dimensions,
please note that this is not the case. Consider the 3D printer again. If
we added a rotating extruder head, the degrees of freedom would equal to
four (or more, depending on setup), but the physical dimensions would
stay at three.



Consider a welder that can position its tool head at any point in a
three dimensional space. This implies three degrees of freedom. We
continue and assume that this welder must be able to position its tool
head orthogonal to the surface of any object it works on. This means the
tool must be able to rotate around in space - basically pan and tilt.
This is two degrees of freedom. Now if we attach the rotating tool head
to the welder, we have five degrees of freedom: 5DOF.

Each joint in a robotic arm typically generates a degree of freedom. To
access any point in space from any angle requires five degrees of
freedom (:math:`x,y,z,pan,tilt`). So why would we need more? Additional
degrees of freedom add flexibility when there are obstacles or
constraints in the system. Consider the human arm. The shoulder rotates
with two degrees of freedom. The elbow is a single degree of freedom.
The wrist can rotate (the twisting in the forearm) as well as limited
two degree motion down in the wrist. Thus the wrist can claim three
degrees of freedom. Without the hand, the arm has six degrees of
freedom. So you can approach an object with your hand from many
different directions. You can drive in a screw from any position.
