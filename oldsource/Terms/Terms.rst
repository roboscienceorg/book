Terminology
-----------

In the Introduction, several terms were introduced such as end effector
or actuator. We will round out the common robotics terminology in the
section.

-  :index:`Manipulator`: the movable part of the robot, often this is the
   robotic arm.

-  :index:`Degrees of Freedom`: the number of independently adjustable or
   controllable elements in the robot. It is also the number of
   parameters that are needed to describe the physical state of the
   robot such as positions, angles and velocities.

-  :index:`End Effector`: the end of the manipulator.

-  :index:`Payload`: the amount the robot can manipulate or lift.

-  :index:`Actuator`: the motor, servo or other device which translates
   commands into motion.

-  :index:`Speed`: the linear or angular speed that a robot can achieve.

-  :index:`Accuracy`: how closely a robot can achieve its desired position.

-  :index:`Resolution`: the numerical precision of the device, usually with
   respect to the end-effector. This can also be measured in terms of
   repeatability. Related to accuracy vs precision in general
   measurements.

-  :index:`Sensor`: any device that takes in environmental information and
   translates it to a signal for the computer such as cameras, switches,
   ultrasonic ranges, etc.

-  *Controller*: can refer to the hardware or software system that
   provides low level control of a physical device (mostly meaning
   positioning control), but may also refer to the robot control
   overall.

-  :index:`Processor`: the cpu that controls the system. There may be multiple
   cpus and controllers or just one unit overall.

-  :index:`Software`: all of the code required to make the system operate.

-  :index:`Open Loop control`: a form of robot control that does not use feedback
   and relies on timed loops for placement.

-  :index:`Closed Loop control`: using sensor feedback to improve the control
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
essentially indicates what it does. A :index:`rotary actuator` allows the
relative angle between the links to change. A :index:`linear actuator` changes
the length of a link. Examples of rotary joints are :index:`revolute`,
:index:`cylindrical`, :index:`helical`, :index:`universal` and :index:`spherical`
joints :numref:`fig:robotjoints`. Linear joints are
also referred to as prismatic joints.

.. _`fig:robotjoints`:
.. figure:: TermsFigures/robotjoints.*
   :width: 70%
   :align: center

   Some common robot joints.

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



Serial and Parallel Chain Manipulators
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Manufacturing robots typically work in a predefined and restricted
space. They usually have very precise proprioception (the knowledge of
relative position and forces) within the space. It is common to name the
design class after the coordinate system which the robot naturally
operates in. For example, a cartesian design (similar to gantry systems)
is found with many mills and routers, heavy lift systems, 3D Printers
and so forth, see :numref:`gantrysample`-left.
Actuation occurs in the coordinate directions and is described by
variable length linear segments (links) or variable positioning along a
segment. This greatly simplifies the mathematical model of the machine
and allows efficient computation of machine configurations.

In two dimensions, one can rotate a linear actuator about a common
center producing a radial design which would use a polar coordinate
description. Adding a linear actuator on the :math:`z` axis gives a
cylindrical coordinate description,
:numref:`gantrysample`-right


.. _`gantrysample`:
.. figure:: TermsFigures/cart_cyli.png
   :width: 90%
   :align: center

   Basic designs.  (left) Cartesian design - Muhammad Furqan, grabcad.com
   (right) Cylindrical design -  Mark Dunn,  grabcad.com


A :index:`serial chain manipulator` is a common design in industrial
robots. It is built as a sequence of links connect by actuated joints
(normally seen as a sequence starting from an attached base and
terminating at the end-effector. By relating the links to segments and
joints as nodes, we see that serial link manipulators can be seen as
graphs with no loops or cycles. The classical robot arm is an example of
a serial chain manipulator, :numref:`serialparallel`-(left).
Robot arms normally employ fixed length links and use rotary joints.
This are often called articulated robots or the arm is called an
articulator. Very general tools exist to construct mathematical
descriptions of arm configuration as a function of joint angles. A
formalism developed by Denavit and Hartenberg can be used to obtain the
equations for position.

.. _`serialparallel`:
.. figure:: TermsFigures/serialparallel.png
   :width: 90%
   :align: center

   Robot arms(left) Articulated -   Ivo Jardim,  grabcad.com,
   (right) Delta Design -  Ivan Volpe,  grabcad.com



.. _`fig:stewart`:
.. figure:: TermsFigures/Stewart.jpg
   :width: 50%
   :align: center

   Stewart Platform -  Micheal Meng, grabcad.com

Another popular approach is the :index:`parallel chain manipulator`, which uses
multiple serial chains to control the end-effector. A couple of examples,
a Delta Robot, seen in :numref:`serialparallel`-(right) and :numref:`fig:stewart`.
Combinations of articulators can built to mimic a human hand as seen in
:numref:`armsample-c`.

.. _`armsample-c`:
.. figure:: TermsFigures/Robotarm.jpg
   :width: 70%
   :align: center

   Articulated with hand gripper - Chris Christofferson,  grabcad.com


Basic Machine Elements
~~~~~~~~~~~~~~~~~~~~~~

We have been designing and using machines for thousands of years.  There
is a wealth of very interesting designs to do a myriad of things.  We will
review a few common designs in terms of basic function.

Sources of force in a robot arise from elecromagnetic devices such as DC
motors or chemical processes such as internal combustion.
The force produced is normally not in correct form for use on the
robot.  It could be in the wrong direction, speed, magnitude of force, etc and
needs to be changed.  This is where gears, joints, rods and other essential
components enter the design.

In robotics we see many elecrically powered systems.   The main method for converting
electrical power into mechanical power is to use a rotational motor design.  For
driving wheels, propellors, or other devices that use rotation energy this works
very well.  Getting linear motion requires some additional components.  Although
direct linear motion is possible through a solenoid type design, it is very common
to find a electric motor and some type of gearing system to create the linear
motion.

Gears are rotating elements with cut teeth that mesh with each other.  They are
capable of transmission of power and can change speed, torque or direction.
Gears can be categorized according to relative relation of their rotation
axes: parallel axes, intersecting axes, nonparallel-nonintersecting, and
other.

Parallel axes contain spur gears, helical gears, internal gears and gear rack designs.  Intersecting
axes include straight and spiral bevel gears, miter gears.  Nonparallel-nonintersecting
gears have worm gears designs and screw gear designs, :numref:`fig:typesofgears`.

.. _`fig:typesofgears`:
.. figure:: TermsFigures/typesofgears.jpg
   :width: 99%
   :align: center

   Sample of different gear designs.

Beyond axes, the way the teeth fall on the gear are important.  An external design
is where the teeth lie on the outside (outer surface) of the gear and internal gears are ones
where the teeth lie on the inside.   Internal gears are nice in that they don't
reverse the shaft rotation direction.   The teeth
can run parallel to the rotation axis such as seen in spur gears or straight cut
gears.  These are the simpliest designs and only work with parallel axes.
Helical gears use teeth that are not parallel to the rotation axis.

In spur designs, the teeth mesh with mostly static contact points and the engagement
is all at once.  Spur gears can be noisy at high rotational speeds.
Helical or spiral designs the teeth engage more gradually and also slide against
each other.  This produces less noise or vibrations at the cost of
higher energy loss and heat production.

Bevel gear designs (especially spiral) are used in differentials to transmit
driveshaft rotation 90 degrees to axle rotation.   Rack and pinon systems
can be used to transmit the rotational motion of a steering wheel to the linear
motion of a rod used to change the wheel angle (steering).  Worm gear designs
can provide significant torque as well as having the property of self-locking
which means they are stationary when no power is applied to the worm gear (the
worm wheel cannot drive the mechanism in reverse).

Most gear designs change angle by 90 degrees (although bevel angles and teeth
designs can work at other angles).  However the angle is fixed for the specific
gear.  If thesee angles are not fixed, which happens when suspension systems
and steering are employed, then one needs joints that can address variable
angles.  Universal and flexible joints are used to allow for variable changes
in rotation axes.

.. _`jointsample`:
.. figure:: TermsFigures/joints.png
   :width: 70%
   :align: center

   Joint examples.  (Left) Universal Joint - Devin Dyke,  grabcad.com, (Right)
   Flexible joint - Chintan (CK) Patel,  grabcad.com

Changing rotational motion to linear motion is important and for us arises often
in manufacturing robots such as CNC machines and 3D Printers.  A simple system
is just a threaded rod and nut design (which is can be seen as a variation of the
worm gear concept).   By spinning the rod and not allowing the
nut to spin, the nut will move up or down the rod.  The relative motion
is determined by the thread pitch.    Because all of the threads
of the nut are engaged, there can be considerable friction.

An improvement over a threaded rod is the ball screw, :numref:`fig:ballscrew` .
The idea is to add small
balls (bearings) between the threads (or teeth if thinking of this like a
worm gear).  This reduces friction and backlash as well as increases accuracy.
Using recirculating balls (in a wormdrive design) is how early automobiles
provided smooth blacklash free lower force steering.

.. _`fig:ballscrew`:
.. figure:: TermsFigures/BallScrews-with-detail-insets.jpg
   :width: 70%
   :align: center

   Ball Screw -  Glenn McKechnie, June 2006, Wikipedia

Other common methods to transfer rotational motion are sprocket and chains.
Using different sizes of sprockets attached via the chain provides changes
in angular speed and torque, called gearing based on the direct analogy to
gears.  The chains can be attached to tracks and a track or tank drive system is
produced.   A toothed belt is a variation of this system and is found from
timing belts and Gilmer belts to the head positioning belts for 3D printers.
