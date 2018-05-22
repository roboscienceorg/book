
Locomotion
----------

The word locomotion means “The act of moving from place to
place.” :cite:`dictionary` from Latin combining location and
motion. Biology has explored a variety of very interesting ways to move
around. Animals can be carried on currents, swim, crawl, slide, walk,
run, jump, and fly.

Most of the locomotion solutions found in nature are hard to imitate
with machines. Although legs are the base for human locomotion, humans
have valued wheels in their motion solutions. The earliest recorded
appearance of wheels is in Mesopotamia (Sumerian) at the mid-4th
millenium BC. The is evidence of independent discovery in the new world,
the lack of domesticated large animals prevented any development beyond
children’s toys. :cite:`wiki:wheel`.

Rolling is very efficient, especially compared to dragging or carrying
materials. At the macroscopic scale, nature has not developed wheels.
This is not surprising since the wheel needs to be disconnected from the
rest of the system for free rotation reasons, but would then not have
the required nutrient supply (blood or something similar) to grow,
develop and maintain the structure. Although nature did not evolve large
wheels, human motion has some similarities to rolling - a rolling
polygon when motion models are examined.

We are no longer bound to wheels as the only choice for motion. We can
implement a number of nature inspired motion solutions. The type of
motion used by a robot is often the most notable aspect of the machine.
Certainly a great deal of interest and entertainment can be found in
implementing novel locomotion into a robot.

As a robotics engineer, you may be asked to choose the type of motion,
meaning you must choose “fly, hop, swim, walk, roll ...” Often the
environment decides for you, for example, if you must operate in the air
or water, or in very rough terrain. You may have other constraints
involved like power consumption, weight or robustness. These constraints
will normally push the design towards one locomotion system.

When looking at a wheel verses an articulator there are some standard
issues that must be addressed. Articulation is much more complicated in
design, control and expense (both energy and financial). Legs
(articulators) require more actuators and thus more control components.
The control system is more complicated than with a wheeled design.
Another consideration is energy. Articulators move their center of mass
for locomotion and may have to move a significant amount of hardware.
Thus there is internal mass movement along with the external mass (the
vehicle). Wheels by their very design keep the center of mass at a
constant distance from the ground. This reduces power usage.

The tradeoff for the efficiency gain is that articulated motion has the
possibility of enhanced environmental robustness. This is clear when
watching any household spider run across a textured ceiling. The most
efficient wheel, the rail wheel is also the least robust in that it does
not operated outside an instrumented environment. Cars use wheels that
can run on a road or flat ground. To go beyond this we need to bring the
rail or road along with - the idea behind tracks. Track systems can
operate in a larger set of environments, but at cost of energy.

.. _`motionenergyspeed`:
.. figure:: MotionFigures/energyusegraph.*
   :width: 60%
   :align: center

   The relations between energy, speed and motion
   type.



Wheels
------

For thousands of years wheels have worked very well. Most vehicles we
imagine are two or four wheeled systems. Two wheels use the gyroscopic
effect to provide stability. Static (passive) stability of the vehicle
is assured by having three wheels and the center of gravity in the
triangle formed by the ground contact points of the wheels. Trikes are
less popular on the road due to concerns about instabilities during
turning. Additional wheels, additional ground contact points, can
improve the stability. Four wheels provides the stability in the turn at
the cost of needing a suspension system (more than three require
suspension). Suspension systems do more than level the ride as they can
keep all the wheels on the ground when traveling over rough terrain.
This provides better traction as well as avoids digging in too deeply.
Larger wheels give greater obstacle traversal due to the decreased angle
of attack which reduces the required torque. Bigger wheels are heavier
and require greater reductions in the gear box however. Selection of
wheels is based on the surface and the application. Hard dry smooth
surfaces may use smooth wheels and rougher or slicker surfaces demand
tires that are rough and maybe soft.

The effectiveness of the wheel is given by the contact area of the
wheel. This is a combination of wheel width and tread design. Angle of
contact combined with tire shape will affect the steering response.
Wheel tread, width and other parameters will affect the rolling friction
and the energy loss in motion. To gain maneuverability, wheels can be
steered or replaced with omni-wheels. This requires additional hardware
and controls which increases complexity, weight and cost. Most designs
do not allow the craft to maneuver and orient simultaneously and
independently which increases the control effort. As with many aspects
of engineering, this is a tradeoff between simple, robust and
inexpensive design verses a flexible, maneuverable, adaptable design.

Omni, Mecanum and Spherical Wheels
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Of all the wheel types, none captures attention like the omni and
Mecanum wheels. Their operation is unexpected and at first seems to defy
intuition. These wheels are capable of motion in the rolling direction
as well as motion along the axle direction which leads to holonomic
robots. Which means the robot can position and orient independently in
the plane. It makes for a very maneuverable robot which is very popular.
These wheels require hard flat surfaces to work properly. If the dirt or
small stones get lodged into the rollers or if the rollers lose contact
with the surface, the holonomic motion is compromised. So these wheels
are used exclusively indoors. Since fine precision maneuvering is
normally required for indoor systems and not outdoor systems, there has
not been much effort expended to make outdoor versions.

.. _`fig:airtrax`:
.. figure:: MotionFigures/airtrax.jpg
   :width: 40%
   :align: center

   The Airtrax forklift.

.. _`fig:airtraxcobra`:
.. figure:: MotionFigures/airtraxcobra.jpg
   :width: 40%
   :align: center

   The Airtrax scissor lift.

The omni wheel’s first patent was in 1919 by Grabowiecki. The Mecanum
wheel was developed by Bengt Erland Ilon in 1972 while working for the
Mecanum company. Airtrax, an American forklift company purchased patent
rights and briefly manufactured forklifts with a heavy duty version of
the Mecanum wheel. These wheels have much less ground friction in a turn
in comparison to a skid steer requiring much less torque.

.. _`gammavarconfig`:
.. figure:: MotionFigures/swedish_angle.*
   :width: 20%
   :align: center

   The :math:`\gamma` measure.

.. _`gammaconfig`:
.. figure:: MotionFigures/omni_mecanum-wheel.png
   :width: 75%
   :align: center

   The (a) :math:`\gamma = 0` configuration
   and (b) :math:`\gamma = 45^\circ` configuration.

For this text, we will combine the omni and Mecanum wheels and just call
them omniwheels. The difference between them is only in the angle the
rollers are mounted on the wheel body.
:numref:`gammaconfig` shows some sample types of
omniwheels using the :math:`\gamma = 0` configuration and
:math:`\gamma = 45^\circ` configuration. Normally the :math:`\gamma=0`
style of wheel is used in non-parallel mounting as shown in the first
robot in the :numref:`gammawheelmounting` and
the parallel mounting is used for the other standard type of wheel
design using :math:`\gamma = 45^\circ`.

.. _`gammawheelmounting`:
.. figure:: MotionFigures/swedish_config.*
   :width: 40%
   :align: center

   Normal mounting style for :math:`\gamma = 0` and
   :math:`\gamma = 45^\circ`.

.. _`meccanumwheelvectors`:
.. figure:: MotionFigures/swedish_mount.*
   :width: 40%
   :align: center

   Force vectors induced by rotation with the :math:`\gamma = 45^\circ`
   configuration.


.. _`meccanumwheelmotion`:
.. figure:: MotionFigures/swedish_mount2.*
   :width: 60%
   :align: center

   Mecanum rotation directions and vector forces for different vehicle
   directions.

.. _`meccanumwheelmotion2`:
.. figure:: MotionFigures/swedish_mount3.*
   :width: 60%
   :align: center

   Summary of wheel motion and directions



-  Driving forward: all four wheels forward

-  Driving backward: all four wheels backward

-  Driving left: 1,4 backwards; 2,3 forward

-  Driving right: 1,4 forward; 2,3 backward

-  Turning clockwise: 1,3 forward; 2,4 backward

-  Turning counterclockwise: 1,3 backward; 2,4 forward


A variation of the omni wheel is the omni ball developed by Kaneko
Higashimori Lab at Osaka University,
see :numref:`fig:omniball`. This wheel will be used to
drive tracks in a very novel approach described in the tracks section
below. This wheel fails to be a true spherical wheel as far as two
directional motion is concerned and has motion equations similar to the
omniwheel systems.

.. _`fig:omniball`:
.. figure:: MotionFigures/omni-ball.jpg
   :width: 60%
   :align: center

   The Omni Ball Wheel developed at the Kaneko Higashimori Lab at Osaka
   University

Omni and Mecanum wheels can be driven on only one direction and only
when combined with other wheels are they able to move against the
rolling directions. To gain two dimensional directional capability the
wheel needs to be a sphere or at least approximate the sphere in a
significant manner. This can be done by reversing the power direction
from the classical mechanical computer mouse. In the mechanical mouse
the ball is forced around which drives small disks inside in the
component directions. By mounting three omniwheels on top of a ball, one
can gain motion in two directions.
:numref:`fig:robotonball` shows one design by
Dr. Masaaki Kumagai, director of the Robot Development Engineering
Laboratory at Tohoku Gakuin University.



.. figure:: MotionFigures/sds-omni-1.jpg
   :width: 60%
   :align: center

   Omniwheel drive system


.. _`fig:robotonball`:
.. figure:: MotionFigures/robotonball.jpg
   :width: 60%
   :align: center

   Omniwheel balancing robot

|

.. figure:: MotionFigures/goodyearsphere.jpg
   :width: 60%
   :align: center

   GoodYear Spherical Wheel Concept Tire


.. figure:: MotionFigures/SDS-omnidirectional-electric-motorcycle4.jpg
   :width: 60%
   :align: center

   Prototype omnidirectional motorcycle

Mobility Issues
~~~~~~~~~~~~~~~

The stability of the craft is given by several factors. Having less than
three contact points requires dynamic balance for a system which is “at
rest”. Having less than 6 contact points means that during locomotion,
the system requires dynamic balance during motion or one is moving at
most two legs at a time making a more complicated control system. The
location of the center of gravity is an important aspect of dynamic
stability. A lower center of gravity helps to avoid falling over.

For ground systems, the terrain will have more influence than with air
or sea. We have to worry about the terrain roughness, slickness, grades
and other issues. The number of wheels, type of wheels, type of
suspension, and steering will all have a large affect on the
effectiveness of motion.


Tracks
~~~~~~

For the purposes of this text, we will treat unsteered tracked systems
(tank treads) as two-wheel differential drive (wheeled) systems. The
modeling is more difficult than with wheels. Modeling the skid-steer
turns requires details about the track system and the surface. Since
rocks, mud and other aspects of the surface can have significant effects
on turning friction, models have limited utility.
