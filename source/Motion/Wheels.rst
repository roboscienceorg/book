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
