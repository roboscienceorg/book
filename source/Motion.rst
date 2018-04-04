Vehicle Motion [Chap:Motion]
============================

In this chapter, we model several motion systems and focus one of the
most common robot drive systems known as the differential drive. We
begin with ground contact components such as wheels and tracks. Move to
steering and drive systems. The differential drive will work as the
canonical example for which other drive systems will be discussed.
Following the differential drive we derive a general approve to modeling
drive systems. This is used to model steered vehicles and several types
of Omniwheel systems.

Locomotion
----------

The word locomotion means “The act of moving from place to
place.” :raw-latex:`\cite{dictionary}` from Latin combining location and
motion. Biology has explored a variety of very interesting ways to move
around. Animals can be carried on currents, swim, crawl, slide, walk,
run, jump, and fly.

Most of the locomotion solutions found in nature are hard to imitate
with machines. Although legs are the base for human locomotion, humans
have valued wheels in their motion solutions. The earliest recorded
appearance of wheels is in Mesopotamia (Sumerian) at the mid-4th
millenium BC. The is evidence of independent discovery in the new world,
the lack of domesticated large animals prevented any development beyond
children’s toys. :raw-latex:`\cite{wiki:wheel}`.

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

.. raw:: latex

   \centering

.. figure:: motion/energyusegraph
   :alt: The relations between energy, speed and motion
   type.[motionenergyspeed]

   The relations between energy, speed and motion
   type.[motionenergyspeed]

Mobility Issues
---------------

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

.. raw:: latex

   \centering

.. figure:: motion/airtrax.jpg
   :alt: The Airtrax forklift. [fig:airtrax]

   The Airtrax forklift. [fig:airtrax]

.. figure:: motion/airtraxcobra.jpg
   :alt: The Airtrax scissor lift. [fig:airtraxcobra]

   The Airtrax scissor lift. [fig:airtraxcobra]

The omni wheel’s first patent was in 1919 by Grabowiecki. The Mecanum
wheel was developed by Bengt Erland Ilon in 1972 while working for the
Mecanum company. Airtrax, an American forklift company purchased patent
rights and briefly manufactured forklifts with a heavy duty version of
the Mecanum wheel. These wheels have much less ground friction in a turn
in comparison to a skid steer requiring much less torque.

.. raw:: latex

   \centering

.. figure:: motion/swedish_angle
   :alt: :math:`\gamma` measure. [gammavarconfig]

   :math:`\gamma` measure. [gammavarconfig]

.. figure:: motion/omni-wheel
   :alt: :math:`\gamma = 0` configuration [gammazeroconfig]

   :math:`\gamma = 0` configuration [gammazeroconfig]

.. figure:: motion/mecanum-wheel
   :alt: :math:`\gamma = 45^\circ` configuration. [gamma45config]

   :math:`\gamma = 45^\circ` configuration. [gamma45config]

For this text, we will combine the omni and Mecanum wheels and just call
them omniwheels. The difference between them is only in the angle the
rollers are mounted on the wheel body.
Figure \ `[gammaconfig] <#gammaconfig>`__ shows some sample types of
omniwheels using the :math:`\gamma = 0` configuration and
:math:`\gamma = 45^\circ` configuration. Normally the :math:`\gamma=0`
style of wheel is used in non-parallel mounting as shown in the first
robot in the Figure \ `[gammawheelmounting] <#gammawheelmounting>`__ and
the parallel mounting is used for the other standard type of wheel
design using :math:`\gamma = 45^\circ`.

.. raw:: latex

   \centering

.. figure:: motion/swedish_config
   :alt: Normal mounting style for :math:`\gamma = 0` and
   :math:`\gamma = 45^\circ`. [gammawheelmounting]

   Normal mounting style for :math:`\gamma = 0` and
   :math:`\gamma = 45^\circ`. [gammawheelmounting]

.. raw:: latex

   \centering

.. figure:: motion/swedish_mount
   :alt: Force vectors induced by rotation with the
   :math:`\gamma = 45^\circ` configuration. [meccanumwheelvectors]

   Force vectors induced by rotation with the :math:`\gamma = 45^\circ`
   configuration. [meccanumwheelvectors]

.. raw:: latex

   \centering

.. figure:: motion/swedish_mount2
   :alt: Mecanum rotation directions and vector forces for different
   vehicle directions. [meccanumwheelmotion]

   Mecanum rotation directions and vector forces for different vehicle
   directions. [meccanumwheelmotion]

.. raw:: latex

   \centering

.. figure:: motion/swedish_mount3
   :alt: Summary of wheel motion and directions [meccanumwheelmotion2]

   Summary of wheel motion and directions [meccanumwheelmotion2]

2

-  Driving forward: all four wheels forward

-  Driving backward: all four wheels backward

-  Driving left: 1,4 backwards; 2,3 forward

.. raw:: latex

   \hspace*{-5mm}

-  Driving right: 1,4 forward; 2,3 backward

-  Turning clockwise: 1,3 forward; 2,4 backward

-  Turning counterclockwise: 1,3 backward; 2,4 forward

.. raw:: latex

   \vspace*{-3mm}

A variation of the omni wheel is the omni ball developed by Kaneko
Higashimori Lab at Osaka University,
see \ `[fig:omniball] <#fig:omniball>`__. This wheel will be used to
drive tracks in a very novel approach described in the tracks section
below. This wheel fails to be a true spherical wheel as far as two
directional motion is concerned and has motion equations similar to the
omniwheel systems.

.. raw:: latex

   \centering

.. figure:: motion/omni-ball.jpg
   :alt: The Omni Ball Wheel developed at the Kaneko Higashimori Lab at
   Osaka University[fig:omniball]

   The Omni Ball Wheel developed at the Kaneko Higashimori Lab at Osaka
   University[fig:omniball]

Omni and Mecanum wheels can be driven on only one direction and only
when combined with other wheels are they able to move against the
rolling directions. To gain two dimensional directional capability the
wheel needs to be a sphere or at least approximate the sphere in a
significant manner. This can be done by reversing the power direction
from the classical mechanical computer mouse. In the mechanical mouse
the ball is forced around which drives small disks inside in the
component directions. By mounting three omniwheels on top of a ball, one
can gain motion in two directions.
Figure \ `[fig:robotonball] <#fig:robotonball>`__ shows one design by
Dr. Masaaki Kumagai, director of the Robot Development Engineering
Laboratory at Tohoku Gakuin University.

.. raw:: latex

   \centering

.. raw:: latex

   \centering

.. figure:: motion/sds-omni-1.jpg
   :alt: Omniwheel drive system

   Omniwheel drive system

.. raw:: latex

   \centering

.. figure:: motion/robotonball.jpg
   :alt: Omniwheel balancing robot[fig:robotonball]

   Omniwheel balancing robot[fig:robotonball]

| 

.. raw:: latex

   \centering

.. figure:: motion/goodyearsphere.jpg
   :alt: GoodYear Spherical Wheel Concept Tire

   GoodYear Spherical Wheel Concept Tire

.. raw:: latex

   \centering

.. figure:: motion/SDS-omnidirectional-electric-motorcycle4.jpg
   :alt: Prototype omnidirectional motorcycle

   Prototype omnidirectional motorcycle

Tracks
~~~~~~

For the purposes of this text, we will treat unsteered tracked systems
(tank treads) as two-wheel differential drive (wheeled) systems. The
modeling is more difficult than with wheels. Modeling the skid-steer
turns requires details about the track system and the surface. Since
rocks, mud and other aspects of the surface can have significant effects
on turning friction, models have limited utility.

Drive Systems
-------------

The most common drive system in robotics is the differential drive.
Differential drive is a two wheeled drive system. For stability a third
support must be employed. A castor wheel or ball is normally used. The
well known Rumba floor cleaning robot uses this system. It is stable,
maneuverable, easy to control, and simple.
Figure \ `[ddrive_pre] <#ddrive_pre>`__ gives the basic layout and
variables involved in the model.

.. raw:: latex

   \centering

.. figure:: motion/ddexample
   :alt: The differential drive robot dimensions and variables.
   [ddrive_pre]

   The differential drive robot dimensions and variables. [ddrive_pre]

Differential Drive
~~~~~~~~~~~~~~~~~~

Recall the Differential Drive
robot \ `[fig:ddriveRecalled2] <#fig:ddriveRecalled2>`__

.. raw:: latex

   \centering

.. figure:: motion/ddrive
   :alt: Simple differential drive robot. [fig:ddriveRecalled2]

   Simple differential drive robot. [fig:ddriveRecalled2]

| and the forward and inverse kinematics:

.. math::

   \boxed{
   \begin{array}{l}
    \dot{x} = \frac{r}{2} (\dot{\phi_1}+\dot{\phi_2})\cos(\theta) \\[5mm]
   \dot{y} = \frac{r}{2} (\dot{\phi_1}+\dot{\phi_2})\sin(\theta) \\[5mm]
   \dot{\theta} = \frac{r}{2L} (\dot{\phi_1}-\dot{\phi_2})
   \end{array}}

.. math::

   \boxed{
   \begin{array}{l}
   v = \sqrt{\dot{x}^2 + \dot{y}^2},\hspace*{2mm}
   \kappa =   \displaystyle  \frac{\dot{x}\ddot{y} - \dot{y}\ddot{x}}{v^3} \\[3mm]
   \dot{\phi_1} = \displaystyle \frac{v}{r}\left(\kappa L + 1\right) \\[3mm]
   \dot{\phi_2} = \displaystyle \frac{v}{r}\left(-\kappa L + 1\right)
   \end{array}}

| 
| where :math:`\dot{\phi_1}` and :math:`\dot{\phi_2}` be the right and
  left wheel rotational speeds (respectively), :math:`r` is wheel radius
  and :math:`L` is the axle length from the center to the wheel (“half
  axle”).

Alternate Form
^^^^^^^^^^^^^^

In some cases we only need to know the forward velocity and the vehicle
rotation rate. By computing :math:`v` from
equation \ `[ddkinematicsmodel] <#ddkinematicsmodel>`__ and using
:math:`\omega = \dot{\theta}`, we obtain

.. math::

   \label{ddkinematicsmodelalt}
   \begin{array}{l}
   v = \frac{r}{2} (\dot{\phi_1}+\dot{\phi_2}) \\[5mm]
   \omega = \frac{r}{2L} (\dot{\phi_1}-\dot{\phi_2})
   \end{array}

 and the inverse of these are

.. math::

   \label{ddinversekinematicsmodelalt}
   \begin{array}{l}
   \dot{\phi_1} = \frac{1}{r} (v+L\omega)\\[5mm]
   \dot{\phi_2} = \frac{1}{r} (v-L\omega)
   \end{array}

Omniwheels
~~~~~~~~~~

Figure \ `[gammaconfig] <#gammaconfig>`__ shows some sample types of
omniwheels using the :math:`\gamma = 0` configuration and
:math:`\gamma = 45^\circ` configuration. Also recall that
:math:`\gamma=0` style of wheel is used in non-parallel mounting as
shown in the first robot in the
Figure \ `[gammawheelmounting] <#gammawheelmounting>`__ and the parallel
mounting is used for the other standard type of wheel design using
:math:`\gamma = 45^\circ`.

.. raw:: latex

   \centering

.. figure:: motion/mecanumdim
   :alt: Dimensions for the Mecanum Kinematics[fig:mecanumdim]

   Dimensions for the Mecanum Kinematics[fig:mecanumdim]

For this section we assume that we have a traditional care design frame
and wheel mounting as described in
Figure \ `[fig:mecanumdim] <#fig:mecanumdim>`__
(:math:`\gamma = 45^\circ`). The following notation is used in the
kinematics:

-  :math:`r` - wheel radius.

-  :math:`L_1` - distance between left and right wheel pairs,
   :math:`L_2` - distance between front and rear wheel pairs.

-  :math:`v_x`, :math:`v_y`, :math:`\omega` - the robot velocity and
   angular velocity in robot coordinates.

-  :math:`\dot{x}`, :math:`\dot{y}`, :math:`\dot{\theta}` - robot
   velocity in :math:`x`, :math:`y` and robot angular velocity in global
   coordinates.

-  :math:`\dot{\phi}_{FL}, \dot{\phi}_{FR},  \dot{\phi}_{BL}, \dot{\phi}_{BR}`
   - front left, front right, back left, back right, radians per minute.

Forward kinematics
^^^^^^^^^^^^^^^^^^

The forward local kinematics for this architecture is

.. math::

   \begin{bmatrix}v_x \\[3mm] v_y \\[3mm] \omega \end{bmatrix}
   =  \frac{r}{4} \begin{bmatrix} 1 & 1 & 1 & 1 \\[3mm]
                           -1 & 1 & 1 & -1\\[3mm]
                            -\frac{1}{(L_1+L_2)} & \frac{1}{(L_1+L_2)} & -\frac{1}{(L_1+L_2)} &
                               \frac{1}{(L_1+L_2)}
            \end{bmatrix}
   \begin{bmatrix}\dot{\phi}_{FL} \\ \dot{\phi}_{FR} \\ \dot{\phi}_{BL} \\ \dot{\phi}_{BR} \end{bmatrix} .

Applying the rotation to move to global coordinates

.. math::

   \begin{bmatrix}\dot{x}\\[3mm] \dot{y}\\[3mm] \dot{\theta} \end{bmatrix}
   =  \frac{r}{4} R(\theta)\begin{bmatrix} 1 & 1 & 1 & 1 \\[3mm]
                           -1 & 1 & 1 & -1\\[3mm]
                            -\frac{1}{(L_1+L_2)} & \frac{1}{(L_1+L_2)} & -\frac{1}{(L_1+L_2)} &
                               \frac{1}{(L_1+L_2)}
            \end{bmatrix}
   \begin{bmatrix}\dot{\phi}_{FL} \\ \dot{\phi}_{FR} \\ \dot{\phi}_{BL} \\ \dot{\phi}_{BR} \end{bmatrix}

.. math::

   =
   \frac{ r}{4} R(\theta)\begin{bmatrix} \dot{\phi}_{FL} + \dot{\phi}_{FR} + \dot{\phi}_{BL} + \dot{\phi}_{BR} \\[3mm]
                           -\dot{\phi}_{FL} + \dot{\phi}_{FR} + \dot{\phi}_{BL} - \dot{\phi}_{BR}  \\[3mm]
                               \frac{1}{(L_1+L_2) } \left( -\dot{\phi}_{FL} + \dot{\phi}_{FR} - \dot{\phi}_{BL} +\dot{\phi}_{BR} \right)
            \end{bmatrix}

.. math::

   =
   \frac{ r}{4} 
   \begin{bmatrix} \left(\dot{\phi}_{FL} + \dot{\phi}_{FR} + \dot{\phi}_{BL} + \dot{\phi}_{BR}\right) \cos(\theta)
                             -\left( -\dot{\phi}_{FL} + \dot{\phi}_{FR} + \dot{\phi}_{BL} - \dot{\phi}_{BR}\right)\sin(\theta) \\[3mm]
                           \left(\dot{\phi}_{FL} + \dot{\phi}_{FR} + \dot{\phi}_{BL} + \dot{\phi}_{BR}\right) \sin(\theta)
                             +\left( -\dot{\phi}_{FL} + \dot{\phi}_{FR} + \dot{\phi}_{BL} - \dot{\phi}_{BR}\right)\cos(\theta)  \\[3mm]
                               \frac{1}{(L_1+L_2) } \left( -\dot{\phi}_{FL} + \dot{\phi}_{FR} - \dot{\phi}_{BL} +\dot{\phi}_{BR} \right)
            \end{bmatrix} .

| So, finally obtain

  .. math::

     \label{meccanumforwardkinematics}
     \begin{bmatrix}\dot{x}\\[3mm] \dot{y}\\[3mm] \dot{\theta} \end{bmatrix}
     =
     \frac{ r}{4} 
     \begin{bmatrix}A\cos(\theta)
                               -B\sin(\theta) \\[3mm]
                             A \sin(\theta)
                               +B\cos(\theta)  \\[3mm]
                                 \frac{1}{(L_1+L_2) } C
              \end{bmatrix}

   where
  :math:`A = \left(\dot{\phi}_{FL} + \dot{\phi}_{FR} + \dot{\phi}_{BL} + \dot{\phi}_{BR}\right)`,
  :math:`B = \left( -\dot{\phi}_{FL} + \dot{\phi}_{FR} + \dot{\phi}_{BL} - \dot{\phi}_{BR}\right)`,
| and
  :math:`C = \left( -\dot{\phi}_{FL} + \dot{\phi}_{FR} - \dot{\phi}_{BL} +\dot{\phi}_{BR} \right)`.

| To perform numerical calculations, we need to discretize the
  differential equations. Using the same process that we used to gain
  equations \ `[discreteDD] <#discreteDD>`__, we discretize the Mecanum
  equations. As before the time step is :math:`\Delta t`,
  :math:`x_k = x(t_k)`, :math:`y_k = y(t_k)`,
  :math:`\theta_k = \theta(t_k)`,
  :math:`\omega_{FL,k}=\dot{\phi}_{FL}(t_k)` ..., and we have

  .. math::

     \label{mecanumforwardkinematics}
     \begin{bmatrix} x_{k+1}\\[3mm] y_{k+1}\\[3mm] \theta_{k+1} \end{bmatrix}
     =   \begin{bmatrix} x_{k}\\[3mm] y_{k}\\[3mm] \theta_{k} \end{bmatrix} +
     \frac{ r\Delta t }{4} \begin{bmatrix} A\cos(\theta_{k})  - B \sin(\theta_{k})   \\[3mm]
     A\sin(\theta_{k})  + B \cos(\theta_{k})                     \\[3mm]
                                 \frac{1}{(L_1+L_2) } C
              \end{bmatrix}

   where
  :math:`A = \left( \omega_{FL,k} + \omega_{FR,k} + \omega_{BL,k} + \omega_{BR,k} \right)`,
| :math:`B = \left(-\omega_{FL,k} + \omega_{FR,k} + \omega_{BL,k} - \omega_{BR,k}  \right)`,
| and
  :math:`C =  \left( -\omega_{FL,k} + \omega_{FR,k} - \omega_{BL,k} +\omega_{BR,k} \right)`.

Inverse Kinematics for the Mecanum
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

We used a traditional care design frame and wheel mounting as described
in Figure \ `[gammaconfig] <#gammaconfig>`__
(:math:`\gamma = 45^\circ`). The inverse kinematics in local coordinates
are given by

.. math::

   \begin{bmatrix}\dot{\phi}_{FL} \\[3mm] \dot{\phi}_{FR} \\[3mm] \dot{\phi}_{BL} \\[3mm] \dot{\phi}_{BR} \end{bmatrix}
   =
   \frac{1}{ r}
   \begin{bmatrix} 1 & -1 & -(L_1+L_2)  \\[3mm]
                   1 & 1 & (L_1+L_2)  \\[3mm]
                   1 & 1 & -(L_1+L_2)  \\[3mm]
                   1 & -1 & (L_1+L_2)  
            \end{bmatrix}
   \begin{bmatrix}v_x \\[3mm] v_y \\[3mm] \omega \end{bmatrix} .

Applying the coordinate transformation we can move to global coordinates

.. math::

   \begin{bmatrix}\dot{\phi}_{FL} \\[3mm] \dot{\phi}_{FR} \\[3mm] \dot{\phi}_{BL} \\[3mm] \dot{\phi}_{BR} \end{bmatrix}
   =
   \frac{1}{ r}
   \begin{bmatrix} 1 & -1 & -(L_1+L_2)  \\[3mm]
                   1 & 1 & (L_1+L_2)  \\[3mm]
                   1 & 1 & -(L_1+L_2)  \\[3mm]
                   1 & -1 & (L_1+L_2)  
    \end{bmatrix}
    R^{-1}(\theta)
   \begin{bmatrix}\dot{x} \\[3mm] \dot{y} \\[3mm] \dot{\theta} \end{bmatrix}

.. math::

   =
   \frac{1}{ r}
   \begin{bmatrix} 1 & -1 & -(L_1+L_2)  \\[3mm]
                   1 & 1 & (L_1+L_2)  \\[3mm]
                   1 & 1 & -(L_1+L_2)  \\[3mm]
                   1 & -1 & (L_1+L_2)  
   \end{bmatrix}
   \begin{bmatrix}\cos(\theta) \dot{x} + \sin(\theta)\dot{y}\\[3mm] -\sin(\theta)\dot{x} + \cos(\theta)\dot{y} \\[3mm] \dot{\theta} \end{bmatrix}

.. math::

   \label{meccanuminversekinematics}
   =
   \frac{1}{ r}
   \begin{bmatrix}  \cos(\theta) \dot{x} + \sin(\theta)\dot{y} + \sin(\theta)\dot{x} - \cos(\theta)\dot{y} -(L_1+L_2)\dot{\theta}  \\[3mm]  
                     \cos(\theta) \dot{x} + \sin(\theta)\dot{y} - \sin(\theta)\dot{x} + \cos(\theta)\dot{y} +(L_1+L_2)\dot{\theta}  \\[3mm] 
                     \cos(\theta) \dot{x} + \sin(\theta)\dot{y} - \sin(\theta)\dot{x} + \cos(\theta)\dot{y} -(L_1+L_2)\dot{\theta}   \\[3mm] 
                    \cos(\theta) \dot{x} + \sin(\theta)\dot{y} + \sin(\theta)\dot{x} - \cos(\theta)\dot{y} +(L_1+L_2)\dot{\theta}  
   \end{bmatrix} .

Steered Systems
~~~~~~~~~~~~~~~

Automobiles are nearly exclusive to a front wheel steering system (for a
variety of reasons not discussed here). There are lots of ways to
approach steering and some work better than others. If the front wheels
are turned, the vehicle starts a circular arc either to the left or
right. Geometrically this generates two concentric circles which are not
the same size. The inside and outside wheel on a given axle do not
rotate at the same speed or point in the same direction. Parallel wheels
will skid on a turn. The mechanical solution to the problem is listed in
a patent by Ackermann, but the solution predates by more than a half
century. We will discuss this issue in greater detail in the motion
modeling chapter.

.. raw:: latex

   \centering

.. figure:: motion/steered
   :alt: Front Wheel Steered System.

   Front Wheel Steered System.

Ackerman
^^^^^^^^

The best known mobile vehicle design currently is the steered wheel,
specifically the Ackerman Steering design. This is our traditional car
implementation. It is a rectangular vehicle with four wheels. The front
two wheels are steered. We begin with the fixed turn angle or simple
steer model.

.. math::

   \displaystyle
   \begin{bmatrix} v \\ \dot{\theta} \end{bmatrix}
   =  r \dot{\phi}
   \begin{bmatrix} 1 \\ \displaystyle \frac{\sin \beta}{L_2} \end{bmatrix} 
   \quad \mbox{and} \quad
   \begin{bmatrix} \dot{\phi}  \\ \beta \end{bmatrix}
   = 
   \begin{bmatrix}\displaystyle  \frac{v}{r} \\ \displaystyle \sin^{-1} \frac{L_2 \dot{\theta}}{v} \end{bmatrix}

There are several issues with the simple design illustrated above.
During a turn the left and right wheels travel different arcs meaning
different distances,
Figure \ `[ackermannsteeringfig] <#ackermannsteeringfig>`__. This will
cause the wheels to skid if their rotation rates are the same. Part of
the solution is to place a differential in the axle to deliver power and
allow for different wheel speeds. The other part is to allow for
differential steering with the Ackerman design. The Ackerman steering
overcomes the issue of side slip due to the outside wheel traveling
farther than the inside wheel.

Some history here is interesting. The invention is claimed by Georg
Lankensperger (Munich) in 1817. However his agent, Rudolf Ackerman,
filed the patent and now has name credit. But, this steering system was
described 50 years earlier by Eramus Darwin (the grandfather of Charles
Darwin) in 1758 according to Desmond King-Halle in 2002 and Mr. Darwin
has claim to the invention.

.. raw:: latex

   \centering

.. figure:: motion/ackermann
   :alt: To avoid skidding, the outside wheel must turn at a different
   angle and rotate at a different speed than the inside
   wheel.[ackermannsteeringfig]

   To avoid skidding, the outside wheel must turn at a different angle
   and rotate at a different speed than the inside
   wheel.[ackermannsteeringfig]

Recall that we had the no-slip and no-slide assumptions for our wheels.
The no-slide assumption means that there is no motion in the direction
of the axle. All of the motion is perpendicular to the axle. This means
for each wheel, the sliding constraint generates a zero motion line
(orthogonal to the wheel plane). The intersection of the zero motion
lines is the ICR - Instantaneous Center of Rotation. Having a common
intersection, an ICR, implies that each wheel is moving on a concentric
circle. If the zero motion lines do not intersect at a single point,
then no motion is possible when we have no-slip and no-slide for our
wheels. We can easily see that this is the case for the simple steering
approach shown above. The rear wheels have overlapping zero motion
lines. The front wheels have parallel non-overlapping zero motion lines.

.. raw:: latex

   \centering

.. figure:: motion/icr
   :alt: ICR - Instantaneous Center of Rotation.

   ICR - Instantaneous Center of Rotation.

To satisfy the constraint placed on by the ICR, the steering system must
satisfy the Ackerman equation:

.. math:: \cot\theta_R - \cot\theta_L = \frac{2L_1}{L_2}

where :math:`\theta_R` is the angle of the right wheel, :math:`\theta_L`
is the angle of the left wheel, :math:`2L_1` is axle length and
:math:`L_2` is the wheel base length,
Figure \ `[Fig:ackermansteerangles] <#Fig:ackermansteerangles>`__. The
effective steering angle, :math:`\theta_S` can be found by

.. math:: \cot\theta_S = \frac{L_1}{L_2} + \cot\theta_R    \quad {\mbox{or} } \quad \cot\theta_S =\cot\theta_L -  \frac{L_1}{L_2}

.. raw:: latex

   \centering

.. figure:: motion/ackermann_steer2
   :alt: The steering angles for the Ackerman
   equation.[Fig:ackermansteerangles]

   The steering angles for the Ackerman
   equation.[Fig:ackermansteerangles]

The Ackerman design is one that approximates the geometric constraints
which produces the ICR. A purely mechanical solution is to embed the
geometry into the steering linkage. A triangle is formed from the
attachment points at the wheels and the center of the rear axle. By
moving the rear axle intersection, one can steer the wheels as well as
keep the zero motion lines intersecting on the rear axle. The attachment
to the wheels is called the *kingpins*. The cross piece between the
Kingpins is called the *tie rod*.

.. raw:: latex

   \centering

.. figure:: motion/icr2
   :alt: The Ackerman steering system.

   The Ackerman steering system.

Other Steered Wheel
^^^^^^^^^^^^^^^^^^^

As you delve into robot drive systems you begin to see that there are
many different ways that people have mounted wheels onto frames and
figured out how to steer the craft. We can only touch on a few designs
in this text and encourage the reader to look beyond this text. It can
be very entertaining to experiment with different wheel and frame
designs. Using components like Actobots
(https://www.servocity.com/actobotics), Lego, or Vex one can quickly
assemble nearly anything that your mind can dream up. One novel approach
to all wheel steering is the Syncro Drive
system \ `[fig:syncrodrive] <#fig:syncrodrive>`__. Using three or four
steered wheels, the wheels are connected by a chain or cable allowing
all wheels to be steered. Each wheel is kept in a parallel mode so that
motion is possible in any direction.

.. raw:: latex

   \centering

.. figure:: motion/syncro
   :alt: Syncro Drive System.[fig:syncrodrive]

   Syncro Drive System.[fig:syncrodrive]

The Dubins, Reeds-Shepps Cars and other drive systems
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

We investigate two vehicle designs which have a similar mechanism for
steering. The first design consists of two axles with four driven
wheels. The centers of the axles are attached to the frame of the robot
using a lockable pivot. In essence, it is two differential drives
attached to a bar with the pivot mechanism (see
Figure \ `[fig:DDD] <#fig:DDD>`__). We will refer to this as the Dual
Differential Drive (DDD). The second design uses four axles (or one can
think of splitting the axles in the DDD design) each with a driven
wheel. The axles are attached to the body of the robot again using
locking pivots (Figure `[fig:FWS] <#fig:FWS>`__). We will focus on
attachment points at the corners of the vehicle but other locations such
as along the center line at either end of the robot would also be
possible. For this design, mounting the pivots at the center of the axle
or at the corners of a chassis has the effect of changing the number of
pivot brakes and the costs, but does not significantly change the
kinematics. This configuration will be known as the Four Wheel Steer
(FWS). A traditional articulated steering design is shown in
Figure \ `[fig:AD] <#fig:AD>`__. The kinematics and motion curves for
this design are essentially the same as the DDD design, and as such will
be treated as a DDD steering mechanism.

.. raw:: latex

   \centering

.. figure:: motion/single_axle
   :alt: Dual Differential Drive (DDD). This vehicle has single or
   connected axle in the front and a single axle in the rear. The axle
   is connect to the frame using a pivot which can be locked (braked) or
   free. [fig:DDD]

   Dual Differential Drive (DDD). This vehicle has single or connected
   axle in the front and a single axle in the rear. The axle is connect
   to the frame using a pivot which can be locked (braked) or free.
   [fig:DDD]

.. raw:: latex

   \centering

.. figure:: motion/split_axle_box
   :alt: Four Wheel Steer (FWS). This vehicle has four axles each is
   connected to the frame by a lockable pivot. In addition to motion see
   in the DDD design, if there is sufficient rotational motion in the
   axles, this conifguration can spin in place. [fig:FWS]

   Four Wheel Steer (FWS). This vehicle has four axles each is connected
   to the frame by a lockable pivot. In addition to motion see in the
   DDD design, if there is sufficient rotational motion in the axles,
   this conifguration can spin in place. [fig:FWS]

.. raw:: latex

   \centering

.. figure:: motion/pivot_brake
   :alt: Articulated Drive (AD). This is a common design in heavy
   equipment like articulated front loaders. The motion is similar to
   that found in the DDD design and can be driven with an unlocked pivot
   (brake not required). [fig:AD]

   Articulated Drive (AD). This is a common design in heavy equipment
   like articulated front loaders. The motion is similar to that found
   in the DDD design and can be driven with an unlocked pivot (brake not
   required). [fig:AD]

When a wheel motor is activated, it will cause the axle to rotate about
the pivot. Once the desired angle is achieved, the pivot is locked
leaving the wheels in the steered configuration. The pivot joints are
binary in the sense that they are completely locked or completely free.
This is done by a normally closed brake attached to the pivots and will
allow free motion when power is applied to the pivot brake. When power
is interrupted, the pivot brake locks down. Expected initial operation
of the test unit is to alternate between a fixed position while aligning
wheels and vehicle motion with the pivot brakes locked.

In terms of movement in the plane, the solid axle system is a dual
differential drive. For the purposes of understanding motion curves we
can view it as a two wheel (bicycle) design. Since we use four drive
motors there is no need for a differential. The FWS axle mounted on the
box can emulate Ackerman steering and does not suffer from wheel slip or
slide. We will see that this design has greater maneuverability in
comparison to a double Ackerman steered vehicle. In either case, we have
two situations with a moving vehicle: driving straight paths and
circular paths. Not found in Ackerman systems, the FWS design can
additionally rotate in place if the axle is allowed to rotate out
:math:`45^\circ` or more.

.. raw:: latex

   \centering

.. figure:: motion/motion
   :alt: The forward motion curves. Left: traditional Dubins Car. Right:
   forward motion of the DDD vehicle.[fig:fmotion]

   The forward motion curves. Left: traditional Dubins Car. Right:
   forward motion of the DDD vehicle.[fig:fmotion]

For the DDD design, using the bicycle approximation, the radius of
curvature is given as a function of the maximum axle rotation and the
wheelbase. Let the axle turn angle be :math:`\theta` and the wheel base
given by :math:`d`, then the radius of curvature is given by

.. math:: r  = d/(2\sin\theta)

\ (Figure `[fig:turngeo] <#fig:turngeo>`__ (left)). In addition, the DDD
can move linearly in directions angled off the forward direction of the
vehicle if the axles are parallel and have nonzero axis angle in
reference to the forward vehicle normal
(Figure`[fig:fmotion] <#fig:fmotion>`__). The direction off of the
forward normal direction is given by the axle angles and if the front
and rear axles are not parallel, then a circular path will occur with
direction off of the forward direction as seen with parallel axles.

.. raw:: latex

   \centering

|Turn geometry for the DDD (left) and FWS (right) designs.
[fig:turngeo]| |Turn geometry for the DDD (left) and FWS (right)
designs. [fig:turngeo]|

The FWS design can adjust to the radius of curvature for both inside and
outside wheels. The radius of curvature for the vehicle center is the
average of the inside and outside circle radii:

.. math:: \overline{r} = (r_1+r_2)/2 = d\left(1/(4\sin\theta_1) + 1/(4\sin\theta_2)\right)

\ (Figure `[fig:turngeo] <#fig:turngeo>`__). For this design, we have
the ability to move as with the DDD and in addition rotate in place.
Both systems can also move forwards and reverse. Thus orientation and
direction may be changed at any point along the trajectory.

For the DDD, there are four motors (with associated electronics) and two
pivot brakes (and associated electronics). The FWS design adds two pivot
brakes in addition to the DDD cost. The operating assumption here is
that mechanical holding torque can be gained more cheaply than
electrical turning torque. The term “cheap” will refer to dollar cost or
to electrical power depending on the context. The dollar cost range for
motors, motor drivers, electromagnetic brakes, etc., varies greatly. In
our application, we found the prices to be fairly close between brakes
and motors but the prices for driving electronics was significantly
cheaper for the brakes as they operate like solenoids and the more
complicated motor driver hardware was not required.

We have found that we don’t need a brake for the DDD system which
removes both financial and electrical costs associated with the
eliminated systems. For the FWS system, we can purchase a normally
locked brake. Power is applied only when adjustments are required thus
removing the need for holding current.

.. _sec:paths:

Configuration space and reach for simple vehicles
-------------------------------------------------

In this section, we determine the reach (and time limited reach) of the
robot from a given point and the possible paths between two points. Does
the reach cover the plane or are there some points in the plane which
cannot be reached? First, we make precise what is meant by reach
:raw-latex:`\cite{lavalle2006}`. Let :math:`X` be the state space,
:math:`{\cal U} \subset X` be the set of all permissible trajectories on
:math:`[0,\infty)` and :math:`R(q_0,{\cal U} )` denote the reachable set
from :math:`x_0`.

We define the reachable set as

.. math:: R(x_0,{\cal U} ) = \left\{  x_1 \in X | \exists \tilde{u}\in {\cal U} \mbox{ and } \exists t \in [0,\infty) \mbox{ s.t. } x(t) = x_1 \right\}

Let :math:`R(q_0,{\cal U} ,t)` denote the time-limited reachable set
from :math:`x_0`.

We define the time-limited reachable set as

.. math:: R(x_0,{\cal U},t ) = \left\{ x_1 \in X | \exists \tilde{u}\in {\cal U} \mbox{ and } \exists \tau \in [0,t] \mbox{ s.t. } x(\tau) = x_1 \right\}

The Dubins Car, :raw-latex:`\cite{dubins}`, is a vehicle that can move
straight forward or turn at any curvature up to some maximum curvature.
This vehicle provides a geometric motion model for automobiles and can
be used to understand basic optimal path planning. The Reeds-Shepps Car,
:raw-latex:`\cite{reeds}`, extends the Dubins vehicle to include reverse
motion. This greatly enhances maneuverability. Small back and forth
motions can realign a vehicle to a new orientation. This means if the
robot arrives at a destination point with the wrong orientation, it can
be corrected locally (assuming sufficient room about the point).

Dubins showed that a vehicle which can go only forward and turn at any
curvature up to some maximum curvature can reach any point in the plane
in the absence of obstacles :raw-latex:`\cite{dubins}`. Optimality of
solutions is discussed in
:raw-latex:`\cite{kelly2013mobile, lavalle2006}`. A slight
generalization is given in :raw-latex:`\cite{reeds}` for a car that can
go forwards and backwards. In
:raw-latex:`\cite{reeds, sussman, lavalle2006}`, it is shown that
optimal solutions are piecewise collections of line segments and maximum
curvature circles. Since the DDD (dual differential drive) and FWS (four
wheel steer) designs have less restrictive motion, we can answer the
reach question. The entire plane can be covered. The question of optimal
paths will be left for a future study. The FWS system we have built is
targeted for an environment filled with obstacles. Our main concern is
reach in the presence of obstacles, for which the reach and the optimal
path results for Dubins and Reeds-Shepps are no longer valid.

Both the DDD and FWS designs are more maneuverable than the Dubins
vehicle, and so we expect more flexibility in dealing with obstacles.
The time limited reach of the Dubins Car is the forward fan seen in
Figure \ `[fig:fmotion] <#fig:fmotion>`__ and the time limited reach of
the Reeds-Shepps car is the open set about the initial point
:raw-latex:`\cite{lavalle2006}`. Since both the DDD and FWS systems
include the motion patterns found in the Reeds-Shepps car, the time
limited reach for these two designs is an open set about the initial
point: there exists a set :math:`U`, open, such that
:math:`U \subset R(x_0,{\cal U},t )`. This is possible due to the
ability to perform back and forth maneuvers like that found in parallel
parking.

Rigid Motion
~~~~~~~~~~~~

The FWS can move from point to point and then adjust orientation as
required. If there exists a path between two points, the FWS axle can
traverse the path via the waypoints, re-orient at each point and reach
the goal location. Thus it can follow a piecewise linear path between
two configuration space locations. A smooth path can be found by using a
b-spline and if curvature exceeds the maximum bound, the vehicle can
stop, re-orient and then continue. Traversal is possible if the start
and goal locations are path connected and that path locations with
curvature above :math:`R` have a disk of radius :math:`r` centered at
the path point which does not intersect any obstacle.

The DDD design has additional constraints compared to the FWS design.
The solution that :raw-latex:`\cite{reeds, sussman, lavalle2006}`
suggest is to perform a series of short adjustment maneuvers as seen in
Figure \ `[fig:deltatheta] <#fig:deltatheta>`__. Although the results
for re-orientation can be applied to arbitrarily small robots and
adjustment regions, in practice for a given robot or vehicle, the region
has some minimum size. Assume that the adjustment maneuvers falls in a
circle of radius :math:`r`. Let :math:`W` be a bounded domain in
:math:`{\Bbb R}^2`, the obstacles be :math:`{\cal O}_i` and the free
space be given by :math:`\Omega = W\setminus \cup_{i}{\cal O}_i`.

.. raw:: latex

   \centering

.. figure:: motion/deltatheta
   :alt: A series of short adjustment maneuvers to re-orient the
   vehicle. [fig:deltatheta]

   A series of short adjustment maneuvers to re-orient the vehicle.
   [fig:deltatheta]

For simplicity here, we assume the domain satisfies a traversability
condition. Let :math:`D(x,r)` be the disk of radius :math:`r` centered
at :math:`x`. :math:`\Omega` is said to be disk traversable if for any
two points :math:`x_0,x_1 \in \Omega`, there exists a continuous
function :math:`p(t)\in{\Bbb R}^2` and :math:`\epsilon >0` such that
:math:`D(p(t),\epsilon)\subset\Omega` for :math:`t\in [0,1]` and
:math:`x_0=p(0)`, :math:`x_1=p(1)`. Note that :math:`p(t)` generates the
curve :math:`C` which is a path in :math:`\Omega` and the path is a
closed and bounded subset of :math:`\Omega`. Navigation along jeep
trails, bike trails and large animal trails (in our case, Cattle and
Bison) produces small corridors though the forest. Along these tracks
there is a corridor produced which we describe as disk traversable.

[disktraverseDDD] If :math:`\Omega` is disk traversable, then the DDD
and FWS vehicles can navigate to the goal ending with the correct
orientation.

**Proof:** See Chapter Appendix.

.. raw:: latex

   \FloatBarrier

Problems
--------

.. raw:: latex

   \setcounter{Exc}{0}

]

Write a Python function to compute wheel angles in the Ackerman system.
The function should have arguments (theta, l1, l2). The function should
return .

What are the motion equations for the Ackerman drive? [Meaning forward
and angular velocity as a function of wheel speed.] Assume wheel radius
is :math:`r`.

A dual Ackerman drive would steer both front and rear wheels using an
Ackerman steering approach. What would the pros and cons for this system
compared to a single Ackerman drive?

What are the motion equations for the Syncro Drive System as a function
of wheel velocity and wheel turn angle? Use :math:`r` for wheel radius.

Assume that you have a rectangular Mechanum robot with
:math:`L_1 = 0.30`\ m, :math:`L_2 = 0.20`\ m and :math:`r=0.08`\ m. Find
the path of the robot for the given wheel rotations:
:math:`\dot{\phi}_1 = 0.75*\cos(t/3.0)`,
:math:`\dot{\phi}_2 = 1.5*cos(t/3.0)`, :math:`\dot{\phi}_3 = -1.0`,
:math:`\dot{\phi}_4 = 0.5`. Start with :math:`x, y, \theta = 0` and set
:math:`t=0`, :math:`\Delta t = 0.05`. Run the simulation for 200
iterations (or for 10 seconds). Keeping the x and y locations in an
array is an easy way to generate a plot of the robot’s path. If x, y are
arrays of x-y locations then try

::

    import pylab as plt
    plt.plot(x,y,'b.')
    plt.show() 

Showing the orientation takes a bit more work. Matplotlib provides a
vector plotting method. You need to hand it the location of the vector
and the vector to be plotted, :math:`(x,y,u,v)`, where :math:`(x,y)` s
the vector location and :math:`(u,v)` are the x and y components of the
vector. You can extract those from :math:`\theta` using
:math:`u = s*\cos(\theta)` and :math:`v = s*\sin(\theta)` where
:math:`s` is a scale factor (to give a good length for the image, e.g.
0.075). The vector plot commands are then

::

    plt.quiver(u,v,c,s,scale=1.25,units='xy',color='g')
    plt.savefig('mecanumpath.pdf')
    plt.show()

::

    from math import *
    import numpy as np
    import pylab as plt

    def fk(r, L1, L2, phi1, phi2, phi3, phi4):
         vx = 0.25*r*(phi1+phi2+phi3+phi4)
         vy = 0.25*r*(-phi1+phi2+phi3-phi4)
         omega = 0.5*r*(-phi1+phi2-phi3+phi4)/(L1+L2)
         return vx, vy, omega

    def rotate(vx,vy,omega,theta):
        xdot = vx*cos(theta) - vy*sin(theta)
        ydot = vx*sin(theta) + vy*cos(theta)
        thetadot = omega
        return xdot, ydot, thetadot

    def main():
        dt = 0.05
        r = .8
        L1 = .30
        L2 = .20
        x = 0
        y = 0
        theta = 0
        t = 0
        z = 0.075
        u = []
        v = []
        s = []
        c = []
        u.append(x)
        v.append(y)
        s.append(z*sin(theta))
        c.append(z*cos(theta))
        for i in range(200):
            phi1 = 0.75*cos(t/3.0)
            phi2 = 1.5*cos(t/3.0)
            phi3 = -1.0
            phi4 = 0.5
            vx,vy,omega = fk(r, L1, L2, phi1, phi2, phi3, phi4)
            xd,yd,thd = rotate(vx,vy,omega,theta)
            x = x + xd*dt
            y = y + yd*dt
            t = t + dt
            theta = theta + thd*dt
            if (i%4==0):
              u.append(x)
              v.append(y)
              s.append(z*(sin(theta)))
              c.append(z*(cos(theta)))

        plt.quiver(u,v,c,s,scale=1.25,units='xy',color='g')
        plt.savefig('mecanumpath.pdf')
        plt.show()

    main()

|image|

Real motion and measurement involves error and this problem will
introduce the concepts. Assume that you have a differential drive robot
with wheels that are 20cm in radius and L is 12cm. Using the
differential drive code (forward kinematics) from the text, develop code
to simulate the robot motion when the wheel velocities are
:math:`\dot{\phi}_1 = 0.25t^2`, :math:`\dot{\phi}_2 = 0.5t`. The
starting location is [0,0] with :math:`\theta = 0`.

#. Plot the path of the robot on :math:`0\leq t \leq 5`. It should end
   up somewhere near [50,60].

#. Assume that you have Gaussian noise added to the omegas each time you
   evaluate the velocity (each time step). Test with :math:`\mu = 0` and
   :math:`\sigma = 0.3`. Write the final location (x,y) to a file and
   repeat for 100 simulations. Hint:

   ::

        mu, sigma = 0.0, 0.3
        xerr = np.random.normal(mu,sigma, NumP)
        yerr = np.random.normal(mu,sigma, NumP)

#. Generate a plot that includes the noise free robot path and the final
   locations for the simulations with noise. Hint:

   ::

       import numpy as np
       import pylab as plt
       ...
       plt.plot(xpath,ypath, 'b-', x,y, 'r.')
       plt.xlim(-10, 90)
       plt.ylim(-20, 80)
       plt.show()

#. Find the location means and 2x2 covariance matrix for this data set,
   and compute the eigenvalues and eigenvectors of the matrix. Find the
   ellipse that these generate. [The major and minor axes directions are
   given by the eigenvectors. Show the point cloud of final locations
   and the ellipse in a graphic (plot the data and the ellipse). Hint:

   ::

       from scipy import linalg
       from matplotlib.patches import Ellipse
       #  assume final locations are in x & y
       mat = np.array([x,y])   
       #  find covariance matrix
       cmat = np.cov(mat)    
       # compute eigenvals and eigenvects of covariance
       eval, evec = linalg.eigh(cmat) 
       #  find ellipse rotation angle 
       angle = 180*atan2(evec[0,1],evec[0,0])/np.pi   
       # create ellipse 
       ell = Ellipse((np.mean(x),np.mean(y)),
                    eval[0],eval[1],angle)  
       #  make the ellipse subplot
       a = plt.subplot(111, aspect='equal')   
       ell.set_alpha(0.1)    #  make the ellipse lighter
       a.add_artist(ell)   #  add this to the plot

#. The path of the robot:

   |image|

#. |image|

#. |image|

Derive equation `[wheelprojection] <#wheelprojection>`__.

Derive Equations
`[meccanumforwardkinematics] <#meccanumforwardkinematics>`__.

Complete the tribot example, see Figure \ `[Fig:Tribot] <#Fig:Tribot>`__
and find the forward kinematic equations of motion.

Assume that you have a square robot which is 50 cm per side and uses
four 15 cm diameter omniwheels with\ :math:`\gamma=0` configuration. The
wheels are mounted at each corner at :math:`45^\circ` to the sides:

|image|

-  Find the kinematic equations of motion.

Describe the different styles of Swedish wheel.

NA

Find the analytic wheel velocities and initial pose for a Mecanum robot
tasked to follow (:math:`r=3`, :math:`L_1 = 10`, :math:`L_2=10` all in
cm) the given paths (path units in m). Plot the paths and compare to the
actual functions to verify.

#. :math:`y=(3/2)x + 5/2`

#. :math:`y = x^{2/3}`

#. :math:`(x-3)^2/16 + (y-2)^2/9 = 1`

In STDR, drive the Mecanum robot along a square with corners (0,0),
(10,0), (10,10), (0,10), :math:`L_1 = 0.30`, :math:`L_2 = 0.20` and
:math:`r=0.08`. You should stop and “turn” at a corner, but keep the
robot faced in the x-axis direction. Drive the edges at unit speed. Use
a video screen capture program to record the results.

In STDR, drive the Mecanum robot in an infinity (:math:`\infty`) shape.
Use a video screen capture program to record the results.

.. raw:: latex

   \Closesolutionfile{Answer}

Appendix
--------

The proof for Theorem \ `[disktraverseDDD] <#disktraverseDDD>`__,
statement reproduced below, is given here.

If :math:`\Omega` is disk traversable, then the DDD and FWS vehicles can
navigate to the goal ending with the correct orientation.

**Proof:** Let :math:`C` be the path from :math:`x_0` to :math:`x_1`. At
each point of the path there exists an open disk of radius
:math:`\epsilon` which does not intersect an obstacle. The intersection
of the curve :math:`C` with the open disk induces an open set in
:math:`C`. The collection of open sets is an open cover of the curve
:math:`C`. Since the curve is a closed and bounded set, and thus
compact, there is a finite subcover of open intervals
:raw-latex:`\cite{munkres2000topology}`. These correspond to a finite
set of open disks which cover the path. The vehicle may travel a
straight line from disk center to disk center. At each center the
vehicle may reorient if required. The time limited reach for the DDD
drive is a proper subset of the FWS reach, and follows from the DDD
result.

.. |Turn geometry for the DDD (left) and FWS (right) designs. [fig:turngeo]| image:: motion/curvature
.. |Turn geometry for the DDD (left) and FWS (right) designs. [fig:turngeo]| image:: motion/curvature2
.. |image| image:: /motion/mecanumpath
.. |image| image:: solutions/MotionModel/p6-14exact
.. |image| image:: solutions/MotionModel/p6-14noise
.. |image| image:: solutions/MotionModel/p6-14ellipse
.. |image| image:: motion/omniwheelmounting

