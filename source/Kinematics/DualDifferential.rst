Dual Differential Drive
-----------------------

This section derives the kinematics for a robot with a single axle. This
will be used to extend the differential drive to the dual differential
drive. All results are with respect to the local robot coordinate
system, with :math:`y` the forward direction, :math:`z` up, and
:math:`x` defined according to the right hand rule. The total length of
the axle is given by :math:`2L`, the robot angle by :math:`\theta`, and
the angle of the axle with respect to the robot given by :math:`\alpha`,
with :math:`\alpha=0` aligning the axle with the :math:`x` axis
:numref:`fig:angles`. The points at the end of the
axle are denoted by :math:`A` and :math:`B`, with :math:`A`
corresponding to the point in the positive :math:`x` direction when
:math:`\alpha=0`.

.. _`fig:angles`:
.. figure:: KinematicsFigures/angle_labels.*
   :width: 70%
   :align: center

   The axis angles.

Simple planar kinematics gives the following relationships between the
velocities at points :math:`A` and :math:`B` and the robot motion. Let
:math:`x,y` denote the center of the axle.

.. math::

   v_{A_x} = \dot{x}-L\dot{\alpha}\sin\alpha, \quad
   v_{A_y} = \dot{y}+L\dot{\alpha}\cos\alpha

Incorporating the non-holonomic constraint on wheel velocity directions
yields

.. math::

   V_A\sin\alpha = L\dot{\alpha}\sin\alpha-\dot{x}, \quad
   V_A\cos\alpha = \dot{y}+L\dot{\alpha}\cos\alpha

where :math:`V_A` is the magnitude of the axle tip velocity. Similarly,
for point :math:`B`

.. math::

   V_B\sin\alpha = -L\dot{\alpha}\sin\alpha-\dot{x}, \quad
   V_B\cos\alpha = \dot{y}-L\dot{\alpha}\cos\alpha

Combining the equations for points :math:`A` and :math:`B` results in

.. math::

   \dot{y} = \frac{V_A+V_B}{2}\cos\alpha, \quad
   \dot{x} = -\frac{V_A+V_B}{2}\sin\alpha, \quad
   \dot{\alpha} = \frac{V_A-V_B}{2L}

The major difference with this current derivation and our previous
version in the Terms Chapter  is that the
coordinate system is rotated by :math:`90^\circ` compared to what we
use.

The analysis now can be easily extended to the case of two axles. Let
the pivots for each of the two axles be separated from the robot
centroid by distance :math:`d` in the :math:`y` direction. Let :math:`A`
and :math:`B` denote the velocities of wheel for the axle offset in the
positive :math:`y` direction from the centroid and :math:`C` and
:math:`D` denote the velocities of wheel for the axle offset in the
negative :math:`y` direction from the centroid. The angle of the front
axle with respect to the robot is given by :math:`\alpha`, whereas the
angle of the rear axle with respect to the robot is given by
:math:`\beta`. Then

.. math::

   \begin{array}{l} V_A\sin\alpha = L\dot{\alpha}\sin\alpha-\dot{x}+d\dot{\theta}, \quad
   V_A\cos\alpha = \dot{y}+L\dot{\alpha}\cos\alpha \\[4mm]
   V_B\sin\alpha = -L\dot{\alpha}\sin\alpha-\dot{x}+d\dot{\theta}, \quad
   V_B\cos\alpha = \dot{y}-L\dot{\alpha}\cos\alpha \end{array}

for the front axle and

.. math::

   \begin{array}{l}  V_C\sin\beta = L\dot{\beta}\sin\beta-\dot{x}-d\dot{\theta}, \quad
   V_C\cos\beta = \dot{y}+L\dot{\beta}\cos\beta \\[4mm]
   V_D\sin\beta = -L\dot{\beta}\sin\beta-\dot{x}-d\dot{\theta}, \quad
   V_D\cos\beta = \dot{y}-L\dot{\beta}\cos\beta\end{array}

for the rear axle.

Combining equations for the dual differential drive case results in

.. math:: \dot{y} = \frac{V_A+V_B}{2}\cos\alpha=\frac{V_C+V_D}{2}\cos\beta

Note that this equation places a constraint on the relationship between
front and rear axle velocities.

.. math::

   \begin{array}{l}
   \displaystyle \dot{\theta} = \frac{(V_A+V_B)\sin\alpha-(V_C+V_D)\sin\beta}{4d}\\[4mm]
   \displaystyle \dot{x} = -\frac{(V_a+V_B)\sin\alpha+(V_C+V_D)\sin\beta}{4}\\[4mm]
   \displaystyle \dot{\alpha} = \frac{V_A-V_B}{2L}, \quad
   \dot{\beta} = \frac{V_C-V_D}{2L}\end{array}

Implementation of the forward kinematics is easily done and can be
simulated for sample wheel speeds without use of the brake.
Figure \ `[fig:DDDpath] <#fig:DDDpath>`__, shows the resulting path for
sample wheel inputs which demonstrate the ability to steer the craft.
The wheel speeds for this figure are

.. math::

   \begin{array}{l}
   V_A, V_B =  5t - t^2 + 1.5 \mp \sin(t), \quad 0 \leq t \leq 5 \\[3mm]
   V_C, V_D = (5t - t^2)\cos(\alpha)/\cos(\beta) \pm \sin(t) ,   \quad 0 \leq t \leq 5 .
   \end{array}

.. _`fig:DDDpath`:
.. figure:: KinematicsFigures/DDDpath1.png
   :width: 40%
   :align: center


   Path for the DDD system demonstrating the ability to steer and
   control the vehicle with free axle pivots.



Four Axle Robot or the Four Wheel Steer Robot
---------------------------------------------

The case of a four axle robot is very similar to the dual differential
drive case. The angles of the four axles are :math:`\alpha`,
:math:`\beta`, :math:`\gamma`, and :math:`\delta`, with :math:`\alpha`
representing the angle of the axle in the first quadrant, :math:`\beta`
the angle of the axle in the second quadrant, :math:`\gamma` the angle
of the axle in the fourth quadrant, and :math:`\delta` the angle of the
axle in the third quadrant. Let the hinge point be located by vector
:math:`\vec{r}` with components of magnitude :math:`r_x` and :math:`r_y`
with respect to the centroid of the robot, and have the wheel located at
distance :math:`L` from the hinge. Then the velocities of the ends of
the axles are given below. The constraints for the front two axles are:

.. math::

   \begin{array}{l}
   V_{A_x} =\dot{x}-r_y\dot{\theta}-\dot{\alpha}L\sin\alpha = -V_A\sin\alpha, \\[4mm]
   V_{A_y} = \dot{y}+r_x\dot{\theta}+\dot{\alpha}L\cos\alpha = V_A\cos\alpha , \\[4mm]
   V_{B_x} =\dot{x}-r_y\dot{\theta}+\dot{\beta}L\sin\beta = -V_B\sin\beta, \\[4mm]
   V_{B_y} = \dot{y}-r_x\dot{\theta}-\dot{\beta}L\cos\beta = V_B\cos\beta , \end{array}

and the constraints for the rear two axles are:

.. math::

   \begin{array}{l}
   V_{C_x} =\dot{x}+r_y\dot{\theta}+\dot{\gamma}L\sin\gamma = -V_C\sin\gamma, \\[4mm]
   V_{C_y} = \dot{y}-r_x\dot{\theta}-\dot{\gamma}L\cos\gamma= V_C\cos\gamma , \\[4mm]
   V_{D_x} =\dot{x}+r_y\dot{\theta}-\dot{\delta}L\sin\delta = -V_C\sin\delta, \\[4mm]
   V_{D_y} = \dot{y}+r_x\dot{\theta}+\dot{\delta}L\cos\delta= V_C\cos\delta  . \end{array}

These equations reduce to the DDD case when the offset is removed, i.e.,
when pivots are located in the center of the robot. The consequence is
that the constraint these equations present is :math:`\alpha=\beta` and
:math:`\gamma = \delta`. For any other angular relationships the wheels’
kinematic constraints would conflict and the robot would be locked in
place. In the general case, we must have a relation
:math:`\alpha=\beta + \epsilon_1` and
:math:`\gamma = \delta+ \epsilon_2` where :math:`\epsilon_1`,
:math:`\epsilon_2` are the corrections due to the offset.

However, clearly there are admissible motions, such as the case in which

.. math:: \begin{array}{l} V_{A_y} = V_{B_y} = V_{C_y} = V_{D_y} = \dot{y},\\[4mm]V_{A_x} = V_{B_x} = V_{C_x} = V_{D_x} = 0, \\[4mm]\dot{\theta} = \alpha = \beta = \gamma = \delta = \dot{x} = 0.\end{array}

\ In other words, a vehicle that already has forward motion could
maintain it with all brakes unlocked. Given the constraint that the
angles must remain equal, the kinematics of the FWS robot are identical
to those of the DDD robot as expected.

The system that emerges is one where the split axles are connected to
the center of the robot as shown in
:numref:`fig:DDDFWS`. The locking mechanism will lock
the axles in line, but leave them free to pivot with respect the frame.
This produces a robot which has DDD motion normally. When the pivot
brakes are released, then the axles can separate and the wheels move to
a configuration that allows in place rotation.


.. _`fig:DDDFWS`:
.. figure:: KinematicsFigures/split_axle.*
   :width: 40%
   :align: center

   Hybrid between the DDD and FWS designs. This places the pivots at the
   center allowing different axle angles. This design also holds costs
   by only using two brakes.

So, based on the kinematics, we see that linear motion is possible for
the both vehicles when the pivot brakes are locked or free. The DDD
vehicle can also turn without locks on the pivots. The kinematic
constraint induced by the body connection between front and rear axles
places constraints on wheel motion (as expected). Violating these will
cause wheel slip and slide. You can think of DDD motion as simply two
differential drive robots moving in tandem.

The FWS system is more complicated and the dynamics do allow unlocked
pivots during a turn as long as not all are unlocked. So, dynamic turns
can be performed by acting on axles sequentially. One may employ motion
sequences such as

#. Unlock rear axle pivots

#. Change rear wheel velocities

#. Lock rear axle pivots

#. Unlock front axle pivots

#. Change front wheel velocities

#. Lock front axle pivots

to turn the robot without performing a complete stop. This configuration
works very much like an Ackerman drive other than the ability to stop
and rotate in place. A simulation is shown of the DDD-FWS hybrid in
:numref:`fig:FWSpath`.

.. _`fig:FWSpath`:
.. figure:: KinematicsFigures/FWSpath1.png
   :width: 60%
   :align: center

   Path for the DDD-FWS hybrid system demonstrating the ability to steer
   and control the vehicle with free axle pivots. The system stops
   halfway and resets pose.
