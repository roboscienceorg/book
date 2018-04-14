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
Figure \ `[fig:DDDFWS] <#fig:DDDFWS>`__. The locking mechanism will lock
the axles in line, but leave them free to pivot with respect the frame.
This produces a robot which has DDD motion normally. When the pivot
brakes are released, then the axles can separate and the wheels move to
a configuration that allows in place rotation.

.. raw:: latex

   \centering

.. figure:: motion/split_axle
   :alt: Hybrid between the DDD and FWS designs. This places the pivots
   at the center allowing different axle angles. This design also holds
   costs by only using two brakes. [fig:DDDFWS]

   Hybrid between the DDD and FWS designs. This places the pivots at the
   center allowing different axle angles. This design also holds costs
   by only using two brakes. [fig:DDDFWS]

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
Figure \ `[fig:FWSpath] <#fig:FWSpath>`__.

.. raw:: latex

   \centering

.. figure:: motion/FWSpath1.png
   :alt: Path for the DDD-FWS hybrid system demonstrating the ability to
   steer and control the vehicle with free axle pivots. The system stops
   halfway and resets pose. [fig:FWSpath]

   Path for the DDD-FWS hybrid system demonstrating the ability to steer
   and control the vehicle with free axle pivots. The system stops
   halfway and resets pose. [fig:FWSpath]
