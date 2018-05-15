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
(Figure `[fig:angles] <#fig:angles>`__). The points at the end of the
axle are denoted by :math:`A` and :math:`B`, with :math:`A`
corresponding to the point in the positive :math:`x` direction when
:math:`\alpha=0`.


.. figure:: KinematicsFigures/angle_labels.*
   :width: 70%
   :align: center

   The axis angles. [fig:angles]

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
version in Chapter \ `[Chap:Terms] <#Chap:Terms>`__ is that the
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


.. figure:: KinematicsFigures/DDDpath1.png
   :width: 40%
   :align: center


   Path for the DDD system demonstrating the ability to steer and
   control the vehicle with free axle pivots. [fig:DDDpath]

.. _subsec:fouraxle:
