General Kinematic Modeling[Chap:Kinematics]
===========================================

Differential drive is a popular approach for lower cost and smaller
robots. When the weight or terrain demand four drive wheels, other drive
systems are better options. In this section, we approach the general
modeling kinematic problem more formally to address different wheel and
drive systems.

.. raw:: latex

   \centering

.. figure:: motion/fixedwheel
   :alt: Basic wheel and axle configuration.[wheelconfig]

   Basic wheel and axle configuration.[wheelconfig]

Fixed Wheel
-----------

We begin with the axle and wheel. Let P be the chassis center, L is the
distance from P to the wheel contact point. Define :math:`\alpha` as the
angle between the wheel axle and :math:`X_R` and :math:`\beta` as the
angle between the wheel axis :math:`A` and the axle. We also define
:math:`A` as the vector in the wheel axle direction and :math:`v` as the
orthogonal vector (the vector in the travel direction of the wheel). See
Figure \ `[wheelconfig] <#wheelconfig>`__.

.. math::

   \label{eq:axledirection}
   A = \left\langle \cos(\alpha+\beta) , \sin(\alpha+\beta) \right\rangle

.. math::

   \label{eq:wheeldirection}
   v = \left\langle \sin(\alpha+\beta) , -\cos(\alpha+\beta) \right\rangle

We will examine the linear and rotational aspects of motion separately.
They can be combined in the following manner:

.. math::

   \left\langle \dot{x}_I , \dot{y}_I , 0 \right\rangle + \left\langle 0 , 0 , \dot{\theta} \right\rangle = 
   \left\langle \dot{x}_I , \dot{y}_I , \dot{\theta} \right\rangle = \dot{\xi}_I

These equations are in the inertial or global coordinate system. Our
derivations will start in the robot or local coordinate system and so
the velocity vectors will need to be rotated into the robot coordinates:
:math:`\dot{\xi}_R = R(\theta)^{-1}\dot{\xi}_I`.

There are two basic constraints we will apply to derive the kinematic
equations. The first is *no slip* which will mean that the wheel has
perfect traction and the distance around the wheel translates into the
linear distance. This condition is broken when we lose traction on
surfaces with ice, snow or mud. The next constraint is the *no slide*
constraint. This constraint means that we always move in the direction
the wheel is pointed and not in the axle direction. This constraint can
be violated on slick surfaces where the rotational motion of the craft
causes the vehicle to slide sideways. For this section we will assume
that we have no slip and no slide conditions enforced for the basic
wheel.

We begin with the no slip constraint. All of the motion of the vehicle
must be accounted for by wheel motion. This means that the total motion
of the craft must be equal to the wheel travel and in the wheel
direction. This seems obvious and can be translated into a mathematical
constraint using the no slip condition. First, we resolve the linear
motion of the craft and then address the rotational aspect. Recall that
magnitude of the linear motion from the wheel ( in the direction of the
wheel) is :math:`r\dot{\phi}`. The direction of motion is given by
:math:`v`. Since we have both linear and rotational motion built into
:math:`\dot{\xi}`, and to keep the dimensions matching up, the vector
:math:`v` is extended to :math:`\langle v_1, v_2, 0 \rangle`. If
:math:`v` is a unit vector, :math:`\| v \| = 1`, then the projection
onto :math:`v` is

.. math:: P^1_v (u) = \frac{v\cdot u}{\| v\|^2} = v \cdot u = \left\langle \sin(\alpha+\beta) , -\cos(\alpha+\beta) , 0 \right\rangle \cdot u.

.. raw:: latex

   \centering

.. figure:: motion/fixedwheel2
   :alt: Motion in the angular direction is shown by the vector
   :math:`w`. [fig:angularradialmotion]

   Motion in the angular direction is shown by the vector :math:`w`.
   [fig:angularradialmotion]

For angular motion, we can break the motion of the wheel vector
(:math:`v`) into radial and angular components,
Figure \ `[fig:angularradialmotion] <#fig:angularradialmotion>`__. The
radial component is in the direction of the :math:`L` vector. The
angular component is :math:`w`. It is the angular and not the radial
component which will contribute to :math:`\dot{\theta}`. The angular
component must have :math:`-L \dot{\theta}` for the angular speed in the
:math:`w` direction. [The negative comes from the direction of
:math:`w`.] Projecting that speed onto :math:`v` gives
:math:`-L\cos(\beta) \dot{\theta}` which means our projection component
is :math:`P^2_v = \left\langle 0 , 0 ,  -L\cos(\beta) \right\rangle`.
Combining the projections :math:`P_v = P^1_v + P^2_v`:

.. math:: P_v =  \left\langle \sin(\alpha+\beta) , -\cos(\alpha+\beta), -L\cos(\beta) \right\rangle

and recall

.. math:: P_v [\dot{\xi}_R]  = P_v [R(\theta)^{-1}\dot{\xi}_I] .

| So we obtain:
| :math:`P_v [R(\theta)^{-1}\dot{\xi}_I]`

.. math::

   \label{wheelprojection}
    = \left\langle \sin(\alpha+\beta) , -\cos(\alpha+\beta), -L\cos(\beta) \right\rangle 
   \cdot R(\theta)^{-1}\left\langle \dot{x}_I , \dot{y}_I , \dot{\theta} \right\rangle .

For *No Slip* we have:

.. math:: P_v [R(\theta)^{-1}\dot{\xi}_I] =r\dot{\phi}

.. math::

   \Rightarrow  \left\langle \sin(\alpha+\beta) , -\cos(\alpha+\beta), -L\cos(\beta) \right\rangle 
   R(\theta)^{-1}\dot{\xi}_I = r\dot{\phi}

For *No Slide*, we want the projection in the direction of A and L to be
zero (a similar derivation as above):

.. math:: P_A [R(\theta)^{-1}\dot{\xi}_I]= 0

.. math::

   \Rightarrow  \left\langle \cos(\alpha+\beta) , \sin(\alpha+\beta), L\sin(\beta) \right\rangle 
   \cdot R(\theta)^{-1}\dot{\xi}_I= 0

Steered Wheel
~~~~~~~~~~~~~

| The only difference for steered wheels compared to fixed wheels is
  that the angle :math:`\beta` varies over time. This does not have an
  effect on the form of the equations at an instanteous time, but will
  when integrated over time.
| For *No Slip* we have:
| :math:`P_v [R(\theta)^{-1}\dot{\xi}_I]`

  .. math::

     =  \left\langle \sin(\alpha+\beta(t)) , -\cos(\alpha+\beta(t)), -L\cos(\beta(t)) \right\rangle 
     R(\theta)^{-1}\dot{\xi}_I = r\dot{\phi}

| For *No Slide*, as before we want the projection to be zero:
| :math:`P_A R(\theta)^{-1}\dot{\xi}_I`

  .. math::

     =  \left\langle \cos(\alpha+\beta(t)) , \sin(\alpha+\beta(t)), L\sin(\beta(t)) \right\rangle 
     \cdot R(\theta)^{-1}\dot{\xi}_I= 0

Castor Wheel
~~~~~~~~~~~~

| For the castor wheel, the no slip condition is the same (as the castor
  offset, d, plays no role in the motion in the direction of the wheel).
  The offset, d, does change the equations in the no slide aspect.
| For *No Slip*:

  .. math::

     \left\langle \sin(\alpha+\beta(t)) , -\cos(\alpha+\beta(t)), -L\cos(\beta(t)) \right\rangle 
     R(\theta)^{-1}\dot{\xi}_I = r\dot{\phi}

For *No Slide*:

.. math::

   \left\langle \cos(\alpha+\beta(t)) , \sin(\alpha+\beta(t)), d + L\sin(\beta(t)) \right\rangle 
   \cdot R(\theta)^{-1}\dot{\xi}_I + d\dot{\beta} = 0

.. raw:: latex

   \centering

.. figure:: motion/castorwheel
   :alt: Castor Wheel

   Castor Wheel

Omni, Swedish, or Mecanum Wheels
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. raw:: latex

   \centering

.. figure:: motion/swedish_angle
   :alt: Swedish Wheel

   Swedish Wheel

| Let :math:`\gamma` be the angle between the roller axis and wheel
  plane (plane orthogonal to the wheel axis) For *No Slip*:

  .. math::

     \left\langle \sin(\alpha+\beta+\gamma) , -\cos(\alpha+\beta+\gamma), -L\cos(\beta +\gamma) \right\rangle 
     R(\theta)^{-1}\dot{\xi}_I

  \ :math:`= r\dot{\phi}\cos(\gamma)`
| For *No Slide*:

  .. math::

     \left\langle \cos(\alpha+\beta +\gamma) , \sin(\alpha+\beta+\gamma),  L\sin(\beta + \gamma) \right\rangle 
     \cdot R(\theta)^{-1}\dot{\xi}_I

  \ :math:`= r\dot{\phi}\sin(\gamma) + r_{sw}\dot{\phi}_{sw}`
| Note that since :math:`\phi_{sw}` is free (to spin), the no slide
  condition is not a constraint in the same manner as the fixed or
  steered wheels.

Multiple Wheel Model and Matrix Formulation
-------------------------------------------

Since nearly all the robots we will work with have three or more wheels.
The equations we derived above can be combined to build a complete
kinematic model. We begin with some basic variables that define the
system.

-  Let :math:`N` denote the total number of wheels

-  Let :math:`N_f` denote the number of fixed wheels

-  Let :math:`N_s` denote the number of steerable wheels

-  Let :math:`\phi_f(t)` and :math:`\beta_f` be the fixed wheel angular
   velocity and wheel position.

-  Let :math:`\phi_s(t)` and :math:`\beta_s(t)` be the steerable wheel
   angular velocity and wheel position.

We bundle the latter two values in a vector for notational ease:

.. math::

   \phi (t) = ( \phi_{f,1}(t), 
   \phi_{f,2}(t), \phi_{f,3}(t), ..., \phi_{s,1}(t), \phi_{s,2}(t), ...)

.. math::

   \beta (t) = ( \beta_{f,1}(t), 
   \beta_{f,2}(t), \beta_{f,3}(t), ..., \beta_{s,1}(t), \beta_{s,2}(t), ...)

Next we collect the no slip constraints, the equations derived above for
the various drive types and place them in a matrix:

.. math:: J_1 R(\theta)^{-1}\dot{\xi}_I = \begin{bmatrix} J_{1f} \\ J_{1s}\end{bmatrix} R(\theta)^{-1} \dot{\xi}_I= J_2 \dot{\phi}

where :math:`J_1` is the matrix with rows made up of the rolling
constraints and :math:`J_2` is a diagonal matrix made from wheel
diameters. In a similar manner we can bundle up the no slide constraints
(fixed and steered):

.. math:: C_1 R(\theta)^{-1}\dot{\xi}_I = \begin{bmatrix} C_{1f} \\ C_{1s}\end{bmatrix} R(\theta)^{-1} \dot{\xi}_I = 0.

This is matrix shorthand to address the kinematic models for a variety
of systems.

.. math:: \begin{bmatrix} J_1 \\ C_1 \end{bmatrix} R(\theta)^{-1} \dot{\xi}_I = \begin{bmatrix} J_2 \\ 0\end{bmatrix} \dot{\phi}

Differential Drive - Rederivation
---------------------------------

To get a feel for the more general approach above, it is worthwhile to
rederive the equations for the differential drive robot. This allows us
to check our result as well as see how the matrices are constructed.
From Figure \ `[ddrive_rederivation] <#ddrive_rederivation>`__ we have
for the left wheel: :math:`\alpha = \pi/2`, :math:`\beta = 0`; and for
the right wheel: :math:`\alpha = -\pi/2`, :math:`\beta = \pi` (to be
consistent with the coordinate system).

.. raw:: latex

   \centering

.. figure:: motion/ddexample
   :alt: The differential drive robot dimensions and variables.
   [ddrive_rederivation]

   The differential drive robot dimensions and variables.
   [ddrive_rederivation]

Recall the left wheel rolling constraint is given by

.. math::

   \left\langle \sin(\alpha+\beta) , -\cos(\alpha+\beta), -L\cos(\beta) \right\rangle = 
   \left\langle 1 , 0, -L \right\rangle

and the right wheel rolling constraint is

.. math::

   \left\langle \sin(\alpha+\beta) , -\cos(\alpha+\beta), -L\cos(\beta) \right\rangle =
   \left\langle 1 , 0, L \right\rangle .

From these two equations we can form the rolling constraint matrix:

.. math:: J_1 = \begin{bmatrix} 1 & 0 & -L \\ 1 & 0 & L \end{bmatrix}

In a similar manner, recall that the left wheel sliding constraint is

.. math::

   \left\langle \cos(\alpha+\beta) , \sin(\alpha+\beta), L\sin(\beta) \right\rangle =
   \left\langle 0 , 1, 0 \right\rangle ,

and the right wheel sliding constraint is

.. math::

   \left\langle \cos(\alpha+\beta) , \sin(\alpha+\beta), L\sin(\beta) \right\rangle
   = \left\langle 0 , 1, 0 \right\rangle  .

Again in a similar manner we can form the sliding constraint matrix:

.. math:: C_1 = \begin{bmatrix} 0 & 1 & 0 \\ 0 & 1 & 0 \end{bmatrix}.

Since the two rows are linearly dependent, we only need to keep one row,
so the matrix looks like

.. math:: C_1 = \begin{bmatrix} 0 & 1 & 0 \end{bmatrix}

The two matrices are stacked to form a single matrix model:

.. math:: \begin{bmatrix}  J_1 \\[4mm] C_1 \end{bmatrix} =  \begin{bmatrix} 1 & 0 & -L \\ 1 & 0 & L \\ 0 & 1 & 0 \end{bmatrix}.

The same can be done with the right hand side arrays

.. math:: \begin{bmatrix} J_2 \\ 0\end{bmatrix} = \begin{bmatrix} r & 0 \\ 0 & r \\0& 0\end{bmatrix}.

The resulting motion model is

.. math::

   \begin{bmatrix} 1 & 0 & -L \\ 1 & 0 & L \\ 0 & 1 & 0 \end{bmatrix} R(\theta)^{-1} \dot{\xi}_I  
   = \begin{bmatrix} r & 0 \\ 0 & r \\0& 0\end{bmatrix} \dot{\phi}

Expanding

.. math::

   \begin{bmatrix} 1 & 0 & -L \\ 1 & 0 & L \\ 0 & 1 & 0 \end{bmatrix} 
    \begin{bmatrix} \cos \theta & \sin \theta & 0 \\ -\sin \theta &
   \cos \theta & 0 \\  0 & 0 & 1  \end{bmatrix}
    \dot{\xi}_I  
   = \begin{bmatrix} r & 0 \\ 0 & r \\0& 0\end{bmatrix} 
   \begin{bmatrix}\dot{\phi}_2 \\ \dot{\phi}_1\end{bmatrix}

| To be consistent with the previous example, we had the left wheel as
  (2) and the right wheel as (1) - hence the reverse ordering on the
  :math:`\phi` terms.
| This is the system to solve. Invert the left hand array first, then
  invert the rotation matrix.

Working out the details:

.. math::

   \begin{bmatrix} \cos \theta & \sin \theta & 0 \\ -\sin \theta &
   \cos \theta & 0 \\  0 & 0 & 1  \end{bmatrix}
    \dot{\xi}_I  
   = \begin{bmatrix} 1 & 0 & -L \\ 1 & 0 & L \\ 0 & 1 & 0 \end{bmatrix}^{-1}
   \begin{bmatrix} r & 0 \\ 0 & r \\0& 0\end{bmatrix} 
   \begin{bmatrix}\dot{\phi}_2 \\ \dot{\phi}_1\end{bmatrix}

.. math::

   \dot{\xi}_I =  \begin{bmatrix}\dot{x} \\ \dot{y} \\ \dot{\theta} \end{bmatrix}
   = \begin{bmatrix} \cos \theta & \sin \theta & 0 \\ -\sin \theta &
   \cos \theta & 0 \\  0 & 0 & 1  \end{bmatrix}^{-1}
   \begin{bmatrix} 1 & 0 & -L \\ 1 & 0 & L \\ 0 & 1 & 0 \end{bmatrix}^{-1}
   \begin{bmatrix} r & 0 \\ 0 & r \\0& 0\end{bmatrix} 
   \begin{bmatrix}\dot{\phi}_2 \\ \dot{\phi}_1\end{bmatrix}

.. math::

   \begin{bmatrix}\dot{x} \\ \dot{y} \\ \dot{\theta} \end{bmatrix}
   = \begin{bmatrix} \cos \theta & -\sin \theta & 0 \\ \sin \theta &
   \cos \theta & 0 \\  0 & 0 & 1  \end{bmatrix}
   \begin{bmatrix} 1/2 & 1/2 & 0 \\ 0 & 0 & 1 \\ -1/(2L) & 1/(2L) & 0 \end{bmatrix}
   \begin{bmatrix} r\dot{\phi}_2 \\ r\dot{\phi}_1 \\ 0\end{bmatrix}

and finally ....

.. math::

   \begin{bmatrix}\dot{x} \\ \dot{y} \\ \dot{\theta} \end{bmatrix}
   = \begin{bmatrix} \cos \theta & -\sin \theta & 0 \\ \sin \theta &
   \cos \theta & 0 \\  0 & 0 & 1  \end{bmatrix}
    \begin{bmatrix} \frac{r}{2}\dot{\phi}_1 + \frac{r}{2}\dot{\phi}_2 \\ 0   \\
    -\frac{r}{2L}\dot{\phi}_2 + \frac{r}{2L}\dot{\phi}_1  \end{bmatrix}

.. math::

   = \begin{bmatrix}  \frac{r}{2}\left(\dot{\phi}_1 + \dot{\phi}_2\right)\cos \theta \\ 
   \frac{r}{2}\left(\dot{\phi}_1 + \dot{\phi}_2\right)\sin \theta \\ 
   \frac{r}{2L}\left(\dot{\phi}_1 -\dot{\phi}_2\right) \end{bmatrix}

(and you didn’t think this was going to work out, did you.) You may
apply this machinery to other systems as well.

Omniwheel Example
-----------------

For this example we look at a Swedish three wheel robot, Figure
`[Fig:Tribot] <#Fig:Tribot>`__. We use an unsteered :math:`90^\circ`
Swedish wheel, so :math:`\beta_i =0` and :math:`\gamma_i = 0` for all
:math:`i`. Going counterclockwise in the figure, we have
:math:`\alpha_1 = \pi/3`, :math:`\alpha_2 = \pi` and
:math:`\alpha_3 = -\pi/3`. You will note that the :math:`C_1` matrix is
of zero rank and so the sliding constraint does not contribute to (nor
is needed for) the model. The equations for motion then are

.. math:: \dot{\xi}_I = R(\theta) J^{-1}_{1f}J_2\dot{\phi}

\ where

.. math::

   J_{1f} = \begin{bmatrix} \sqrt{3}/2 & -1/2 & -L \\ 0 & 1 & -L \\ -\sqrt{3}/2 & -1/2 & -L \end{bmatrix},
    \quad 
   J_2 = \begin{bmatrix} r & 0 & 0 \\ 0 & r & 0 \\ 0 & 0 & r \end{bmatrix}

.. raw:: latex

   \centering

.. figure:: motion/tribot
   :alt: The Omniwheel can be configured in a three wheel [Fig:Tribot]

   The Omniwheel can be configured in a three wheel [Fig:Tribot]

.. _subsec:twoaxle:

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

.. raw:: latex

   \centering

.. figure:: motion/angle_labels
   :alt: The axis angles. [fig:angles]

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

.. raw:: latex

   \centering

.. figure:: motion/DDDpath1.png
   :alt: Path for the DDD system demonstrating the ability to steer and
   control the vehicle with free axle pivots. [fig:DDDpath]

   Path for the DDD system demonstrating the ability to steer and
   control the vehicle with free axle pivots. [fig:DDDpath]

.. _subsec:fouraxle:

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

Maneuverability
---------------

In this section we study the ability of a particular wheel configuration
to move around in the environment. Each wheel must respect its sliding
constraint which limits the motion in the workspace. We will see that
the ability to move in the environment is a combination of the mobility
provided by the sliding constraint and the available steering.

The basic wheel constraint prevents motion in the direction of the axle.
Recall that

.. math:: \begin{bmatrix} C_{1f} \\ C_{1s} \end{bmatrix}R(\theta)^{-1}\dot{\xi}_I = 0

which means that :math:`R(\theta)^{-1}\dot{\xi}_I` is in the nullspace
of the array
:math:`C_1 = \begin{bmatrix} C_{1f} \\ C_{1s} \end{bmatrix}`. The
*Nullspace* of the matrix :math:`A` is the collection of vectors
:math:`v` such that :math:`Av=0`.

We will see that this idea can be related to the ICR (instantaneous
center of rotation). The axle of a standard wheel can be extended into
what is called the zero motion line. For multiple fixed wheels (this can
be extended to steered wheels), the zero motion lines intersect at the
ICR. Recall that if the zero motion lines do not intersect at a single
point then the craft does not move. This was argued in the case of the
standard automobile design, four wheels, two fixed and two steered. We
argued that the four wheels must all line up on two concentric circles
and the wheel direction must be tangent to the circle it was on. If the
ICR is a finite value in the plane, then we have a circle with finite
radius. If the ICR is out at infinity, then the radius of the circle is
infinite and we are traveling on a straight line.

To explore this connection, we delve more into the linear algebra of the
constraint matrices. The kinematic formulas we derived are a function of
the wheel constraints. It is the number of independent constraints that
are important are contribute to the robot kinematics. This is connected
to the rank of the constraint matrix. The rank of :math:`C_1`

is the number of independent constraints or the number of linearly
independent rows. The greater the rank, the more constrained the
vehicle. Clearly

.. math:: 0 \leq \mbox{rank}(C_1) \leq 3.

Each constraint is related to the wheel’s zero motion line. The zero
motion line is the axle direction which from
equation \ `[eq:axledirection] <#eq:axledirection>`__:
:math:`A = \left\langle \cos(\alpha+\beta) , \sin(\alpha+\beta) \right\rangle`.
Contrary to what you may expect, adding steerable wheels increases the
number of constraints. Keep in mind adding omniwheels or Mecanum wheels
does not increase the number of constraints since those do not enforce a
no slide condition.

Example for the differential drive: :math:`\alpha_1=\pi/2`,
:math:`\beta_1 = 0`, :math:`\alpha_2=-\pi/2`, :math:`\beta_2 = 0`

.. math:: C_1 = \begin{bmatrix} 0 & 1 & 0\\ 0 & -1 & 0 \end{bmatrix}, \quad \mbox{rank}(C_1) = 1.

The two wheels in this case share the zero motion line. And so we only
have one zero motion line. Using the formula for :math:`A` above we gain
:math:`A = \left\langle 0 , 1 \right\rangle` which is clearly the first
two components of the constraint. Not surprising as this is by
construction. The zero motion line is then the vertical line spanned by
:math:`A`.

.. raw:: latex

   \centering

.. figure:: motion/bikeicr
   :alt: [fig:fixedbikewheel]A fixed turn bike wheel.

   [fig:fixedbikewheel]A fixed turn bike wheel.

Next we examine the bicycle. For the bike, we have :math:`L_1 = L_2=L`,
:math:`\beta_1=\beta_2 = \pi/2`, :math:`\alpha_1=0`,
:math:`\alpha_2=\pi`. From these two wheels we obtain the two
constraints which are loaded into :math:`C_1`.

.. math:: C_1 = \begin{bmatrix} 0 & 1 & L\\ 0 & -1 & L \end{bmatrix}, \quad \mbox{rank}(C_1) = 2.

The first two components of the constraints agree with the differential
drive example, but we have a different result for the constraint matrix.

An Ackerman drive can be modeled as two bicycles attached together. To
drive the front wheels must respect the Ackerman angle constraint,
Figure \ `[fig:ackermanzeromotion] <#fig:ackermanzeromotion>`__. Because
of the Ackerman steering angle constraint, with the back wheels and one
front free, you have selected the location of the ICR. This means that
the other steered wheel zero motion line is prescribed. So, it must mean
that the four rows in the constraint matrix are linearly dependent. We
know that the two rows for the fixed wheels are the same line. This
tells us one fixed wheel and one steered wheel are sufficient. Thus we
have two linearly independent rows for the the constraint matrix.

.. raw:: latex

   \centering

.. figure:: motion/ackermanzeromotion
   :alt: The Ackerman Design[fig:ackermanzeromotion]

   The Ackerman Design[fig:ackermanzeromotion]

The algebra for the general case is difficult, however, we can put some
values on this diagram. Let right front wheel (steered) be wheel 1, left
front wheel be wheel 2 and the left rear wheel be wheel 3. We don’t need
the fourth wheel. The diagram has wheels 1 and 2 fully labeled and for
wheel 3, the same conventions are followed. Assume that :math:`A = 5`,
:math:`B=3` and :math:`W=2`. This means that

.. math::

   \delta_1 = \mbox{atan2}(B,A) = \mbox{atan2}(3,5), \quad 
   \delta_2 = \mbox{atan2}(B,A+W) = \mbox{atan2}(3,7), \quad 
   \delta_3=0

and

.. math:: \alpha_1 =  -\mbox{atan2}(W/2,B/2) = -\mbox{atan2}(1,3/2)

.. math:: \alpha_2 =  -\alpha_1,  \quad   \alpha_3 = \pi - \alpha_2

This provides us with

.. math::

   \beta_1 = -\pi/2 - \alpha_1 - \delta_1, \quad  
   \beta_2 = \pi/2 - \alpha_2 - \delta_2, \quad 
   \beta_3 = -\pi/2 - \alpha_3 - \delta_3

we can plug each into the constraint equation

.. math:: \left\langle \cos(\alpha+\beta) , \sin(\alpha+\beta), L\sin(\beta) \right\rangle

to build the matrix C. This is done with the following program.

::

     import numpy as np
    import numpy.linalg as lin
    import math

    A = 5
    B = 3
    W = 2
    d1 = math.atan2(B,A)
    d2 = math.atan2(B,A+W)
    d3 = 0.0
    a1 = -math.atan2(W/2.0,B/2.0)
    a2 = ath.atan2(W/2.0,B/2.0)
    a3 = math.pi - a2

    b1 = -math.pi/2.0 - a1 - d1
    b2 = math.pi/2.0 - a2 - d2
    b3 = -math.pi/2.0 - a3 - d3
    L = math.sqrt(W*W+B*B)/2.0

    C = np.array([
    [math.cos(a1+b1) , math.sin(a1+b1), L*math.sin(b1)],
    [math.cos(a2+b2) , math.sin(a2+b2), L*math.sin(b2)],
    [math.cos(a3+b3) , math.sin(a3+b3), L*math.sin(b3)]])

    print C

    r = lin.matrix_rank(C)
    print r

The output

::

    [[ -5.14495755e-01  -8.57492926e-01  -1.80073514e+00]
     [  3.93919299e-01   9.19145030e-01   9.84798246e-01]
     [ -3.82856870e-16  -1.00000000e+00   1.50000000e+00]]
    2

In general, if the rank of :math:`C_1` is greater than one then the
vehicle at best can only travel a line or a circle. Rank = 3 means no
motion at all. We can define the *degree of mobility* =
:math:`\delta_m`, also known as *DDOF - differential degrees of
freedom*,

.. math:: \delta_m \equiv \mbox{dim} {\cal N}(C_1) = 3 - \mbox{rank}(C_1)

 This is the robot’s degrees of freedom or a measure of the local
mobility of the robot.

For a differential drive the degree of mobility is :math:`\delta_m = 2`.
We define the *degree of steerability*, :math:`\delta_s` as

.. math:: \delta_s \equiv \mbox{rank} (C_{1,s}).

Note that increasing this rank increases steerability, but since
:math:`C_1` contains :math:`C_{1,s}`, it will decrease mobility. We can
also define DOF, *the degrees of freedom*, which is based on the
workspace dimension which is two or three.

We have :math:`N_f = 2` and :math:`N_s=2`.

.. math:: \mbox{rank}(C_{1f})=1

(since they share an axle). Since all axle lines must intersect in a
point for the vehicle to move (example above), once you prescribe on
wheel, you have prescribed both wheels.

.. math:: \mbox{rank}(C_{1s})=1

So:

.. math:: \mbox{rank}\begin{bmatrix} C_{1f} \\ C_{1s}\end{bmatrix} = 2

Thus :math:`\delta_m=1` and :math:`\delta_s =1`.

We can contrast this with the equal steer angle vehicle. This has
:math:`N_f = 2` and :math:`N_s=2` just like the Ackerman. However the
three rows are linearly independent (rank is 3). This provides us with
:math:`\delta_m=0` and :math:`\delta_s =1`.

An important concept is the Degree of Maneuverability, :math:`\delta_M`,

.. math:: \delta_M = \delta_m + \delta_s.

This measures the degrees of freedom the robot can operate in a global
sense. So even if the robot does not have full mobility in a local
sense, the robot can operate through a series of movements in this
larger sense. A differential drive robot for example
:math:`\delta_M = \delta_m + \delta_s = 2`.

Degree of Maneuverability is equivalent to control degrees of freedom. A
*holonomic* robot is a robot with ZERO nonholonomic constraints. A
holonomic kinematic constraint can be expressed as an explicit function
of position variables alone. A robot is holonomic if and only if DDOF =
DOF. A robot is said to be omnidirectional if it is holonomic and DDOF =
3. This means that the robot can *Maneuver* and *Orient*.

.. raw:: latex

   \centering

| |Summary of some common configurations.[fig:summaryconfigurations]|

+-----------------+-----------------+-----------------+-----------------+
| Configuration   | Maneuverability | Mobility        | Steerability    |
+=================+=================+=================+=================+
| A. Omniwheel    | :math:`\delta_M | :math:`\delta_m | :math:`\delta_s |
|                 |  = 3`           |  = 3`           |  = 0`           |
+-----------------+-----------------+-----------------+-----------------+
| B. Differential | :math:`\delta_M | :math:`\delta_m | :math:`\delta_s |
|                 |  = 2`           |  = 2`           |  = 0`           |
+-----------------+-----------------+-----------------+-----------------+
| C. Omni-Steer   | :math:`\delta_M | :math:`\delta_m | :math:`\delta_s |
|                 |  = 3`           |  = 2`           |  = 1`           |
+-----------------+-----------------+-----------------+-----------------+
| D. Tricycle     | :math:`\delta_M | :math:`\delta_m | :math:`\delta_s |
|                 |  = 2`           |  = 1`           |  = 1`           |
+-----------------+-----------------+-----------------+-----------------+
| E. Two-Steer    | :math:`\delta_M | :math:`\delta_m | :math:`\delta_s |
|                 |  = 3`           |  = 1`           |  = 2`           |
+-----------------+-----------------+-----------------+-----------------+

.. raw:: latex

   \FloatBarrier

Problems
--------

.. raw:: latex

   \setcounter{Exc}{0}

]

.. |Summary of some common configurations.[fig:summaryconfigurations]| image:: motion/summary

