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
