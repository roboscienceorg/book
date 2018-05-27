Manipulators: Arms and Legs
===========================

.. Rewrite this chapter

Legs
----

We now move over to a more biological approach. The use of legs in
locomotion has been very successful. Animals range in sizes from single
millimeter to multiple meter range. Legs have proved invaluable at many
space and speed dimensions. In the robotics view, a leg is articulated
manipulator (serial chain). This means that it requires many of the same
controls that a robot arm would require, but adapted to the specific
task of moving the robot. Although a hobby level robot can implement 6
simple articulators to produce insect like motion, getting natural,
efficient, robust and fast bipedal and quadrupedal motion is very
difficult.

Aspects of Articulated Based Motion
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Engineers are experimenting with 2 - 8 leg designs to learn more about
articulator locomotion as well as what it can teach us about the animals
that have similar designs. We know that for static stability, at least
three points of contact are required. So, three legs are required to
stand still. When we move, some of the legs must move forward to
initiate the gait. If we want three points on the ground, this means six
legs is the minimum for stable walking. Robots using six or more legs do
not need a balance control system.

Systems using four legs have a static balance when still, but must use a
dynamic control approach to maintain balance during the step. This is a
variation of the inverted pendulum problem which is discussed in the
chapter on motion control, Chapter \ `[Chap:Control] <#Chap:Control>`__.
Finally our system of using two legs requires a control system for
moving and standing.

.. raw:: latex

   \centering

|image|

Increasing the number of legs will increase weight, power requirements,
coordination problems and control hardware. Adding legs, as mentioned
before, will help with stability and provide a greater number of contact
points. Increasing ground contact can increase traction and robustness,
depending on the contact area, angle of contact, friction, surface
roughness and friction. For static stability we want the center of mass
to fall inside the span of the legs. Unlike wheels, the center of mass
for a leg moves up and down as the robot walks. This decreases power
efficiency, increases the chance of a shifting center of gravity which
makes path planing more difficult. In complicated environments, one may
have to plan the motion of each articulator. The configuration space for
a 6 legged robot with three servos is 18, which is rather large for path
planning and might fail due to size.

.. raw:: latex

   \centering

.. figure:: motion/legjoint.png
   :alt: Leg joints and their use.

   Leg joints and their use.

-  | Two DOF is required:
   | lift and swing

-  | Three DOF is needed in most cases:
   | lift, swing and position

-  | Fourth DOF is needed for stability:
   | ankle joint - improves balance and walking

Consider a humanoid robot. How complex are they? A leg has a hip (two
degrees of freedom) and knee plus ankle which gives another two degrees
of freedom. So a leg is a 4 DOF (degrees of freedom) structure. An arm
is at least 5 DOF. A head has pan and tilt so at least 2 DOF. This adds
up to 20 DOF for a humanoid style robot. Motion planning and control is
challenging for high DOF robots. Good tools for doing this is an active
area of research.

.. raw:: latex

   \centering

|image|

There are several attempts to combine legs and wheels. The Shrimp is one
such design, Figure \ `[shrimp] <#shrimp>`__. Many fun and interesting
innovations come from the suspension system. Adaptive (passive or
active) suspension is a current area of development,
Figure \ `[adaptivesuspension] <#adaptivesuspension>`__. Other lines of
development look to blending sensing with the wheel or suspension
system. One example is the flexible wheel,
Figure \ `[flexiwheel] <#flexiwheel>`__.

.. raw:: latex

   \centering

.. figure:: motion/shrimp
   :alt: Walking Wheels[shrimp]
   :width: 50.0%

   Walking Wheels[shrimp]

.. raw:: latex

   \centering

.. figure:: motion/adaptivesuspension
   :alt: Adaptive Suspension[adaptivesuspension]
   :width: 80.0%

   Adaptive Suspension[adaptivesuspension]

.. raw:: latex

   \centering

.. figure:: motion/flexiwheel
   :alt: Flexible Wheel[flexiwheel]
   :width: 30.0%

   Flexible Wheel[flexiwheel]

Frames
------

| Represent a rigid body in space.
| Define the principle direction (front) as the approach direction -
  vector :math:`a`.
| A second orthogonal direction to :math:`a`, the normal to :math:`a` is
  :math:`n`.
| A third direction selected :math:`o = a \times n`.
| We can load into a matrix

  .. math::

     F = \begin{pmatrix}
              n_x & o_x & a_x  \\
              n_y & o_y & a_y \\
              n_z & o_z & a_z  \end{pmatrix}

We can see that this acts like a coordinate system, let
:math:`c = [c_1,c_2,c_3]`

.. math:: c' = Fc = c_1  n + c_2 o + c_3 a

and :math:`F` transforms from one coordinate system to another.

:math:`F` can generate scalings, rotations, reflections, shears.

Does not translate since :math:`F0 = 0`. To translate we need

.. math:: c' = Fc + T

Known as an affine map.

Homogeneous Coordinates
-----------------------

An translation or offset can be described by using a homogenous
coordinate system.

.. math::

   F =  \begin{pmatrix}
            n_x & o_x & a_x & p_x \\
            n_y & o_y & a_y & p_y\\
            n_z & o_z & a_z & p_z \\
            0  &  0  &  0 & 1 \end{pmatrix}

Homogeneous coordinates

.. math:: \xi = \begin{pmatrix}x \\ y \\ z \\ 1 \end{pmatrix}

Allows for general transforms: :math:`\xi' = A\xi`, thus,

.. math::

   \xi' = \begin{pmatrix}1 & 0 & 0 & t_1 \\
            0 & 1 & 0 & t_2\\ 0 &0 & 1 & t_3 \\
            0& 0& 0& 1 \end{pmatrix}
   \begin{pmatrix}\cos\theta & -\sin\theta & 0 & 0 \\
            \sin\theta & \cos\theta & 0 & 0\\ 0 &0 & 1 & 0 \\
            0& 0& 0& 1 \end{pmatrix}  \xi

Rotations - Rotation matrix

-  About :math:`z`

   .. math::

      R_z = \begin{pmatrix}\cos\theta & -\sin\theta & 0 & 0 \\
               \sin\theta & \cos\theta & 0 & 0\\ 0 &0 & 1 & 0 \\
               0& 0& 0& 1 \end{pmatrix}

-  About :math:`x`

   .. math::

      R_x = \begin{pmatrix}1 & 0 & 0 & 0 \\ 0 & \cos\theta & -\sin\theta & 0  \\
               0& \sin\theta & \cos\theta & 0 \\
               0& 0& 0& 1 \end{pmatrix}

-  About :math:`y`

   .. math::

      R_y = \begin{pmatrix}\cos\theta & 0 & -\sin\theta & 0  \\ 0 & 1 & 0 & 0\\
               \sin\theta &0& \cos\theta & 0 \\
               0& 0& 0& 1 \end{pmatrix}

Translation - Translation matrix

.. math::

   T = \begin{pmatrix}1 & 0 & 0 & t_1 \\
            0 & 1 & 0 & t_2\\ 0 &0 & 1 & t_3 \\
            0& 0& 0& 1 \end{pmatrix}

| Successive motion can be computed by matrix multiplication.
| Let :math:`R` be a rotation and :math:`T` be a translation. Then

  .. math:: M = TR

is the matrix that describes the rotation by :math:`R` followed by
translation by :math:`T`.

Assume that you are given the following motions: Rotate about the x-axis
30 degrees, translate in y by 3cm, and rotate about the z axis 45
degrees. Find the coordinate transformation. [1]_

.. math::

   R_1 = \begin{pmatrix}1 & 0 & 0 & 0 \\ 0 & \cos 30 & -\sin 30 & 0  \\
            0& \sin 30 & \cos 30 & 0 \\
            0& 0& 0& 1 \end{pmatrix},  \quad R_2 =
            \begin{pmatrix}\cos 45 & -\sin 45 & 0 & 0 \\
            \sin 45 & \cos 45 & 0 & 0\\ 0 &0 & 1 & 0 \\
            0& 0& 0& 1 \end{pmatrix}

.. math::

   T = \begin{pmatrix}1 & 0 & 0 & 0 \\
            0 & 1 & 0 & 3\\ 0 &0 & 1 & 0 \\
            0& 0& 0& 1 \end{pmatrix}

Then the transformation is :math:`M = R_2TR_1`

.. math::

   = \begin{pmatrix}\cos 45 & -\sin 45 & 0 & 0 \\
            \sin 45 & \cos 45 & 0 & 0\\ 0 &0 & 1 & 0 \\
            0& 0& 0& 1 \end{pmatrix}
            \begin{pmatrix}1 & 0 & 0 & 0 \\
            0 & 1 & 0 & 3\\ 0 &0 & 1 & 0 \\
            0& 0& 0& 1 \end{pmatrix}
            \begin{pmatrix}1 & 0 & 0 & 0 \\ 0 & \cos 30 & -\sin 30 & 0  \\
            0& \sin 30 & \cos 30 & 0 \\
            0& 0& 0& 1 \end{pmatrix}

.. math::

   =
   \begin{pmatrix}\cos 45 & -\sin 45 & 0 & 0 \\
            \sin 45 & \cos 45 & 0 & 0\\ 0 &0 & 1 & 0 \\
            0& 0& 0& 1 \end{pmatrix}
   \begin{pmatrix}1 & 0 & 0 & 0 \\
            0 & \cos 30 & -\sin 30 & 3\\ 0 &\sin 30 & \cos 30 & 0 \\
            0& 0& 0& 1 \end{pmatrix}

.. math::

   =
   \begin{pmatrix}
            \cos 45 & -\sin 45 \cos 30 & -\sin 45 \sin 30 & -3\sin 45 \\
            \sin 45 & \cos 45 \cos 30 & -\cos 45 \sin 30 & 3\cos 45\\
            0       & \sin 30 & \cos 30 & 0 \\
            0       & 0& 0& 1 \end{pmatrix}

Combined Transforms
-------------------

Begin with a point :math:`x` in space. An application of a
transformation, :math:`T_1`, with respect to the global frame carries
this point to a new point :math:`x'`:

.. math:: x' = T_1x

In essense, the fixed frame views the point as in motion. Now apply
another transformation :math:`T_2` to the new point :math:`x'`:

.. math:: x" = T_2x' = T_2(x') = T_2(T_1x) = T_2T_1x

Note that each transform was done with respect to the fixed frame.

| Again, begin with a point :math:`x` in space. If we view the
  transformation, :math:`T` from the perspective of the point (which
  will be fixed), then it appears that the "fixed" frame is moving
| AND
| that the motion is in the *opposite* direction of the fixed frame
  transformation. Opposite here would be the inverse transformation:
  :math:`T^{-1}`. Thus combined transformations from the point’s “point
  of view”:

  .. math:: T^{-1} = T_2^{-1}T_1^{-1}, \quad \mbox{or}\quad T = \left(T_2^{-1} T_1^{-1} \right)^{-1}

.. math:: T = T_1T_2

Reverse order.

Successive transformations relative to the global frame are left
multiplied:

.. math:: T = T_n T_{n-1} \dots T_1 T_0

For example, take a rotation about :math:`z` of 30 degrees, :math:`R_1`,
followed by a rotation about :math:`x` by 60 degrees, :math:`R_2`:

.. math::

   R = R_2R_1= \begin{pmatrix}1 & 0 & 0 & 0 \\ 0 & \cos 60 & -\sin 60 & 0  \\
            0& \sin 60 & \cos 60 & 0 \\
            0& 0& 0& 1 \end{pmatrix}\begin{pmatrix}\cos 30 & -\sin 30 & 0 & 0 \\
            \sin 30 & \cos 30 & 0 & 0\\ 0 &0 & 1 & 0 \\
            0& 0& 0& 1 \end{pmatrix}

Successive transformations relative to the moving frame are right
multiplied:

.. math:: T = T_0 T_{1} \dots T_{n-1} T_n

For example, take a rotation about x by 45 degrees, :math:`R`, followed
by a translation in z by 4 cm, :math:`T`:

.. math::

   M = TR= \begin{pmatrix}1 & 0 & 0 & 0 \\ 0 & \cos 60 & -\sin 60 & 0  \\
            0& \sin 60 & \cos 60 & 0 \\
            0& 0& 0& 1 \end{pmatrix}\begin{pmatrix}1 & 0 & 0 & 0 \\
            0 & 1 & 0 & 0\\ 0 &0 & 1 & 4 \\
            0& 0& 0& 1 \end{pmatrix}

Inverse Transforms
------------------

Inverting transformation matrices ...

.. math::

   T^{-1} = \left( T_n T_{n-1} \dots T_1 T_0 \right)^{-1}
     = T_0^{-1} T_{1}^{-1} \dots T_{n-1}^{-1} T_n^{-1}

How does one invert the transformations?

Rotation matrices are orthogonal and so

.. math:: R^{-1} = R^T

For example, the inverse of the 60 degree rotation mentioned above:

.. math::

   \begin{pmatrix}1 & 0 & 0 & 0 \\ 0 & \cos 60 & -\sin 60 & 0  \\
            0& \sin 60 & \cos 60 & 0 \\
            0& 0& 0& 1 \end{pmatrix}^{-1} =
            \begin{pmatrix}1 & 0 & 0 & 0 \\ 0 & \cos 60 & \sin 60 & 0  \\
            0& -\sin 60 & \cos 60 & 0 \\
            0& 0& 0& 1 \end{pmatrix}

Translation matrices are simple as well. One just negates the
translation components.

Thus:

.. math::

   \begin{pmatrix}1 & 0 & 0 & a \\ 0 & 1 & 0 & b  \\
            0& 0 & 1 & c \\
            0& 0& 0& 1 \end{pmatrix}^{-1} =
            \begin{pmatrix}1 & 0 & 0 & -a \\ 0 & 1 & 0 & -b  \\
            0& 0 & 1 & -c \\
            0& 0& 0& 1 \end{pmatrix}

Thus we can just undo the transformations individually.

RPY Angles and Euler Angles
---------------------------

RPY angles provide the position and orientation of a craft by using a
translation to body center and then three rotation matrices for craft
pose.

-  Rotation about :math:`a` (z axis) - Roll

-  Rotation about :math:`o` (y axis) - Pitch

-  Rotation about :math:`n` (x axis) - Yaw

.. raw:: latex

   \vspace*{1cm}

.. math:: M = R_nR_oR_aT

Euler angles provide the position and orientation of a craft by using a
translation to body center and then three rotation matrices for craft
pose. However - reference is with respect to the body, not the world
coordinates.

-  Rotation about :math:`a` (z axis) - Roll

-  Rotation about :math:`o` - Pitch

-  Rotation about :math:`a` - Roll

.. raw:: latex

   \vspace*{1cm}

.. math:: M = R_aR_oR_aT

Forward and Inverse Kinematics
------------------------------

Given joint angles and actuator lengths it is straightforward to compute
end effector position. Thus it is easy to find effector path as a
function of rotations.\

.. math::

   \begin{pmatrix} \theta_1(t), ... , \theta_n(t)
              \end{pmatrix}\to p(t)

It is MUCH harder to find the angle functions if you are given the end
effector path:

.. math::

   p(t) \to \begin{pmatrix} \theta_1(t), ... , \theta_n(t)
              \end{pmatrix}

| Represent points by :math:`\begin{pmatrix}x\\y\\z\\1 \end{pmatrix}`
  and vectors by :math:`\begin{pmatrix}v_x\\v_y\\v_z\\0 \end{pmatrix}`
| Recall a rotation about x by 30 degrees

  .. math::

     \begin{pmatrix}x'\\y'\\z'\\1 \end{pmatrix} =
      \begin{pmatrix}\cos 30 & -\sin 30 & 0 & 0 \\
              \sin 30 & \cos 30 & 0 & 0\\ 0 &0 & 1 & 0 \\
              0& 0& 0& 1 \end{pmatrix} \begin{pmatrix}x\\y\\z\\1 \end{pmatrix}

Denavit-Hartenberg Parameters
-----------------------------

| Provides a standard way to build kinematic models for a robot.
| Simple concept.
| Follow out the links of the manipulator, and see them as rotations and
  translations of the coordinate system:

  .. math:: P = P_0 P_1 ...P_{n-1} P_n

where :math:`P_k = R_z T_z T_x R_x`

-  Each link is assigned a number. Normally start with the base and work
   towards the effector.

-  All joints are represented by the z axis, :math:`z_i` where the z
   axis is the axis of revolution (right hand rule for orientation).

-  :math:`\theta_i` will represent the rotation about the joint.

-  The x axis, :math:`x_i` is in the direction that connects the links.
   [Well, connects the z axes of each joint.]

-  :math:`a_i` is link length.

-  :math:`\alpha_i` will be the angles between z axes (if they are not
   parallel).

-  :math:`d_i` will represent the offset along the z axis.

|image|

Thus, the translation from one joint to the next involves a rotation,
translation, translation and a rotation:

-  Rotate about the local z axis angle :math:`\theta`.

-  Translate along the z axis amount :math:`d`.

-  Translate along x amount :math:`a`.

-  Rotate about the new x axis (the joint twist) amount :math:`\alpha`.

This set of transformations will then change the coordinate system to
the next link in the serial chain.

:math:`A_{n+1} =`

.. math::

   \begin{pmatrix}\cos \theta_{n+1} & -\sin \theta_{n+1} & 0 & 0 \\
            \sin \theta_{n+1} & \cos \theta_{n+1} & 0 & 0\\ 0 &0 & 1 & 0 \\
            0& 0& 0& 1 \end{pmatrix}
            \begin{pmatrix}1 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0  \\
            0& 0 & 1 & d_{n+1} \\
            0& 0& 0& 1 \end{pmatrix}
            \begin{pmatrix}1 & 0 & 0 & a_{n+1} \\ 0 & 1 & 0 & 0  \\
            0& 0 & 1 & 0 \\
            0& 0& 0& 1 \end{pmatrix}

.. math::

   \times
    \begin{pmatrix}1 & 0 & 0 & 0 \\ 0 & \cos \alpha_{n+1} & -\sin \alpha_{n+1} & 0  \\
            0& \sin \alpha_{n+1} & \cos \alpha_{n+1} & 0 \\
            0& 0& 0& 1 \end{pmatrix}

:math:`A_{n+1} =`

|

  .. math::

     \begin{pmatrix}\cos \theta_{n+1} & -\sin \theta_{n+1}\cos \alpha_{n+1} & \sin \theta_{n+1}\sin \alpha_{n+1} & a_{n+1}\cos \theta_{n+1} \\
              \sin \theta_{n+1} & \cos \theta_{n+1}\cos \alpha_{n+1} & -\cos \theta_{n+1}\sin \theta_{n+1}  & a_{n+1}\sin \theta_{n+1} \\ 0 & \sin \alpha_{n+1}& \cos \alpha_{n+1} & d_{n+1} \\
              0& 0& 0& 1 \end{pmatrix}
| A parameter table keeps track for each link, the values of
  :math:`\theta`, :math:`d`, :math:`a` and :math:`\alpha`.

Starting from the base of the robot, we can built the transformation
that defines the kinematics:

.. math:: A = A_1A_2 \dots A_n

D-H Two Link Example
--------------------

+------+------------------+-----------+-------------+----------------+
| Link | :math:`\theta`   | :math:`d` | :math:`a`   | :math:`\alpha` |
+======+==================+===========+=============+================+
| 1    | :math:`\theta_1` | 0         | :math:`a_1` | 0              |
+------+------------------+-----------+-------------+----------------+
| 2    | :math:`\theta_2` | 0         | :math:`a_2` | 0              |
+------+------------------+-----------+-------------+----------------+

.. math::

   A_1 =\begin{pmatrix}\cos \theta_1 & -\sin \theta_1 & 0 & a_1 \cos \theta_1 \\
            \sin \theta_1 & \cos \theta_1 & 0 & a_1 \sin \theta_1
           \\ 0 &0 & 1 & 0 \\
            0& 0& 0& 1 \end{pmatrix}

.. math::

   A_2 =\begin{pmatrix}\cos \theta_2 & -\sin \theta_2 & 0 & a_2 \cos \theta_2 \\
            \sin \theta_2 & \cos \theta_2 & 0 & a_2 \sin \theta_2 \\ 0 &0 & 1 & 0 \\
            0& 0& 0& 1 \end{pmatrix}

So,

.. math::

   A = A_1A_2 =
     \begin{pmatrix}\cos (\theta_1+\theta_2) & -\sin (\theta_1+\theta_2) & 0 & a_2 \cos (\theta_1+\theta_2) + a_1 \cos \theta_1 \\
   \sin (\theta_1 +\theta_2) & \cos (\theta_1 +\theta_2) & 0 & a_2 \sin (\theta_1 +\theta_2) + a_1\sin \theta_1\\
            0 &0 & 1 & 0 \\
            0& 0& 0& 1
     \end{pmatrix}

Inverse Kinematics
------------------

How can we use this technology to solve the inverse kinematics problem?

.. math::

   T^{-1}
     = T_0^{-1} T_{1}^{-1} \dots T_{n-1}^{-1} T_n^{-1}

In each matrix one can solve algebraically for :math:`\theta_i` in terms
of the orientation and displacement vectors. What does this look like
for the two link manipulator?

.. _denavit-hartenberg-parameters-1:

Denavit-Hartenberg Parameters
-----------------------------

| Provides a standard way to build kinematic models for a robot.
| Simple concept.
| Follow out the links of the manipulator, and see them as rotations and
  translations of the coordinate system:

  .. math:: P = P_0 P_1 ...P_{n-1} P_n

where :math:`P_k = R_z T_z T_x R_x`

-  Each link is assigned a number. Normally start with the base and work
   towards the effector.

-  All joints are represented by the z axis, :math:`z_i` where the z
   axis is the axis of revolution (right hand rule for orientation).

-  :math:`\theta_i` will represent the rotation about the joint.

-  The x axis, :math:`x_i` is in the direction that connects the links.
   [Well, connects the z axes of each joint.]

-  :math:`a_i` is link length.

-  :math:`\alpha_i` will be the angles between z axes (if they are not
   parallel).

-  :math:`d_i` will represent the offset along the z axis.

|image|

Thus, the translation from one joint to the next involves a rotation,
translation, translation and a rotation:

-  Rotate about the local z axis angle :math:`\theta`.

-  Translate along the z axis amount :math:`d`.

-  Translate along x amount :math:`a`.

-  Rotate about the new x axis (the joint twist) amount :math:`\alpha`.

This set of transformations will then change the coordinate system to
the next link in the serial chain.

:math:`A_{n+1} =`

.. math::

   \begin{pmatrix}\cos \theta_{n+1} & -\sin \theta_{n+1} & 0 & 0 \\
            \sin \theta_{n+1} & \cos \theta_{n+1} & 0 & 0\\ 0 &0 & 1 & 0 \\
            0& 0& 0& 1 \end{pmatrix}
            \begin{pmatrix}1 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0  \\
            0& 0 & 1 & d_{n+1} \\
            0& 0& 0& 1 \end{pmatrix}
            \begin{pmatrix}1 & 0 & 0 & a_{n+1} \\ 0 & 1 & 0 & 0  \\
            0& 0 & 1 & 0 \\
            0& 0& 0& 1 \end{pmatrix}

.. math::

   \times
    \begin{pmatrix}1 & 0 & 0 & 0 \\ 0 & \cos \alpha_{n+1} & -\sin \alpha_{n+1} & 0  \\
            0& \sin \alpha_{n+1} & \cos \alpha_{n+1} & 0 \\
            0& 0& 0& 1 \end{pmatrix}

:math:`A_{n+1} =`

|

  .. math::

     \begin{pmatrix}\cos \theta_{n+1} & -\sin \theta_{n+1}\cos \alpha_{n+1} & \sin \theta_{n+1}\sin \alpha_{n+1} & a_{n+1}\cos \theta_{n+1} \\
              \sin \theta_{n+1} & \cos \theta_{n+1}\cos \alpha_{n+1} & -\cos \theta_{n+1}\sin \theta_{n+1}  & a_{n+1}\sin \theta_{n+1} \\ 0 & \sin \alpha_{n+1}& \cos \alpha_{n+1} & d_{d+1} \\
              0& 0& 0& 1 \end{pmatrix}
| A parameter table keeps track for each link, the values of
  :math:`\theta`, :math:`d`, :math:`a` and :math:`\alpha`.

Starting from the base of the robot, we can built the transformation
that defines the kinematics:

.. math:: A = A_1A_2 \dots A_n

Inverse Kinematics for the two link example
-------------------------------------------

+------+------------------+-----------+-------------+----------------+
| Link | :math:`\theta`   | :math:`d` | :math:`a`   | :math:`\alpha` |
+======+==================+===========+=============+================+
| 1    | :math:`\theta_1` | 0         | :math:`a_1` | 0              |
+------+------------------+-----------+-------------+----------------+
| 2    | :math:`\theta_2` | 0         | :math:`a_2` | 0              |
+------+------------------+-----------+-------------+----------------+

.. math::

   A_1 =\begin{pmatrix}\cos \theta_1 & -\sin \theta_1 & 0 & a_1 \cos \theta_1 \\
            \sin \theta_1 & \cos \theta_1 & 0 & a_1 \sin \theta_1
           \\ 0 &0 & 1 & 0 \\
            0& 0& 0& 1 \end{pmatrix}

.. math::

   A_2 =\begin{pmatrix}\cos \theta_2 & -\sin \theta_2 & 0 & a_2 \cos \theta_2 \\
            \sin \theta_2 & \cos \theta_2 & 0 & a_2 \sin \theta_2 \\ 0 &0 & 1 & 0 \\
            0& 0& 0& 1 \end{pmatrix}

So,

:math:`A = A_1A_2 =`

.. math::

   \begin{pmatrix}\cos (\theta_1+\theta_2) & -\sin (\theta_1+\theta_2) & 0 & a_2 \cos (\theta_1+\theta_2) + a_1 \cos \theta_1 \\
   \sin (\theta_1 +\theta_2) & \cos (\theta_1 +\theta_2) & 0 & a_2 \sin (\theta_1 +\theta_2) + a_1\sin \theta_1\\
            0 &0 & 1 & 0 \\
            0& 0& 0& 1
     \end{pmatrix}

Then we have that the transformation carries the frame to some frame
description :math:`A = F`:

.. math::

   A = \begin{pmatrix}\cos (\theta_1+\theta_2) & -\sin (\theta_1+\theta_2) & 0 & a_2 \cos (\theta_1+\theta_2) + a_1 \cos \theta_1 \\
   \sin (\theta_1 +\theta_2) & \cos (\theta_1 +\theta_2) & 0 & a_2 \sin (\theta_1 +\theta_2) + a_1\sin \theta_1\\
            0 &0 & 1 & 0 \\
            0& 0& 0& 1
     \end{pmatrix}

.. math::

   =
     \begin{pmatrix}
            n_x & o_x & a_x & p_x \\
            n_y & o_y & a_y & p_y\\
            n_z & o_z & a_z & p_z \\
            0  &  0  &  0 & 1 \end{pmatrix} = F

Then the location of the end effector :math:`(x,y,z) = (p_x, p_y, p_z)`:

.. math::

   \begin{pmatrix}
   x \\ y \\ z
   \end{pmatrix}
   =
   \begin{pmatrix}
   a_2\cos (\theta_1+\theta_2) + a_1 \cos \theta_1 \\
   a_2 \sin (\theta_1 +\theta_2) + a_1\sin \theta_1 \\
   0
   \end{pmatrix}

How can we use this technology to solve the inverse kinematics problem?

.. math::

   T^{-1}
     = T_0^{-1} T_{1}^{-1} \dots T_{n-1}^{-1} T_n^{-1}

In each matrix one can solve algebraically for :math:`\theta_i` in terms
of the orientation and displacement vectors. What does this look like
for the two link manipulator?

Recall that

.. math::

   A_1 =\begin{pmatrix}\cos \theta_1 & -\sin \theta_1 & 0 & a_1 \cos \theta_1 \\
            \sin \theta_1 & \cos \theta_1 & 0 & a_1 \sin \theta_1
           \\ 0 &0 & 1 & 0 \\
            0& 0& 0& 1 \end{pmatrix}

.. math::

   A_2 =\begin{pmatrix}\cos \theta_2 & -\sin \theta_2 & 0 & a_2 \cos \theta_2 \\
            \sin \theta_2 & \cos \theta_2 & 0 & a_2 \sin \theta_2 \\ 0 &0 & 1 & 0 \\
            0& 0& 0& 1 \end{pmatrix}

Thus

.. math::

   A = A_1(\theta_1)A_2(\theta_2) = \begin{pmatrix}
            n_x & o_x & a_x & p_x \\
            n_y & o_y & a_y & p_y\\
            n_z & o_z & a_z & p_z \\ 0 & 0 & 0 & 1\end{pmatrix}

Right multiply to decouple: :math:`A_1 = A A_2^{-1}`

.. math::

   =\begin{pmatrix}\cos \theta_1 & -\sin \theta_1 & 0 & a_1 \cos \theta_1 \\
            \sin \theta_1 & \cos \theta_1 & 0 & a_1 \sin \theta_1
           \\ 0 &0 & 1 & 0 \\
            0& 0& 0& 1 \end{pmatrix}

.. math::

   = \begin{pmatrix}
            n_x & o_x & a_x & p_x \\
            n_y & o_y & a_y & p_y\\
            n_z & o_z & a_z & p_z \\ 0 & 0 & 0 & 1\end{pmatrix}
            \begin{pmatrix}\cos \theta_2 & -\sin \theta_2 & 0 & -a_2  \\
            \sin \theta_2 & \cos \theta_2 & 0 & 0 \\ 0 &0 & 1 & 0 \\
            0& 0& 0& 1 \end{pmatrix}

Note that :math:`a_1\cos\theta_1 = p_x - a_2n_x` and
:math:`a_1\sin\theta_1 = p_y - a_2n_y`

This provides us with

.. math:: \theta_1 = \mbox{atan2}\left(\frac{p_y - a_2n_y}{a_1} , \frac{p_x - a_2n_x}{a_1}\right)

| From :math:`\cos \theta_1 = \cos \theta_2 n_x - \sin \theta_2o_x` and
  :math:`-\sin \theta_1 = \sin \theta_2 n_x + \cos \theta_2o_x`
| we can solve for :math:`\theta_2`.

.. math::

   \begin{pmatrix} \cos \theta_1 \\ -\sin \theta_1 \end{pmatrix}
     = \begin{pmatrix}n_x & -o_x  \\ n_x & o_x \end{pmatrix}
     \begin{pmatrix} \cos \theta_2 \\ \sin \theta_2 \end{pmatrix}

.. math::

   \begin{pmatrix} \cos \theta_2 \\ \sin \theta_2 \end{pmatrix}
     = \frac{1}{2n_xo_x}\begin{pmatrix}o_x & o_x  \\ -n_x & n_x \end{pmatrix}
     \begin{pmatrix} \cos \theta_1 \\ -\sin \theta_1 \end{pmatrix}

So ...
:math:`\theta_2 = \mbox{atan2} \left( o_x(\cos \theta_1 -\sin \theta_1 ), -n_x(\cos \theta_1 +\sin \theta_1 )\right)`

There is a problem. The two link example has two degrees of freedom. The
assumption here is that you have four variables to input (four degrees
of freedom): :math:`p_x, p_y, n_x, n_y`. You may not know
:math:`n_x, n_y`. [2]_ For general systems this approach will succeed if
you have enough degrees of freedom in your robot.

.. _inverse-kinematics-1:

Inverse Kinematics
------------------

The general approach is to form matrix :math:`A` analytically and set to
final pose matrix. Then by applying inverses :math:`A_k^{-1}`, examine
intermediate results looking for terms which provide one of the angle
variables: :math:`\theta_j`.

Producing actual robot motion means moving the end effector along some
path :math:`(x(t), y(t), z(t))`.

One really wants

.. math::

   \begin{pmatrix} \theta_1(t), ... , \theta_n(t)
              \end{pmatrix} = f^{-1}(p(t), n(t), o(t), a(t))

There is no reason to expect that there exists a solution, that you can
find the solution, or that the solution is unique.

Kinematic equations are derived by the developer of the robot. Inverse
kinematic formulas are derived in an “ad hoc” manner.

| How?

  .. math::

     p(t) \to \begin{pmatrix} \theta_1(t), ... , \theta_n(t)
                \end{pmatrix}
| Assume that you have :math:`(\theta_1, ..., \theta_n) = f(p,n,o,a)`.
| For each :math:`t`, solve

  .. math::

     \begin{bmatrix}
      {\theta_1}_k \\ {\theta_2}_k \\ \vdots \\ {\theta_n}_k
     \end{bmatrix}
     =
     \begin{bmatrix}
      {\theta_1}(t_k) \\ {\theta_2}(t_k) \\ \vdots \\ {\theta_n}_k
     \end{bmatrix}
     =
     \begin{bmatrix}
      f_1(p(t_k),n(t_k),o(t_k),a(t_k)) \\
      f_2(p(t_k),n(t_k),o(t_k),a(t_k)) \\ \vdots \\
      f_n(p(t_k),n(t_k),o(t_k),a(t_k))
     \end{bmatrix}

.. [1]
   the text has examples as well

.. [2]
   We will address the specific situation in a few slides.

.. |image| image:: motion/legs.png
.. |image| image:: motion/humanoid.png
   :width: 65.0%
.. |image| image:: kinematics/DH_frame.png
