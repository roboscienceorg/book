Representation of objects in space
-----------------------------------

These are slides  ...

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
~~~~~~~~~~~~~~~~~~~~~~~~~~

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
~~~~~~~~~~~~~~~~~~~~~~

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


.. math:: M = R_nR_oR_aT

Euler angles provide the position and orientation of a craft by using a
translation to body center and then three rotation matrices for craft
pose. However - reference is with respect to the body, not the world
coordinates.

-  Rotation about :math:`a` (z axis) - Roll

-  Rotation about :math:`o` - Pitch

-  Rotation about :math:`a` - Roll


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

.. [1]
   the text has examples as well
