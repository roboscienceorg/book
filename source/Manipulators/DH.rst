Denavit-Hartenberg Parameters
-----------------------------

.. Note::  Again, these are converted slides and need to be written up
   properly.   The slow part for these sections is getting good images created.


Given joint angles and actuator lengths it is straightforward to compute
end effector position. Thus it is possible to compute the effector path as a
function of arm movements.

.. math::

   \begin{pmatrix} \theta_1(t), ... , \theta_n(t)
              \end{pmatrix}\to p(t)

It is MUCH harder to find the angle functions if you are given the end
effector path:

.. math::

   p(t) \to \begin{pmatrix} \theta_1(t), ... , \theta_n(t)
              \end{pmatrix}


DH formalism
~~~~~~~~~~~~~~~~

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

.. figure:: ManipulatorsFigures/DH_frame.png
   :align: center
   :width: 50%

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

.. math::

  \begin{pmatrix}\cos \theta_{n+1} & -\sin \theta_{n+1}\cos \alpha_{n+1} & \sin \theta_{n+1}\sin \alpha_{n+1} & a_{n+1}\cos \theta_{n+1} \\
  \sin \theta_{n+1} & \cos \theta_{n+1}\cos \alpha_{n+1} & -\cos \theta_{n+1}\sin \theta_{n+1}  & a_{n+1}\sin \theta_{n+1} \\ 0 & \sin \alpha_{n+1}& \cos \alpha_{n+1} & d_{n+1} \\
  0& 0& 0& 1 \end{pmatrix}

A parameter table keeps track for each link, the values of :math:`\theta`, :math:`d`, :math:`a` and :math:`\alpha`.

Starting from the base of the robot, we can built the transformation
that defines the kinematics:

.. math:: A = A_1A_2 \dots A_n

D-H Two Link Example
~~~~~~~~~~~~~~~~~~~~~

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


DH Inverse Kinematics
-----------------------

How can we use this technology to solve the inverse kinematics problem?

.. math::

   T^{-1}
     = T_0^{-1} T_{1}^{-1} \dots T_{n-1}^{-1} T_n^{-1}

In each matrix one can solve algebraically for :math:`\theta_i` in terms
of the orientation and displacement vectors. What does this look like
for the two link manipulator?



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

How?

  .. math::

     p(t) \to \begin{pmatrix} \theta_1(t), ... , \theta_n(t)
                \end{pmatrix}


Assume that you have :math:`(\theta_1, ..., \theta_n) = f(p,n,o,a)`.

For each :math:`t`, solve

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


.. [2]
   We will address the specific situation in a few slides.
