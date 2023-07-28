Representation of objects in space
-----------------------------------

Before we can derive the forward kinematics for a serial chain manipulator,
we need to be able to describe in a very general manner, the changes in
position or orientation of a solid object in space.

As indicated before, we represent a point or a vector
in space (focused on three dimensions for now) via

.. math::

   p = \begin{pmatrix}
   a \\
   b \\
   c \end{pmatrix}


It is common to extend this notation to four components by appending a scale factor :math:`w>0`:

.. math::

   p = \begin{pmatrix}
   a \\
   b \\
   c \\
   w \end{pmatrix}, \quad
   x = a/w, \quad y = b/w, \quad z = c/w

The length of the vector is given by

.. math::

   \| p\| = \sqrt{x^2 + y^2 + z^2}

The factor :math:`w` will scale the length of the vector as :math:`w` changes from 0
to infinity.  When :math:`w=1`, the length is unchanged.   For :math:`w=0`, the length of the scaled vector would be infinite.  In this extended notation, it is commonly used to represent a direction vector.   We will find this
notation useful later when we introduce homogeneous coordinates.

Example
^^^^^^^^^^

What is the length of :math:`[1,-2,3,4]`?

.. math::

   \sqrt{(1/4)^2 + (-2/4)^2 + (3/4)^2} = 1/16 + 1/4 + 9/16 = 14/16 = 7/8

So, we begin by reviewing how we represent a rigid body in space.    For this chapter and
the next, robotic arms will be constructed from rigid elements.   To represent a point
in space, we only need three variables which is three degrees of freedom.  Solid objects will also have an orientation
in space which is tracked by another three variable giving six degrees of freedom.   The orientation
of the object will be tracked by a matrix description called a frame.
A frame is a collection of mutually orthonormal vectors which act as a local coordinate
system which can be used describe an object from a different perspective or to orient a robotic arm.



A full discussion of change of basis or change of coordinate systems can be found in
any text on linear algebra.  We will restrict our discussion to the elements directly
applicable to what we need for modeling robot arms.   For here we are just focused on
the change of frames (which are made up of orthogonal basis vectors) in three dimensional
space.   We start with a simple frame, Figure :numref:`Fig:frame`.  This means that
frame G has three basis vectors for the three directions:  :math:`A, B, C`.

.. _`Fig:frame`:
.. figure:: MathFigures/frame.*
   :width: 10%
   :align: center

   Simple coordinate frame G.

Assume that you have a vector :math:`U` which is described in frame L, Figure :numref:`Fig:frame1`:

.. _`Fig:frame1`:
.. figure:: MathFigures/frame1.*
   :width: 20%
   :align: center

   Vector :math:`U` in the frame.


We can represent :math:`U` as components in each of the coordinate directions via

.. math::

   U = [ (U_A) , (U_B) , (U_C)  ]

where :math:`U_A, U_B, U_C` are the projections or lengths in the basis directions :math:`A, B, C`.  See Figure
:numref:`Fig:frame2`



.. _`Fig:frame2`:
.. figure:: MathFigures/frame2.*
   :width: 25%
   :align: center

   The projection of :math:`U` onto basis element :math:`A`.



Assume that :math:`U` is unit length and that you have two additional vectors :math:`V, W` which form a orthonormal
frame, call it L. [#f1]_   Any vector, say :math:`Q` can be represented in either coordinate system.   Since
we can express each vector :math:`U, V, W` in the frame G.   Doing so give us a way to relate these two
coordinate systems mathematically.   Figure :numref:`Fig:frames` gives the idea (although we have not
yet discussed translations, only rotations, but it is nice to separate for readability).


.. _`Fig:frames`:
.. figure:: MathFigures/frames.*
   :width: 25%
   :align: center

   Mapping from one coordinate frame to another.

The way to relate vectors in Frame L to Frame G is to multiply the vector (in Frame L) by an orthogonal
rotation matrix

.. math::

   R = \begin{pmatrix}
   U_A & V_A & W_A  \\
   U_B & V_B & W_B \\
   U_C & V_C & W_C  \end{pmatrix}

An easy check will show this works.    The vector :math:`[1,0,0]` in the Frame L is :math:`U`.
We can compute

.. math::

   R \begin{pmatrix} 1 \\ 0 \\ 0\end{pmatrix}
   =
   \begin{pmatrix}
   U_A & V_A & W_A  \\
   U_B & V_B & W_B \\
   U_C & V_C & W_C  \end{pmatrix} \begin{pmatrix} 1 \\ 0 \\ 0\end{pmatrix}
   =
   \begin{pmatrix}
   U_A   \\
   U_B  \\
   U_C  \end{pmatrix}
   = U \mbox{  in Frame G}

Note that in general transformations (given by non-singular matrices) :math:`M` can generate scalings, rotations, reflections, shears.
and are called transformation matrices. Also, these are linear transformations
and so they do not translate the  vectors (since :math:`R0 = 0`).


So, how does this relate to robotics?
We will dive into the details of robotic arms in the next chapter.  For now,
suffice it to say that one will need to
track the tool end of a robotic arm (e.g. where a drill bit might be located).
The direction that the tool tip faces is the approach direction or the principle
direction.  We will use  the vector :math:`a` to indicate the unit vector pointing
in the approach direction.   A second orthogonal direction to :math:`a` can be
found and will be called :math:`n`.
A third direction, :math:`o`,
selected using the cross-product :math:`o = a \times n`.  Note that some texts will
use :math:`x = a`, :math:`y = n` and :math:`z = o`.
At this point we can apply the transformations given above.   We will abuse the
notation a bit and have :math:`x, y, z` be the directions of the world
coordinate system or global frame.
Load the three vectors column-wise into a matrix

.. math::

   R = \begin{pmatrix}
   n_x & o_x & a_x  \\
   n_y & o_y & a_y \\
   n_z & o_z & a_z  \end{pmatrix}

and since these are mutually orthogonal vectors,
we can see that this acts like a coordinate system and
what we are doing is the mathematical operation of
a change of coordinates or change of reference frame.


Let :math:`c = [c_1,c_2,c_3]`

.. math:: c' = Rc = c_1  n + c_2 o + c_3 a

and as indicated above, :math:`R` transforms from one coordinate system to another.

To perform a translation we need
to augment by a displacement vector, :math:`D`.

.. math:: c' = Mc = Rc + D

This coordinate transformation and translation is known as an affine map, :math:`M`.
Although the affine map works well as a way to shift coordinate systems,
the linear transformation property (:math:`L(0) = 0`) will turn out to be important and so
to gain rotations, scalings as well as the translation, but keeping the
linearity property, we inflate our matrix and introduce
homogeneous coordinates and homogeneous transformation matrices.




Homogeneous Coordinates and Transforms
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The homogeneous transforms act on four component vectors.  We extend
the vectors by adding a fourth element.
Homogeneous coordinates are defined by appending a "1" at the bottom
of a normal 3 component position vector giving

.. math:: \xi = \begin{pmatrix}x \\ y \\ z \\ 1 \end{pmatrix}

Allows for general transforms: :math:`\xi' = A\xi`, which are linear
transforms.   In most of our applications, we will be interested in
a rotation and then a translation.  Shear and reflection are not an issue
here since these changes in coordinates will apply to rigid robot hardware
which (for now) does not experience reflection and shear.

We can represent a rigid body in space by giving the body a frame and then representing that frame
in space.   The rotation and translation of the frame can be combined into a single
transformation matrix.  Specifically, the translation will be appended as a final column in the matrix and
a unit basis vector is added to the last row giving us

.. math::

   T =  \begin{pmatrix}
            n_x & o_x & a_x & p_x \\
            n_y & o_y & a_y & p_y\\
            n_z & o_z & a_z & p_z \\
            0  &  0  &  0 & 1 \end{pmatrix}.

This turns out to be a rotation followed by a
translation.  To get a feel of these operations, we will look at
translations and rotations separately.

In the setup, we saw the displacement or translation as an additive operation.

.. math::

   T + D' =
   \begin{pmatrix}
            n_x & o_x & a_x & p_x \\
            n_y & o_y & a_y & p_y\\
            n_z & o_z & a_z & p_z \\
            0  &  0  &  0 & 1 \end{pmatrix}
    +
    \begin{pmatrix}0& 0 & 0 & t_1 \\
             0 & 0 & 0 & t_2\\ 0 &0 & 0 & t_3 \\
             0& 0& 0& 0 \end{pmatrix}
   =
   \begin{pmatrix}
            n_x & o_x & a_x & p_x + t_1\\
            n_y & o_y & a_y & p_y + t_2\\
            n_z & o_z & a_z & p_z + t_3\\
            0  &  0  &  0 & 1 \end{pmatrix}

However, we can write this as a matrix multiplication by combining the 4x4 identity with the displacement matrix
and so the pure translation matrix can be formed by

.. math::

   D = I + D' = \begin{pmatrix}1 & 0 & 0 & t_1 \\
            0 & 1 & 0 & t_2\\ 0 &0 & 1 & t_3 \\
            0& 0& 0& 1 \end{pmatrix}



You will note that it has the property that if you apply a translation to
the frame:

.. math::

   DR =
   \begin{pmatrix}1 & 0 & 0 & t_1 \\
            0 & 1 & 0 & t_2\\ 0 &0 & 1 & t_3 \\
            0& 0& 0& 1 \end{pmatrix}
   \begin{pmatrix}
            n_x & o_x & a_x & p_x \\
            n_y & o_y & a_y & p_y\\
            n_z & o_z & a_z & p_z \\
            0  &  0  &  0 & 1 \end{pmatrix}
   =
   \begin{pmatrix}
            n_x & o_x & a_x & p_x + t_1\\
            n_y & o_y & a_y & p_y + t_2\\
            n_z & o_z & a_z & p_z + t_3\\
            0  &  0  &  0 & 1 \end{pmatrix}

A very simple example for this case, translate the point :math:`p=[5,12,13]` by :math:`v=<3,4,5>`.
We can point to :math:`p` using the matrix T:

.. math::
   T =    \begin{pmatrix}1 & 0 & 0 & 5 \\ 0 & 1 & 0 & 12\\ 0 &0 & 1 & 13 \\ 0& 0& 0& 1 \end{pmatrix}

and then add the displacement :math:`v`:

.. math::
    T_v T =  \begin{pmatrix}1 & 0 & 0 & 3 \\ 0 & 1 & 0 & 4\\ 0 &0 & 1 & 5 \\ 0& 0& 0& 1 \end{pmatrix}
    \begin{pmatrix}1 & 0 & 0 & 5 \\ 0 & 1 & 0 & 12\\ 0 &0 & 1 & 13 \\ 0& 0& 0& 1 \end{pmatrix}
    = \begin{pmatrix}1 & 0 & 0 & 8 \\ 0 & 1 & 0 & 16\\ 0 &0 & 1 & 18 \\ 0& 0& 0& 1 \end{pmatrix}


Let :math:`R` be a rotation matrix.  Rotation can be expressed by

.. math::

   R = \begin{pmatrix}
            n_x & o_x & a_x & 0 \\
            n_y & o_y & a_y & 0\\
            n_z & o_z & a_z & 0 \\
            0  &  0  &  0 & 1 \end{pmatrix}


It is useful to review the basic rotations about the three axes:

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


It is not hard to show that :math:`R^{-1} = R^T`.  We can also
verify that replacing :math:`\theta` with :math:`-\theta` is the reverse rotation and gives the
same thing as  :math:`R^{-1}` and :math:`R^T`.


Example,  rotate the point :math:`[1,2,3]` by 30 degrees about the y-axis.  Let :math:`v` point to :math:`[1,2,3]`.

.. math::

      w = R_yv = \begin{pmatrix}\cos 30^\circ & 0 & -\sin 30^\circ & 0  \\ 0 & 1 & 0 & 0\\
               \sin 30^\circ &0& \cos 30^\circ & 0 \\  0& 0& 0& 1 \end{pmatrix}v
          = \begin{pmatrix}\sqrt{3}/2 & 0 & -1/2 & 0  \\ 0 & 1 & 0 & 0\\
                   1/2 &0& \sqrt{3}/2& 0 \\  0& 0& 0& 1 \end{pmatrix} \begin{pmatrix} 1 \\ 2 \\ 3\\ 1\end{pmatrix}
          = \begin{pmatrix} \sqrt{3}/2 - 3/2 \\ 2 \\ 1/2 + 3\sqrt{3}/2\\ 1\end{pmatrix}


These operations can be chained together and this is the basis for the matrix T we began with.
We then see the matrix T as the representation of the orientation and position of some frame that describes
solid body.

.. math::

    T  =
   \begin{pmatrix}
   R  & d \\
   0 & 1\\
   \end{pmatrix}

We will multiply this matrix often and this should be done blockwise.  Let :math:`u = (x,y,z,1)`, :math:`w = (x,y,z)`

.. math::

   Tu = \begin{pmatrix} R & d \\ 0 & 1 \end{pmatrix} u = \begin{pmatrix} Rw + d \\ 1 \end{pmatrix}


For example a rotation about the :math:`z` axis and then a translation of
:math:`(t_1, t_2, t_3 )`  would
have the following tansformation matrix.

.. math::

   \xi' =
   \begin{pmatrix}
   \cos\theta & -\sin\theta & 0 & t_1 \\
   \sin\theta & \cos\theta & 0 & t_2\\
   0 &0 & 1 & t_3 \\
   0& 0& 0& 1
   \end{pmatrix}  \xi


As an aside, we can chain as many of these as we would like.
Assume that you are given the following motions: Rotate about the x-axis
30 degrees, translate in y by 3cm, and rotate about the z axis 45
degrees. Find the coordinate transformation.

.. math::

   R_1 = \begin{pmatrix}1 & 0 & 0 & 0 \\ 0 & \cos 30 & -\sin 30 & 0  \\
            0& \sin 30 & \cos 30 & 0 \\
            0& 0& 0& 1 \end{pmatrix},  \quad R_2 =
            \begin{pmatrix}\cos 45 & -\sin 45 & 0 & 0 \\
            \sin 45 & \cos 45 & 0 & 0\\ 0 &0 & 1 & 0 \\
            0& 0& 0& 1 \end{pmatrix}

.. math::

   D = \begin{pmatrix}1 & 0 & 0 & 0 \\
            0 & 1 & 0 & 3\\ 0 &0 & 1 & 0 \\
            0& 0& 0& 1 \end{pmatrix}

Then the transformation is :math:`M = R_2DR_1`

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

Going forward we will just have T represent the rotation and displacement pair, and then chain those.
It is also useful to have the inverse of the transformation.
How does one invert the transformations?  For us this is simplified
since we are restricted to rotations and translations which are easily
inverted.   Rotation matrices are orthogonal and so

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



You may guess that the
inverse of the combined transformation must include the transpose of the rotation and the negative of the
displacement.  By trial and error you can find it.  Here is the result:

.. math::

   T^{-1} =  \begin{pmatrix}
            n_x & n_y & n_z & -p\cdot n \\
            o_x & o_y & o_z & -p\cdot o\\
            a_x & a_y & a_z & -p\cdot a \\
            0  &  0  &  0 & 1 \end{pmatrix}.



Successive transformations
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Once you can relate one frame (coordinate system) to another, we can chain these
to relate additional coordinate systems.   Each new frame is related to the
previous frame by a transformation.

.. _`Fig:frames2`:
.. figure:: MathFigures/frames2.*
   :width: 50%
   :align: center

   Successive changes of frames

Successive motion can be computed by matrix multiplication.
This is done by multiplication from left to right:  :math:`T_1 T_2 T_3`.
Any type of transformation will work here.  We can mix rotations and translations.  For
example, let :math:`R` be a rotation and :math:`D` be a translation. Then

  .. math:: T = DR

is the matrix that describes the rotation by :math:`R` followed by
translation by :math:`D`.

.. math::

   \begin{pmatrix}
   n_x & o_x & a_x & p_x \\
   n_y & o_y & a_y & p_y\\
   n_z & o_z & a_z & p_z \\
   0& 0& 0& 1 \end{pmatrix}
   =
   \begin{pmatrix}1 & 0 & 0 & p_x \\
   0 & 1 & 0 & p_y\\
   0 &0 & 1 & p_z \\
   0& 0& 0& 1 \end{pmatrix}
   \begin{pmatrix}
   n_x & o_x & a_x & 0 \\
   n_y & o_y & a_y & 0 \\
   n_z & o_z & a_z & 0 \\
   0& 0& 0& 1 \end{pmatrix}

It is useful to
have a feel for the difference in the postmultiplication we are doing
and the premultiplication you may have seen in other context's such
as the LU factorization.

The core idea illustrated in Figure :numref:`Fig:frames2` is that starting
with :math:`T_1` and successive postmultiplication by :math:`T_2, T_3, ...`
is layer by layer creating the transformation that will take an object described
in the final frame (coordinate system) and represent it in the first frame (coordinate system).
It rotates the frames from first frame to final frame.
When you are transforming a vector from position to position (rotating, translating, etc)
then you would perform a sequence of premultiplications.   It is
a difference in view whether you are holding the outer frame and moving a vector
verses having a fixed vector and moving the reference frame.  Both are
valuable ways to look at these transformations.  To see this, we illustrate
with specific points.

Begin with a point :math:`x` in space. An application of a
transformation, :math:`T_1`, with respect to the global frame carries
this point to a new point :math:`x'`:

.. math:: x' = T_1x

We can think of the new point :math:`x'` as movement of the original
point :math:`x`.   This can be repeated.
Apply
another transformation :math:`T_2` to the new point :math:`x'`:

.. math:: x" = T_2x' = T_2(x') = T_2(T_1x) = T_2T_1x

Note that each transform was done with respect to the fixed frame.

Again, begin with a point :math:`x` in space. If we view the
transformation, :math:`T` from the perspective of the point (which
will be fixed), then it appears that the "fixed" frame is moving AND
that the motion is in the *opposite* direction of the fixed frame
transformation. Opposite here would be the inverse transformation:
:math:`T^{-1}`. Thus combined transformations from the point’s “point
of view”:

  .. math:: T^{-1} = T_2^{-1}T_1^{-1}, \quad \mbox{or}\quad T = \left(T_2^{-1} T_1^{-1} \right)^{-1}

.. math:: T = T_1T_2

This places the list of operations in reverse order.
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


The formula for inverting products of transformation matrices is given by

.. math::

   T^{-1} = \left( T_n T_{n-1} \dots T_1 T_0 \right)^{-1}
     = T_0^{-1} T_{1}^{-1} \dots T_{n-1}^{-1} T_n^{-1}




RPY Angles and Euler Angles
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Roll-Pitch-Yaw (RPY) angles provide the position and orientation of a craft by using a
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

-  Rotation about :math:`o` (y axis) - Pitch

-  Rotation about :math:`a` - Roll


.. math:: M = R_aR_oR_aT




.. rubric:: Footnotes

.. [#f1] Technically saying "assume orthonormal frame" for us would be redundant since we defined frames as made up from orthonormal sets of vectors.
