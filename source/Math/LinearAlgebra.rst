Linear Algebra Concepts
-----------------------

When we attempt to integrate multiple sensors or when we compute paths
from discrete points, the methods use the tools from linear algebra. No
attempt here is made to be complete or expository. This is intended to
review the language and concepts only. The reader who is unfamiliar with
Linear Algebra as a subject is strongly encouraged to explore it. [1]_
Calculus, Linear Algebra and Probability are three legs to the
mathematical stool every engineer should have.

If :math:`x, y\in {\mathbb R}^n` are vectors, and
:math:`a, b\in {\mathbb R}` are real numbers, we say that :math:`ax+by`
is a *linear combination* of :math:`x` and :math:`y`. Over all possible
values for :math:`a` and :math:`b`, we say :math:`ax+by` is a span of
:math:`x` and :math:`y`. Spanning sets arise in all sort of
applications. It is a way to decompose sets into basic components. For
example, the span of :math:`x = \left< 1, 0 \right>` and
:math:`y = \left< 0, 1 \right>` is the plane and the vectors :math:`x`
and :math:`y` are a known as a :index:`basis`. The term basis is a minimal
spanning set and the number of linear independent basis elements is the
dimension. More information on these ideas can be found in most linear
algebra textbooks.

We can represent a line through the origin by
:math:`t \left< a  , b \right>`

where :math:`t\in {\mathbb R}` (:math:`t` is the scale factor).
Geometrically we are scaling the vector into spanning a line. The vector
we are using is :math:`\left< a  , b \right>`. Another example is the
collection of all :math:`2\times 2` matrices:

.. math:: \begin{pmatrix} a & b \\ c & d\end{pmatrix}

which is the linear combination of

.. math::

   \begin{pmatrix} 1 & 0 \\ 0 & 0\end{pmatrix},
   \begin{pmatrix} 0 & 1 \\ 0 & 0\end{pmatrix},
   \begin{pmatrix} 0 & 0 \\ 1 & 0\end{pmatrix},
   \begin{pmatrix} 0 & 0 \\ 0 & 1\end{pmatrix}.

One consequence of these ideas is that of a :index:`vector space`. It is the span
of a collection of vectors (or all linear combinations of the vectors).
More formally, :math:`V` is a vector space if :math:`x, y\in V` are
vectors, and :math:`a, b\in {\mathbb R}`, then :math:`ax+by \in V`.

The two examples above are vector spaces: the line through the origin
and the collection of :math:`2\times 2` matrices. Note that in the
figure below, the solid line is a vector space is, and the dotted is
not. A vector space must include the zero element and the dotted line
does not.

.. _`fig:lineisnotvectorspace`:
.. figure:: MathFigures/lines.*
   :width: 40%
   :align: center

A :index:`subspace` is a subset of a vector space :math:`V` that is also a vector
space. For example, a line through the origin is a subspace of the
plane. Also, a plane through the origin is a subspace of three space,
such as the span of

.. math::

   \left\{\begin{pmatrix} 1 \\ 0 \\ 0\end{pmatrix},
   \begin{pmatrix} 0 \\ 1 \\ 0\end{pmatrix}\right\}.

The reason these concepts are discussed is that when solving linear
systems or doing least squares (optimization), you are often working
with vector spaces and subspaces. The literature uses this terminology
and the concepts have a very rich geometric structure which can be
helpful in understanding the problems.

A very well studied subspace is the *Nullspace* of a matrix, :math:`N`.
It is defined as all :math:`w` such that :math:`Aw=0`. Note that if
:math:`Au=0` and :math:`Av=0` then

.. math:: A(cu+dv) = cAu + dAv = c(0) + d(0) = 0

thus it is correctly called a subspace. Also, :math:`u=0` is trivially
in the nullspace. If a matrix has a nullspace, then the associated
linear systems problem :math:`Ax = b` will not have a unique solution
which is important to know if you need a solution to your problem.

An example of this issue is if you wanted to solve :math:`Ax = b` where

.. math::

   A = \begin{pmatrix} 1 & 0 & -1\\ 0 & 0 & 0 \\ 0 & 0 & 0\end{pmatrix},
   \quad b = \begin{pmatrix} 1  \\ 0 \\ 0\end{pmatrix} .

Can this be solved for :math:`x`? In this trivial example you can see
that it can be and :math:`x = \left< 1, 0 , 0\right>` works. However the
solution is not unique. Without going into the details, we see that
there are two vectors which span the Nullspace:

.. math::

   v_1 = \begin{pmatrix} 1  \\ 0 \\ 1\end{pmatrix},
   \quad v_2 = \begin{pmatrix} 0  \\ 1 \\ 0\end{pmatrix}

i.e. :math:`Av_1 = 0` and :math:`Av_2 = 0`. So we actually gain a two
dimensional family of solutions (meaning a plane)

.. math:: x = \begin{pmatrix} 1  \\ 0 \\ 0\end{pmatrix} + c_1\begin{pmatrix} 1  \\ 0 \\ 1\end{pmatrix}  +  c_2\begin{pmatrix} 0  \\ 1 \\ 0\end{pmatrix}

Another popular subspace is known as the *Column Space*. It is the span
of the columns (treated as vectors) of :math:`A`. This tells you the
range space of the matrix. Using the last :math:`A` as the working
example:

.. math:: A = \begin{pmatrix} 1 & 0 & -1\\ 0 & 0 & 0 \\ 0 & 0 & 0\end{pmatrix}

the range is given by the span of the columns. So we have

.. math:: \left\{\begin{pmatrix} 1 \\ 0\\ 0\end{pmatrix}\right\}

Note that a similar notion is the span of the rows, called the *Row
Space*.

Eigenvalues and Eigenvectors
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Let :math:`x` solve :math:`Ax=\lambda x` (the invariant directions
problem).

.. math:: Ax-\lambda x=0 \quad\Rightarrow\quad (A-\lambda I)x=0\quad \Rightarrow \quad x\in {\cal N}(A-\lambda I)

The latter saying that :math:`x` must be in the Nullspace of
:math:`A-\lambda I`. This implies the following polynomial equation
which is solved for roots :math:`\lambda`.

.. math:: \det (A-\lambda I)=0 \quad \Rightarrow \quad \lambda

We can numerically solve for :math:`(\lambda , x)` and these are known
as an eigenvalue, eigenvector pair. An example of the SciPy eigenvalue
solver is given below.

Eigenvalues for Symmetric Matrices
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Assume that :math:`A` is a real symmetric matrix and that
:math:`(\lambda, v)` is an eigenvalue, eigenvector pair. If :math:`v` is
complex valued then :math:`\| v \|^2 = v \cdot \bar{v}` where
:math:`\bar{v}` is the complex conjugate of :math:`v`. Then we have

.. math:: \lambda \| v \|^2 =  \lambda v \cdot \bar{v} = Av  \cdot \bar{v} = v \cdot A \bar{v} =  v \cdot  \overline{Av} =  v \cdot  \overline{\lambda v}  = \bar{\lambda} v \cdot \bar{v} = \bar{\lambda} \| v \|^2

So this implies that :math:`\lambda = \bar{\lambda}` or that
:math:`\lambda` is real valued.

Orthogonal
~~~~~~~~~~

The last concept we will review is orthogonality. The basic term means
perpendicular. Two vectors, :math:`x` and :math:`y` are said to be
orthogonal if their dot product is zero:

.. math:: x\cdot y =0.

A matrix, :math:`Q`, is said to be orthogonal if its columns treated as
vectors are mutually orthogonal and of unit length. This turns out to be
mathematically equivalent to a matrix satisfying

.. math:: QQ^T = I

where :math:`I` is the identity matrix. We will see orthogonal matrices
later when we compute rotations in space. These matrices will be the
foundations of the coordinate transformations used in robotic arms.

The Pseudo-Inverse
~~~~~~~~~~~~~~~~~~

We will at several occasions run into the problem of solving what is
known as the *overdetermined* problem. This is the linear systems
problem for which there are more equations than there are unknowns
(variables).

The problem is then

.. math::

   \begin{array}{c} a_{11}x_1 + a_{12}x_2 + .... + a_{1n}x_n = b_1 \\ a_{21}x_1 + a_{22}x_2 + .... + a_{2n}x_n = b_2 \\ \vdots
     \\ a_{m1}x_1 + a_{m2}x_2 + .... + a_{mn}x_n = b_m \end{array}, m > n

Just as before we can use the matrix notation to write this in a very
compact form:

.. math:: \Rightarrow\quad  Ax = b

where

.. math::

   A = \left( \begin{array}{ccc}a_{11}&\dots&a_{1n}\\ \dots & \dots & \dots
     \\ a_{m1} & \dots & a_{mn}\end{array}\right), \quad x = \left(\begin{array}{c} x_1 \\ x_2 \\ \vdots
     \\ x_n \end{array}\right) , \quad
     b =  \left(\begin{array}{c} b_1 \\ b_2 \\ \vdots
     \\ b_m \end{array}\right) .



.. figure:: MathFigures/vrect.*
   :width: 20%
   :align: center

   Overdetermined System of Equations[fig:overdetermined]

This leads to a non-square matrix which is not invertible. There is no
exact solution: :math:`Ax \neq b` for all possible :math:`x` in this
case. So instead of trying to solve the problem exactly, we ask about
getting as close as possible. In other words, this problem is not
solvable by regular methods such as the LU factorization or Gauss-Jordan
elimination, but can be addressed by minimizing the error using the
method of least squares.

The columns must be linearly independent for this method to succeed so
we assume that for now. With the columns linearly independent, the core
issue geometrically is that the vector :math:`b` is not in the span of
the columns of :math:`A`. The best we can ask is to get as close as
possible. Thus we optimize:

.. math:: \min \| Ax - b\|

where we will call the minimizer :math:`\hat{x}`. To minimize we express
the norm as a matrix multiply:

.. math:: \| Ax - b\|^2 =  (Ax - b)^T(Ax - b) =  (Ax)^T(Ax) - b^T(Ax) -  (Ax)^Tb +  b^Tb .

Note that :math:`b^TAx  =  (Ax)^Tb`, and :math:`(Ax)^T = x^TA^T`, so we
have

.. math:: \| Ax - b\|^2 = x^TA^T Ax -2x^TA^Tb  +   b^Tb.

Next we form the gradient of the norm with respect to :math:`x`. We
leave to a homework to show :math:`\nabla [x^TA^T Ax] = 2 A^TAx` and
:math:`\nabla [x^TA^Tb] = A^T b`. Then we have

.. math:: \nabla \| Ax - b\|^2 = 2 A^TAx  - 2A^T b  .

To find the minimizer, set :math:`\nabla \| Ax - b\|^2 = 0` so we obtain

.. math:: A^TA\hat{x}  = A^T b .

These are known as the *Normal Equations*.

The matrix :math:`A^T A` is symmetric and if the columns of :math:`A`
are linearly independent, then :math:`A^T A` is invertible. This yields
the solution

.. math:: \hat{x} = \left( A^T A\right)^{-1} A^T b .

This formula is known by several names. It is called the :index:`Pseudo-Inverse`
or :index:`Moore-Penrose` Pseudo-Inverse. It is also called the left-sided
pseudo-inverse (because it acts on the left side).

**Example** Find the least squares solution to

.. math:: \begin{pmatrix} 1 & 0 \\ 1 & 1 \\ 0 & 2 \end{pmatrix}\begin{pmatrix} x_1 \\ x_2 \end{pmatrix} = \begin{pmatrix} 1 \\ 2 \\ 1 \end{pmatrix}

Forming the normal equations

.. math::

   \begin{pmatrix} 1 & 1 & 0 \\ 0 & 1 & 2 \end{pmatrix}
    \begin{pmatrix} 1 & 0 \\ 1 & 1 \\ 0 & 2 \end{pmatrix}\begin{pmatrix} x_1 \\ x_2 \end{pmatrix} = \begin{pmatrix} 1 & 1 & 0 \\ 0 & 1 & 2 \end{pmatrix}
    \begin{pmatrix} 1 \\ 2 \\ 1 \end{pmatrix}

and multiplying out

.. math:: \begin{pmatrix} 2 & 1 \\ 1 & 5 \end{pmatrix}\begin{pmatrix} x_1 \\ x_2 \end{pmatrix} = \begin{pmatrix} 3 \\ 4 \end{pmatrix} .

Solving the two by two system, we obtain

.. math:: \begin{pmatrix} x_1 \\ x_2 \end{pmatrix} = \begin{pmatrix} \frac{11}{9} \\[1mm] \frac{5}{9} \end{pmatrix} .

Does this actually solve the problem?

.. math:: \begin{pmatrix} 1 & 0 \\ 1 & 1 \\ 0 & 2 \end{pmatrix}\begin{pmatrix} \frac{11}{9} \\[1mm] \frac{5}{9} \end{pmatrix} = \begin{pmatrix}  \frac{11}{9} \\[1mm] \frac{16}{9}\\[1mm]  \frac{10}{9} \end{pmatrix} \neq  \begin{pmatrix} 1 \\ 2 \\ 1 \end{pmatrix}

It does not solve the problem. What about residual (error)?

.. math:: \| \begin{pmatrix}  \frac{11}{9} \\[1mm] \frac{16}{9}\\[1mm]  \frac{10}{9} \end{pmatrix} -  \begin{pmatrix} 1 \\ 2 \\ 1 \end{pmatrix} \| = \sqrt{(2/9)^2 + (2/9)^2 + (1/9)^2} = 1/9

Can we do any better? For any value :math:`x = \left< x_1, x_2\right>`,
is it possible for

.. math:: \|  \begin{pmatrix} 1 & 0 \\ 1 & 1 \\ 0 & 2 \end{pmatrix}u -  \begin{pmatrix} 1 \\ 2 \\ 1 \end{pmatrix} \| < 1/9?

We will minimize the square of the norm to avoid issues with the square
root. The first derivatives must be zero and we apply the second
derivative test if the error is a minimum.

.. math:: f(x_1,x_2) = (x_1 - 1)^2 + (x_1+x_2 - 2)^2 + (2x_2-1)^2

.. math:: f_{x_1} = 2(x_1-1)  + 2(x_1+x_2-2), \quad f_{x_2} =  2(x_1+x_2-2) + 4(2x_2-1)

We see that

.. math:: f_{x_1}(11/9, 5/9) = 0, \quad  f_{x_2} (11/9, 5/9) = 0

and

.. math:: f_{x_1x_1} = 4, \quad f_{x_2x_2} =  10, \quad f_{x_1x_2} =2

The second derivative test gives :math:`D = 40- 4=36` which means our
surface is curved up at the critical point and thus :math:`(11/9, 5/9)`
is a local min. The function :math:`f` is a parabolic surface and so
:math:`(11/9, 5/9)` is the global min. Meaning it is the best that we
can do.

The other variation of the non-square linear system is the
*underdetermined* problem. In this case we have more columns than rows
and so has the structure shown in
Figure  :numref:`Fig:underdetermined]`

.. _`Fig:underdetermined]`:
.. figure:: MathFigures/hrect.*
   :width:  20%
   :align: center

   An underdetermined system

The columns cannot be linearly independent and so :math:`A^TA` is not
invertible which means the left sided pseudo-inverse
:math:`\left(A^TA\right)^{-1}` does not exist. So, we need to go another
route.

This time instead of assuming the columns are linearly independent we
will assume the rows are linearly independent. So although :math:`A^T A`
is not invertible, we have that :math:`\left(A A^T\right)` is of full
rank, or invertible. Using :math:`\left(A A^T\right)` on the right side
gives us the result. Admittedly this version is less intuitive.

.. math:: Ax = b \quad\Rightarrow\quad   Ax = I b

.. math:: A x = \left(A A^T\right) \left(A A^T\right)^{-1} b

.. math:: Ax = AA^T \left(A A^T\right)^{-1} b

.. math:: \hat{x} = A^T \left(A A^T\right)^{-1} b

Pseudo-Inverse Formulas
^^^^^^^^^^^^^^^^^^^^^^^

| Left Moore-Penrose Pseudo-Inverse (:math:`A` has linearly independent
  columns):
| :math:`A^+ = \left(A^TA\right)^{-1} A^T`, and :math:`A^+ A =I`
| Right Moore-Penrose Pseudo-Inverse (:math:`A` has linearly independent
  rows):
| :math:`A^+ = A^T \left(AA^T\right)^{-1}`, and :math:`A A^+ =I`

Applying the pseudo-inverse to the curve fitting problem
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

We return to our system which arose from the curve fitting problem.
Recall we had the linear system formulation for the curve fitting
problem \ `[eqn:curvefittingmatrix] <#eqn:curvefittingmatrix>`__:

.. math:: y = X a

We assume that we have many data points but wish a low degree polynomial
to fit the data points, :math:`k >> n+1` where :math:`k` is the number
of points and :math:`n` is the degree of the polynomial. This is an
overdetermined problem and presents us with a non-square matrix
:math:`A`. Using the tools just presented, that of a left-sided
pseudo-inverse, we form the normal equations

.. math:: X^T y = X^TXa

we obtain a solvable system. If :math:`X^T X` is of full rank, then we
can invert

.. math:: a = \left(X^T X\right)^{-1} X^Ty

Once :math:`a` is found then we may use

.. math:: \hat{y} = a_n x^n + a_{n-1}x^{n-1} + \dots + a_1x + a_0

as the “fit” to the data.

Curve Fit Example[ex:curvefitexample]
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

For this example, we have 20 points for which we would like to fit a
quadratic equation. Assume the data is contained in a file named
“data.txt” (with the same formatting), we can plot this using:



:math:`x_i` :math:`y_i`

::

    0.026899  1.367895
    0.115905  1.295606
    0.250757  1.156797
    0.413750  1.144025
    0.609919  0.862480
    0.669044  0.827181
    0.868043  0.693536
    1.080695  0.528216
    1.233052  0.549789
    1.312322  0.741778
    1.402371  0.879171
    1.724433  0.784356
    1.844290  0.912907
    1.901078  0.902587
    2.117728  1.032718
    2.235872  1.133116
    2.331574  1.331071
    2.607533  1.768845
    2.719074  1.723766
    2.853608  1.898702

.. figure:: MathFigures/quadpts.*
   :width:  50%
   :align: center


Assume that the model for the data is :math:`y = a_2x^2 + a_1x +a_0`.
Find :math:`a_2, a_1, a_0`. Note that the system arises:


  .. math::

     \begin{array}{c}
        1.367895 = a_2(0.026899)^2 + a_1(0.026899) + a_0\\
        1.295606 = a_2(0.115905)^2 + a_1(0.115905) + a_0\\
        1.156797 = a_2(0.250757)^2 + a_1(0.250757) + a_0\\
        \vdots
       \end{array}

which can be written as

.. math::

   \begin{bmatrix}
   (0.026899)^2 & 0.026899 & 1\\
   (0.115905)^2 & 0.115905 & 1\\
   (0.250757)^2 & 0.250757 & 1\\
   \vdots & \vdots & \vdots
   \end{bmatrix}
   \begin{bmatrix}
    a_2 \\ a_1 \\ a_0
   \end{bmatrix}
   =
   \begin{bmatrix}
    1.367895\\
     1.295606\\
    1.156797\\
   \vdots
   \end{bmatrix}

The Normal Equations can be formed

.. math::

   \begin{bmatrix}
    (0.026899)^2 & (0.115905)^2 & (0.250757)^2 & \dots \\
    0.026899& 0.115905 & 0.250757 & \dots \\
   1 & 1 & 1 & \dots
   \end{bmatrix}
   \begin{bmatrix}
   (0.026899)^2 & 0.026899 & 1\\
   (0.115905)^2 & 0.115905 & 1\\
   (0.250757)^2 & 0.250757 & 1\\
   \vdots & \vdots & \vdots
   \end{bmatrix}
   \begin{bmatrix}
    a_2 \\ a_1 \\ a_0
   \end{bmatrix}

.. math::

   =
   \begin{bmatrix}
    (0.026899)^2 & (0.115905)^2 & (0.250757)^2 & \dots \\
    0.026899& 0.115905 & 0.250757 & \dots \\
   1 & 1 & 1 & \dots
   \end{bmatrix}
   \begin{bmatrix}
    1.367895\\
     1.295606\\
    1.156797\\
   \vdots
   \end{bmatrix}


One can solve :math:`X^TX a = X^T y`: :math:`a = (X^TX)^{-1} X^T y`


  .. math::

     \begin{bmatrix}
     286.78135686  & 122.11468009 &  55.44347326 \\
      122.11468009 &  55.44347326  & 28.317947 \\
       55.44347326 &  28.317947  &   20.
     \end{bmatrix}
     \begin{bmatrix}
     a_2 \\ a_1 \\ a_0
     \end{bmatrix}
     =
     \begin{bmatrix}
       72.4241925 \\  33.380646 \\ 21.534542
     \end{bmatrix}

.. math::

   \begin{bmatrix}
   a_2 \\ a_1 \\ a_0
   \end{bmatrix}
   \approx
   \begin{bmatrix}
    0.4930957 \\ -1.212858 \\ 1.42706\\
   \end{bmatrix}

The curve is approximately :math:`y = 0.49x^2 - 1.21x + 1.42`,
Figure  :numref:`plot:quadgraph`

.. _`plot:quadgraph`
.. figure:: MathFigures/quadgraph.*
   :width: 70%
   :align: center

   The plot of :math:`y = 0.49x^2 - 1.21x + 1.42`.

Singular Value Decomposition
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

For the normal equations to be invertible the columns of the matrix
:math:`A` must be linearly independent, meaning as vectors they point in
different directions. This is fine in the theoretical context, but in
practice a data set can produce columns which point in similar
directions. This can cause problems with the accuracy of the solution to
the normal equations. In addition, the product of :math:`A` times the
transpose of :math:`A` can increase the ill-conditioning of the matrix.

The standard method to address numerical problems such as this is to
compute the pseudo-inverse through the :index:`Singular Value Decomposition`
(SVD). We will present the SVD first and then show how it applies to the
pseudo-inverse.

(details needed here) The SVD of :math:`A = U \Sigma V^T`. :math:`U,V`
are orthogonal. :math:`\Sigma` is diagonal.

The pseudo-inverse of :math:`A` is :math:`A^+ = V \Sigma^+ U^T`.

Note that the SVD pseudo-inverse has one formulation which makes it a
nice for applications which may be deficient in both row and column
rank.

Weighted Least Squares
~~~~~~~~~~~~~~~~~~~~~~

Traditional least squares is formulated by minimizing using the normal
:index:`inner product`:

.. math:: x^Ty = \sum_i x_iy_i.

\ Let :math:`x, y\in R^n`. No weights are referred to as uniform
weighting. Non-uniform weights are just termed as weights. If the inner
product is weighted:

.. math:: \left< x, y \right> = \sum_{i=1}^n x_i y_i q_i = x^T Q y

where :math:`Q` is a :math:`n \times n` square matrix then what is

least squares solution to :math:`A x = b`? One simple modification to
the previous least squares process is required. We multiply both sides
by the weight matrix :math:`Q`:

.. math:: QAx= Qb

then follow the earlier derivation:

.. math:: A^T QAx = A^T Qb .

Assuming that :math:`A^T Q A` is full rank,

.. math:: x = \left(A^T Q A\right)^{-1} A^TQb .

The matrix :math:`Q` is any matrix for which the inner product above is
a valid. However, we will often select :math:`Q` as a diagonal matrix
containing the reciprocals of the variances (the reason shown below in
the covariance computation):

.. math::

   Q =
   \begin{pmatrix} q_1 & 0 & \dots & 0 & 0   \\
   0 & q_2 & \dots & 0 & 0   \\
   && \ddots  &&\\
   0 & 0 & 0 & q_{n-1} & 0   \\
   0 & 0 & 0 & 0 & q_n
   \end{pmatrix}
   =
   \begin{pmatrix} 1/\sigma_1^2 & 0 & \dots & 0 & 0   \\
   0 & 1/\sigma_2^2 & \dots & 0 & 0   \\
   && \ddots  &&\\
   0 & 0 & 0 & 1/\sigma_{n-1}^2 & 0   \\
   0 & 0 & 0 & 0 & 1/\sigma_n^2
   \end{pmatrix} .

Assume that you have an :math:`x`-:math:`y` data set,
Figure :numref:`Fig:weightedLSdata`. Using the
process above we compute the uniformly weighted least squares fit to a
line, shown in blue, and the weighted least squares fit to a line, shown
in green, Figure :numref:`Fig:weightedLSplot`. The
weight function weights more heavily towards the origin (using
:math:`w_i = 1.0/i^3`). In this example, the weights are scaled so the
sum of the weights is one.

.. Owned by Roboscience
.. _`Fig:weightedLSdata`
.. figure:: MathFigures/weightedleastsquaredata.*
   :width: 70%
   :align: center

   Sample noisy data to fit a line.


.. Owned by Roboscience
.. _`Fig:weightedLSplot`
.. figure:: MathFigures/weightedleastsquareplot.*
   :width: 70%
   :align: center

   Least squares line fit. Uniform weighting in
   blue and weighted to the origin in green.
