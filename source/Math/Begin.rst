Mathematical Notation
---------------------

This text will use italicized letters for the variables in formulas:
:math:`x`. We will not distinguish scalar (single) variables from vector
(array) variables. Scalar and vector quantities will normally be lower
case (unless we need to be consistent with a well known formula). When
possible, elements of a vector will be indexed using a subscript
:math:`x_k`. Matrices will be indicated using italicized uppercase,
:math:`M`. Discrete time steps will be indicated by superscripts,
:math:`x^k`.

Notation related to robotics:

-  Real line: :math:`{\mathbb R}`

-  Plane: :math:`{\mathbb R}^2`

-  Space: :math:`{\mathbb R}^3`

-  Workspace: :math:`{\cal W}`

-  Workspace Obstacles: :math:`{\cal W}{\cal O}_i`

-  Free space: :math:`{\cal W}\setminus \bigcup_i {\cal W}{\cal O}_i`

-  Point in space:  :math:`x = (x_1, x_2, x_3)`

-  Vector: :math:`\vec{v} \in {\mathbb R}^n`, :math:`v = [v_1, v_2, \dots , v_n]^T`.

-  Bounded workspace:   :math:`{\cal W} \subset B_r(x)` :math:`\equiv \{ y \in {\mathbb R}^n | d(x,y) < r\}` for some :math:`0 < r < \infty`.

We will use several forms of notation for derivatives. Let :math:`f(x)`
be a differentiable function, both Newton’s notation, :math:`f'(x)`, and
Leibniz’s notation, :math:`df/dx`, will be employed. Specifically for
derivatives of functions of time, we will also use the superscript dot
notation, :math:`\dot{g}`. The dot product and cross product of two
vectors will be denoted by :math:`x \cdot y` and :math:`x \times y`
respectively. Matrix vector and matrix matrix products will be denoted
by :math:`Ax` and :math:`AB`. The normal matrix multiplication is
implied. Rarely will we need to do element-wise matrix and vector
operations. Those will be written out since they don’t occur often
enough to warrant custom notation.

There is a temptation for some authors to present material in the most
abstract setting possible. It can be argued that this provides the most
general case (widest application). In addition, one should remove the
specifics of any application so the abstract formulation represents the
core concept at its purest form. In many cases this is true. At times it
is also true that the author is trying to impress his or her audience
with their mathematical talent using an assault of symbols. At times I
am sure you will feel this way, but we have made significant effort to
keep the mathematical level in the first two years of an engineering
curriculum (Calculus, Differential Equations, Linear Algebra,
Probability).


Parametric Form
---------------

Say you want to traverse a path :math:`C`, shown in
Figure  :numref:`Fig:intro-path`

.. _`Fig:intro-path`:
.. figure:: MathFigures/path1.*
   :width: 75%
   :align: center

   A path for an explicitly defined function.

.. _`Fig:intro-path2`:
.. figure:: MathFigures/path2.*
   :width: 75%
   :align: center

   A path for a parametric function.

The path :math:`C` often will come from some function description of the
curve :math:`y = f(x)`. This type of description will work for many
paths, but fails for a great number of interesting paths like circles:
:math:`x^2 + y^2 = 1`. We want to be able to wander around in the plan
crossing our own path which certainly is not the graph of a function.
So, we must move to a parametric description of the path (actually a
piecewise parametric description). You want to prescribe
:math:`x(t), y(t)` and obtain :math:`\dot{\phi_1},\dot{\phi_2}`. Clearly
if you have :math:`x(t), y(t)`, differentiation will yield
:math:`\dot{x}(t), \dot{y}(t)`, so we may assume that we know
:math:`\dot{x}(t), \dot{y}(t)`. Using :math:`\dot{x}` and
:math:`\dot{y}` we may drive the robot along the curve of interest. How
does one follow an arbitrary curve?

The first step is to write in :index:`parametric form`: :math:`x(t)`,
:math:`y(t)`. Example: convert :math:`y=x^2` to parametric

.. math:: \mbox{Let } x = t  \quad \to \quad y = x^2 = t^2

| Note that there are an infinite number of choices :
| Let

  .. math::

     \begin{array}{l}
     x = 2t  \quad \to \quad y = x^2 = 4t^2 \\
     x = e^t  \quad  \to \quad y = x^2 = e^{2t} \\
     x = \tan(t) \quad \to \quad y = x^2 = \tan^2(t)
     \end{array}

and so forth.

All the parametric forms provide the same curve, same shape, same
geometry. They vary in the speed. Think of the function form telling you
the shape, like the shape of a road, but not the velocity. The
parametric form gives you both path shape and velocity. We will assume
that you can find parametric functions :math:`x = \phi(t)` and
:math:`y = \psi(t)` such that the graph is :math:`y=f(x)` which
generates the path :math:`C` of interest.

Example Functions
^^^^^^^^^^^^^^^^^

Some examples of parametric forms may help in getting good at writing
these down.

Line
    :math:`x(t) = t`, :math:`y(t) = mt + b`, where :math:`m` is the
    slope and :math:`b` is the intercept.

Circle
    :math:`x(t) = R \cos(t) + h`, :math:`y(t) = R \sin(t) + k`, where
    the radius is :math:`R` and the center is :math:`(h,k)`.

Ellipse
    :math:`x(t) = A \cos(t) + h`, :math:`y(t) = B \sin(t) + k`, where
    :math:`A` and :math:`B` describe the major and minor axes and the
    center is :math:`(h,k)`.

Lissajous
    :math:`x(t) = A\sin(at)`, :math:`y(t) = B \sin(bt)`
    (Figure `[Fig:intro-path2] <#Fig:intro-path2>`__ :math:`A=1`,
    :math:`B=1`, :math:`a=3`, :math:`b=4`). Infinity: :math:`A=1`,
    :math:`B=0.25`, :math:`a=1`, :math:`b=2`

Root
    :math:`x(t) =  t^2`, :math:`y(t) = t`.

Heart
    :math:`x(t) = 16\sin^3(t)`,
    :math:`y(t) = 13\cos(t) - 5\cos(2t) -2\cos(3t) - \cos(4t)`



Vectors, Matrices and Linear Systems
------------------------------------

A vector is a list of numbers. It can be used to represent physical
quantities like force and direction. It can be expressed as

.. math:: \vec{x} = \left< x_1, x_2, x_3, \dots , x_n \right>.

The notation for a point in n-dimensional space and a n-dimensional
vector are similar: :math:`\vec{x}\in \mathbb{R}^n`:
:math:`\vec{x} = (x_1, x_2, ... x_n)`, and also written as

.. math::

   \vec{x} = \left(\begin{array}{c} x_1 \\ x_2 \\ \vdots
   \\ x_n \end{array}\right).

If the context is understood, the small arrow above the variable is left
off, so :math:`\vec{x}` becomes :math:`x`. The basic datatype used in
scientific computing is the array. Arrays are used to store points,
vectors, matrices and other mathematical constructs. The basic
operations defined on vectors are listed below. Let
:math:`c\in \mathbb{R}` and :math:`x,y \in
\mathbb{R}^n`, then

-  Sum: :math:`x+y = \{ x_1 + y_1, x_2 + y_2, \dots, x_n + y_n\}`

-  Scalar multiplication: :math:`cx = \{ cx_1, cx_2, \dots , cx_n\}`

-  Inner product (related to angle): :math:`x \cdot y = \sum_{i=1}^n x_iy_i`

-  Norm (length): :math:`\| x \| = \sqrt{\sum_{i=1}^n x_i^2}`

-  Norm as multiplication: :math:`\| x \|^2 = x^T x`

We will make use of matrix algebra and will follow the normal
conventions. Let :math:`A, B \in \mathbb{R}^{n\times n}`,

.. math::

   A =
   \left( \begin{array}{ccc}a_{11}&\dots&a_{1n}\\ \dots & \dots & \dots
   \\ a_{n1} & \dots & a_{nn}\end{array}\right), \quad B =
   \left( \begin{array}{ccc}b_{11}&\dots&b_{1n}\\ \dots & \dots & \dots
   \\ b_{n1} & \dots & b_{nn}\end{array}\right).

Matrix addition and multiplication are defined in the standard manner as

-  :math:`A+B=\left( \begin{array}{ccc}a_{11}+b_{11}&\dots&a_{1n}+b_{1n}\\ \dots & \dots & \dots \\ a_{n1}+b_{n1} & \dots & a_{nn}+b_{nn}\end{array}\right)`

-  :math:`AB =\left( \begin{array}{ccc}c_{11}&\dots&c_{1n}\\ \dots & \dots & \dots\\ c_{n1} & \dots & c_{nn}\end{array}\right)`,
where the entries are :math:`c_{ij} = \sum_k a_{ik}b_{kj}`

Matrix vector multiplication occurs often and is given by

-  :math:`Ax = \left( \begin{array}{ccc}a_{11}&\dots&a_{1n}\\ \dots & \dots & \dots\\ a_{n1} & \dots & a_{nn}\end{array}\right)\left(\begin{array}{c} x_1 \\ x_2 \\ \vdots\\ x_n \end{array}\right) =   \left(\begin{array}{c} \sum_k a_{1k}x_k \\ \sum_k a_{2k}x_k \\ \vdots\\ \sum_k a_{nk}x_k \end{array}\right)`

The identity element and the matrix transpose are given by

-  :math:`I=\left( \begin{array}{ccccc}1&0&\dots&0&0\\ 0&1&\dots&0&0\\ \vdots&\vdots & \ddots & \vdots & \vdots\\ 0& 0 & \dots& 1 & 0  \\ 0& 0 &  \dots &0& 1  \end{array}\right)`

-  Transpose: :math:`A^T`: :math:`\{ a_{ij}\}^T = \{ a_{ji}\}`
   Example: If :math:`A =
   \left( \begin{array}{ccc}1 & 2 & 3 \\ 4 & 5 & 6
   \\ 7 & 8 & 9\end{array}\right)` then :math:`A^T =
   \left( \begin{array}{ccc}1 & 4 & 7 \\ 2 & 5 & 8
   \\ 3 & 6 & 9\end{array}\right)`

Some additional matrix terms and properties:

-  The matrix determinant is indicated by det(\ :math:`A`)

-  The transpose formula is given by :math:`(AB)^T=B^TA^T`

-  The determinant formula is given by det(\ :math:`AB`) = det(\ :math:`A`)det(\ :math:`B`)

-  A symmetric matrix is defined by :math:`A^T = A`

-  A symmetric positive definite matrix satisfies :math:`x^T A x >0` for :math:`x \neq 0`.


Linear Systems
~~~~~~~~~~~~~~

One of the most common mathematical operations is solving simultaneous linear equations:

.. math::

   \begin{array}{c} a_{11}x_1 + a_{12}x_2 + .... + a_{1n}x_n = b_1 \\ a_{21}x_1 + a_{22}x_2 + .... + a_{2n}x_n = b_2 \\ \vdots
   \\ a_{n1}x_1 + a_{n2}x_2 + .... + a_{nn}x_n = b_n \end{array}

Using the matrix notation defined above we may write this in a very
compact form:

.. math:: \Rightarrow\quad  Ax = b

where

.. math::

   A = \left( \begin{array}{ccc}a_{11}&\dots&a_{1n}\\ \dots & \dots & \dots
   \\ a_{n1} & \dots & a_{nn}\end{array}\right), \quad x = \left(\begin{array}{c} x_1 \\ x_2 \\ \vdots
   \\ x_n \end{array}\right) , \quad
   b =  \left(\begin{array}{c} b_1 \\ b_2 \\ \vdots
   \\ b_n \end{array}\right) .

One approach to solve the equations is :index:`Gaussian Elimination`. The
industry version of Gaussian Elimination is the :index:`LU factorization`. An LU
factorization decomposes the matrix :math:`A` into the product of a
lower triangular matrix, :math:`L`, and an upper triangular matrix,
:math:`U`. The strength of this approach is that the LU factorization is
done for :math:`A` once. Once done, solving :math:`Ax = b` for different
:math:`b`\ ’s can be done relatively easily. You don’t actually have to
know how to do this, only how to call the system solvers.

Inverses
^^^^^^^^

The inverse of :math:`A` is notated :math:`A^{-1}`:

.. math::

 A(A^{-1}) = I =
 (A^{-1})A

Given the inverse:

.. math:: Ax=b \to x = A^{-1}b


Is this a good approach to solving :math:`Ax=b`?

No. The fast multiplication algorithms are not numerically stable. Best
to use a Gauss-Jordan based approach like the LU factorization. LU can
also make good use of matrix structure. Possible that an algorithm may
list an inverse, but this can often be converted to a linear solve. For
example if the formula lists :math:`y^* = y + BC^{-1}x`, then solve
:math:`Cz = x` first and then find :math:`y^*=y+Bz`.

Finding curves from data
~~~~~~~~~~~~~~~~~~~~~~~~

Say that you have a data set:

.. math:: (x_i, y_i),\quad  i=1, \dots, k

and you want to fit a model to it:

.. math:: y = a_n x^n + a_{n-1}x^{n-1} + \dots + a_1x + a_0

or in general

.. math:: y = a_n \phi_n(x) + a_{n-1}\phi_{n-1}(x) + \dots + a_0 \phi_0(x) .

How does one use the data to find the coefficients of the model?

Plug the data into the model:

.. math::

   \begin{array}{l}
    y_1 = a_n x_1^n + a_{n-1}x_1^{n-1} + \dots + a_1x_1 + a_0 \\[3mm]
    y_2 = a_n x_2^n + a_{n-1}x_2^{n-1} + \dots + a_1x_2 + a_0 \\[3mm]
   \vdots \\[3mm]
    y_{k-1} = a_n x_{k-1}^n + a_{n-1}x_{k-1}^{n-1} + \dots + a_{k-1}x_{k-1} + a_0 \\[3mm]
    y_k = a_n x_k^n + a_{n-1}x_k^{n-1} + \dots + a_1x_k + a_0
   \end{array} .

This can be rewritten in the language of matrix algebra.

Plug the data into the model:

.. math::

   \underbrace{\begin{bmatrix} y_1 \\[3mm] y_2 \\[3mm] \vdots \\[3mm] y_k \end{bmatrix}}_y =
   \underbrace{ \begin{bmatrix} x_1^n & x_1^{n-1} & \dots & x_1 & 1 \\[3mm]
   x_2^n & x_2^{n-1} & \dots & x_2 & 1 \\[3mm]
   \vdots &\vdots & & \vdots & \vdots\\[3mm]
   x_k^n & x_k^{n-1} & \dots & x_k & 1
   \end{bmatrix} }_X
   \underbrace{\begin{bmatrix}
   a_n \\[3mm] a_{n-1} \\[3mm] \vdots \\[3mm] a_1 \\[3mm] a_0
   \end{bmatrix}}_a     .

.. math::

   \label{eqn:curvefittingmatrix}
   y = Xa

If :math:`k = n+1`, then this system is square, meaning it has the same
number of equations as unknowns. For the polynomial, the matrix will be
invertible and we can solve the system. For larger values of :math:`n`,
the system becomes ill-conditioned and has some numerical accuracy
problems, but can be solved in the theoretical sense. Specific formulas
have been created to avoid the numerical errors. These are the Lagrange
formulas presented in the next section.

Simple Example
^^^^^^^^^^^^^^

Fit a quadratic to (0,1), (1,2), (2,5). The quadratic is
:math:`y = a_2 x^2 + a_1 x + a_0`. The matrix model is

.. math::

   \underbrace{\begin{bmatrix} 1 \\[3mm] 2 \\[3mm] 5 \end{bmatrix}}_y =
   \underbrace{ \begin{bmatrix}
   0 & 0  & 1 \\[3mm]
   1 & 1 &  1 \\[3mm]
   4 & 2 & 1
   \end{bmatrix} }_X
   \underbrace{\begin{bmatrix}
   a_2 \\[3mm] a_1 \\[3mm] a_0
   \end{bmatrix}}_a     .

Then we must row reduce

.. math::

   \begin{bmatrix}
   0 & 0  & 1 & 1\\[3mm]
   1 & 1 &  1 & 2 \\[3mm]
   4 & 2 & 1  & 5
   \end{bmatrix}
   \to
   \begin{bmatrix}
   0 & 0  & 1 & 1\\[3mm]
   1 & 1 &  0 & 1 \\[3mm]
   4 & 2 & 0  & 4
   \end{bmatrix}
   \to
   \begin{bmatrix}
   0 & 0  & 1 & 1\\[3mm]
   1 & 1 &  0 & 1 \\[3mm]
   1 & 0 & 0  & 1
   \end{bmatrix}
   \to
   \begin{bmatrix}
   0 & 0  & 1 & 1\\[3mm]
   0 & 1 &  0 & 0 \\[3mm]
   1 & 0 & 0  & 1
   \end{bmatrix}

You can read off the coefficients here: :math:`a_2=1`, :math:`a_1=0` and
:math:`a_0=1`. Thus we obtain :math:`y = x^2 +1` which checks with the
data. The next section gives you a way to do this without a matrix
solve.
