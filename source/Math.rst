Mathematical and Computational Background [Chap:Math]
=====================================================

Robotics as an academic subject can be very mathematical in nature. This
text does not attempt to place the subject on a rigorous mathematical
foundation, but it is necessary to use the mathematical formalism to
avoid confusion. So, we jump in with some common notation and then
proceed to the standard vocabulary that all robotics professionals use.

Note: This is under significant development.

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

2

-  Real line: :math:`{\mathbb R}`

-  Plane: :math:`{\mathbb R}^2`

-  Space: :math:`{\mathbb R}^3`

-  Workspace: :math:`{\cal W}`

-  Workspace Obstacles: :math:`{\cal W}{\cal O}_i`

-  Free space: :math:`{\cal W}\setminus \bigcup_i {\cal W}{\cal O}_i`

-  | Point in space:
   | :math:`x = (x_1, x_2, x_3)`

-  | Vector:
   | :math:`\vec{v} \in {\mathbb R}^n`,
   | :math:`v = [v_1, v_2, \dots , v_n]^T`.

-  | Bounded workspace:
   | :math:`{\cal W} \subset B_r(x)`
   | :math:`\equiv \{ y \in {\mathbb R}^n | d(x,y) < r\}`
   | for some :math:`0 < r < \infty`.

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
Figure \ `[Fig:intro-path] <#Fig:intro-path>`__.

.. raw:: latex

   \centering

.. figure:: control/path1
   :alt: A path for an explicitly defined function.[Fig:intro-path1]

   A path for an explicitly defined function.[Fig:intro-path1]

.. figure:: control/path2
   :alt: A path for a parametric function.[Fig:intro-path2]

   A path for a parametric function.[Fig:intro-path2]

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

The first step is to write in parametric form: :math:`x(t)`,
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

Vectors and Matrices
--------------------

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

-  Inner product (related to angle):
   :math:`x \cdot y = \sum_{i=1}^n x_iy_i`

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

-  | Transpose: :math:`A^T`: :math:`\{ a_{ij}\}^T = \{ a_{ji}\}`
   | Example: If :math:`A =
     \left( \begin{array}{ccc}1 & 2 & 3 \\ 4 & 5 & 6
       \\ 7 & 8 & 9\end{array}\right)` then :math:`A^T =
     \left( \begin{array}{ccc}1 & 4 & 7 \\ 2 & 5 & 8
       \\ 3 & 6 & 9\end{array}\right)`

Some additional matrix terms and properties:

-  The matrix determinant is indicated by det(\ :math:`A`)

-  The transpose formula is given by :math:`(AB)^T=B^TA^T`

-  The determinant formula is given by det(\ :math:`AB`) =
   det(\ :math:`A`)det(\ :math:`B`)

-  A symmetric matrix is defined by :math:`A^T = A`

-  A symmetric positive definite matrix satisfies :math:`x^T A x >0` for
   :math:`x \neq 0`.

Linear Systems
--------------

One of the most common mathematical operations is solving simultaneous
linear equations:

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

One approach to solve the equations is Gaussian Elimination. The
industry version of Gaussian Elimination is the LU factorization. An LU
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

\ Is this a good approach to solving :math:`Ax=b`?

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

\ and you want to fit a model to it:

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

Interpolation
-------------

Lagrange Interpolation
~~~~~~~~~~~~~~~~~~~~~~

There are times when you would like to just prescribe the points and
generate the polynomial without the worry of having numerical issues in
the linear solver. There are a couple of approaches to finding the
polynomial, one popular method is known as Lagrange Interpolation. For a
set of :math:`N` points, we can find a polynomial of degree :math:`N-1`
that can interpolate the points. A well known approach is to use
Lagrange polynomials:

.. math::

   x(t) = \sum_{i=0}^{N} x_i q_i(t), \quad y(t) = \sum_{i=0}^{N} y_i q_i(t)
   \quad \mbox{where}\quad
    q_i(t) = \prod_{j =0 \atop j \neq i}^N \frac{t-t_j}{t_i-t_j}

Since this is a parametric form, we have freedom to select the
:math:`\{ t_i \}` values.

.. raw:: latex

   \centering

.. figure:: motion/poly.pdf
   :alt: Polynomial Interpolant of data[Fig:PolynomialInterpolant]

   Polynomial Interpolant of data[Fig:PolynomialInterpolant]

Assume that you are given the points (0,1), (1,2), (2,5). Find the
Lagrange interpolant. First we define :math:`t_i = i` and compute the
Lagrange polynomials:

.. math:: q_0(t) = \left(\frac{t-1}{0-1}\right) \left(\frac{t-2}{0-2}\right) = \frac{1}{2} (t-1)(t-2)

.. math:: q_1(t) =  \left(\frac{t-0}{1-0}\right) \left(\frac{t-2}{1-2}\right) =  -(t)(t-2)

.. math:: q_2(t) =  \left(\frac{t-0}{2-0}\right) \left(\frac{t-1}{2-1}\right) = \frac{1}{2} (t)(t-1)

Then using the interpolation formula:

.. math:: x(t) = -t(t-2) + t(t-1)  = t,

.. math:: y(t) =  \frac{1}{2} (t-1)(t-2) - 2t(t-2) + \frac{5}{2} (t)(t-1)= t^2+1 .

This process can be used for arbitrary many points. However, the greater
the number of points, the higher degree polynomial and several problems
arise. Clearly the formulas get more complicated as well as the
computation effort. The central problem is that the interpolant can
oscillate between data points. Although the polynomial includes the data
points, a poor path emerges. Another approach is to fix the degree of
polynomial and attempt a least squares approximation. In this case, the
path will have less oscillation, but could miss many or possibly all of
the data points.

So, why not just limit the number of points used? Say we pick two or
three points at a time? Two points will give rise to a linear
interpolant and three will give rise to a quadratic interpolant. We just
take two or three at a time computing the interpolants as we travel.
This would have the added benefit that we don’t even need to know all of
them when we start. And this idea takes us to a tool known as cubic
splines - which can be done in an iterative fashion as well as having
smooth connections.

Cubic Splines[text:cubicspline]
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The straight line connection between two points discussed above uses a
linear polynomial. To gain the smoothness in the transition from point
to point, we need a higher degree polynomial. At minimum for matching at
a point requires both the location and direction. Direction is
prescribed by the derivative. This is four data items: left position,
left derivative, right position and right derivative. A quadratic only
has three degrees of freedom which would result in some points not
having a smooth transition, so we move to a cubic polynomial.

The method of Cubic Splines is one of the most popular interpolation
methods. There are several methods that can be used to find the cubic
spline given the endpoint data. In addition to fitting the data, it also
will minimize the curvature along the interpolant. This is exactly the
tool we need. It can be used iteratively as data points arrive in the
path queue and can be used iteratively to produce wheel velocities.
Assume that you have two points :math:`t_0: (x_0,y_0)` and
:math:`t_1: (x_1, y_1)`. Also assume that you have a derivative at each
point :math:`t_0: (\dot{x}_0, \dot{y}_0)` and
:math:`t_1: (\dot{x}_1, \dot{y}_1)`. The cubic spline is

.. math:: x(t) = (1-z)x_0 + z x_1 + z(1-z)\left[ a(1-z) +b z\right]

.. math:: y(t) = (1-z)y_0 + z y_1 + z(1-z)\left[ c(1-z) +d z\right]

\ where

.. math:: a = \dot{x}_0(t_1-t_0)-(x_1-x_0), \quad b = -\dot{x}_1(t_1-t_0)+(x_1-x_0)

.. math:: c = \dot{y}_0(t_1-t_0)-(y_1-y_0), \quad d = -\dot{y}_1(t_1-t_0)+(y_1-y_0)

.. math:: z = \displaystyle \frac{t - t_0}{t_1-t_0}

When we are working with signal filters we end up with a large number of
sample points. One of the filter techniques is to “fit" a polynomial to
the points. However, we will want to limit the degree of the polynomial
and this gives rise to non-square systems (more equations and unknowns).
This problem is addressed below in the least squares section.

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
and :math:`y` are a known as a basis. The term basis is a minimal
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

One consequence of these ideas is that of a vector space. It is the span
of a collection of vectors (or all linear combinations of the vectors).
More formally, :math:`V` is a vector space if :math:`x, y\in V` are
vectors, and :math:`a, b\in {\mathbb R}`, then :math:`ax+by \in V`.

The two examples above are vector spaces: the line through the origin
and the collection of :math:`2\times 2` matrices. Note that in the
figure below, the solid line is a vector space is, and the dotted is
not. A vector space must include the zero element and the dotted line
does not.

.. raw:: latex

   \centering

.. figure:: math/lines
   :alt: [fig:lineisnotvectorspace] Not all linear sets are vector
   spaces. The blue is and the red line is not.

   [fig:lineisnotvectorspace] Not all linear sets are vector spaces. The
   blue is and the red line is not.

A subspace is a subset of a vector space :math:`V` that is also a vector
space. For example, a line through the origin is a subspace of the
plane. Also, a plane through the origin is a subspace of three space,
such as the span of

.. math::

   \left\{\begin{pmatrix} 1 \\ 0 \\ 0\end{pmatrix}, 
   \begin{pmatrix} 0 \\ 1 \\ 0\end{pmatrix}\right\}.

The reason these concepts are brought up is that when solving linear
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

.. raw:: latex

   \centering

.. figure:: math/vrect.png
   :alt: Overdetermined System of Equations[fig:overdetermined]

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

This formula is known by several names. It is called the Pseudo-Inverse
or Moore-Penrose Pseudo-Inverse. It is also called the left-sided
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
Figure \ `[Fig:underdetermined] <#Fig:underdetermined>`__.

.. raw:: latex

   \centering

.. figure:: math/hrect
   :alt: An underdetermined system[Fig:underdetermined]

   An underdetermined system[Fig:underdetermined]

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

.. raw:: latex

   \hspace*{5mm}

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

.. raw:: latex

   \hfill

|image|

| Assume that the model for the data is :math:`y = a_2x^2 + a_1x +a_0`.
  Find :math:`a_2, a_1, a_0`. Note that the system arises:
| 

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

| One can solve :math:`X^TX a = X^T y`: :math:`a = (X^TX)^{-1} X^T y`
| 

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
Figure \ `[plot:quadgraph] <#plot:quadgraph>`__.

.. raw:: latex

   \centering

.. figure:: math/quadgraph
   :alt: The plot of :math:`y = 0.49x^2 - 1.21x + 1.42`.[plot:quadgraph]

   The plot of :math:`y = 0.49x^2 - 1.21x + 1.42`.[plot:quadgraph]

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
compute the pseudo-inverse through the Singular Value Decomposition
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
innerproduct:

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

\ The matrix :math:`Q` is any matrix for which the innerproduct above is
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
Figure \ `[Fig:weightedLSdata] <#Fig:weightedLSdata>`__. Using the
process above we compute the uniformly weighted least squares fit to a
line, shown in blue, and the weighted least squares fit to a line, shown
in green, Figure \ `[Fig:weightedLSplot] <#Fig:weightedLSplot>`__. The
weight function weights more heavily towards the origin (using
:math:`w_i = 1.0/i^3`). In this example, the weights are scaled so the
sum of the weights is one.

.. raw:: latex

   \centering

.. figure:: math/weightedleastsquaredata
   :alt: [Fig:weightedLSdata] Sample noisy data to fit a line.

   [Fig:weightedLSdata] Sample noisy data to fit a line.

.. raw:: latex

   \centering

.. figure:: math/weightedleastsquareplot
   :alt: [Fig:weightedLSplot] Least squares line fit. Uniform weighting
   in blue and weighted to the origin in green.

   [Fig:weightedLSplot] Least squares line fit. Uniform weighting in
   blue and weighted to the origin in green.

Probability
-----------

Let :math:`X` denote a random variable. Let :math:`P` denote the
probability that :math:`X` takes on a specific value :math:`x`:
:math:`P(X=x)`. If :math:`X` takes on discrete values we say that
:math:`X` is a discrete variable. If :math:`X` takes on continuous
values we say that :math:`X` is a continuous variable.

Normally :math:`P(X=x)` makes sense for discrete spaces and we use
:math:`P(x_1 <
X < x_2)` for continuous spaces. For continuous spaces we define the
probability density function (pdf) :math:`p(x)` in the following manner:

.. math:: P(x_1 \leq X \leq x_2) = \int_{x_1}^{x_2} p(x)\, dx

Uncertainty and Distributions
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Recall that random variables are drawn from some probability
distribution function. Often these are the normal distributions seen in
many areas of the sciences, but can be any shape as long as the area
under the curve is one. Specifically, the normal distribution is given
by

.. math:: p(x) = \frac{1}{\sqrt{2\pi}\, \sigma}e^{-(x-\mu)^2/2\sigma^2}

where :math:`\mu` is the mean and :math:`\sigma^2` is the variance
(:math:`\sigma` is the standard deviation). For multivariate
distributions (vector valued random variables) we can exend to

.. math:: p(x) = \frac{1}{(2\pi)^{n/2}\sqrt{\det(\Sigma)}}e^{-\frac{1}{2}(x-\mu)^T\Sigma^{-1}(x-\mu)}

where :math:`\mu` - mean vector, :math:`\Sigma` - covariance matrix
(symmetric positive definite).

.. raw:: latex

   \centering

.. figure:: math/pdf
   :alt: [fig:pdfplot] Probability Distribution Function

   [fig:pdfplot] Probability Distribution Function

Let :math:`X,Y` be two random variables, the joint distribution is

.. math:: P(x,y) = P(X=x~\mbox{and}~Y=y).

We say the the variables are independent if

.. math:: P(x,y) = P(x)P(y)

Conditional probability: what is the probability of :math:`x` if we know
:math:`y` has occurred? Denoted :math:`P(x|y)`,

.. math:: P(x|y) = \frac{P(x,y)}{P(y)}

If they are independent

.. math:: P(x|y) = \frac{P(x,y)}{P(y)}=\frac{P(x)P(y)}{P(y)} = P(x)

Total probability (relax the uppercase formalism)

.. math:: p(x) = \sum_{y} p(x|y)p(y)\quad \left[= \int_Y p(x|y)p(y)\, dy \right]

**Bayes Rule** (way to invert conditional probabilities)

.. math:: p(x|y) = \frac{p(y|x)p(x)}{p(y)}

**Expectation** or the mean or average for a distribution is given by

.. math:: E(x) = \sum x p(x) \quad \left[ =\int_X x p(x)\, dx \right]

Moments for a distribution are given by

.. math:: \tilde{\mu_r} = E(x^r) = \int_X x^rp(x)\, dx

.. math:: \mu = \tilde{\mu_1} = \quad \mbox{Mean - expected value}

Moments about the mean

.. math:: \mu_r = \int_X (x-\mu)^rp(x) \,dx

Second moment about the mean is called the *Variance*: :math:`\mu_2 =
\sigma^2`, where :math:`\sigma` is called the *Standard Deviation*. Note
that variance :math:`=E[(x-\mu)^2]` and covariance
:math:`E(X\cdot Y)-\mu\nu`

where :math:`\mu`, :math:`\nu` are the means for :math:`X` and
:math:`Y`.

The **Covariance** Matrix is given by :math:`\Sigma =`

.. math::

   \left( \begin{array}{cccc}E[(x_1-\mu_1)(x_1-\mu_1)^T]& \dots & E[(x_1-\mu_1)(x_n-\mu_n)^T] 
    \\     \dots & \ddots & \dots 
     \\ E[(x_n-\mu_n)(x_1-\mu_1)^T]  & \dots &
     E[(x_n-\mu_n)(x_n-\mu_n)^T]\end{array}\right)

.. math:: = E[(x-\mu)(x-\mu)^T]

There are many terms to describe the variance of a set of random
variables. Variance, covariance and cross-variance, variance-covariance
are a few example terms. We will use variance for scalar terms and
covariance for vector terms.

Sample covariance
^^^^^^^^^^^^^^^^^

If you know the population mean, the covariance is given by

.. math:: Q = \frac{1}{N} \sum_{k=1}^{N}(x_k - E(x))(x_k - E(x))^T

and if you don’t know the mean the covariance is given by

.. math:: Q = \frac{1}{N-1} \sum_{k=1}^{N}(x_k - \overline{x})(x_k - \overline{x})^T

Note: :math:`(x_1-\overline{x})`, :math:`(x_2-\overline{x})`,
:math:`(x_2-\overline{x})` has :math:`n-1` residuals (since they sum to
zero).

.. raw:: latex

   \FloatBarrier

Problems
--------

.. raw:: latex

   \setcounter{Exc}{0}

]

Find the smallest degree polynomial that interpolates (-1,2), (0,4), (1,
1).

Find a cubic polynomial that goes through (0,0) and (1,1); and has
derivative at the left side :math:`\{ 1\}` and on the right side is
:math:`\{ -2)\}`.

What is the parametric form for :math:`(x-2)^2/9 + (y-1)^2/4 = 1`?

Find the cubic spline (parametric) that interpolates (0,0) with
derivatives (tangent vector) :math:`<2,0>` and (1,1) with derivatives
(tangent vector) :math:`<0,-1>`. [cubicsplineddhw]

Assume that you have two cublic splines to connect together. The first
spline interpolates (0,0) and (1,1). The second spline interpolates
(1,1) and (2,2). Assume that the direction vector at (0,0) is
:math:`<1,0>` and the tangent vector at (2,2) is :math:`<0,1>`. What
condition must hold so that the resulting curve is smooth? Solve for the
two splines assuming that the resulting curve is smooth and of minimal
curvature.[connectedsplineshw]

Let :math:`A \in \mathbb{R}^{m\times m}`, :math:`x \in  \mathbb{R}^{n}`,
:math:`b \in  \mathbb{R}^{m}`. Show :math:`\nabla [x^TA^T Ax] = 2 A^TAx`
and :math:`\nabla [x^TA^Tb] = A^T b`.

Assume the data set (0, -2), (1, -1), (2,2) is a noisy observation of a
straight line. What is the best (least squares) estimate of the line?

Fit the following points to a quadratic model: (-4.5,50), (-3.6, 40),
(-3, 30), (-2, 13.5), (-1, 6), (0.7, -1), (2, 1.25), (3,8), (4.6, 23)
using uniform weights.

IP

Fit the following points to a quadratic model: (-4.5,50), (-3.6, 40),
(-3, 30), (-2, 13.5), (-1, 6), (0.7, -1), (2, 1.25), (3,8), (4.6, 23)
using :math:`w = e^{-x^2}`. You don’t have to normalize the weights.

IP

Fit the previous set of points to a quadratic model using the iterative
approach using uniform weights and using :math:`w_i = 1/i` where
:math:`i` is the point index.

Starting with the curve :math:`y = 0.25x^2 + 2x +5`, create a dataset of
100 points on the interval [0,10]. Add Gaussian noise in x
(:math:`\mu = 0`, :math:`\sigma = 0.2`) and y (:math:`\mu = 0`,
:math:`\sigma = 2.0`) to the points. Fit to a linear model using (a)
uniform weights and (b) :math:`w = e^{-x^2}`.

One can generate the points by

::

    N = 100
    mu1=0.0
    mu2 = 0.0
    sigma1 = 0.2
    sigma2 = 2.0

    x = np.linspace(0,10,N)+np.random.normal(mu1,sigma1, N) 
    y = 0.25*x*x + 2.0*x + 5.0 + np.random.normal(mu2,sigma2, N)
    plt.plot(x,y,'r.')
    plt.show()

Starting with the curve :math:`y = -x^2 + 4x`, create a dataset of 100
points on the interval [0,4]. Add Gaussian noise in x (:math:`\mu = 0`,
:math:`\sigma = 0.2`) and y (:math:`\mu = 0`, :math:`\sigma = 2.0`) to
the points. Compare the results of fitting a quadratic and a trig
function to the data.

One can generate the points by

::

    N = 100
    mu1=0.0
    mu2 = 0.0
    sigma1 = 0.2
    sigma2 = 2.0

    x = np.linspace(0,4,N)+np.random.normal(mu1,sigma1, N) 
    y = - x*x + 4.0*x +  np.random.normal(mu2,sigma2, N)
    plt.plot(x,y,'r.')
    plt.show()

.. raw:: latex

   \Closesolutionfile{Answer}

.. [1]
   Gilbert Strang - Linear Algebra, see the online text.

.. |image| image:: math/quadpts

