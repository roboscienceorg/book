.. role:: math(raw)
   :format: html latex
..

Mathematical and Computational Background [Chap:Math]
*****************************************************

Robotics, especially the controls literature, can be very mathematical
in nature. This text does not attempt to place the subject on a firm
mathematical foundation, but at times it is necessary to use the
mathematical formalism to avoid confusion. So, we jump in with some
common notation and then proceed to the standard vocabulary that all
robotics professionals use.

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
   | :math:` \equiv \{ y \in {\mathbb R}^n | d(x,y) < r\}`
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
am sure you will feel this way, but I have made significant effort to
keep the mathematical level in the first two years of an engineering
curriculum. [1]_

I have also decided to review the mathematics inline with the Python
Scientific Computing environment called SciPy. The point of this
material is not to introduce the math nor bask in glory of experiencing
the mathematics in its solitary form, but to use it in our goal to
effectively build working robots. The following material assumes that
you are familiar with Python. Python reads like pseudocode and so it is
possible to follow along without a background in Python if you have seen
some other programming language. A quick introduction is given in the
Appendices for those who want to ramp up before reading on.

SciPy and Mathematics
---------------------

SciPy, , is a collection of open-source packages for Scientific
Computing. One of the packages, redundantly named, SciPy library is a
collection of numerical methods including special functions,
integration, optimization, linear algebra, interpolation, and other
standard mathematics routines. NumPy is an open-source Python package
supporting data structures and low level algorithms for scientific
computing which is used by SciPy. [2]_ The main data structure of
interest to us from numpy is an array type and efficient methods to
access array elements.

Many of the implementations of iPython load NumPy and SciPy (as well as
the plotting package matlibplot) automatically. The idea is that most
users of iPython are going to use these. To use the NumPy or SciPy
libraries you need to import them. Since the scientific libraries are
large, we don’t want to drop them into the main namespace. The Python
community now uses the standard names for the namespaces:

::

    >>> import numpy as np
    >>> import scipy as sp
    >>> import matplotlib as mpl
    >>> import matplotlib.pyplot as plt

Scipy sub-packages need to be imported separately, for example:

::

    >>> from scipy import linalg, optimize

As stated above, some versions of iPython (some IDEs) will import
certain libraries for you. Say you are tired of typing the five import
lines above each time you run iPython. There is a full configuration
system available. To find the location of the config files, type at the
command prompt

.. code:: bash

    $ ipython locate
    /home/yourloginname/.ipython

    $ ipython locate profile foo
    /home/yourloginname/.ipython/profile_foo

| You may create a profile for each of the different iPython activities.
  We will stick with the default which is profile\_default. The startup
  files, files that get run when you start iPython, are located in the
  startup subdirectory. In my case this is:
| /Users/jmcgough/.ipython/profile\_default/startup

Inside the startup directory, I created a file: 05-early.py containing

::

    import numpy as np
    import scipy as sp
    import matplotlib as mpl
    import matplotlib.pyplot as plt

which then runs those import commands each time iPython is invoked. In
this next section, we will review some needed mathematics and introduce
SciPy as we proceed.

Vectors and Arrays
~~~~~~~~~~~~~~~~~~

A vector is a list of numbers. It can be used to represent physical
quantities like force and direction. It can be expressed as

.. math:: \vec{x} = \left< x_1, x_2, x_3, \dots , x_n \right>.

 The notation for a point in n-dimensional space and a n-dimensional
vector are similar: :math:`\vec{x}\in \mathbb{R}^n`:
:math:`\vec{x} = (x_1, x_2, ... x_n)`, and also written as

.. math::

   \vec{x} = \left(\begin{array}{c} x_1 \\ x_2 \\ \vdots
     \\ x_n \end{array}\right).

 If the context is understood, the small arrow above the variable is
left off, so :math:`\vec{x}` becomes :math:`x`. The basic datatype used
in scientific computing is the array. Arrays are used to store points,
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

Creation of an array is easy:

::

    In [1]: import numpy as np

    In [2]: x = np.array([2,3,6,4,5,0])

    In [3]: x
    Out[3]: array([2, 3, 6, 4, 5, 0])

    In [4]: len(x)
    Out[4]: 6

The array command takes a list and converts it to an array object.
Arrays are stored like C does it, in row major order. To create an array
of numbers from 0 to 10:

::

    In [2]: x = np.arange(10)

    In [3]: x
    Out[3]: array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9])

    In [4]: 2*x+1
    Out[4]: array([ 1,  3,  5,  7,  9, 11, 13, 15, 17, 19])

    In [5]: x*x
    Out[5]: array([ 0,  1,  4,  9, 16, 25, 36, 49, 64, 81])

    In [6]: np.sqrt(x)
    Out[6]:
    array([ 0.        ,  1.        ,  1.41421356,  1.73205081,  2.        ,
            2.23606798,  2.44948974,  2.64575131,  2.82842712,  3.        ])

    In [7]: np.sin(0.2*x)
    Out[7]:
    array([ 0.        ,  0.19866933,  0.38941834,  0.56464247,  0.71735609,
            0.84147098,  0.93203909,  0.98544973,  0.9995736 ,  0.97384763])

    In [8]: np.sin(0.2*x)[3]
    Out[8]: 0.56464247339503548

    In [9]: x < 7
    Out[9]: array([ True,  True,  True,  True,  True,
                      True,  True, False, False, False], dtype=bool)

    In [10]: x.sum()
    Out[10]: 45

    In [11]: x.max()
    Out[11]: 9

    In [12]: x.min()
    Out[12]: 0

    In [13]: x.mean()
    Out[13]: 4.5

    In [14]: x.std()
    Out[14]: 2.8722813232690143

    In [15]: np.where(x < 7)
    Out[15]: (array([0, 1, 2, 3, 4, 5, 6]),)

Note that indexing works like normal Python lists. A few vector
operations are also available as methods.

::

    In [2]: x = np.arange(10)

    In [3]: y = np.ones(10)

    In [4]: x
    Out[4]: array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9])

    In [5]: y
    Out[5]: array([ 1.,  1.,  1.,  1.,  1.,  1.,  1.,  1.,  1.,  1.])

    In [6]: np.dot(x,y)
    Out[6]: 45.0

    In [7]: np.outer(x,y)
    Out[7]:
    array([[ 0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.],
           [ 1.,  1.,  1.,  1.,  1.,  1.,  1.,  1.,  1.,  1.],
           [ 2.,  2.,  2.,  2.,  2.,  2.,  2.,  2.,  2.,  2.],
           [ 3.,  3.,  3.,  3.,  3.,  3.,  3.,  3.,  3.,  3.],
           [ 4.,  4.,  4.,  4.,  4.,  4.,  4.,  4.,  4.,  4.],
           [ 5.,  5.,  5.,  5.,  5.,  5.,  5.,  5.,  5.,  5.],
           [ 6.,  6.,  6.,  6.,  6.,  6.,  6.,  6.,  6.,  6.],
           [ 7.,  7.,  7.,  7.,  7.,  7.,  7.,  7.,  7.,  7.],
           [ 8.,  8.,  8.,  8.,  8.,  8.,  8.,  8.,  8.,  8.],
           [ 9.,  9.,  9.,  9.,  9.,  9.,  9.,  9.,  9.,  9.]])

Matrices
~~~~~~~~

We will make use of matrix algebra and will follow the normal
conventions. Let :math:`A, B \in \mathbb{R}^{n\times n}`,

.. math::

   A =
   \left( \begin{array}{ccc}a_{11}&\dots&a_{1n}\\ \dots & \dots & \dots
     \\ a_{n1} & \dots & a_{nn}\end{array}\right), \quad B =
   \left( \begin{array}{ccc}b_{11}&\dots&b_{1n}\\ \dots & \dots & \dots
     \\ b_{n1} & \dots & b_{nn}\end{array}\right).

 Matrix addition and multiplication are defined in the standard manner
as

-  :math:`A+B=\left( \begin{array}{ccc}a_{11}+b_{11}&\dots&a_{1n}+b_{1n}\\ \dots & \dots & \dots
     \\ a_{n1}+b_{n1} & \dots & a_{nn}+b_{nn}\end{array}\right)`

-  :math:`AB =
   \left( \begin{array}{ccc}c_{11}&\dots&c_{1n}\\ \dots & \dots & \dots
     \\ c_{n1} & \dots & c_{nn}\end{array}\right)`, where the entries
   are :math:`c_{ij} = \sum_k a_{ik}b_{kj}`

Matrix vector multiplication occurs often and is given by

-  :math:`Ax = \left( \begin{array}{ccc}a_{11}&\dots&a_{1n}\\ \dots & \dots & \dots
     \\ a_{n1} & \dots & a_{nn}\end{array}\right)\left(\begin{array}{c} x_1 \\ x_2 \\ \vdots
     \\ x_n \end{array}\right) =   \left(\begin{array}{c} \sum_k a_{1k}x_k \\ \sum_k a_{2k}x_k \\ \vdots
     \\ \sum_k a_{nk}x_k \end{array}\right)`

The identity element and the matrix transpose are given by

-  :math:`I=\left( \begin{array}{ccccc}1&0&\dots&0&0\\ 0&1&\dots&0&0\\ \vdots&\vdots & \ddots & \vdots & \vdots
     \\ 0& 0 & \dots& 1 & 0  \\ 0& 0 &  \dots &0& 1  \end{array}\right)`

-  | Transpose: :math:`A^T`: :math:`\{ a_{ij}\}^T = \{ a_{ji}\}`
   | Example: If :math:`A =
     \left( \begin{array}{ccc}1 & 2 & 3 \\ 4 & 5 & 6
       \\ 7 & 8 & 9\end{array}\right)` then :math:`A^T =
     \left( \begin{array}{ccc}1 & 4 & 7 \\ 2 & 5 & 8
       \\ 3 & 6 & 9\end{array}\right)`

Some NumPy examples using 2D arrays (or matrices):

::

    In [2]: A = np.array([[1,2,3],[4,5,6]])

    In [3]: print A
    [[1 2 3]
     [4 5 6]]

    In [4]: B = np.array([[9,8],[7,6],[5,4]])

    In [5]: print B
    [[9 8]
     [7 6]
     [5 4]]

    In [6]: A*B
    --------------------------
    ValueError                             Traceback (most recent call last)
    <ipython-input-6-e2f71f566704> in <module>()
    ----> 1 A*B

    ValueError: operands could not be broadcast together with shapes
    (2,3) (3,2)

    In [7]: np.dot(A,B)
    Out[7]:
    array([[ 38,  32],
           [101,  86]])

    In [8]: A.T
    Out[8]:
    array([[1, 4],
           [2, 5],
           [3, 6]])

    In [9]: A.T + B
    Out[9]:
    array([[10, 12],
           [ 9, 11],
           [ 8, 10]])

Note: Most of the python overloaded math operators are defined
elementwise. As such :math:`*` does not make sense for :math:`A*B` since
the arrays are not the same dimension. The point is that you need to be
careful and in this case you need to call the correct function to do
matrix multiplication and not array multiplication.

One can easily create a two dimensional array by reshaping:

::

    In [10]: z = np.arange(16)

    In [11]: z
    Out[11]: array([ 0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12,
                                    13, 14, 15])

    In [12]: z.shape = (4,4)

    In [13]: z
    Out[13]:
    array([[ 0,  1,  2,  3],
           [ 4,  5,  6,  7],
           [ 8,  9, 10, 11],
           [12, 13, 14, 15]])

    In [14]: z[1,3]
    Out[14]: 7

    In [15]: z[1,-4]
    Out[15]: 4

Some additional matrix terms and properties:

-  The matrix determinant is indicated by det(\ :math:`A`)

-  The transpose formula is given by :math:`(AB)^T=B^TA^T`

-  The determinant formula is given by det(\ :math:`AB`) =
   det(\ :math:`A`)det(\ :math:`B`)

-  A symmetric matrix is defined by :math:`A^T = A`

-  A symmetric positive definite matrix satisfies :math:`x^T A x >0` for
   :math:`x \neq 0`.

Using previous examples of :math:`A` and :math:`B`:

::

    In [16]: import numpy.linalg as npl

    In [17]: npl.det(np.dot(A,B))
    Out[17]: 35.99999999999968

Linear Systems
~~~~~~~~~~~~~~

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

 Is this a good approach to solving :math:`Ax=b`?

No. The fast multiplication algorithms are not numerically stable. Best
to use a Gauss-Jordan based approach like the LU factorization. LU can
also make good use of matrix structure. Possible that an algorithm may
list an inverse, but this can often be converted to a linear solve. For
example if the formula lists :math:`y^* = y + BC^{-1}x`, then solve
:math:`Cz = x` first and then find :math:`y^*=y+Bz`.

We use both NumPy and SciPy for Linear Algebra problems. NumPy is used
to provide the array data structure and the numerical methods are
provided in SciPy.

::

    In [1]: import numpy as np

    In [2]: import scipy as sp

    In [3]: from scipy import linalg as spl

    In [4]: A = np.array([[3,1,0],[1,5,1],[0,2,6]])

    In [5]: A
    Out[5]:
    array([[3, 1, 0],
           [1, 5, 1],
           [0, 2, 6]])

    In [6]: b = np.array([[3,2,1]]).T

    In [7]: b
    Out[7]:
    array([[3],
           [2],
           [1]])

    In [8]: x1 = spl.inv(A).dot(b)  # x = inverse(A)*b

    In [9]: x1
    Out[9]:
    array([[ 0.93589744],
           [ 0.19230769],
           [ 0.1025641 ]])

    In [10]: x2 = spl.solve(A,b)  # solve Ax = b

    In [11]: x2
    Out[11]:
    array([[ 0.93589744],
           [ 0.19230769],
           [ 0.1025641 ]])

    In [12]: A.dot(x1)
    Out[12]:
    array([[ 3.],
           [ 2.],
           [ 1.]])

One question that arises is regarding performance. There is a
significant difference between plain Python and NumPy. This author’s
experiments have shown that NumPy performs very well and has fallen
within 10-20% of plain C in some cases. Given how powerful the
Python-NumPy combination is, this is a small price.

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

 You can read off the coefficients here: :math:`a_2=1`, :math:`a_1=0`
and :math:`a_0=1`. Thus we obtain :math:`y = x^2 +1` which checks with
the data. The next section gives you a way to do this without a matrix
solve.

Parametric Form
~~~~~~~~~~~~~~~

Say you want to traverse a path :math:`C`, shown in
Figure [Fig:intro-path].

0.485 |Paths for explicit and parametric functions.[Fig:intro-path]|

0.485 |Paths for explicit and parametric functions.[Fig:intro-path]|

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

| The first step is to write in parametric form: :math:`x(t)`,
  :math:`y(t)`. Example: convert :math:`y=x^2` to parametric

  .. math:: \mbox{Let } x = t  \quad \to \quad y = x^2 = t^2

   Note that there are an infinite number of choices :
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

Graphing parametric functions
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The Python plot command (well, this is actually the MatPlotLib library
for Python) takes an array of x values and an array of y values. This
means that it is very easy to generate explicit plots, :math:`y=f(x)` or
parametric plots, :math:`x=f(t)`, :math:`y=g(t)`. So, for example one
can easily plot a regular function via

::

    import numpy as np
    import pylab as plt

    x = np.linspace(0,5,25) # 25 equally spaced points on [0,5]
    y = 0.15*x*x*x  #  Generate the y values from y = 0.15x^3

    plt.plot(x,y,'bo')  #  Plot x-y values using blue dots
    plt.show()

    plt.plot(x,y,'b-')  #  Plot x-y values using a blue line
    plt.show()

0.49 |Plots generated by Python.[Fig:exampleplots]|

0.49 |Plots generated by Python.[Fig:exampleplots]|

The two plots should look like Figure [Fig:exampleplots]. You will
notice that the line plot hides the fact that the underlying data is
actually discrete. The point plot provides the actual points. The same
thing can be done using a parametric version making the small change in
the code:

::

    t = np.linspace(0,5,25)
    x = t
    y = 0.15*t*t*t

You will also notice that the space between the points is not the same
even though x (or t) was generated using uniform spacing. The x spacing
is uniform, but the y value is s nonlinear function of x and the spacing
between is not constant.

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
    (Figure [Fig:intro-path2] :math:`A=1`, :math:`B=1`, :math:`a=3`,
    :math:`b=4`). Infinity: :math:`A=1`, :math:`B=0.25`, :math:`a=1`,
    :math:`b=2`

Root
    :math:`x(t) =  t^2`, :math:`y(t) = t`.

Heart
    :math:`x(t) = 16\sin^3(t)`,
    :math:`y(t) = 13\cos(t) - 5\cos(2t) -2\cos(3t) - \cos(4t)`

Code Sample (heart):
^^^^^^^^^^^^^^^^^^^^

::

    import numpy as np
    import pylab as plt
    import math
    t = np.linspace(-math.pi,math.pi,200)
    x = 16*(np.sin(t))**3
    y = 13*np.cos(t) - 5*np.cos(2*t) - 2*np.cos(3*t) - np.cos(4*t)
    plt.plot(x,y,'r-')
    plt.show()

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
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

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

 where

.. math:: a = \dot{x}_0(t_1-t_0)-(x_1-x_0), \quad b = -\dot{x}_1(t_1-t_0)+(x_1-x_0)

.. math:: c = \dot{y}_0(t_1-t_0)-(y_1-y_0), \quad d = -\dot{y}_1(t_1-t_0)+(y_1-y_0)

.. math:: z = \displaystyle \frac{t - t_0}{t_1-t_0}

[cubicsplineexample] Assume you want the spline that connects the points
(1,-1) with (3,4). Also assume that the derivative at (1,-1) is given by
:math:`<1,-3>` and at (3,4) is given by :math:`<0,2>`. We can take
:math:`t_0=0` and :math:`t_1 = 1`. This gives :math:`z = t`,
:math:`\dot{z} = 1`, :math:`a = 1 - 2 = -1`, :math:`b = 2`,
:math:`c = -8`, :math:`d = 3`. This gives us the two splines for the
parametric description of the curve:

.. math:: x(t) = (1-t) + 3t + t(1-t)[-1(1-t) + 2t]  = -3 t^3+4 t^2+t+1

.. math:: y(t) = -(1-t) + 4t + t(1-t)[-4(1-t)+3t] =  -11 t^3+19 t^2-3 t-1

.. math:: \dot{x} = -9t^2+8t+1, \quad \ddot{x} =   -18t+8

.. math:: \dot{y} =   -33t^2 +38t -3, \quad \ddot{y} =  -66t+38

 See Figure [cubicsplinefigure] for a plot.

::

    t0, t1 = 0, 1
    x0, y0 = 1, -1
    x1, y1 = 3, 4
    xd0 , yd0 = 1, -3
    xd1 = 0
    yd1 = 2
    dt = (t1-t0)
    dx = (x1-x0)
    dy = (y1-y0)
    a = xd0*dt- dx
    b = -xd1*dt+dx
    c = yd0*dt-dy
    d = -yd1*dt+dy
    t = np.linspace(t0,t1,100)
    dotz = 1.0/dt
    z = (dotz)*(t-t0)
    x = (1-z)*x0 + z*x1+z*(1-z)*(a*(1-z)+b*z)
    y = (1-z)*y0 + z*y1+z*(1-z)*(c*(1-z)+d*z)
    ptx = np.array([x0,x1])
    pty = np.array([y0,y1])

    plt.figure()
    plt.xlim(0,4)
    plt.ylim(-2,5)
    plt.plot(ptx,pty, 'ro',x,y,'g-')
    plt.legend(['Data', 'Interpolant'],loc='best')
    plt.title('Cubic Spline')
    plt.show()

.. figure:: control/cubicspline.pdf
   :alt: Graph of the spline for
   example [cubicsplineexample].[cubicsplinefigure]

   Graph of the spline for
   example [cubicsplineexample].[cubicsplinefigure]

When we are working with signal filters we end up with a large number of
sample points. One of the filter techniques is to “fit" a polynomial to
the points. However, we will want to limit the degree of the polynomial
and this gives rise to non-square systems (more equations and unknowns).
This problem is addressed below in the least squares section.

Some terms from Linear Algebra
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

When we attempt to integrate multiple sensors or when we compute paths
from discrete points, the methods use the tools from linear algebra. No
attempt here is made to be complete or expository. This is intended to
review the language and concepts only. The reader who is unfamiliar with
Linear Algebra as a subject is strongly encouraged to explore it. [3]_
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
:math:`t \left< a  , b \right>` where :math:`t\in {\mathbb R}`
(:math:`t` is the scale factor). Geometrically we are scaling the vector
into spanning a line. The vector we are using is
:math:`\left< a  , b \right>`. Another example is the collection of all
:math:`2\times 2` matrices:

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

::

    In [2]: import scipy.linalg as lin

    In [3]: a = np.array([[3, 1, 0], [1, 5, 1],  [0, 2, 6]])

    In [4]: lin.eig(a)
    Out[4]:
    (array([ 2.48586307+0.j,  4.42800673+0.j,  7.08613020+0.j]),
     array([[ 0.86067643,  0.39715065,  0.11600488],
           [-0.44250554,  0.5671338 ,  0.47401104],
           [ 0.25184308, -0.72154737,  0.87284386]]))

Eigenvalues pop up all through engineering computations and we will use
the built in SciPy routines to compute them. The most common application
later will be finding the error ellipses for variance-covariance
matrices in the Kalman Filter.

Eigenvalues for Symmetric Matrices
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Assume that :math:`A` is a real symmetric matrix and that
:math:`(\lambda, v)` is an eigenvalue, eigenvector pair. If :math:`v` is
complex valued then :math:`\| v \|^2 = v \cdot \bar{v}` where
:math:`\bar{v}` is the complex conjugate of :math:`v`. Then we have

.. math:: \lambda \| v \|^2 =  \lambda v \cdot \bar{v} = Av  \cdot \bar{v} = v \cdot A \bar{v} =  v \cdot  \overline{Av} =  v \cdot  \overline{\lambda v}  = \bar{\lambda} v \cdot \bar{v} = \bar{\lambda} \| v \|^2

 So this implies that :math:`\lambda = \bar{\lambda}` or that
:math:`\lambda` is real valued.

Error Ellipses
^^^^^^^^^^^^^^^

In the section on Kalman filters, we will want to track the progress of
the filter by tracking the error of the estimate. It is normally
represented by an error ellipse where the ellipse size is the variances
or standard deviations of the Kalman estimate. Thus the larger the
standard deviations then the larger the ellipse. As you will see later
Kalman process produces a covariance, :math:`P`. The eigenvalues and
eigenvectors of :math:`P` can be used for the basic variance
information. The eigenvectors represent the major and minor axis
directions and the eigenvalues represent the lengths of those axes.
Note: in some applications it makes sense to graph the standard
deviations instead of the variances and so one should take the square
root of the eigenvalues. The algorithm follows.

-  Compute the eigenvalues and eigenvectors of :math:`P`:
   :math:`(\lambda_1, v_1)`, :math:`(\lambda_2, v_2)`. Call the larger
   one :math:`a` and the smaller one :math:`b`.

-  Compute the square roots of the eigenvalues IF desired (if the
   variances are really small or really huge).

-  Compute the smaller angle between the eigenvector and the
   :math:`x`-axis. Call this :math:`\theta` and assume it is for
   :math:`v_1`.

-  Call an ellipse routine to plot.

Let a, b be the major and minor axis lengths, x0, y0 be the center and
angle be the tilt angle. The function to plot an rotated ellipse is
given by:

::

    def Ellipse(a,b,angle,x0,y0):
        points=100
        cos_a,sin_a=math.cos(angle*math.pi/180),math.sin(angle*math.pi/180)
        theta=np.linspace(0,2*np.pi,points)
        X=a*np.cos(theta)*cos_a-sin_a*b*np.sin(theta)+x0
        Y=a*np.cos(theta)*sin_a+cos_a*b*np.sin(theta)+y0
        return X,Y

The following is an example of how to plot an error ellipse for the
covariance matrix

.. math:: P = \begin{pmatrix} 0.9 & 0.1 \\ 0.1 & 0.5 \end{pmatrix}

about the point :math:`(4,5)`. We use the eigenvalues and eigenvectors
to plot the major and minor axes. The following is a quick example on
how to extract eigenvalues and plot an ellipse.

::

    import math
    import numpy as np
    import pylab as plt
    from numpy import linalg
    P = np.array([[0.9, 0.1],[0.1, 0.5]])
    w, v = linalg.eig(P)
    angle = 180*math.atan2(v[1][0],v[0][0])/math.pi
    u,v = Ellipse(w[0],w[1],angle, 4,5)
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(u,v,'b-')
    ax.set_aspect('equal')
    fig.savefig("Ellipse.pdf")
    plt.show()

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

A Pseudo-Inverse for linear systems
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

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

 where we will call the minimizer :math:`\hat{x}`. To minimize we
express the norm as a matrix multiply:

.. math:: \| Ax - b\|^2 =  (Ax - b)^T(Ax - b) =  (Ax)^T(Ax) - b^T(Ax) -  (Ax)^Tb +  b^Tb .

 Note that :math:`b^TAx  =  (Ax)^Tb`, and :math:`(Ax)^T = x^TA^T`, so we
have

.. math:: \| Ax - b\|^2 = x^TA^T Ax -2x^TA^Tb  +   b^Tb.

 Next we form the gradient of the norm with respect to :math:`x`. We
leave to a homework to show :math:`\nabla [x^TA^T Ax] = 2 A^TAx` and
:math:`\nabla [x^TA^Tb] = A^T b`. Then we have

.. math:: \nabla \| Ax - b\|^2 = 2 A^TAx  - 2A^T b  .

 To find the minimizer, set :math:`\nabla \| Ax - b\|^2 = 0` so we
obtain

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
and so has the structure shown in Figure [Fig:underdetermined].

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
^^^^^^^^^^^^^^^^^^^^^^^^

| Left Moore-Penrose Pseudo-Inverse (:math:`A` has linearly independent
  columns):
| :math:`A^+ = \left(A^TA\right)^{-1} A^T `, and :math:`A^+ A =I`
| Right Moore-Penrose Pseudo-Inverse (:math:`A` has linearly independent
  rows):
| :math:`A^+ = A^T \left(AA^T\right)^{-1} `, and :math:`A A^+ =I`

Applying the pseudo-inverse to the curve fitting problem
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

We return to our system which arose from the curve fitting problem.
Recall we had the linear system formulation for the curve fitting
problem [eqn:curvefittingmatrix]:

.. math:: y = X a

 We assume that we have many data points but wish a low degree
polynomial to fit the data points, :math:`k >> n+1` where :math:`k` is
the number of points and :math:`n` is the degree of the polynomial. This
is an overdetermined problem and presents us with a non-square matrix
:math:`A`. Using the tools just presented, that of a left-sided
pseudo-inverse, we form the normal equations

.. math:: X^T y = X^TXa

 we obtain a solvable system. If :math:`X^T X` is of full rank, then we
can invert

.. math:: a = \left(X^T X\right)^{-1} X^Ty

 Once :math:`a` is found then we may use

.. math:: \hat{y} = a_n x^n + a_{n-1}x^{n-1} + \dots + a_1x + a_0

 as the “fit” to the data.

Curve Fit Example
~~~~~~~~~~~~~~~~~

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

|image|

::

    import numpy as np
    import pylab as plt
    x = []
    y = []
    f = open('data.txt','r')
    for line in f:
      item = line.split()
      xt = eval(item[0])
      yt = eval(item[1])
      x.append(xt)
      y.append(yt)

    plt.plot(x,y, 'ro')
    plt.show()

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
Figure [plot:quadgraph].

.. figure:: math/quadgraph
   :alt: The plot of :math:`y = 0.49x^2 - 1.21x + 1.42`.[plot:quadgraph]

   The plot of :math:`y = 0.49x^2 - 1.21x + 1.42`.[plot:quadgraph]

::

    import numpy as np
    import pylab as plt
    from scipy import linalg

    xl = []
    yl = []
    f = open('data.txt','r')
    for line in f:
      item = line.split()
      xt = eval(item[0])
      yt = eval(item[1])
      xl.append(xt)
      yl.append(yt)

    N = len(xl)
    x = np.array(xl)
    y = np.array(yl)
    xx = x*x
    A = np.array([xx, x, np.ones((N))]).T
    AT = np.array([xx, x, np.ones((N))])
    AA = np.dot(AT,A)
    ATy = np.dot(AT,y)

    c = linalg.solve(AA,ATy)
    t = np.arange(0,3, 0.1)
    tt = t*t
    B = np.array([tt,t,np.ones(len(t))]).T
    s = np.dot(B,c)

    plt.plot(t,s, 'b-', x,y, 'ro')
    plt.xlim(0,3)
    plt.ylim(0,2)
    plt.show()

Note that NumPy/SciPy provides some built in functions to fit
polynomials to lines. The NumPy function linalg.lstsq will compute the
pseudoinverse via the normal equations directly and the NumPy function
polyfit will do this assuming you are fitting a polynomial. In terms of
speed, doing it ourselves tends to be fastest, with the next fastest is
the lstsq function and the polyfit function the slowest.

Overfitting
~~~~~~~~~~~

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
^^^^^^^^^^^^^^^^^^^^^^

Traditional least squares is formulated by minimizing using the normal
innerproduct:

.. math:: x^Ty = \sum_i x_iy_i.

Let :math:`x, y\in R^n`. No weights are referred to as uniform
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

 The matrix :math:`Q` is any matrix for which the innerproduct above is
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
Figure [Fig:weightedLSdata]. Using the process above we compute the
uniformly weighted least squares fit to a line, shown in blue, and the
weighted least squares fit to a line, shown in green,
Figure [Fig:weightedLSplot]. The weight function weights more heavily
towards the origin (using :math:`w_i = 1.0/i^3`). In this example, the
weights are scaled so the sum of the weights is one.

.. figure:: math/weightedleastsquaredata
   :alt: [Fig:weightedLSdata] Sample noisy data to fit a line.

   [Fig:weightedLSdata] Sample noisy data to fit a line.

.. figure:: math/weightedleastsquareplot
   :alt: [Fig:weightedLSplot] Least squares line fit. Uniform weighting
   in blue and weighted to the origin in green.

   [Fig:weightedLSplot] Least squares line fit. Uniform weighting in
   blue and weighted to the origin in green.

Assume that you have the raw data ready in arrays :math:`x` and
:math:`y`. Then

::

    one = np.ones((N))
    A = np.array([ x, one]).T
    AT = A.T
    AA = np.dot(AT,A)
    ATy = np.dot(AT,y)
    t = np.arange(0,10, 0.2)
    B = np.array([t,np.ones(len(t))]).T

    c = linalg.solve(AA,ATy)
    line1 = np.dot(B,c)

    weights =[]
    sum = 0
    for i in range(1,N+1):
        v = 1.0/(i*i*i)
        sum = sum + v
        weights.append(v)

    for i in range(N):
        weights[i] = weights[i]/sum

    ww = np.diagflat(weights)
    A1 = np.dot(ww,A)
    AA = np.dot(AT,A1)
    y1 = np.dot(ww,y)
    ATy = np.dot(AT,y1)
    coeff2 = linalg.solve(AA,ATy)
    line2 = np.dot(B,coeff2)

    # Plot result: red is data, blue is uniformly weighted,
    #  green is weighted to points near the origin.
    plt.plot(t,line1, 'b-', t,line2, 'g-', x,y, 'r.')
    plt.show()

Probability
~~~~~~~~~~~

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
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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

.. figure:: math/pdf
   :alt: [fig:pdfplot] Probability Distribution Function

   [fig:pdfplot] Probability Distribution Function

Let :math:`X,Y` be two random variables, the joint distribution is

.. math:: P(x,y) = P(X=x~\mbox{and}~Y=y).

 We say the the variables are independent if

.. math:: P(x,y) = P(x)P(y)

 Conditional probability: what is the probability of :math:`x` if we
know :math:`y` has occurred? Denoted :math:`P(x|y)`,

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
:math:`E(X\cdot Y)-\mu\nu` where :math:`\mu`, :math:`\nu` are the means
for :math:`X` and :math:`Y`.

The **Covariance** Matrix is given by :math:`\Sigma =`

.. math::

   \left( \begin{array}{cccc}E[(x_1-\mu_1)(x_1-\mu_1)^T]& \dots & E[(x_1-\mu_1)(x_n-\mu_n)^T]
    \\     \dots & \ddots & \dots
     \\ E[(x_n-\mu_n)(x_1-\mu_1)^T]  & \dots &
     E[(x_n-\mu_n)(x_n-\mu_n)^T]\end{array}\right)

.. math:: = E[(x-\mu)(x-\mu)^T]

 There are many terms to describe the variance of a set of random
variables. Variance, covariance and crossvariance, variance-covariance
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

Generation of Noise
~~~~~~~~~~~~~~~~~~~

You will see later on we go to great efforts to remove noise from a
dataset. So, it might seem odd to have a section on generating noise.
However, it is very useful to be able to generate noise for more
realistic simulations and to test the filters that are intended to
remove the noise. The Numpy library supports the generation of random
numbers as well as some convenient functions to draw numbers from
certain types of distributions. Most of our work will use normal
distributions. The numpy function random.normal will generate random
(well, approximately) values drawn from a normal distribution. For
example, the following code will generate a scatter plot, see
Figure [fig:samplescatterplot].

::

    mu1 = 1.0
    sigma1 = 0.5
    mu2 = 2.0
    sigma2 = 1.0
    x = np.random.normal(mu1,sigma1,100)
    y = np.random.normal(mu2,sigma2,100)
    plt.plot(x,y,'b.')
    plt.show()

This code will generate a sampled line with noise, see
Figure [fig:samplenoisylineplot].

::

    mu = 0.0
    sigma = 1.0
    error = np.random.normal(mu,sigma,100)
    x = np.linspace(0,5,100)
    y = 2*x+1.0 + error
    plt.plot(x,y,'b.')
    plt.show()

0.485 |An example of drawing from a normal
distribution.[fig:randomvalueplot]|

0.485 |An example of drawing from a normal
distribution.[fig:randomvalueplot]|

Above we are sampling from a single normal distribution (univariate),
however, later on we will need to sample from multivariate distribution.
We provide the algorithm below or this can be done with
np.random.multivariate\_normal.

::

    >>> mean = [0,0]
    >>> covar = [[.5,.05],[.05,1.0]]
    >>> N = 10
    >>> np.random.multivariate_normal(mean,covar,N)
    array([[ 0.88598172, -0.4423436 ],
           [ 0.13454988, -0.72543919],
           [-0.37652703,  0.74301719],
           [ 0.25273237, -0.63923146],
           [-1.43009133, -0.53752537],
           [ 0.27189567, -0.56165933],
           [-0.23506435,  0.82847583],
           [ 0.47206316,  0.46425447],
           [-0.33998358,  0.4583102 ],
           [-1.07647896,  0.90586496]])
    >>>

If you want to do this by hand:

#. Generate the random numbers for each variable.

#. Place them into an array.

#. Compute their variance-covariance matrix.

#. Perform a Cholesky factorization on the variance-covariance matrix.

#. Invert the Cholesky factor and multiple it by the random matrix data.
   This normalizes the dataset.

#. Compute a Cholesky factorization of the desired variance-covariance
   matrix.

#. Multiply the last Cholesky factor times the normalized data.

::

    import numpy as np

    N = 100
    sigma = 1.0

    # Create two vectors of random numbers
    #
    ex = np.random.normal(0,sigma,N)
    ey = np.random.normal(0,sigma,N)

    # Stack them into an array
    #
    D = np.vstack((ex,ey))

    # Normalize the distribution
    M = np.cov(D)
    MC = np.linalg.cholesky(M)
    MCI = np.linalg.inv(MC)
    MD = np.dot(MCI,D)

    # Enter the desired covariance matrix:
    #
    W = np.array([[0.1, 0.01],[0.01,0.2]])

    # Perform the Cholesky decomposition
    #
    L = np.linalg.cholesky(W)

    # Multiply the Cholesky factor L with the data
    # (which transforms the data to having the correct
    # covariance)
    #
    # LD is the random data with the correct covariance
    LD = np.dot(L,MD)

    # Print the result to check if it is close to w
    #print(np.cov(LD))

MatPlotLib
----------

MatPlotLib is the SciPy library used for generating plots. We will be
using the *pyplot* functions from it. The standard import convention is
import matplotlib.pyplot as plt. The basic tool is the function plot.
Let x and y be lists of numbers representing the points
:math:`(x_i , y_i)`. Simple plots can be made using plot(x,y).

::

    In [1]: x = [0,1,2,3,4]

    In [2]: y = [0,1,4,9,16]

    In [3]: plt.plot(x,y)
    Out[3]: [<matplotlib.lines.Line2D at 0x105079490>]

    In [4]: plt.show()

    In [5]: plt.plot([1,2,3,4], [1,4,9,16], 'ro')
    Out[5]: [<matplotlib.lines.Line2D at 0x110a586d0>]

    In [6]: plt.axis([0, 6, 0, 20])
    Out[6]: [0, 6, 0, 20]

    In [7]: plt.show()

This code produces the following two plots:

|image| |image|

This is efficiently done using NumPy arrays instead of lists and using
NumPy functions to generate the arrays.

::

    In [1]: x = np.arange(0,10,0.1)

    In [2]: y = np.sin(x)

    In [3]: plt.plot(x,y,'b-')
    Out[3]: [<matplotlib.lines.Line2D at 0x104880490>]

    In [4]: plt.show()

    In [5]: z = np.cos(x)

    In [6]: plt.plot(y,z)
    Out[6]: [<matplotlib.lines.Line2D at 0x105ea7810>]

    In [7]: plt.show()

|image| |image|

Surface plots may be done by importing the library
mpl\_toolkits.mplot3d. For surface plotting to work, a meshgrid needs to
be created. This can be easily built from the x and y array data. The 3D
plotting support is in a toolit shipped wiht matplotlib. It is accessed
via the axis setting in the figure function:

::

    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

An example of a quadratic surface [plot:basicsurfaceplot] Many other
plot examples can be found at the MatPlotLib website.

::

    from mpl_toolkits.mplot3d import Axes3D
    import matplotlib.pyplot as plt
    import numpy as np
    x = np.arange(0, 10, 0.2)
    y = np.arange(0, 10, 0.2)
    N,M = x.size, y.size

    x,y = np.meshgrid(x,y)
    z = (x-5)*(x-5) + (y-6)*(y-6)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot_surface(x, y, z, rstride=1, cstride=1, color='b')
    plt.show()

Another example will illustrate both the plotting capability as well as
linear regression [plot:fitcurveexample].

::

    import numpy as np
    import matplotlib.pyplot as plt
    from scipy import linalg

    xl = [  0.        ,   1.11111111,   2.22222222,   3.33333333,
          4.44444444,   5.55555556,   6.66666667,   7.77777778,
          8.88888889,  10.        ]
    yl = [  1.86113482,   3.81083902,   4.1465256 ,   7.37843476,
          10.76437019,  11.99975421,  14.59486508,  16.0576472 ,
          20.77206089,  20.4204027 ]

    N = len(xl)
    x = np.array(xl)
    y = np.array(yl)
    A = np.array([x, np.ones((N))]).T
    AT = np.array([x, np.ones((N))])
    AA = np.dot(AT,A)
    ATy = np.dot(AT,y)

    c = linalg.solve(AA,ATy)
    t = np.arange(0,10, 0.25)
    B = np.array([t,np.ones(len(t))]).T
    s = np.dot(B,c)

    plt.plot(t,s, 'b-', x,y, 'ro')
    plt.xlim(0,10)
    plt.ylim(0,20)
    plt.show()

0.45 |Line fit and plot example.[plot:fitcurveexample]|

0.45 |Line fit and plot example.[plot:fitcurveexample]|

Animation
~~~~~~~~~

Animation is done using the draw command. Create a plot with the plot
command and then update the lists using the set\_ydata command. The draw
commend will draw the updated data into the existing plot window.

::

    from pylab import *
    import time

    ion()

    tstart = time.time()               # for profiling
    x = arange(0,2*pi,0.01)            # x-array
    line, = plot(x,sin(x))
    for i in arange(1,200):
        line.set_ydata(sin(x+i/10.0))  # update the data
        draw()                         # redraw the canvas

    print 'FPS:' , 200/(time.time()-tstart)

Interactive mode needs to be toggled using ion() and an empty plot
created. Next a loop runs through the positions of the points. The setp
command updates the plot data values. Appended to the plot values (the
plot comand) is the previous points to give the effect of a traced path.
After the animation, interactive mode is toggled, ioff() and the show()
command is executed to hold the image.

::

    import numpy as np
    import matplotlib.pyplot as plt
    import time
    from math import *

    plt.ion()

    line = plt.plot([],[],'ro')
    plt.xlim(0, 10)
    plt.ylim(0, 10)
    plt.xlabel('x')
    plt.ylabel('y')
    plt.draw()
    dt = 0.1

    for t in np.arange(0,8,dt):
        x = t
        y =  x*(8-x)/2.0
        plt.setp(line,xdata = x, ydata = y)
        plt.draw()
        plt.plot([x],[y],'b.')

    plt.ioff()
    plt.show()

|image|

Another animation example is to give virtual velocity commands to move a
point. Say you wanted to animate an object which was moving by

.. math::

   \displaystyle \left(\frac{dx}{dt}, \frac{dy}{dt}\right) =
   \left\{
   \begin{array}{ll}
   (0.5, 0.0),  & 0 \leq t < 2, \\[3mm]
   (0.25, 1.0),  & 2 \leq t < 5, \\[3mm]
   (1.0, 0.0),  & 5 \leq t < 8, \\[3mm]
   (0.3, -1.0), & 8 \leq t < 10,
   \end{array}
   \right.

 and starting at :math:`t=0`, :math:`(x,y)  = (0.1, 3)`. Using the
approximation of the derivative

.. math::

   \displaystyle \frac{dx}{dt} \approx \frac{x(t+\Delta t) - x(t)}{\Delta t}
   \quad\quad \Rightarrow \quad\quad
    \left[ x_\text{current} + \left(\frac{dx}{dt}\right) \Delta t \right] \rightarrow   x_\text{new}

::

    import numpy as np
    import matplotlib.pyplot as plt
    import time
    from math import *

    plt.ion()

    line, = plt.plot([],[],'bo')
    plt.xlim(0, 10)
    plt.ylim(0, 10)
    plt.xlabel('x')
    plt.ylabel('y')
    plt.draw()
    x = 0.1
    y = 3
    dt = 0.1


    for t in np.arange(0,10,dt):
        if t < 2:
            x = x + 0.5*dt
        if (t>=2) and (t<5):
            x = x + 0.25*dt
            y = y + dt
        if (t>=5) and (t<8):
            x = x + dt
        if (t>=8):
            x = x+0.3*dt
            y = y - dt
        line.set_xdata([x])
        line.set_ydata([y])
        plt.draw()
        time.sleep(0.1)

    plt.ioff()
    plt.show()

Problems
--------

[Math\_ans]

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

.. [1]
   Calculus, Linear Algebra, Probability and Statistics.

.. [2]
   Thanks to the NumPy and SciPy online tutorials for great examples.

.. [3]
   Gilbert Strang - Linear Algebra, see the online text.

.. |Paths for explicit and parametric functions.[Fig:intro-path]| image:: control/path1
.. |Paths for explicit and parametric functions.[Fig:intro-path]| image:: control/path2
.. |Plots generated by Python.[Fig:exampleplots]| image:: control/plot1
.. |Plots generated by Python.[Fig:exampleplots]| image:: control/plot2
.. |image| image:: math/quadpts
.. |An example of drawing from a normal distribution.[fig:randomvalueplot]| image:: math/randomvalues
.. |An example of drawing from a normal distribution.[fig:randomvalueplot]| image:: math/randomvalues2
.. |image| image:: math/plot_1
.. |image| image:: math/plot_2
.. |image| image:: math/plot_3
.. |image| image:: math/plot_4
.. |Line fit and plot example.[plot:fitcurveexample]| image:: math/plot_5
.. |Line fit and plot example.[plot:fitcurveexample]| image:: math/plot_6
.. |image| image:: math/plot_8
