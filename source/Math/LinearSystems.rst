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
