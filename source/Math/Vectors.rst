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
