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
