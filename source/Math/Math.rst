Mathematical and Computational Background [Chap:Math]
=====================================================

Robotics as an academic subject can be very mathematical in nature. This
text does not attempt to place the subject on a rigorous mathematical
foundation, but it is necessary to use the mathematical formalism to
avoid confusion. So, we jump in with some common notation and then
proceed to the standard vocabulary that all robotics professionals use.

Note: This is under significant development.

.. include:: MathematicalNotation.rst

.. include:: Parametric.rst

.. include:: Vectors.rst

.. include:: LinearSystems.rst

.. include:: Interpolation.rst

.. include:: LinearAlgebra.rst

.. include:: Probability.rst

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

