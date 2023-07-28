Interpolation
-------------


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
   :label: eqn:curvefittingmatrix

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


Lagrange Interpolation
~~~~~~~~~~~~~~~~~~~~~~

There are times when you would like to just prescribe the points and
generate the polynomial without the worry of having numerical issues in
the linear solver. There are a couple of approaches to finding the
polynomial, one popular method is known as Lagrange Interpolation. For a
set of :math:`N` points, we can find a polynomial of degree :math:`N-1`
that can interpolate the points. A well known approach is to use
:index:`Lagrange polynomials`:

.. math::

   x(t) = \sum_{i=0}^{N} x_i q_i(t), \quad y(t) = \sum_{i=0}^{N} y_i q_i(t)
   \quad \mbox{where}\quad
    q_i(t) = \prod_{j =0 \atop j \neq i}^N \frac{t-t_j}{t_i-t_j}

Since this is a parametric form, we have freedom to select the
:math:`\{ t_i \}` values.

.. _`Fig:PolynomialInterpolant`:
.. figure:: MathFigures/poly.*
   :width: 50%
   :align: center

   Polynomial Interpolant of data.

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

.. _`text:cubicspline`:

Cubic Splines
~~~~~~~~~~~~~~

The straight line connection between two points discussed above uses a
linear polynomial. To gain the smoothness in the transition from point
to point, we need a higher degree polynomial. At minimum for matching at
a point requires both the location and direction. Direction is
prescribed by the derivative. This is four data items: left position,
left derivative, right position and right derivative. A quadratic only
has three degrees of freedom which would result in some points not
having a smooth transition, so we move to a cubic polynomial.

The method of :index:`Cubic Splines` is one of the most popular interpolation
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

When we are working with signal filters we end up with a large number of
sample points. One of the filter techniques is to “fit" a polynomial to
the points. However, we will want to limit the degree of the polynomial
and this gives rise to non-square systems (more equations and unknowns).
This problem is addressed in the least squares section.



There are four free parameters when you set a cubic spline which are endpoint values and endpoint derivatives.  If there are more than two points, then the intermediate points should glue together smoothly.   A tridiagonal linear system arises when solving for the coefficients on the connected set of splines.  This is left to textbooks on numerical methods.  The equations can also be found in the wikipedia page on Cubic Splines.



:index:`Natural Splines`
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The obvious question that arises is “what if I don’t know the endpoint
derivatives". Most often the question is focused on the terminal
endpoint and not the initial endpoint of the spline since you can infer
the derivatives from the direction of the vehicle at the initial point.
So, what does one do? If you have multiple points, you can glue the
splines together by setting the derivatives at the point equal. A system
of equations will arise for the derivative values on those interior
points. This can be solved if you provide the derivative values on the
extreme ends. Again, we have the same problem. One way to approach the
issue is to assume that the curve straightens out (becomes linear). This
would mean that the curvature goes to zero at :math:`t=t_1`. Before we
proceed, we will simplify the algebra by setting :math:`t_0=0` and
:math:`t_1=1`. Assume that your planning algorithm gives you two points
which we will call :math:`(x_p,y_p)` at :math:`t=0` and
:math:`(x_q, y_q)` at :math:`t=1`. The endpoint derivatives are given by
:math:`(\dot{x}_p,\dot{y}_p)` and :math:`(\dot{x}_q, \dot{y}_q)`. We can
compute the cubic spline coefficients:

.. math:: a = \dot{x}_p-(x_q-x_p), \quad b = -\dot{x}_q+(x_q-x_p)

.. math:: c = \dot{y}_p-(y_q-y_p), \quad d = -\dot{y}_q+(y_q-y_p).

.. math:: \dot{x} = x_q -x_p  + (1 - 2t)\left[ a(1-t) +b t\right] + t(1-t)\left[ b-a\right]

.. math:: \dot{y} =   y_q -y_p  + (1 - 2t)\left[ c(1-t) +d t\right] + t(1-t)\left[ d-c\right]

.. math:: \ddot{x} = -2\left[ a(1-t) +b t\right] + 2(1-2t)\left[ b-a\right]

.. math:: \ddot{y} = -2\left[ c(1-t) +d t\right] + 2(1-2t)\left[ d-c\right]

We can compute the formula for the :math:`t=1` endpoint:

.. math:: \kappa =   \displaystyle  \frac{\dot{x}(1)\ddot{y}(1) - \dot{y}(1)\ddot{x}(1)}{v^3} = 0 \Rightarrow   \dot{x}(1)\ddot{y}(1) = \dot{y}(1)\ddot{x}(1)

.. math:: \dot{x}_q (2\dot{y}_p + 4\dot{y}_q - 6(y_q-y_p)) = \dot{y}_q (2\dot{x}_p + 4\dot{x}_q- 6(x_q-x_p))


The condition that :math:`\kappa = 0` does not prescribe the size of the derivatives, only the slope of the curve at at :math:`t=1`. So, we
may without loss of generality, set :math:`\dot{x}_q = 1` and solve for :math:`\dot{y}_q` and get the derivative on the right as:

.. math:: <\dot{x}_q,\dot{y}_q> = \left< 1, \frac{ \dot{y}_p - 3(y_q-y_p)}{\dot{x}_p - 3(x_q-x_p) } \right>.

This vector may be scaled to the required length. This will determine  the coefficents for

.. math:: x(t) = (1-t)x_p + t x_q + t(1-t)\left[ a(1-t) +b t\right]

.. math:: y(t) = (1-t)y_p + t y_q + t(1-t)\left[ c(1-t) +d t\right]

Compute the spline that connects (0,0) to
(2,3) with left hand derivative :math:`<0,1>` and a natural endpoint on
the right. See Figure.

The Python code can be found below.

.. code-block::
   :name: lst:spline

    import numpy as np
    import scipy as sp
    import pylab as plt
    delta = 0.005
    z = 1.0  # scale factor
    xp =0.0  # endpoints
    xq =2.0
    yp =0.0
    yq =3.0
    dxp =0.0  # endpoint derivatives
    dxq =z*1.0
    dyp =4.0
    dyq = z*(dyp - 3*(yq-yp))/(dxp - 3*(xq-xp))
    a = dxp-(xq-xp)  # spline coefficients
    b = -dxq+(xq-xp)
    c = dyp-(yq-yp)
    d = -dyq+(yq-yp)
    t=np.arange(0,1+delta,delta)  # points to eval spline
    x = (1-t)*xp + t*xq + t*(1-t)*(a*(1-t) +b *t)
    y = (1-t)*yp + t*yq + t*(1-t)*(c*(1-t) +d *t)
    dx = xq -xp  + (1 - 2*t)*(a*(1-t) +b *t)+ t*(1-t)*( b-a)
    dy = yq -yp  + (1 - 2*t)*(c*(1-t) +d *t) + t*(1-t)*( d-c)
    ddx = -2*( a*(1-t) +b* t) + 2*(1-2*t)*( b-a)
    ddy = -2*( c*(1-t) +d* t) + 2*(1-2*t)*( d-c)
    v = np.sqrt(dx*dx + dy*dy)
    k= (dx*ddy - dy*ddx)/(v*v*v)
    plt.plot(x,y)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Traversed Path')
    plt.show()
    plt.plot(t,k)
    plt.xlabel('T')
    plt.ylabel('K')
    plt.title('Curvature')
    plt.show()



Multiple points
~~~~~~~~~~~~~~~

In the preceeding sections we focused on computing the interpolating
polynomial for two points. A planner may return many points to describe
a path. One approach is to take the current point and the next point,
then compute the cubic spline interpolant. Each pair can be taken to
produce a spline for the current interval. Another approach is to use
the machinery developed to solve for the spline collection. The Python
SciPy library provides an Interpolation Class which includes spline
methods.

Compute the natural spline that interpolates
(0,0), (1,2), (2,4), (3,3), 4,2), (5,1), (6,2), (7,4), (8,5), (9,7).
Figure :numref:`Fig:splineplot` shows the spline produced by the
Python code below:

.. code-block::
   :name: lst:spline2
   :dedent: 1

    import numpy as np
    import pylab as plt
    from scipy import interpolate

    x = np.array([0,1,2,3,4,5,6,7,8,9])
    y = np.array([0,2,4,3,2,1,2,4,5,7])
    tck,u = interpolate.splprep([x,y],s=0)
    t = np.arange(0,1.01,0.01)
    out = interpolate.splev(t,tck)
    plt.figure()
    plt.plot(x,y,'gs',out[0],out[1],'b')
    plt.legend(['Data', 'Cubic Spline'])
    plt.title('Multipoint Spline')
    plt.show()


.. _`Fig:splineplot`:
.. figure:: MathFigures/parametricmultispline.*
   :width: 60.0%

   Graph of the spline for code above.



Derivatives
~~~~~~~~~~~~

In some of our applications we will need the derivatives at interpolating points in addition to the interpolating points.   There are two ways to approach the problem.   If you have constructed the splines using the formulas above, you can also compute derivatives of those formulas.

Recall from above

.. math::

   x(t) = (1-z)x_0 + z x_1 + z(1-z)\left[ a(1-z) +b z\right]

   y(t) = (1-z)y_0 + z y_1 + z(1-z)\left[ c(1-z) +d z\right]

where

.. math::

   a = \dot{x}_0(t_1-t_0)-(x_1-x_0), \quad b = -\dot{x}_1(t_1-t_0)+(x_1-x_0)

   c = \dot{y}_0(t_1-t_0)-(y_1-y_0), \quad d = -\dot{y}_1(t_1-t_0)+(y_1-y_0)

   z = \displaystyle \frac{t - t_0}{t_1-t_0}

We can determine the :index:`spline derivative`.   Using a mix of chain and product rules we obtain for :math:`t_0 \leq t \leq t_1`:

.. math::
   \displaystyle \frac{dx}{dt} =  \frac{dx}{dz}  \frac{dz}{dt} = \left\{x_1 -x_0  + (1 - 2z)\left[ a(1-z) +b z\right] + z(1-z)\left[ b-a\right]\right\}\dot{z}

   \displaystyle \frac{dy}{dt} =  \frac{dy}{dz}  \frac{dz}{dt} = \left\{y_1 -y_0  + (1 - 2z)\left[ c(1-z) +d z\right] + z(1-z)\left[ d-c\right]\right\}\dot{z}

   \displaystyle \frac{d^2x}{dt^2} =  \frac{d\dot{x}}{dz}  \frac{dz}{dt} = \left\{-2\left[ a(1-z) +b z\right] + 2(1-2z)\left[ b-a\right]\right\}\dot{z}^2

   \displaystyle \frac{d^2y}{dt^2} =  \frac{d\dot{y}}{dz}  \frac{dz}{dt} = \left\{-2\left[ c(1-z) +d z\right] + 2(1-2z)\left[ d-c\right]\right\}\dot{z}^2

   z = \displaystyle \frac{t - t_0}{t_1-t_0}, \quad  \dot{z} = \frac{1}{t_1-t_0}


Another method is to approximate using finite differences.

.. math::

   \frac{dx(t^*)}{dt} \approx  \frac{x(t^* + \Delta t) - x(t^*)}{\Delta t}
