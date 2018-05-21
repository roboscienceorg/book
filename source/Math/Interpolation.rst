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
This problem is addressed below in the least squares section.
