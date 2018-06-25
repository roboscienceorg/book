Interpolation
-------------

Natural Splines
~~~~~~~~~~~~~~~

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

 The condition that :math:`\kappa = 0` does not prescribe the size of
the derivatives, only the slope of the curve at at :math:`t=1`. So, we
may without loss of generality, set :math:`\dot{x}_q = 1` and solve for
:math:`\dot{y}_q` and get the derivative on the right as:

.. math:: <\dot{x}_q,\dot{y}_q> = \left< 1, \frac{ \dot{y}_p - 3(y_q-y_p)}{\dot{x}_p - 3(x_q-x_p) } \right>.

 This vector may be scaled to the required length. This will determine
the coefficents for

.. math:: x(t) = (1-t)x_p + t x_q + t(1-t)\left[ a(1-t) +b t\right]

.. math:: y(t) = (1-t)y_p + t y_q + t(1-t)\left[ c(1-t) +d t\right]

[singlesplinenaturalexample] Compute the spline that connects (0,0) to
(2,3) with left hand derivative :math:`<0,1>` and a natural endpoint on
the right. See Figure [singlesplinenatural].

The Python code can be found below.

::

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

|Graph of the spline and spline’s curvature for
example [singlesplinenaturalexample].[singlesplinenatural]| |Graph of
the spline and spline’s curvature for
example [singlesplinenaturalexample].[singlesplinenatural]|

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

[parametricmultispline] Compute the natural spline that interpolates
(0,0), (1,2), (2,4), (3,3), 4,2), (5,1), (6,2), (7,4), (8,5), (9,7).
Figure [parametricmultisplinefig] shows the spline produced by the
Python code below:

::

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

.. figure:: motion/parametricmultispline
   :alt: Graph of the spline for
   example [parametricmultispline].[parametricmultisplinefig]
   :width: 60.0%

   Graph of the spline for
   example [parametricmultispline].[parametricmultisplinefig]

Problems
--------

[Appendix\_ans]

Complete the Python simulation server discussed in this Chapter. Fully
develop the API and test it with the client “remote control” program
from the text. Start with the code in the text and build an extensive
API. For example, instead of having a command named “f”, build a forward
command that takes an argument (distance). When starting up the server,
prompt the user to enter the start and goal locations. Place the robot
at the start location. You will need to add commands to give feedback on
location, goal location, robot heading, impact, goal achieved, etc. You
are assuming that the obstacle has been inflated by robot radius and
that the robot is just a point (or pixel). The text gives an impact
detection example.

Draw a tree using python and turtle graphics: write a recursive drawing
function to graphically display a tree.

What is the built in turtle command to close the Tk window?

NA

.. |image| image:: netpbm/Tiny6pixel.png
   :width: 20.0%
.. |image| image:: slam/projection
   :width: 45.0%
.. |image| image:: slam/projection_a
   :width: 45.0%
.. |image| image:: slam/projection2
   :width: 45.0%
.. |image| image:: slam/projection2a
   :width: 45.0%
.. |image| image:: slam/projection_blur
   :width: 90.0%
.. |image| image:: slam/projection2_blur
   :width: 90.0%
.. |image| image:: slam/projection2a_blur
   :width: 90.0%
.. |Graph of the spline and spline’s curvature for example [singlesplinenaturalexample].[singlesplinenatural]| image:: motion/singlesplinenat
   :width: 52.0%
.. |Graph of the spline and spline’s curvature for example [singlesplinenaturalexample].[singlesplinenatural]| image:: motion/singlesplinenatk
   :width: 52.0%
