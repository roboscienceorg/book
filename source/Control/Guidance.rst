Guidance
--------

First a few definitions are in order. According to Wikipedia, GNC,
Guidance, Navigation and Control:

-  Guidance refers to the determination of the desired path of travel
   (the "trajectory") from the vehicle’s current location to a
   designated target, as well as desired changes in velocity, rotation
   and acceleration for following that path.

-  Navigation refers to the determination, at a given time, of the
   vehicle’s location and velocity (the "state vector") as well as its
   attitude.

-  Control refers to the manipulation of the forces, by way of steering
   controls, thrusters, etc., needed to execute guidance commands whilst
   maintaining vehicle stability.

Planners can find paths through free space. They may be planning based
on certain requirements such as minimal path, minimal curvature or
maximum distance to obstacle. Some but not all will respect the
kinematic constraints. Computationally, it may be rather expensive to
have the planner compute the path at a high resolution. The resolution
may be sufficiently coarse that it provides online performance in terms
of the planning, but opens the door for excessive drift. One may
interpolate between distant points using a polynomial interpolant. Cubic
splines are a very common interpolant which allows for endpoint values
and slope to be selected. Cubics will also provide smooth paths which
minimize curvature.

It should be noted that we are not trying to find :math:`x(t)` and
:math:`y(t)` to plug into the inverse kinematics for some robot. We will
use the parametric equations to create a finer grid of points which is
in turn handed to the speed and heading controller. Between the fine
grid points, the controller is driving and we are not using the inverse
kinematics.

.. _example-1:

Example
~~~~~~~~

Assume that your planner has provided the following points (0,0),
(10,50), (30,20). Also assume that you start with zero derivative at
(0,0), would like to pass through (10,50) with slope :math:`m=1` and
have slope :math:`m=-1` at (30,20). You would like to create set of
points on the curve separated by a distance of roughly 1 and not 10 or
50. How can you do this? The solution is to create two cubic splines
which will match slopes at the three points. First we convert the
problem into a parametric problem: :math:`(t,x,y)`: (0,0,0), (10,10,50)
and :math:`(t,x,y)`: (0,10,50), (20,30,20) This was an arbitrary choice
for the time values. Working on the first segment and following
:numref:`text:cubicspline`, for
:math:`t_0 = 0`, :math:`(x,y) = (0,0)` and
:math:`(\dot{x}, \dot{y} ) = (1,0)` and for :math:`t_1 = 10`,
:math:`(x,y) = (10,50)` and :math:`(\dot{x}, \dot{y} ) = (1,1)`. From
the data we then obtain for the first segment
:math:`z= 0.1t, a = 0, b = 0, c = -50, d = 40`, and on the next segment
:math:`z= 0.05t, a = 0, b = 0, c = 50, d = -10`.

.. _`fig:cubicsplineexample1`:
.. figure:: ControlFigures/cubicexample.*
   :width: 40%
   :align: center

   The two cubic splines from the three data
   points.

.. _`fig:cubicsplineexample2`:
.. figure:: ControlFigures/cubicexample2.*
   :width: 40%
   :align: center

   Sampling the two splines to get guidance
   data.

The plot,
:numref:`fig:cubicsplineexample1` is
produced by the following code with setting the plot command to lines,
g-. The following code as is produces
:numref:`fig:cubicsplineexample2`.

::

    import numpy as np
    import pylab as plt

    def spline(t0,t1, x0, x1, y0, y1, xd0 , yd0, xd1, yd1, N):
      dt = (t1-t0)
      dx = (x1-x0)
      dy = (y1-y0)
      a = xd0*dt- dx
      b = -xd1*dt+dx
      c = yd0*dt-dy
      d = -yd1*dt+dy
      t = np.linspace(t0,t1,N)
      dotz = 1.0/dt
      z = (dotz)*(t-t0)
      x = (1-z)*x0 + z*x1+z*(1-z)*(a*(1-z)+b*z)
      y = (1-z)*y0 + z*y1+z*(1-z)*(c*(1-z)+d*z)
      ptx = np.array([x0,x1])
      pty = np.array([y0,y1])
      return x, y, ptx, pty

    N = 20
    t0, t1 = 0, 10
    x0, y0 = 0, 0
    x1, y1 = 10, 50
    xd0 , yd0 = 1, 0
    xd1, yd1 = 1, 1
    xc1, yc1, ptx1, pty1 = spline(t0,t1, x0, x1, y0, y1, xd0 , yd0, xd1, yd1, N)

    t0, t1 = 0, 20
    x0, y0 = 10,50
    x1, y1 = 30, 20
    xd0 , yd0 = 1, 1
    xd1, yd1 = 1, -1
    xc2, yc2, ptx2, pty2 = spline(t0,t1, x0, x1, y0, y1, xd0 , yd0, xd1, yd1, N)

    plt.figure()
    plt.xlim(-5,35)
    plt.ylim(-5,65)
    plt.plot(ptx1,pty1, 'ro')
    plt.plot(ptx2,pty2, 'ro')
    plt.plot(xc1,yc1,'g.')
    plt.plot(xc2,yc2,'g.')
    plt.legend(['Data', 'Interpolant'],loc='best')
    plt.title('Cubic Spline')
    plt.savefig("cubicexample2.pdf")
    plt.show()
