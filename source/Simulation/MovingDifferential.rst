Moving the Differential Drive robot
-----------------------------------

In the last section, we moved the two link articulator by updating the
position. It is certainly possible to simulate a robot moving through
space by simply jumping positions. Motion effect is produced like a
movie projector gives the impression of motion. So, for animation, this
approach can and often does suffice. However, objects in the world don’t
jump positions. Momentum, inertia and limits on acceleration and
velocity do play a significant role. To move a ground robot, the
position should be controlled by velocity. In reality the position is
controlled by control signals to a motor which in turn generates a
velocity, but we will assume we have a perfect motor controller for now;
one that can take a velocity command and achieve that velocity.

In this section we simulate the motion of the differential drive robot
that we introduced in the Terms Chapter shown in
:numref:`ddriveRecalled`.

.. _`ddriveRecalled`:
.. figure:: SimulationFigures/ddrive.*
   :width: 30%
   :align: center

   Simple differential drive robot.

and the associated
equations :eq:`ddkinematicsmodel`

.. math::
   :label: `ddkinematicsmodel`

   \boxed{
   \begin{array}{l}
    \dot{x} = \frac{r}{2} (\dot{\phi_1}+\dot{\phi_2})\cos(\theta) \\[5mm]
   \dot{y} = \frac{r}{2} (\dot{\phi_1}+\dot{\phi_2})\sin(\theta) \\[5mm]
   \dot{\theta} = \frac{r}{2L} (\dot{\phi_1}-\dot{\phi_2})
   \end{array}}

where :math:`\dot{\phi_1}` and :math:`\dot{\phi_2}` are the right and
left wheel rotational speeds (respectively), :math:`r` is wheel radius
and :math:`2L` is the axle length.

What can be said about these equations? Can these be partially solved so
we can run simulations? Our first attempt is to solve the differential
equations by integration. Starting with the third equation, the one for
the angular velocity,

.. math:: \dot{\theta} =\frac{d\theta}{dt} = \frac{r}{2L} (\dot{\phi_1}-\dot{\phi_2})

integrate from :math:`0` to :math:`t` (and be careful about integration
variables)

.. math:: \int_0^t\frac{d\theta}{d\tau}\, d\tau = \int_0^t\frac{r}{2L} (\dot{\phi_1}-\dot{\phi_2})\, d\tau

and we have

.. math:: \theta(t) = \theta(0) + \int_0^t \frac{r}{2L} \left(\frac{d\phi_1}{d\tau}-\frac{d\phi_2}{d\tau}\right)d\tau

Normally one can determine :math:`\dot{\phi_i}`, but it might not have a
standard functional form. These values are wheel velocities and do
correspond with the standard collection of calculus functions. For the
moment, assume that you know :math:`\phi_i(t)`, then what can you say?
From :math:`\dot{\phi}_i(t)` we can compute :math:`\theta` by
integrating the last equation. This will be used in the formulas for
:math:`x` and :math:`y`. Integrating the formulas for :math:`x` and
:math:`y`

.. math::

   \begin{array}{l}
    x(t)  = x(0)+\displaystyle\int_0^t \frac{r}{2} \left(\frac{d\phi_1}{d\tau}+\frac{d\phi_2}{d\tau}\right)\cos(\theta(\tau))d\tau \\[5mm]
   y(t)  = y(0) + \displaystyle\int_0^t\frac{r}{2} \left(\frac{d\phi_1}{d\tau}+\frac{d\phi_2}{d\tau}\right)\sin(\theta(\tau))d\tau
   \end{array}

These equations are easy to integrate if you know the wheel velocities
are constants. First integrate the :math:`\theta` equation:

.. math:: \theta(t) = (r/2L)(\omega_1 - \omega_2)t + \theta(0).

Theta can be plugged into the :math:`x` and :math:`y` equations and
integrated, under the assumption that :math:`\omega_1\neq \omega_2` or
:math:`\omega_1 \neq -\omega_2`:

.. math::

   x(t) = \frac{L(\omega_1 + \omega_2)}{(\omega_1 - \omega_2)} \left[ \sin((r/2L)(\omega_1 - \omega_2)t + \theta(0)) -
    \sin(\theta(0))\right]

.. math:: y(t) = -\frac{L(\omega_1 + \omega_2)}{(\omega_1 - \omega_2)} \left[ \cos((r/2L)(\omega_1 - \omega_2)t + \theta(0)) - \cos( \theta(0)) \right]

This solution is a sequence of circular arcs. For the special case where
:math:`\omega_1=\omega_2=\omega`, we have that :math:`d\theta / dt = 0`,
so,

.. math::

   \begin{array}{l}
    x = r\omega\cos(\theta_0)t + x_0\\[2mm]
    y = r\omega\sin(\theta_0)t + y_0\\[2mm]
   \theta = \theta_0 .
   \end{array}

And when :math:`\omega_1 = -\omega_2 = \omega`, we have :math:`dx/dt=0`
and :math:`dy/dt=0`, so

.. math::

   \begin{array}{l}
    x = x_0\\[2mm]
    y = y_0\\[2mm]
   \theta = \displaystyle \frac{r\omega}{L} t + \omega_0 .
   \end{array}

As long as you have piecewise constant angular velocities on the wheels,
you have the robot path made up from circular arcs. A simulation program
can connect these up to produce a path for any sequence of wheel
velocities. The path is made up of combinations of lines and arcs. Note
that a pivot in place is possible so the resulting path need not be
differentiable.
:numref:`fig:piecewisecirculararcs`
shows a sample path.

.. _`fig:piecewisecirculararcs`:
.. figure:: SimulationFigures/piecewisecircular.*
   :width: 50%
   :align: center

   Piecewise circular/linear arc paths

In practice it is not possible to instantaneously jump wheel speeds.
Inertia in the system (mass, inductance, power limits) means that it is
not possible to instantaneous jumps in velocity. In addition, it is not
possible to have perfect velocities when surfaces and power are not
consistent. So what if we relax the constant velocity assumption. This
gives rise to two additional issues. The first is that you may not be
able to gain an antiderivative of the wheel velocities to find
:math:`\theta(t)`. If you are able to find :math:`\theta`, the right
hand sides for :math:`\dot{x}` and :math:`\dot{y}` normally are not
integrable. A simple example below demonstrates issues with finding
antiderivatives.

Let :math:`\dot{\phi_1} = e^{-t^2}` and
:math:`\dot{\phi_2} = t`

.. math:: \theta(t) = \theta(0) + \int_0^t \frac{r}{2L} \left(e^{-\tau^2}-\tau\right)d\tau = ???
   :label: `ddexamplenotworkable`


This integral cannot be resolved. Meaning we cannot find an analytic
antiderivative. It is possible to approximate it either with a Taylor
expansion or numerical formulation, but it is an example of a vast
number of functions which we must stop at this step.

There is another problem that this example indicates. In general,
looking for an analytic function for the position is not possible.
Practically you don’t actually have a function representation of
:math:`\phi(t)` and are normally measuring the wheel angular velocity
during runtime? How should we formulate and proceed in that case.

A numerical approach
~~~~~~~~~~~~~~~~~~~~

We will use Euler’s (“Oil-ler’s”) method for solving the differential
equations. Euler’s method approximates the derivatives with a forward
finite difference and converts the differential equation into a
difference equation. The difference equations are algebraic and can be
evaluated numerically. This is also known as a finite difference method.
Let the time between measurements be denoted by :math:`\Delta t`. We
discretize (or approximate) the time variable and the three state
variables using discrete variables. This simple means we have a sequence
of numbers :math:`\{x_k\}` instead of a function :math:`x(t)`.
Technically we should use a different variable, but I will often be
efficient [#f1]_ and reuse the variable even though one denotes a function
of time and one denotes a sequence.

.. math:: t_k \equiv k\Delta t, \quad t_{k+1} = (k+1)\Delta t

.. math:: x_k \equiv x(t_k), ~~~ y_k \equiv y(t_k)

.. math::

   \omega_{1, k}\equiv \dot{\phi}_{1}(t_k), ~~~
   \omega_{2, k}\equiv \dot{\phi}_{2}(t_k)

Recall that if :math:`x` is position then :math:`\dot{x}=dx/dt` is
velocity (and :math:`\ddot{x}=d^2x/dt^2` is acceleration). From basic
calculus, we recall that we may approximate a derivative using a forward
finite difference:

.. math:: \dot{x} \approx \frac{x(t+\Delta t) - x(t)}{\Delta t}.

Using this we can take a time step of :math:`\Delta t` forward (meaning
:math:`t_{k+1} = t_k + \Delta t`) and Euler’s method gives us

.. math::

   x(t_{k+1}) = x(t_k) + (\Delta t)x'(t_k) \quad \mbox{and}
   \quad y(t_{k+1}) = y(t_k) + (\Delta t)y'(t_k).

And so we can write our differential equations as difference equations,

.. math::

   \begin{array}{l}
   \displaystyle \frac{x(t+\Delta t) - x(t)}{\Delta t}\approx \dot{x} = \frac{r}{2} (\dot{\phi_1}+\dot{\phi_2})\cos(\theta) \\[5mm]
   \displaystyle \frac{y(t+\Delta t) - y(t)}{\Delta t}\approx \dot{y} = \frac{r}{2} (\dot{\phi_1}+\dot{\phi_2})\sin(\theta) \\[5mm]
   \displaystyle \frac{\theta (t+\Delta t) - \theta (t)}{\Delta t}\approx \dot{\theta} = \frac{r}{2L} (\dot{\phi_1}-\dot{\phi_2})
   \end{array}

After some algebra, we obtain:

.. math::

   \begin{array}{l}
    x(t+\Delta t) \approx x(t) +\frac{r\Delta t}{2} (\dot{\phi_1}+\dot{\phi_2})\cos(\theta) \\[5mm]
    y(t+\Delta t) \approx y(t) +\frac{r\Delta t}{2} (\dot{\phi_1}+\dot{\phi_2})\sin(\theta) \\[5mm]
   \theta (t+\Delta t) \approx \theta (t) +\frac{r\Delta t}{2L} (\dot{\phi_1}-\dot{\phi_2}).
   \end{array}

Using the discrete (sample) variables, :math:`x(t_k) \to x_k`, etc, we
can rewrite the expression in terms of the discrete variables. Given
starting configuration and wheel velocity measurements, we have the
following difference equations:

.. math::
   :label: discreteDD

   \begin{array}{l}
    x_{k+1} = x_k + \frac{r\Delta t}{2} (\omega_{1, k}+\omega_{2, k})\cos(\theta_k) \\[5mm]
   y_{k+1} = y_k + \frac{r\Delta t}{2} (\omega_{1, k}+\omega_{2, k})\sin(\theta_k) \\[5mm]
   \theta_{k+1} = \theta_k + \frac{r\Delta t}{2L} (\omega_{1, k}-\omega_{2, k})
   \end{array}

These equations are the main model for approximating motion of a
differential drive robot. It has also been used as a first approximation
for a tractor or tank drive system. This function is easily coded into
Python:

::

    def ddstep(xc, yc, qc,r,l,dt,w1,w2):
       xn = xc + (r*dt/2.0)*(w1+w2)*cos(qc)
       yn = yc + (r*dt/2.0)*(w1+w2)*sin(qc)
       qn = qc + (r*dt/(2.0*l))*(w1-w2)
       return (xn,yn,qn)

You will need to bring in the math functions:

::

    from math import *

Assume that :math:`r=1`, :math:`dt = 0.1`, :math:`w1=w2=2` and
:math:`l=6` and take the initial pose to be :math:`x=1`, :math:`y=2` and
:math:`\theta = q =0.7`. The following is a Python program to take 10
steps with the 0.1 time step:

::

    xc = 1; yc = 2; qc  = 0.7
    t = 0
    dt = 0.1
    for i in range(10):
       xc, yc, qc = ddstep(xc, yc, qc,1.0,6.0,dt,2.0,2.0)
       t = t + dt
       print t, xc, yc, qc

The output:

::

    0.1 1.15296843746 2.12884353745 0.7
    0.2 1.30593687491 2.2576870749 0.7
    0.3 1.45890531237 2.38653061234 0.7
    0.4 1.61187374983 2.51537414979 0.7
    0.5 1.76484218728 2.64421768724 0.7
    0.6 1.91781062474 2.77306122469 0.7
    0.7 2.0707790622 2.90190476213 0.7
    0.8 2.22374749966 3.03074829958 0.7
    0.9 2.37671593711 3.15959183703 0.7
    1.0 2.52968437457 3.28843537448 0.7

The Euler approximation amounts to assuming the vehicle has constant
wheel velocity over the interval :math:`\Delta t`, see
:numref:`fig:piecewiseconst`. The assumption
of piecewise constant velocity does not hold in the general case and so
we see accumulating drift when comparing the robot’s true path and the
approximated one.

.. _`fig:piecewiseconst`:
.. figure:: SimulationFigures/piecewiseconst.*
   :width: 50%
   :align: center

   Piecewise Constant nature of the Euler
   Approximation.

A simple modification of the code can accept other wheel speeds. For
example, if the wheel speeds are given by :math:`w1 = 0.1 + 2*t` and
:math:`w2 = 0.1`, we would have

::

    xc = 1; yc = 2; qc  = 0.7
    t = 0;  dt = 0.1
    for i in range(10):
       w1 = 0.1 + 2*t
       w2 = 0.1
       xc, yc, qc = ddstep(xc, yc, qc,1.0,6.0,dt,w1,w2)
       t = t + dt
       print t, xc, yc, qc

::

    0.1 1.00764842187 2.00644217687 0.7
    0.2 1.02294526562 2.01932653062 0.701666666667
    0.3 1.0458582885 2.03869127648 0.705
    0.4 1.07632275057 2.06461262966 0.71
    0.5 1.11424084437 2.09720431822 0.716666666667
    0.6 1.15948081421 2.13661681787 0.725
    0.7 1.21187577374 2.18303629886 0.735
    0.8 1.27122223402 2.23668327131 0.746666666667
    0.9 1.33727835762 2.29781091264 0.76
    1.0 1.40976195869 2.36670305715 0.775

You can plot the motion in Python. Another example with circular motion:

::

    import pylab as plt
    import numpy as np
    from math import *
    N=200
    x = np.zeros(N)
    y = np.zeros(N)
    q = np.zeros(N)
    x[0] = 1; y[0] = 2; q[0]  = 0.7
    t = 0;  dt = 0.1
    for i in range(N-1):
       w1 = 0.1
       w2 = 0.5
       x[i+1], y[i+1], q[i+1] = ddstep(x[i], y[i], q[i],1.0,6.0,dt,w1,w2)
       t = t + dt

    plt.plot(x,y,'b')
    plt.show()

Differential Drive Inverse Kinematics
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Recall the DD forward kinematics:

.. math::

   \begin{array}{l}
    \dot{x} = \frac{r}{2} (\dot{\phi_1}+\dot{\phi_2})\cos(\theta) \\[5mm]
   \dot{y} = \frac{r}{2} (\dot{\phi_1}+\dot{\phi_2})\sin(\theta) \\[5mm]
   \dot{\theta} = \frac{r}{2L} (\dot{\phi_1}-\dot{\phi_2})
   \end{array}

Starting with the velocity :math:`v = \sqrt{\dot{x}^2 + \dot{y}^2}`,
plug in the first two differential equations:

.. math:: v = \sqrt{\left(\frac{r}{2} (\dot{\phi_1}+\dot{\phi_2})\cos(\theta)\right)^2 + \left(\frac{r}{2} (\dot{\phi_1}+\dot{\phi_2})\sin(\theta)\right)^2}

.. math:: = \sqrt{\left(\frac{r}{2} (\dot{\phi_1}+\dot{\phi_2})\right)^2 \left(\cos^2(\theta) + \sin^2(\theta)\right)}

.. math:: = \frac{r}{2} |\dot{\phi_1}+\dot{\phi_2}|.

So, we finally have:

.. math:: |\dot{\phi_1}+\dot{\phi_2}| = \frac{2v}{r}.

Using the third differential equation,
:math:`\dot{\phi_1} = \dot{\phi_2} + \frac{2L\dot{\theta}}{r}`, we can
solve for :math:`\dot{\phi_2}`. We get,

.. math:: |\dot{\phi_2} + \frac{L\dot{\theta}}{r}| = \frac{v}{r}.

Solving for :math:`\dot{\phi_2}` and then plugging back in for
:math:`\dot{\phi_1}`, we have

.. math::

   \dot{\phi_1} =  \frac{L\dot{\theta}}{r} \pm \frac{v}{r}, \quad
   \dot{\phi_2} = -\frac{L\dot{\theta}}{r} \pm \frac{v}{r}

The direction of the robot is the direction of the curve shown in
:numref:`intro-tangent`.

.. _`intro-tangent`:
.. figure:: SimulationFigures/tantheta.*
   :width: 30%
   :align: center

   The relation between :math:`\theta` and :math:`\dot{x}`,
   :math:`\dot{y}`.

.. math:: \theta = \arctan \frac{\dot{y}}{\dot{x}}~.

Differentiation gives

.. math:: \dot{\theta} = \frac{\dot{x}\ddot{y} - \dot{y}\ddot{x}}{\dot{x}^2 + \dot{y}^2}

Plugging in we have

.. math::

   \begin{array}{l}
   \dot{\phi_1} = \displaystyle \frac{L}{r}\left( \frac{\dot{x}\ddot{y} - \dot{y}\ddot{x}}{\dot{x}^2 + \dot{y}^2}\right) \pm \frac{\sqrt{\dot{x}^2 + \dot{y}^2}}{r} \\[3mm]
   \dot{\phi_2} = \displaystyle -\frac{L}{r}\left(\frac{\dot{x}\ddot{y} - \dot{y}\ddot{x}}{\dot{x}^2 + \dot{y}^2}\right) \pm \frac{\sqrt{\dot{x}^2 + \dot{y}^2}}{r}
   \end{array}

Direction along the path is selected depending on the :math:`\pm`. We
will pick the positive root to be consistent with the front of the
robot.

.. math::
   :label: `inverseddequations`

   \boxed{
   \begin{array}{l}
   \dot{\phi_1} = \displaystyle \frac{L}{r}\left( \frac{\dot{x}\ddot{y} - \dot{y}\ddot{x}}{\dot{x}^2 + \dot{y}^2}\right) + \frac{\sqrt{\dot{x}^2 + \dot{y}^2}}{r} \\[3mm]
   \dot{\phi_2} = \displaystyle -\frac{L}{r}\left(\frac{\dot{x}\ddot{y} - \dot{y}\ddot{x}}{\dot{x}^2 + \dot{y}^2}\right) + \frac{\sqrt{\dot{x}^2 + \dot{y}^2}}{r}
   \end{array} }

Note that the curvature of a parameterized plane curve is given by

.. math::

   \kappa   = \frac{\dot{x}\ddot{y} - \dot{y}\ddot{x}}{(\dot{x}^2 + \dot{y}^2)^{3/2}}
   = \frac{\dot{x}\ddot{y} - \dot{y}\ddot{x}}{v(\dot{x}^2 + \dot{y}^2)} =  \frac{\dot{\theta}}{v}

and we can rewrite the inverse kinematic equations, IK, as

.. math::
   :label:  `inverseddequationskappa`

   \boxed{
   \begin{array}{l}
   v = \sqrt{\dot{x}^2 + \dot{y}^2}\\[3mm]
   \kappa =   \displaystyle  \frac{\dot{x}\ddot{y} - \dot{y}\ddot{x}}{v^3} = \frac{\dot{\theta}}{v}\\[3mm]
   \dot{\phi_1} = \displaystyle \frac{v}{r}\left(\kappa L + 1\right) \\[3mm]
   \dot{\phi_2} = \displaystyle \frac{v}{r}\left(-\kappa L + 1\right)
   \end{array}}

Find the wheel velocities for a robot moving in a circle of radius 20.
Assume that :math:`r=1` and :math:`L = 4` and using the following
parameterization:

.. math:: x = R\cos(t/R), \quad y = R\sin(t/R), \quad \mbox{where } t \in [0, 2\pi R]

and so for our example we have that

.. math:: x = 20\cos(t/20), \quad y = 20\sin(t/20), \quad \mbox{where } t \in [0, 40\pi].

First we must compute,
:math:`v = \sqrt{\dot{x}^2 + \dot{y}^2} =  \sqrt{\sin^2(x) + \cos^2(x)} =1`.
Next we compute :math:`\kappa`:

.. math::

   \kappa =  \dot{x}\ddot{y} - \dot{y}\ddot{x} =
   \frac{\sin^2(t/20)}{20} + \frac{\cos^2(t/20)}{20}  =  \frac{1}{20} .

This makes sense since we know the curvature is the reciprocal of the
radius. By selecting to go counter-clockwise (increasing :math:`\theta`)
we use “+" in :eq:`inverseddequations`. Plugging the
values into :eq:`inverseddequations`,
we obtain wheel velocities

.. math::

   \begin{array}{l}
   \dot{\phi_1} = 6/5 \\[3mm]
   \dot{\phi_2} = 4/5
   \end{array}

Assume that you want to follow the path

.. math:: x(t) = t^2, \quad y(t) = t

with a differential drive robot (leaving :math:`L` and :math:`r` as
variables). We must first compute the derivatives

.. math:: \dot{x} = 2t,\quad \ddot{x} = 2,\quad \dot{y} = 1,\quad \ddot{y} = 0

and then plug into the equations

.. math:: \kappa = \frac{(2t)(0) - (1)(2)}{\left((2t)^2 + (1)^2\right)^{3/2}} = -\frac{2}{\left(4t^2 + 1\right)^{3/2}}

.. math:: v = \sqrt{(2t)^2 + 1^2} = \sqrt{4t^2 + 1}

.. math:: \dot{\phi_1} =  \frac{v}{r}\left( \kappa L + 1\right) , \quad \dot{\phi_2} = \frac{v}{r}\left( - \kappa L  + 1\right).

::

    N=100
    t0 = 0.0
    t1 = 2.0
    t = np.linspace(t0,t1,N)
    dt = (t1-t0)/N
    one = np.ones((N))
    xp = np.zeros((N))
    yp = np.zeros((N))
    th = np.zeros((N))

    x = t*t
    y = t

    plt.figure()
    plt.plot(x,y,'g-')
    plt.legend(['Path'],loc='best')
    plt.title('Quadratic Path')
    plt.show()

Generate wheel speeds:

::

    doty=one
    dotx=2*t
    ddoty=0
    ddotx=2*one

    r = 1.0
    L = 4.0
    v = np.sqrt(dotx*dotx + doty*doty)
    kappa = (dotx*ddoty - doty*ddotx)/(v*v*v)
    dotphi1 = (v/r)*(kappa*L +1)
    dotphi2 = (v/r)*(-kappa*L+1)

    plt.plot(t,dotphi1,'b-', t,dotphi2,'g-')
    plt.title('Wheel Speeds')
    plt.legend(['Right', 'Left'],loc='best')
    plt.show()

And the section of code to check:

::

    xp[0] = 0.0
    yp[0] = 0.0
    th[0] = 1.5707963267949

    for i in range(N-1):
        xp[i+1] = xp[i] + (r*dt/2.0)*(dotphi1[i]+dotphi2[i])*math.cos(th[i])
        yp[i+1] = yp[i] + (r*dt/2.0)*(dotphi1[i]+dotphi2[i])*math.sin(th[i])
        th[i+1] = th[i] + (r*dt/(2.0*L))*(dotphi1[i]-dotphi2[i])

    plt.figure()
    plt.plot(x,y,'g-', xp, yp, 'bx')
    plt.legend(['Original Path', 'Robot Path'],loc='best')
    plt.title('Path')
    plt.show()


.. _`quadraticpathexample2`:
.. figure:: SimulationFigures/quadpolyphis.*
   :width: 60%
   :align: center

   The wheel velocities.


.. _`quadraticpathexample3`:
.. figure:: SimulationFigures/quadpoly1.*
   :width: 60%
   :align:  center

   Comparison of the path and driven path.

On a robot, the motor controllers will be taking digital commands which
means the wheel velocities are discrete. This implies that the robot has
fixed wheel velocities during the interval between velocity updates. We
know in the case of the differential drive robot, fixed wheel speeds
means the robot is driving a line or circle. Therefor the DD robot in
this case is following a connected path made up of line or circle
segments, see :numref:`fig:piecewiseconst`.
Even when we do have functional forms for the wheel speeds, the
implementation is still discrete.

It makes sense to treat this as a discrete formula and to write as such:

.. math::
   :label: ddikpartial

   \boxed{
   \begin{array}{l}
   v_k = \sqrt{\dot{x}(t_k)^2 + \dot{y}(t_k)^2} , \quad\quad
   \displaystyle  \kappa_k = \frac{\dot{x}(t_k) \ddot{y}(t_k ) -  \dot{y}(t_k) \ddot{x}(t_k)}{v_k^3}, \\[3mm]
   \displaystyle  \omega_{1,k} = \frac{v_k}{r}(\kappa_k L + 1), \quad\quad
   \displaystyle  \omega_{2,k} = \frac{v_k}{r}(-\kappa_k L + 1)
   \end{array} }

Determine the wheel velocities to drive through the way points (0,1),
(1,2), (2,5). First we compute the derivatives

.. math:: \dot{x} = 1,\quad \ddot{x} = 0,\quad \dot{y} = 2t,\quad \ddot{y} = 2

and then plug into the equations

.. math:: \kappa = \frac{(1)(2) - (2t)(0)}{\left(1 + 4t^2\right)^{3/2}} = \frac{2}{\left(1 + 4t^2\right)^{3/2}} ,

.. math:: \dot{\phi_1} =  \frac{v}{r}\left( \kappa L + 1\right) , \quad \dot{\phi_2} = \frac{v}{r}\left( - \kappa L  + 1\right).

Limitations
~~~~~~~~~~~

In the previous sections we have shown how to drive a robot along any
path that the kinematics admits. In the mathematical examples, there are
no problems with following a precomputed path. However, this is an
example of open loop control and it suffers from many types of error
such as discretization error, non-uniform components, variations in
power, signals and an unpredictable environment. The robot will drift
from the intended path. This drift grows over time.

In practice, we will normally not compute the analytic path from which
to compute the derivatives and such to plug into the inverse kinematics.
We will use more traditional control algorithms to direct the robot such
as a PID controller. We may have a path to follow, but we will not plug
that path into the inverse kinematics. Instead we will extract samples
from the path and feed destination points into the control algorithm.
This does not mean that our efforts working out the inverse kinematics
was wasted. Very much to the contrary. We will still use the IK formulas
in our controllers. Understanding the IK will help in the controller
design. The IK can often help isolate aspects of the system dynamics
which eases controller development or makes it possible to gain a stable
controller.

.. rubric:: Footnotes

.. [#f1] that would be a *codeword* for sloppy
