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


Our work last chapter has given us

.. math:: \theta(t) = \theta(0) + \int_0^t \frac{r}{2L} \left(\frac{d\phi_1}{d\tau}-\frac{d\phi_2}{d\tau}\right)d\tau

If you have the mathematical form to determine :math:`\dot{\phi_i}`, you can proceed like in the last chapter.  But it might not have a
standard functional form.    In general,
looking for an analytic function for the position is not possible.
Practically you don’t actually have a function representation of
:math:`\phi(t)`.  We are normally sampling the wheel angular velocity
during runtime. How should we formulate and proceed in that case?



So what if we relax the constant velocity assumption. This
gives rise to two additional issues. The first is that you may not be
able to gain an antiderivative of the wheel velocities to find
:math:`\theta(t)`. If you are able to find :math:`\theta`, the right
hand sides for :math:`\dot{x}` and :math:`\dot{y}` normally are not
integrable. A simple example below demonstrates issues with finding
antiderivatives.


Let :math:`\dot{\phi_1} = e^{-t^2}` and
:math:`\dot{\phi_2} = t`

.. math:: \theta(t) = \theta(0) + \int_0^t \frac{r}{2L} \left(e^{-\tau^2}-\tau\right)d\tau = ???
   :label: ddexamplenotworkable

This integral cannot be resolved. Meaning we cannot find an analytic
antiderivative. It is possible to approximate it either with a Taylor
expansion or numerical formulation, but it is an example of a vast
number of functions which we must stop at this step.

In practice it is not possible to instantaneously jump wheel speeds as we did in our analytical approach previously.  
Inertia in the system (mass, inductance, power limits) means that it is
not possible to instantaneous jumps in velocity. In addition, it is not
possible to have perfect velocities when surfaces and power are not
consistent. These errors in wheel velocites can over time
translate to significant errors in position. 


A numerical approach
~~~~~~~~~~~~~~~~~~~~

We will use Euler’s (*Oil-ler’s*) method for solving the differential
equations. :index:`Euler’s method` approximates the derivatives with a forward
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
:index:`finite difference`:

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

    
    
.. rubric:: Footnotes

.. [#f1] that would be a *codeword* for sloppy


