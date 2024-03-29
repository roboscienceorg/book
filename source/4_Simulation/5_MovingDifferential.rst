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


In practice you want to prescribe
:math:`x(t), y(t)` to  obtain :math:`\dot{\phi_1},\dot{\phi_2}`. Clearly
if you have :math:`x(t), y(t)`, differentiation will yield
:math:`\dot{x}(t), \dot{y}(t)`, so we may assume that we know
:math:`\dot{x}(t), \dot{y}(t)`. Using :math:`\dot{x}` and
:math:`\dot{y}` we may drive the robot along the curve of interest.
We will explore methods to do this below.


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
~~~~~~~~~~~~~~~~~~~~~~~~

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
for a tractor or tank drive system.


.. code-block:: julia
   :caption: Differential Drive single time step function
   :name: listDDstep
   :dedent: 1

    function ddstep(xc, yc, qc,r,l,dt,w1,w2)
       xn = xc + (r*dt/2.0)*(w1+w2)*cos(qc)
       yn = yc + (r*dt/2.0)*(w1+w2)*sin(qc)
       qn = qc + (r*dt/(2.0*l))*(w1-w2)
       return (xn,yn,qn)
    end



Assume that :math:`r=1`, :math:`dt = 0.1`, :math:`w1=w2=2` and
:math:`l=6` and take the initial pose to be :math:`x=1`, :math:`y=2` and
:math:`\theta = q =0.7`. The following is a Python program to take 10
steps with the 0.1 time step:

.. code-block:: julia
   :caption: Differential Drive wrapper to call single step function
   :name: listDDstepwrapper
   :dedent: 1

    xc = 1; yc = 2; qc  = 0.7
    t = 0;  dt = 0.1
    for i= 1:10
       global xc, yc, qc, t, dt
       xc, yc, qc = ddstep(xc, yc, qc,1.0,6.0,dt,2.0,2.0)
       t = t + dt
       println(round(t,digits=3), " ", round(xc,digits=5), " ", round(yc,digits=5), " ",round(qc,digits=5))
    end


The output:

::

   0.1 1.15297 2.12884 0.7
   0.2 1.30594 2.25769 0.7
   0.3 1.45891 2.38653 0.7
   0.4 1.61187 2.51537 0.7
   0.5 1.76484 2.64422 0.7
   0.6 1.91781 2.77306 0.7
   0.7 2.07078 2.9019 0.7
   0.8 2.22375 3.03075 0.7
   0.9 2.37672 3.15959 0.7
   1.0 2.52968 3.28844 0.7


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

.. code-block:: julia
   :caption: Differential Drive variable wheel speeds
   :name: listDDvarwheel
   :dedent: 1

    xc = 1; yc = 2; qc  = 0.7
    t = 0;  dt = 0.1
    for i = 1:10
        global t, dt, xc, yc, qc
        w1 = 0.1 + 2*t
        w2 = 0.1
        xc, yc, qc = ddstep(xc, yc, qc,1.0,6.0,dt,w1,w2)
        t +=  dt
        println(round(t,digits=3), " ", round(xc,digits=5), " ", round(yc,digits=5), " ",round(qc,digits=5))
    end

The output:

::

    0.1 1.00765 2.00644 0.7
    0.2 1.02295 2.01933 0.70167
    0.3 1.04586 2.03869 0.705
    0.4 1.07632 2.06461 0.71
    0.5 1.11424 2.0972 0.71667
    0.6 1.15948 2.13662 0.725
    0.7 1.21188 2.18304 0.735
    0.8 1.27122 2.23668 0.74667
    0.9 1.33728 2.29781 0.76
    1.0 1.40976 2.3667 0.775


You can plot the motion in Python. Another example with circular motion:

.. code-block:: julia
   :caption: Differential Drive Plot
   :name: listDDplot
   :dedent: 1


    N = 200
    x = zeros(N)
    y = zeros(N)
    q = zeros(N)

    x[1] = 1.0; y[1] = 2.0; q[1]  = 0.7
    t = 0;  dt = 0.1
    for i = 1:N-1
        global t, dt, x, y, q
        w1 = 0.1
        w2 = 0.5
        x[i+1], y[i+1], q[i+1] = ddstep(x[i], y[i], q[i],1.0,6.0,dt,w1,w2)
        t +=  dt
    end

    display(plot(x,y))
    readline()




.. code-block:: julia
   :caption: Plots example
   :name: listPlotExample
   :dedent: 1


    using Plots
    N=100
    t0 = 0.0
    t1 = 2.0
    t = LinRange(t0,t1,N)
    dt = (t1-t0)/N
    one = ones((N))
    xp = zeros((N))
    yp = zeros((N))
    th = zeros((N))

    x = t .* t
    y = t

    p = plot(x,y, title = "Quadratic Path")
    display(p)
    readline()


Generate wheel speeds:


.. code-block:: julia
   :caption: Wheek speeds
   :name: listwheelspeed
   :dedent: 1


    using Plots
    N=100
    t0 = 0.0
    t1 = 2.0
    t = LinRange(t0,t1,N)
    one = ones((N))
    doty=one
    dotx=2*t
    ddoty=0
    ddotx=2*one

    r = 1.0
    L = 4.0
    v = sqrt.(dotx.*dotx .+ doty.*doty)
    kappa = (dotx.*ddoty .- doty.*ddotx)./(v.*v.*v)
    dotphi1 = (v./r).*(kappa*L .+1)
    dotphi2 = (v./r).*(-kappa*L.+1)

    p=plot(t,[dotphi1,dotphi2],title = "Wheel Speeds",label = ["Right" "Left"])
    display(p)
    readline()


And the section of code to check:

.. code-block:: julia
   :caption: Wheel speed check
   :name: listcheckwheelspeed
   :dedent: 1


    xp[1] = 0.0
    yp[1] = 0.0
    th[1] = 1.5707963267949

    for i=1:N-1
        xp[i+1] = xp[i] + (r*dt/2.0)*(dotphi1[i]+dotphi2[i])*cos(th[i])
        yp[i+1] = yp[i] + (r*dt/2.0)*(dotphi1[i]+dotphi2[i])*sin(th[i])
        th[i+1] = th[i] + (r*dt/(2.0*L))*(dotphi1[i]-dotphi2[i])
    end

    plot(x,y, label = "Original Path" )
    p = plot!(xp,yp,  title="Path", label = "Robot Path")
    display(p)
    readline()



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
(1,2), (2,5).  Recall that we found an interpolating polynomial in the Lagrange
Interpolation section.  This provided us with the following:

.. math:: x(t) = -t(t-2) + t(t-1)  = t,

.. math:: y(t) =  \frac{1}{2} (t-1)(t-2) - 2t(t-2) + \frac{5}{2} (t)(t-1)= t^2+1 .

Next we compute the derivatives

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
