Basic Controls
--------------

In the Simulation Chapter, we saw that a
parametric curve can be used to generate a path for a robot. Two issues
quickly arise. First, noise, errors in velocity and path discretization
will cause the vehicle to drift off of the computed path. The formulas
continue to drive the robot as if we are still on the correct
trajectory. This is known as *Open Loop* control. This means there is no
feedback from the sensors to correct for drift, discretization or path
error. The drift will accumulate over time and no matter how accurately
you compute the path, the robot will continue to stray from the path.
The only way to correct the drift is to have some type of feedback in
the control code.

The second issue is that a given parametric curve sets velocity as part
of the path. Later in this chapter we will separate the path from
velocity in the parametric functions, but it still leaves no room to
adapt to changes. Just as the position can drift, so can the velocity.
We need feedback for velocity as well. In addition, it is important to
be able to adapt to the environment, for example if we need to slow down
at some point to avoid a collision. So, feedback in our systems turns
out to be essential.

.. _`fig:pathdriftcorrection`:
.. figure:: ControlFigures/splinemiss.*
   :width: 40%
   :align: center

   Using position feedback as a way to mitigate
   drift.

As we discussed in the simulation chapter, the approach to minimize the
drift is to replace the parametric path with a feedback control system
based on sensing the environment. Thus if we missed hitting the target
point in the last segment, we don’t use next point in the precomputed
path, but have the controller provide a modified direction, see
:numref:`fig:pathdriftcorrection`. This
keeps the robot headed towards the desired path by using position
feedback. A simple stop, rotate to orient and drive approach will work
in very simple applications. How about an approach that corrects as we
drive? When we drive a car, we continually monitor our position on the
road and correct as needed. [#f1]_

Heading to Goal
~~~~~~~~~~~~~~~

After driving a few robots around on less than perfect terrain, it is
clear that driving a straight line can be hard. Assuming the goal is
actually driving to a goal and not as much staying on the line, how can
we correct during the route to keep our orientation towards the goal
point? Assume that you know your orientation error,
:numref:`fig:ddorientationerror`.

.. _`fig:ddorientationerror`:
.. figure:: ControlFigures/ddcontrol.*
   :width: 40%
   :align: center

   Orientation error of :math:`\alpha`.

Recall the alternate form of the differential drive equations:

.. math::

   \begin{array}{l}
   v = \frac{r}{2} (\dot{\phi_1}+\dot{\phi_2}) \\[3mm]
   \dot{\theta} = \frac{r}{2L} (\dot{\phi_1}-\dot{\phi_2}) .
   \end{array}

We are interested in finding wheel velocities such that :math:`v` is
constant, :math:`v_c` and :math:`\alpha \to 0`. So we start with wheel
velocities of the form

.. math::

   \begin{array}{l}
   \dot{\phi_1} = \frac{1}{r} (v_c+Lk\alpha)\\[3mm]
   \dot{\phi_2} = \frac{1}{r} (v_c-Lk\alpha)
   \end{array}

This gives us

.. math::

   \begin{array}{l}
   v = v_c \\[3mm]
   \dot{\theta} = k\alpha .
   \end{array}

Assume that the robot is relatively far from the goal with respect to
robot speed. If so, then :math:`\dot{\beta}` is small. Since
:math:`\beta = \omega + \alpha` it implies that
:math:`\dot{\alpha} \approx -\dot{\omega}` and we have

.. math:: \dot{\alpha} = -k\alpha.

The solution to this differential equation is

.. math:: \alpha(t) = \alpha(0) e^{-kt}

which has :math:`\alpha(t) \to 0` as :math:`t\to 0` if :math:`k>0`.
Large values of :math:`k` produce fast response times and small values
produce slow response. However, in implementation one works with a
discrete (digital) time controller. Large values can cause the robot to
sweep past the target and oscillate about the goal line. These
oscillations can even grow causing unstable motion. Normally we select
small values to start and observe how well it controls the robot.
Increasing :math:`k` as needed when the response is insufficient.
Selecting good values of :math:`k`, known as the gain, is discussed in
the PID control section.

Classical Controls
~~~~~~~~~~~~~~~~~~

Controls is a large interdisciplinary subject. Many engineers have
expressed the sentiment that robotics and controls are the same subject.
That view is up for debate, however everyone agrees that controls is a
very important aspect to robotics. In the next few sections, we briefly
touch on the topic. We begin with some terms found in the controls
literature.

-  Open Loop - device control without using sensor feedback.

-  Closed Loop - device control using sensor feedback.

-  Bang-Bang (On-off) Control - A control approach that turns the
   actuator or motor completely on or off without using proportional
   values.

-  P Control - Proportional control, using the error between the desired
   state and sensed state to control the device.

-  PD Control - Proportional-Derivative control, using the error between
   the desired state and sensed state and rate of change of the error to
   control the device.

-  PID Control - Proportional-Integral-Derivative control, using the
   error, rate of change of the error and a history of the error to
   control the device.

PID is one of the most popular control approaches in industry. It has
wide application due to ease of use and reasonable effectiveness.
Related to PID are PD, PI and P which are just versions of PID where
selected terms are set to zero in the formulas. From holding a
temperature in your house or oven, to cruise control on your car, to
managing flow rates in industrial plants to many more applications,
control systems are an essential aspect to an engineered solution. We
will be using these algorithms to set position, velocity and force in
our robots.

.. _`Fig:motorfeedback`:
.. figure:: ControlFigures/feedback.*
   :width: 85%
   :align: center

   Feedback and Control for motor speed.

:numref:`Fig:motorfeedback` shows the basic
feedback loop for controlling the speed on a motor. We will assume this
encoder returns angular velocity (rpm) although in practice they return
a signal which needs to be translated to rpm. This can be easily done as
part of the controller block. The controller block is the part that
takes the two signals, desired speed and actual speed, and decides what
command to give to the motor driver. The motor driver will then power
the motor.

Very little must change in the diagram if we wish to build a servo or
manipulator control system. For a servo, the encoder returns absolute
angle information. If the motor is attached to a mechanical arm, then
the encoder is the device that returns the location of the end-effector.
Most of our discussion below will use motor speed as an example, but
these ideas apply to articulated systems as well.

We will be concerned with three quantities here. First, the desired
configuration, which in
:numref:`Fig:motorfeedback` is :math:`v_d`.
Second, the measured or actual configuration, :math:`v_m`. And third,
the control signal delivered to the actuator control unit, :math:`u(t)`.

Bang-Bang Control
~~~~~~~~~~~~~~~~~

Bang-Bang control or On-Off control attempts move the system to the
desired configuration by using a series of on/off signals:

.. math:: u(t) = \left\{ \begin{array}{lr} u_c & \mbox{~if~} v_m(t) < v_d \\  0 & \mbox{~if~} v_m(t)\geq v_d \end{array} \right.

where :math:`u_c` is a constant. When :math:`v_m < v_d` a constant
power is applied to the motor. Since the system is digital, the signal
is sampled every :math:`\Delta t` seconds. There are also delays in the
time required to run the control algorithm. So, the power will normally
be applied past the point where the motor speed exceeds the desired
speed.

In this system, a fixed control effort is used and no attempt at scaling
it based on measured speeds is done. The obvious result is an
oscillation of the actual speed around the set speed,
:numref:`Fig:bangbang`. This approach is fine for
systems with slow dynamics (significant inertial) where one only wants
to be close to the set value; such as house temperature. For higher
response systems, this approach can feel rough as it jumps from off to
on and back. It can also become unstable with very rapid response times.

.. _`Fig:bangbang`:
.. figure:: ControlFigures/bangbang.*
   :width: 40%
   :align: center

   Bang-Bang or On-Off Control.

We can smooth this out a bit by placing a range for turning on and off
the motor. Assume that :math:`v_{\mbox{on}} < v_d < v_{\mbox{off}}` then

.. math:: u(t) = \left\{ \begin{array}{lr} u_c & \mbox{~if~} v_m(t) \leq v_{\mbox{on}}  \\  0 & \mbox{~if~} v_m(t)\geq v_{\mbox{off}} \end{array} \right.   .

This is not a good velocity or position control system for the robot.
We clearly want to take into account how far off the set value we are
and adjust our control effort. Meaning we want to base our control
effort on the amount of error. This is the approach used with the family
of P, PD and PID controllers described below.

Proportional Control
~~~~~~~~~~~~~~~~~~~~

The idea of proportional control is to set the control effort
proportionally to the error, :math:`e(t) = v_d - v_m`:

.. math:: u(t) = K_P e(t) .

This is an intuitive thing to try and in some applications works well.
A proportional control is mathematically more complicated that the Bang
Bang control discussed above. It overcomes some of the issues in on-off
controllers since they can continuously vary their output. The constant
:math:`K_P` is known as a gain, in the case the proportional gain. A
high proportional gain results in a large change in the output for a
given change in the error. If the proportional gain is too high, the
system can become unstable, oscillation (like we saw with the On-Off
controller) can occur. In contrast, a small gain results in a small
output response to a large input error, and a less responsive or less
sensitive controller.

Example of how to use a p-controller to drive a robot along a path
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In this example, we extend the previous example of holding a fixed
heading. Assume that you have two points :math:`t_0: (x_0,y_0)` and
:math:`t_1: (x_1, y_1)`. If the robot is at the first point with an
unknown orientation, how does one drive the robot to the second point in
a smooth motion? We can define two error terms

.. math:: e_1(t) = \sqrt{(x_1 - x_m)^2 +  (y_1 - y_m)^2},

.. math:: e_2(t) = \mbox{atan2}((y_1 - y_m), (x_1 - x_m)) - \theta_m .

:math:`e_1` measures the difference between current location and goal
location, and :math:`e_2` measures the difference between current
vehicle direction and the direction to the goal. We can try a
proportional control on the orientation of the vehicle by proportionally
adjusting the differences in the wheel velocities. Let :math:`v` be the
base speed. Then select the speed for the wheels:

.. math:: \dot{\phi}_1 = v + k_1 e_2(t), \quad \dot{\phi}_2 = v - k_1 e_2(t).

The robot can move at a fixed speed until it gets close to goal and
then can ramp down the speed by using the :math:`e_1` error value. If
:math:`e_1(t) < d` then

.. math:: \dot{\phi}_1 = k_2e_1(t)(v + k_1 e_2(t)) , \quad \dot{\phi}_1 = k_2e_1(t)(v - k_1 e_2(t)).

The value :math:`k_2` can be selected so the speed is continuous across
the :math:`e_1(t) < d` jump. Selecting some arbitrary values,
:math:`r=20`, :math:`L=12`, :math:`\Delta t =0.01`, :math:`k_1=2.0` and
:math:`k_2=0.2`. :math:`k_2` is selected for continuity on wheel speed.
:math:`k_1` was derived experimentally. The start point is (0,0) and
endpoint is (40,60). The result is given in
:numref:`Fig:pcontrolDDbot`.

.. _`Fig:pcontrolDDbot`:
.. figure:: ControlFigures/pcontrolDDbot.*
   :width: 40%
   :align: center

   A noise free example.

.. figure:: ControlFigures/noisepcontrolDDbot.*
   :width: 40%
   :align: center

   Adding noise to the wheels.

::

    r = 20.0
    l = 12.0
    dt  = 0.01
    Tend = 6.0
    N = int(Tend/dt)

    xend = 40
    yend = 60
    v = 1.0
    k1 = 2.0
    k2 = 0.2

    x = np.zeros(N)
    y = np.zeros(N)
    th = np.zeros(N)

    i= 0
    while(i<N-1):
        th_err = atan2(yend - y[i], xend - x[i]) - th[i]
        d1 = abs(x[i] - xend)
        d2 = abs(y[i] - yend)
        w = v
        d = sqrt(d1*d1+d2*d2)
        if (d<0.5):
            break
        if (d > 100):  break
        w1 = w + k1*th_err
        w2 = w - k1*th_err
        if (d<5):
            w1, w2 = k2*d*(w + k1*th_err), k2*d*(w - k1*th_err)
        dx = (r*dt/2.0)*(w1+w2)*cos(th[i])
        dy = (r*dt/2.0)*(w1+w2)*sin(th[i])
        dth = (r*dt/(2.0*l))*(w1-w2)
        x[i+1] = x[i] + dx
        y[i+1] = y[i] + dy
        th[i+1] = th[i] + dth
        i = i+1
    
    
    
.. code-block:: julia



using PyCall
    np = pyimport("numpy")

    r = 20.0
    l = 12.0
    dt  = 0.01
    Tend = 6.0
    N = Int64(Tend/dt)


    xend = 40
    yend = 60
    v = 1.0
    k1 = 2.0
    k2 = 0.2

    x = np.zeros(N)
    y = np.zeros(N)
    th = np.zeros(N)

    i = 1
    while(i<N)
        th_err = atan(yend - y[i], xend - x[i]) - th[i]
        d1 = abs(x[i] - xend)
        d2 = abs(y[i] - yend)
        w = v
        d = sqrt(d1*d1+d2*d2)
        if (d<0.5)
            break
        end
        if (d > 100):  break
        end
        w1 = w + k1*th_err
        w2 = w - k1*th_err
        if (d<5)
            w1, w2 = k2*d*(w + k1*th_err), k2*d*(w - k1*th_err)
        dx = (r*dt/2.0)*(w1+w2)*cos(th[i])
        dy = (r*dt/2.0)*(w1+w2)*sin(th[i])
        dth = (r*dt/(2.0*l))*(w1-w2)
        x[i+1] = x[i] + dx
        y[i+1] = y[i] + dy
        th[i+1] = th[i] + dth
        i = i+1
        end
    end

A simple modification can take a sequence of points and navigate the
robot along the path of points. Place the goal points into an array. Set
the counter to the first array index. When the robot is within a small
distance of the goal point, increment the counter. The controller will
adjust. :numref:`Fig:pcontrolDDbotpath`
demonstrates this algorithm.

.. _`Fig:pcontrolDDbotpath`:
.. figure:: ControlFigures/pcontrolDDbotpath.*
   :width: 40%
   :align: center

   Starting direction :math:`\theta =0`.

.. figure:: ControlFigures/pcontrolDDbotpath2.*
   :width: 40%
   :align: center

   Starting direction :math:`\theta = \pi /2`.

There are two main issues reported with proportional control. The first
is oscillation which can be produced by setting the gain to large. The
second is persistent offset error. This is a constant difference between
the desired value (set point) and the measured value. In some systems,
turning the gain down to avoid oscillations produces higher offset
error. Turing the gain up to remove the offset error introduces or
increases oscillations. These systems may not have a "sweet spot" or
interval of values for which neither issue is presented. So we need
additional machinery to correctly control the system.

PID Control Overview
~~~~~~~~~~~~~~~~~~~~

To address the oscillations, overshoot and instability, we use a more
robust control term, :math:`u(t)`, that includes the error, the change
in the error and the error history:

.. math:: u(t) = k_P  e(t)  + k_D \frac{de(t)}{dt}  + k_I \int_0^t e(\tau)d\tau

-  :math:`e(t)` - Error :math:`=v_{des}(t) - v_{act}(t)`

-  :math:`k_P` - Proportional gain

-  :math:`k_I` - Integral gain

-  :math:`k_D` - Derivative gain

PID - Proportional Term
^^^^^^^^^^^^^^^^^^^^^^^

Within the PID control, the proportional control contributes in the same
manner as it does alone.

..  NOT public domain

.. figure:: ControlFigures/Change_with_Kp.png
   :width: 40%
   :align: center

   Proportional control.


PID - Integral Term
^^^^^^^^^^^^^^^^^^^

The contribution from the integral term is proportional to both the
magnitude of the error and the duration of the error. The integral in a
PID controller is the sum of the instantaneous error over time and gives
the accumulated offset that should have been corrected previously. The
accumulated error is then multiplied by the integral gain (:math:`k_i`)
and added to the controller output.

The integral term accelerates the movement of the process towards
setpoint and eliminates the residual steady-state error that occurs with
a pure proportional controller. However, since the integral term
responds to accumulated errors from the past, it can cause the present
value to overshoot the setpoint value. This is known as integrator
windup.


.. figure:: ControlFigures/Change_with_Ki.png
   :width: 40%
   :align: center

   Proportional-Integral control.


PID - Derivative Term
^^^^^^^^^^^^^^^^^^^^^

The derivative of the process error is calculated by determining the
slope of the error over time and multiplying this rate of change by the
derivative gain :math:`k_d`. The magnitude of the contribution of the
derivative term to the overall control action is termed the derivative
gain, :math:`k_d`. The derivative term slows the rate of change of the
controller output. Derivative control is used to reduce the magnitude of
the overshoot produced by the integral component and improve the
combined controller-process stability. However, the derivative term
slows the transient response of the controller.

Also, differentiation of a signal amplifies noise and thus this term in
the controller is highly sensitive to noise in the error term, and can
cause a process to become unstable if the noise and the derivative gain
are sufficiently large.


.. figure:: ControlFigures/Change_with_Kd.png
   :width: 40%
   :align: center

   Proportional-Integral-Derivative control.


PI Control Discretization
^^^^^^^^^^^^^^^^^^^^^^^^^

Set :math:`K_D=0`

.. math:: u(t) = k_P  e(t) + k_I \int_0^t e(\tau)d\tau

Use of the controllers in a computer requires discretization. Let
:math:`t_n` be the discrete times, :math:`\Delta t` the time step,
:math:`e_n = e(t_n)`, and :math:`U_n = u(t_n)`. The discrete form can be
converted to a basic recursion:

.. math:: U_n = k_P e_n + k_I \Delta t \sum_{i=1}^n \frac{e_i + e_{i-1}}{2}

.. math:: U_n - U_{n-1} = k_P(e_n - e_{n-1}) + k_I \Delta t \left( \frac{e_n + e_{n-1}}{2}\right)

.. math::
   :label: eq:PIdiscreteformula

   U_n = U_{n-1} + K_P(e_n - e_{n-1}) + K_I (e_n + e_{n-1})

where :math:`K_P = k_p`, :math:`K_I = k_I \Delta t  / 2`.

PD Control Discretization
^^^^^^^^^^^^^^^^^^^^^^^^^

.. math:: u(t) = k_P e(t) + k_D \frac{de(t)}{dt}

The expression can be converted into a recursive relation:

.. math:: U_n  = k_P e_n + k_D \frac{e_n - e_{n-1}}{\Delta t}

.. math:: U_n - U_{n-1} = k_P(e_n - e_{n-1}) + k_D \frac{e_n - 2e_{n-1} +e_{n-2}}{\Delta t}

.. math::
   :label: eq:PDdiscreteformula

   U_n = U_{n-1} + K_P(e_n - e_{n-1}) + K_D (e_n - 2e_{n-1} +e_{n-2})

where :math:`K_P = k_p`, :math:`K_D = k_D / \Delta t`.

PID Control Discretization
^^^^^^^^^^^^^^^^^^^^^^^^^^

.. math:: u(t) = k_P e(t) + k_I \int_0^t e(\tau)d\tau + k_D \frac{de(t)}{dt}

In a similar fashion as above, the expression can be converted into a
recursive relation:

.. math:: U_n  = k_P e_n + k_I \Delta t \sum_{i=1}^n \frac{e_i + e_{i-1}}{2}  + k_D \frac{e_n - e_{n-1}}{\Delta t}

.. math:: U_n - U_{n-1} = k_P(e_n - e_{n-1}) + k_I \Delta t \left( \frac{e_n + e_{n-1}}{2}\right)  + k_D \frac{e_n - 2e_{n-1} +e_{n-2}}{\Delta t}

.. math::
   :label: eq:PIDdiscreteformula

   U_n = U_{n-1} + K_P(e_n - e_{n-1}) + K_I (e_n + e_{n-1}) + K_D (e_n - 2e_{n-1} +e_{n-2})

where :math:`K_P = k_p`, :math:`K_I = k_I \Delta t  / 2`,
:math:`K_D = k_D / \Delta t`.

PID Application
~~~~~~~~~~~~~~~

For this example, we want to control our differential drive robot to
follow a path. Assume the path is given by a list of close points
:math:`(x_n, y_n)` for :math:`0 \leq n \leq N`. As before we are
interested in finding wheel velocities such that :math:`v` is constant,
:math:`v_c`, and we are driving from point to point. Recall the
differential drive equations:

.. math::

   \begin{array}{l}
   v = \frac{r}{2} (\dot{\phi_1}+\dot{\phi_2}) \\[3mm]
   \dot{\theta} = \frac{r}{2L} (\dot{\phi_1}-\dot{\phi_2}) .
   \end{array}

We will use a PID control approach to control the wheel velocities so
we can closely track the path defined by the points. Using the same
structure we assume wheel velocities of the form

.. math::

   \begin{array}{l}
   \dot{\phi_1} = \frac{1}{r} (v_c+Lu(t))\\[3mm]
   \dot{\phi_2} = \frac{1}{r} (v_c-Lu(t))
   \end{array}

where :math:`u(t)` is obtained from a PID control strategy. In practice
on a computer this would use the discrete form of the PID control:

.. math::

   \begin{array}{l}
   \omega_{1,n} = \frac{1}{r} (v_c+LU_n)\\[3mm]
   \omega_{2,n} = \frac{1}{r} (v_c-LU_n)
   \end{array}

Since :math:`U_n` is a function of the error :math:`e_n`, we need to
figure out what we want for error. The idea is to head to :math:`n`-th
point in the list until we are within some neighborhood of the point,
then we increment the counter :math:`n` meaning target the following
point. This requires control on the heading like the example that
started our controls journey, but we cannot assume the points are far
away, so we need to track position and angles.

Assuming you have your current location as :math:`(x,y)` and target
location :math:`(x_n,y_n)`, the required heading would be
:math:`\left\langle x_n -x, y_n - y \right\rangle`. If you have a
compass on the robot, then you know the current heading. The heading
error we used before is

.. math:: e_n =  \mbox{atan2}( x_n - x , y_n - y) - \theta_n .

\ What if you don’t have a compass? The current heading can be estimated
by :math:`\left\langle x-x_{n-1} , y - y_{n-1}\right\rangle` which is
just an estimate of the derivative. And we have

.. math:: e_n =  \mbox{atan2}( x_n - x , y_n - y) - \mbox{atan2}( x-x_{n-1} , y - y_{n-1}) .

Up to this point we have computed the heading error without discussion.
Specifically the approach has been to take the current heading vector
and compute the heading angle via atan2, then subtract from the desired
heading (which might have also been computed via atan2). This gives
formulas that look like atan2(vy,vx) - atan2(uy,ux).

Example:
^^^^^^^^

-  | For ux = 0.8, uy = 0.5 and vx = 1.1, vy = -.2
   | then atan2(vy,vx) - atan2(uy,ux) :math:`\approx` -0.73845 .

-  | For ux = -0.8, uy = 0.5 and vx = -1.1, vy = -.2
   | then atan2(vy,vx) - atan2(uy,ux) :math:`\approx` -5.544732 .

If you graph these, the pair of vectors are reflections about the y-axis
and so should give the same result. What is the issue? The sum of those
two appears to be :math:`-2\pi`. Not surprisingly the problem lies in
the ambiguity of which angle is desired. From calculus we know that the
angle between two vectors can be determined by the dot product.

.. math:: u \cdot v = \cos(\theta) \| u\| \|v\|  \quad \rightarrow\quad  \theta = \cos^{-1} \left(\frac{u \cdot v }{ \| u\| \|v\|}\right) .

It is worthwhile to write a function that correctly determines the
signed angle between vectors. Using the cross product:

::

    def angle(u1, u2, v1, v2):
       n1 = math.sqrt(u1*u1+u2*u2)
       n2 = math.sqrt(v1*v1+v2*v2)
       dot = u1*v1+u2*v2
       cross = u1*v2 - v1*u2
       if cross == 0.0:  return 0.0
       if cross > 0:  sign = 1
       if cross < 0:  sign =-1
       theta = sign*math.acos(dot/(n1*n2))
       return theta
 
 
 
.. code-block:: julia



using PyCall
math = pyimport("math")

function angle(u1, u2, v1, v2)
       n1 = math.sqrt(u1*u1+u2*u2)
       n2 = math.sqrt(v1*v1+v2*v2)
       dot = u1*v1+u2*v2
       cross = u1*v2 - v1*u2
       if cross == 0.0
        return 0.0
       end
       if cross > 0
        sign = 1
        end
       if cross < 0
        sign =-1
       end
       theta = sign*math.acos(dot/(n1*n2))
       return theta
end
Returning to the control problem we have:

.. math::

   \begin{array}{l}
   e_n=  \mbox{angle}( \cos(\theta_n), \sin(\theta_n), x_n - x , y_n - y )\\[3mm]
   U_n = U_{n-1} + K_P(e_n - e_{n-1}) + K_I (e_n + e_{n-1}) + K_D (e_n - 2e_{n-1} +e_{n-2})\\[3mm]
   \omega_{1,n} = \frac{1}{r} (v_c+LU_n)\\[3mm]
   \omega_{2,n} = \frac{1}{r} (v_c-LU_n)
   \end{array}

The last aspect is to work out the transition to the next point. Assume
you want to get within :math:`\delta` of the point. This means that you
want to drive to :math:`(x_n, y_n)` until

.. math:: d = \sqrt{(x-x_n)^2 + (y - y_n)^2} < \delta

then increment :math:`n` to switch to the new point. If you are using
the non-compass formula, you will want save your location when you
switch. Using the saved location instead of :math:`(x_{n-1}, y_{n-1})`
will give you better heading accuracy.

PID Parameter Tuning
~~~~~~~~~~~~~~~~~~~~

Tuning a PID control can be a bit of an art. There are a number of
approaches in the literature and we provide two below. The first tuning
method has you work the gains like dials starting with the proportional
gain, the addressing the derivative gain and finishing with the integral
gain.

#. Select a typical operating setting for the desired speed, turn off
   integral and derivative parts. Increase :math:`K_P` to maximum or
   until oscillation occurs.

#. If system oscillates, divide :math:`K_P` by 2.

#. Increase :math:`K_D` and observe behaviour when increasing or
   decreasing the desired speed by 5%. Select a value of :math:`K_D`
   which gives a damped response.

#. Slowly increase :math:`K_I` until oscillation starts. Then divide
   :math:`K_I` by 2 or 3.

#. Check overall controller performance under typical conditions.

PID Parameter Tuning: Ziegler-Nichols method
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Another heuristic tuning method is formally known as the Ziegler -
Nichols method, introduced by John G. Ziegler and Nathaniel B. Nichols
in the 1940s. As in the method above, the :math:`K_i` and :math:`K_d`
gains are first set to zero. The P gain is increased until it reaches
the ultimate gain, :math:`K_u`, at which the output of the loop starts
to oscillate. :math:`K_u` and the oscillation period :math:`T_u` are
used to set the gains as shown in
:numref:`tab:ZieglerNicholsmethod`

.. _`tab:ZieglerNicholsmethod`:
.. table:: Ziegler - Nichols method values
   :widths: auto

   +--------------+-----------------+-----------------+-------------------+
   | Control Type | :math:`K_p`     | :math:`K_i`     | :math:`K_d`       |
   +==============+=================+=================+===================+
   | P            | :math:`0.50K_u` | -               | -                 |
   +--------------+-----------------+-----------------+-------------------+
   | PI           | :math:`0.45K_u` | :math:`T_u/1.2` | -                 |
   +--------------+-----------------+-----------------+-------------------+
   | PD           | :math:`0.8K_u`  | -               | :math:`T_u / 1.2` |
   +--------------+-----------------+-----------------+-------------------+
   | PID          | :math:`0.60K_u` | :math:`T_u/2`   | :math:`T_u / 8`   |
   +--------------+-----------------+-----------------+-------------------+

Velocity and Position Control
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

When working with robot arms and vehicles, it is rarely safe to jump
from one speed to another (or one force to another). Safety of the
humans and the system requires that changes in the system state be
controlled. Clearly a robot arm that jumps from one position to another
is dangerous. As would an autonomous vehicle which skidded off from the
light. It is also hard on the mechanical systems to have this form of
bang-bang control applied to changes in set points. For vehicles, high
wheel torques can cause slip and slide which introduces errors in the
navigation.

This is addressed by having speed ramp functions to control the
transition to new set points. This is no more than what we all do in our
cars by slowly pressing down on the accelerator until we reach the
desired speed. :numref:`fig:speedramp0` shows one
sample ramp function. There are times when one needs coordinated control
between multiple devices. This is necessary with any vehicle that has
more than one drive motor, for example a differential drive,
:numref:`fig:speedramp1`.

.. _`fig:speedramp0`:
.. figure:: ControlFigures/changesetpoint.*
   :width: 40%
   :align: center

   A speed ramp function for a single motor


.. _`fig:speedramp1`:
.. figure:: ControlFigures/dualmotor1.*
   :width: 40%
   :align: center

   Coordinating two motors separately with a P
   controller.

Ramping up each motor to the same speed does not assure straight motion.
Variations between ramp ups can cause significant errors in orientation
for differential drive. Both motors must be ramped up in the same manner
so must be fed the same ramp up function,
:numref:`fig:speedramp2`. This can be done with a
P, PI or PID control; a PID version is shown in
:numref:`fig:speedramp3`.


.. _`fig:speedramp2`:
.. figure:: ControlFigures/dualmotor2.*
   :width: 40%
   :align: center

   Coordinating two motors with the same ramp function.

.. _`fig:speedramp3`:
.. figure:: ControlFigures/dualmotor3.*
   :width: 40%
   :align: center

   Coordinating two motors with dual P/PI/PID
   controllers.

Kinematics vs Physics Engine
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Kinematics is concerned with the geometry of motion. It describes the
geometry based on the constraints of motion without concerns for the
causes of the motion such as the forces acting on the system. A physics
engine models the forces and the subsequent motion. A kinematics
simulation has no knowledge of mass, inertia, friction, momentum and
accelerations. It is possible to have jumps in velocity, which would
correspond to infinite forces/accelerations. A physics engine will
provide a more realistic simulation at the cost of increased
computation. This includes the control strategy and so in practice you
may not be able achieve the desired control due to limits on forces
available.

.. rubric:: Footnotes

.. [#f1] Texting while driving is an example of open loop control.  Not looking at the road means you do not use sensor feedback to correct the path resulting in a collision.
