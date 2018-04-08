Simulation Basics [Chap:Simulation]
===================================

Why simulate?
-------------

Learning how to operate any robotic system can be rather rough on your
budget. A sufficiently robust robot that can support the standard array
of sensors, processing and drive system can run well into the thousands
of dollars. And there is nothing more annoying than having a silly
little software error send the robot tumbling down a flight of stairs.
Based on cost and possible system damage, many researchers and
instructors elect to run the robot in simulation. Physical robots can
take a long time to build, configure and get operational. They require
all the details to be just right or it will not power up.

Compare this to simulation. Getting the initial simulation software
running can be very time consuming, but once this is done, changes to
simulation software can be fast. Realistic simulations are very hard
however. It requires that all of the physical parts are carefully
modeled in the code. So the robot design or geometry with masses and
components needs to be included. Next the physics needs to be carefully
entered. All of the servo or motor dynamics, friction, torque,
acceleration, etc involved must be modeled. In many cases, building the
robot is faster than building a very accurate simulation.

So why would one simulate? One answer is that one can focus on a part of
the system and not worry about the entire design to be competed. The
simulation does not need all the components of the robot included and
you can focus on parts of the system. You can swap out parts quickly and
simulate components that don’t actually exist. A simulation can act as a
proof of concept leading to a new design. It can be cheaper than
prototyping. Simulations don’t wear out or run out of battery power.

Of course, your design might, by accident, optimize aspects unique to
the simulation and not work in the real world. The conclusion of many
roboticists is that simulation is a valuable tool in the initial
prototyping stage, but there is no substitute for a physical robot. Much
of this applies to a first course in robotics. Every reader can run
simulations at no cost (well assuming that we are using an open source
package). The simulated robot cannot be broken. It won’t run out of
power. The most valuable aspect is that the simulation can be restricted
to just a few elements making it much simpler. This avoids overwhelming
the novice and so will be our approach.

There are some really great motion simulation packages available. Often
these codes fall under more general motion simulation codes and overlap
gaming systems, kinematics and dynamics codes. These use physics engines
like the

For the roboticist interested in simulating a robot, it is very nice not
to have to look at the simulation code and focus on the control code.
This allows more time devoted to the robotics control problem. However,
if we have a single code base, the code must be designed to be
“compiled" together. This is a challenge in the face of shared
resources. By separating the code into different programs which
communicate via messages, we achieve data encapsulation and security,
modularity, module language independence, and location independence. ROS
will allow us to do just that.

But, looking back over the code, is it really worth go to the trouble of
having separate programs control the robot? Most of us just sit at one
computer and running multiple computers can be challenging. There are
several reasons to consider. Large blocks of code are hard to design,
develop and, mostly, debug. Good software practice would have us develop
classes or modules that address specific functions in the software. So,
separation of the graphics from the control code is good design. Of
course, we can do this without using our socket based design.

One reason for this design is that we can select the best environment
and programming language for the windowing and then the best environment
and language for the control code. The graphics might be written in C++
or Java. The control code could be written in Python.

The windowing code and the driving code are fundamentally different.
They run concurrently yet they operate on different schedules and
interact asynchronously. As we incorporate more detail into the graphics
side, it will require additional resources. It makes sense to assign
these different tasks to different cpus. The graphics side will want to
add maps, simulated sensors and other simulated hardware which needs to
be separate from the control code. When multiple developers become
involved, the interfaces between the device code must be established -
otherwise code chaos will emerge.

Simulating Motion
-----------------

As stated before, producing motion is not difficult. Deciding on the
correct actions and controlling the system are the more challenging
aspects. The first is known as *Motion Planning* or just *Planning* and
the second is called *Controls*. To get started we will borrow
algorithms from nature since we see so many successful autonomous agents
in the biological world. Worms and insects are very successful animals.
They can sense the world and move around in it. We can borrow from
notions in physics and chemistry when we see simple systems moving in
constrained manners. The simplest solution is the best solution. It is
best to use no more components or technology than necessary. Beyond
basic elegance is the fact that the more components something has, the
greater probability the system will fail. This is true in our
simulations as well.

We return to the two examples in the previous section, the Two Link
Manipulator and the Mobile Disk Robot. Using these two systems, we will
introduce methods to simulate motion. These very basic systems can be
used as the prototypes for developing a simulation and for the simple
motion planning algorithms.

You can download the simulators by following the links on D2L. To get
started, again you need to be in your Ubuntu session and run the ROS
Master:

::

    >  roscore

You can run the Two Link Manipulator simulator we will use by typing

::

    >  python twolinksimple.py

and you should see what is indicated in
Figure \ `[Fig:twolinksimulator1] <#Fig:twolinksimulator1>`__-(a). In
another terminal, run Python and type

::

    >>> import rospy
    >>> from std_msgs.msg import String
    >>> pub = rospy.Publisher('TwoLinkTheta', String, queue_size=10)
    >>> rospy.init_node('talker', anonymous=True)
    >>> message = "20:10:0"
    >>> pub.publish(message)

.. raw:: latex

   \centering

|(a) The two link simulator. (b) Published angle to the
simulator.[Fig:twolinksimulator1]| |(a) The two link simulator. (b)
Published angle to the simulator.[Fig:twolinksimulator1]|

You should see the link arm move as shown in
Figure \ `[Fig:twolinksimulator1] <#Fig:twolinksimulator1>`__-(b). The
API is very simple. You need to publish a string formatted as
"theta1:theta2:pen". The values theta1 and theta2 are in degrees (int or
float), and pen is an int. Pen is set to 1 to draw and 0 to not draw.
The program DialCntrl.py is an example of a Tk widget that uses two
sliders to set the angle,
Figure \ `[Fig:tksliderexample] <#Fig:tksliderexample>`__\ (a). To gain
an understanding of the ROS Node structure, one may list out the ROS
nodes (example, your numbers will vary):

::

    rosnode list
    /DialController_5943_1473004072330
    /TwoLinkSimulation_5785_1473004028541
    /rosout

To view the resulting node graph we can use the ROS tool rqt_graph:

::

    rosrun rqt_graph rqt_graph

In this case it produces Figure \ `[Fig:rosgraph0] <#Fig:rosgraph0>`__.

.. raw:: latex

   \centering

.. figure:: sim/rosgraph0.png
   :alt: The ROS Node Graph Tool rqt_graph. [Fig:rosgraph0]

   The ROS Node Graph Tool rqt_graph. [Fig:rosgraph0]

If you are curious about the messages flowing on a topic, recall ROS can
echo those to a terminal for debugging purposes. In a free terminal,
type

::

    rostopic echo /TwoLinkTheta
     

The move one of the sliders. You will see the message on the
TwoLinkTheta topic echoed. If you have source code you can clearly print
out the messages. It is nice to see what is actually going across. If
you don’t have source code, then this tool is very handy.

A Tk control that can set position is given in the next example
PositionCntrl.py and shown in
Figure \ `[Fig:tksliderexample] <#Fig:tksliderexample>`__\ (b). The
widget PositionCntrl.py publishes :math:`(x,y)` coordinates. An
intermediate node IK.py is used to convert the :math:`(x,y)` values to
:math:`(\theta_1, \theta_2)` and these values are published to the Two
Link Simulator.

.. raw:: latex

   \centering

|(a) The servo angle control widget and (b) the position control
widget.[Fig:tksliderexample]| |(a) The servo angle control widget and
(b) the position control widget.[Fig:tksliderexample]|

::

    # Libraries
    from math import *
    import rospy
    from std_msgs.msg import String

::

    # Call back function
    def capture(data):
        var = data.data.split(":")
        x = float(var[0])
        y = float(var[1])
        a1 = float(var[2])
        a2 = float(var[3])
        pen = int(var[4])
        inverse(x,y,a1,a2,pen)

::

    # Compute IK and send to simulator    
    def inverse(x,y,a1,a2,pen):
        if (sqrt(x*x+y*y) > a1+a2):
          print "(x,y) out of reach for links"
        else:
          d =  (x*x+y*y-a1*a1-a2*a2)/(2.0*a1*a2)
          t2 = atan2(-sqrt(1.0-d*d),d)
          t1 = atan2(y,x) - atan2(a2*sin(t2),a1+a2*cos(t2))
          dt1 = (180.0*t1/pi)
          dt2 = (180.0*t2/pi)
          print x,y, dt1, dt2
          sliders = str(dt1) + ':' + str(dt2) + ':' + str(pen)
          pub.publish(sliders)

::

    # ROS management
    pub = rospy.Publisher('TwoLinkTheta', String, queue_size=10)
    rospy.init_node('Converter', anonymous=True)
    rospy.Subscriber("TwoLinkCoords", String, capture)
    rospy.spin()

.. raw:: latex

   \centering

.. figure:: sim/rosgraph1.png
   :alt: The ROS Node Graph Tool rqt_graph. [Fig:rosgraph1]

   The ROS Node Graph Tool rqt_graph. [Fig:rosgraph1]

Animation of the Two Link Manipulator
-------------------------------------

[example_twolinkmanipulator] For the arm in the two link example,
determine the joint angles to trace out a circle centered at (10,8) of
radius 5. The circle can be parametrized by
:math:`x(t) = 5\cos (t) + 8`, :math:`y(t) = 3 \sin(t) + 10`,
:math:`-\pi \leq t \leq \pi`. Generate an array of points on the circle
and plug them into the inverse kinematics.

Bring up the two link simulator. Then run the following code in Python.
You should see an animation of the two link arm drawing a circle. The
final position is given in
Figure \ `[Fig:twolinkcircleexample] <#Fig:twolinkcircleexample>`__.

::

    # Bring in libraries
    import rospy
    from std_msgs.msg import String
    import numpy as np
    import time
    from math import *

::

    #Setup Arrays
    step = 0.1
    t = np.arange(-pi, pi, step)
    x = 5.0*np.cos(t) + 8.0
    y = 3.0*np.sin(t) + 10.0

::

    #Initialize variables
    a1 = 10.0
    a2 = 10.0
    d = (x*x + y*y - a1*a1 - a2*a2)/(2*a1*a2)   
    t2 = np.arctan2(-np.sqrt(1.0-d*d),d)
    t1 = np.arctan2(y,x) - np.arctan2(a2*np.sin(t2),a1+a2*np.cos(t2))

::

    # Setup ROS and publish joint data
    pub = rospy.Publisher('TwoLinkTheta', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)

    for i in range(t.size):
       print t1[i], "  ", t2[i]
       m = str(180*t1[i]/np.pi) + ":" + str(180*t2[i]/np.pi) + ":" + str(1)
       time.sleep(0.25)
       pub.publish(m)

.. raw:: latex

   \centering

.. figure:: sim/twolinkcircleexample.png
   :alt: The output of the circle inverse kinematics
   code.[Fig:twolinkcircleexample]

   The output of the circle inverse kinematics
   code.[Fig:twolinkcircleexample]

In this example, we generate an array named t which is used for the
parametric equations of the circle to generate the x and y arrays. We
may use the inverse kinematic formulas to determine the arrays for
:math:`\theta_1` and :math:`\theta_2` called t1 and t2. The
:math:`\theta_1` and :math:`\theta_2` would be the values sent to the
joint actuators.
Figure \ `[Fig:twolinkcircleexample] <#Fig:twolinkcircleexample>`__
shows the results.

You can modify the data arrays to plot a line:

::

    #Setup Arrays
    t = np.arange(-5, 8, step)
    x = t
    y = x + 5

The inverse kinematics can be placed into a separate ROS node. The
driving program follows (same headers as before). To connect to the
simulation program, we use the inverse kinematics node as before

::

    #Setup Arrays
    a1 = 10
    a2 = 10
    step = 0.1
    t = np.arange(-pi, pi, step)
    x = 5.0*np.cos(t) + 8.0
    y = 3.0*np.sin(t) + 10.0

    pub = rospy.Publisher('TwoLinkCoords', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)

    for i in range(t.size):
       locs = str(x[i]) + ":" + str(y[i]) + ":" + str(10) + ":" + str(10) 
                       +":" + str(1)
       time.sleep(0.25)
       pub.publish(locs)

.. raw:: latex

   \centering

|Movement between the points. a) moving both linearly. b) moving the
servos sequentially. [Fig:twolinkcoarseexample]| |Movement between the
points. a) moving both linearly. b) moving the servos sequentially.
[Fig:twolinkcoarseexample]|

This simulation gives an idea about how to move the robotic arm and the
path is correct. The motion however is not smooth. This is because we
are moving the arm from position to position. This is known as position
control. If you look at the curve produced, it is not a smooth curve but
is a curve made of of connected segments like a polygon,
Figure \ `[Fig:twolinkcoarseexample] <#Fig:twolinkcoarseexample>`__.
Note that the output is not actually a polygon; the sides are not
straight line segments.

In between the control points, the system moves according to how the
controllers are programmed. They will move the joint angles in a linear
fashion. If they are moved together you will see
Figure \ `[Fig:twolinkcoarseexample] <#Fig:twolinkcoarseexample>`__\ (a).
If they are moved one at a time you will see
Figure \ `[Fig:twolinkcoarseexample] <#Fig:twolinkcoarseexample>`__\ (b)

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
that we introduced in Chapter \ `[Chap:Terms] <#Chap:Terms>`__ shown in
Figure \ `[ddriveRecalled] <#ddriveRecalled>`__.

.. raw:: latex

   \centering

.. figure:: motion/ddrive
   :alt: Simple differential drive robot. [ddriveRecalled]

   Simple differential drive robot. [ddriveRecalled]

and the associated
equations \ `[ddkinematicsmodel] <#ddkinematicsmodel>`__:

.. math::

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
Figure \ `[fig:piecewisecirculararcs] <#fig:piecewisecirculararcs>`__
shows a sample path.

.. raw:: latex

   \centering

.. figure:: sim/piecewisecircular
   :alt: Piecewise circular/linear arc paths[fig:piecewisecirculararcs]

   Piecewise circular/linear arc paths[fig:piecewisecirculararcs]

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

[ddexamplenotworkable] Let :math:`\dot{\phi_1} = e^{-t^2}` and
:math:`\dot{\phi_2} = t`

.. math:: \theta(t) = \theta(0) + \int_0^t \frac{r}{2L} \left(e^{-\tau^2}-\tau\right)d\tau = ???

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
efficient [1]_ and reuse the variable even though one denotes a function
of time and one denotes a sequence.

.. math:: t_k \equiv k\Delta t, \quad t_{k+1} = (k+1)\Delta t

.. math:: x_k \equiv x(t_k), \hspace*{1cm} y_k \equiv y(t_k)

.. math::

   \omega_{1, k}\equiv \dot{\phi}_{1}(t_k), \hspace*{1cm}
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

   \label{discreteDD}
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
Figure \ `[fig:piecewiseconst] <#fig:piecewiseconst>`__. The assumption
of piecewise constant velocity does not hold in the general case and so
we see accumulating drift when comparing the robot’s true path and the
approximated one.

.. raw:: latex

   \centering

.. figure:: sim/piecewiseconst.pdf
   :alt: Piecewise Constant nature of the Euler
   Approximation.[fig:piecewiseconst]

   Piecewise Constant nature of the Euler
   Approximation.[fig:piecewiseconst]

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
Figure \ `[intro-tangent] <#intro-tangent>`__.

.. raw:: latex

   \centering

.. figure:: motion/tantheta
   :alt: The relation between :math:`\theta` and :math:`\dot{x}`,
   :math:`\dot{y}`. [intro-tangent]

   The relation between :math:`\theta` and :math:`\dot{x}`,
   :math:`\dot{y}`. [intro-tangent]

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

   \label{inverseddequations}
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

   \label{inverseddequationskappa}
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
we use “+" in
equations \ `[inverseddequations] <#inverseddequations>`__. Plugging the
values into equations \ `[inverseddequations] <#inverseddequations>`__
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

.. raw:: latex

   \centering

.. figure:: motion/quadpolyphis
   :alt: The wheel velocities. [quadraticpathexample2]

   The wheel velocities. [quadraticpathexample2]

.. figure:: motion/quadpoly1
   :alt: Comparison of the path and driven path.[quadraticpathexample3]

   Comparison of the path and driven path.[quadraticpathexample3]

On a robot, the motor controllers will be taking digital commands which
means the wheel velocities are discrete. This implies that the robot has
fixed wheel velocities during the interval between velocity updates. We
know in the case of the differential drive robot, fixed wheel speeds
means the robot is driving a line or circle. Therefor the DD robot in
this case is following a connected path made up of line or circle
segments, see Figure \ `[fig:piecewiseconst] <#fig:piecewiseconst>`__.
Even when we do have functional forms for the wheel speeds, the
implementation is still discrete.

It makes sense to treat this as a discrete formula and to write as such:

.. math::

   \label{eq:ddikpartial}
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

The Ground Robot World
----------------------

One of the main differences many see between a vehicle and a robotic
vehicle is whether or not a person is “onboard". If you are driving a
car, then we would not call this a robot. But if your car was remotely
operated, then some would call it a robotic car.  [2]_ Can we make the
robot simulation remotely operated? In this case we mean, *can this be
controlled from an external program?* The answer is yes.

The previous robot code examples allow the user to move a simulated
device around an open rectangle. The world has obstacles and a
simulation should reflect this. So, how should we include obstacles? The
simulation is in two dimensions and so the obstacle will also be in 2D.
The obstacle is then represented as a 2D shape as viewed from above. The
presentation of the simulation is in a window which means at some point
the robot and obstacles are presented on a grid or in a discrete
fashion. This means we have some choices on how to represent the world,
obstacles and other objects,
Figure \ `[fig:enviromodel] <#fig:enviromodel>`__.

The environment can be represented in three different manners:
continuous, discrete and topological. Continuous is how we tend to think
about the world. All of the locations and distances for objects,
ourselves and the robots use floating point values. For example, the
center of the robot would be located by a pair of floating point values
and exact information about the robot shape stored in a database,
Figure \ `[fig:metricmap] <#fig:metricmap>`__.

For a discrete representation, the world is discretized and objects are
located using integer values,
Figure \ `[fig:discretemap] <#fig:discretemap>`__. The world is then a
large checkerboard with a square (pixel) either occupied or not
occupied. Simple two or three color bitmaps then suffice (two for object
maps and optionally a third to track the robot). Painting a pixel white
will indicate that pixel or location is unoccupied. Painting it colored
indicates the pixel is occupied. This approach is known as an occupancy
grid. The obstacle is simply the collection of black pixels on the
occupancy map. A B/W image file can then be used to generate obstacle
maps. [One handy way to accomplish this task is to use a paint program
(or image editing tool) which can export the image into a format that is
easy to read. ]

.. raw:: latex

   \centering

.. raw:: latex

   \centering

.. figure:: planning/envrep
   :alt: How one should represent the environment.[fig:enviromodel]

   How one should represent the environment.[fig:enviromodel]

.. raw:: latex

   \centering

.. figure:: slam/metricmap
   :alt: Continuous environmental representation.[fig:metricmap]

   Continuous environmental representation.[fig:metricmap]

.. raw:: latex

   \centering

.. raw:: latex

   \centering

.. figure:: slam/discretemap
   :alt: Discrete environmental representation. [fig:discretemap]

   Discrete environmental representation. [fig:discretemap]

.. raw:: latex

   \centering

.. raw:: latex

   \centering

.. figure:: slam/topomap
   :alt: Topological representation. [fig:topomap]

   Topological representation. [fig:topomap]

Topological representations do not include metric information like the
other two, Figure \ `[fig:topomap] <#fig:topomap>`__. Relationships are
through graphs that indicate two things are connected via a path. How
they are connected is another issue. This is very much how humans store
maps. You probably know that to get to your favorite restaurant, you
have to pass the Home Depot and take the next right. Then you keep going
until you pass the Whole Foods market. Then a quick left and there you
are. In this description, no distances were provided and even the notion
of left and right are flexible since we don’t require the streets
intersect at right angles.

For the case of the robot simulation, the choice has been partially
made. The robot’s world appears as an image which is a discretization or
a grid. Thus we have a discrete environment. We might decide to go with
an obstacle map. Each obstacle is just written into the map and then
disappears in to the large collection of filled pixels. Or we may elect
to keep our obstacles in a continuous representation. However, this
means that translations between the continuous and discrete forms must
happen often.

Continuous and discrete forms each have strengths and weaknesses. We
have very precise information in the continuous form. To increase
precision in the discrete world, we must decrease pixel size which
increases the array storage dramatically or forces a more sophisticated
data format over a simple 2D array. Although storage has increased, many
operations in the discrete world are much easier.

Consider the problem of simulating a robot impact on a object. Say that
the object has an irregular shape. This shape can be approximated by the
pixelized version in the discrete world or by a cubic spline
approximation using a continuous approximation. True that you have much
better accuracy with the cubic spline. The problem is in determining
intersection of the robot boundary with the object boundary. In the
continuous world, we need to take both of the functions and look for
intersecting boundaries at each time step. This requires a complex
nonlinear equation solving routine. [Just work out the algebra for two
circles intersecting.] For the bitmap version we just check that the
front of the robot is on an occupied pixel or cell (if cell[i][j] == 1
then ....).

The continuous version will keep objects as objects. For example, if you
have disks that touch, the continuous representation will track the
centers and radii of the two disks. You always know you have multiple
objects. Once converted to a bit map, it could be two adjacent objects
or one connected object or multiple partial objects, etc. It is the
difference between high and low level representations. A topological
representation takes this approach to the next level by removing metric
information and just keeping object description in a connectivity graph.
Many factors enter into the choice of representation. It is always a
trade off between speed, accuracy and simplicity.

Simple Obstacles
~~~~~~~~~~~~~~~~

The simplest object to study is a disk. It is simple not only in
geometry, but in the more difficult task of determining collision. We
know that if any part of our robot is within a radius of the center, we
have collided. Our robots are round, so collision is just checking the
distance between centers minus the radii. It makes a good stage for a
first path planning exercise. We assume for the moment that our robot
can move freely around the plane (in the open space) and that the plane
is covered with disk shaped obstacles. We also assume that the robot
knows its coordinate location and heading. For a given obstacle map, can
we find a path connecting two points in the plane?

The Python code to check if two disks intersect is fairly
straightforward:

::

    def collide(center1, r1, center2, r2):
        x1 = center1[0]
        y1 = center1[1]
        x2 = center2[0]
        y2 = center2[1]
        d = sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2))-r1-r2
        return d

Where center is a list and r is the radius.  [3]_

.. raw:: latex

   \centering

.. figure:: turtle/collision
   :alt: Collision detection with circular robots. [circlecollide]

   Collision detection with circular robots. [circlecollide]

To check for intersection, we only need to check that :math:`d` is
small. Using this we may build a method for a contact sensor. You can
treat a contact sensor as a disk of zero radius and use the formula
above (adjusting for the relation between the center of the robot and
the sensor). Many early robots had sensors placed in a ring around the
body of the robot, Figure \ `[turtlesensors] <#turtlesensors>`__. For
this example, they will be contact or touch sensors, but in experimental
units often low cost ultrasonic ranging sensors would be used.

.. raw:: latex

   \centering

.. figure:: turtle/turtlesensors
   :alt: A circular robot (like a Create) with touch sensors mounted
   around the body. [turtlesensors]

   A circular robot (like a Create) with touch sensors mounted around
   the body. [turtlesensors]

Assume that you have a circular robot with a ring of touch or bump
sensors around the body. Knowing the direction of travel, it is possible
to estimate the boundary of the obstacle relative to the robot,
Figure \ `[turtleboundary] <#turtleboundary>`__. The boundary normal can
be estimated from the vector created by the sensor location to the robot
center. This is a local estimate only as
Figure \ `[turtleboundary] <#turtleboundary>`__ shows. Being able to
estimate the boundary means that a robot can follow the boundary. The
tangent to the boundary is required for this task.

.. raw:: latex

   \centering

|a) Estimating the object boundary. b) Bump sensors can only determine
the nature of the boundary at the contact location. c) Using touch
sensors to estimate the boundary normal and tangent. [turtleboundary]|
|a) Estimating the object boundary. b) Bump sensors can only determine
the nature of the boundary at the contact location. c) Using touch
sensors to estimate the boundary normal and tangent. [turtleboundary]|
|a) Estimating the object boundary. b) Bump sensors can only determine
the nature of the boundary at the contact location. c) Using touch
sensors to estimate the boundary normal and tangent. [turtleboundary]|

Using the normal vector, :math:`\hat{n} = <n_1, n_2>`, the tangent to
the boundary is computed via

.. math:: T = \pm <n_2, -n_1>

where the sign is taken so that motion is to the right (right hand
rule). This tangent direction will provide the motion direction for a
boundary following approach. Estimation of the tangent or the direction
of travel can be done with a ring of touch sensors,
Figure \ `[turtleboundary] <#turtleboundary>`__.

Using a range sensor
^^^^^^^^^^^^^^^^^^^^

Recall the components in
Figure \ `[intro-components] <#intro-components>`__. There was not a
touch or impact sensor listed. However, there are two types of range
sensors shown. One is a LIDAR and the other is a Kinect. The next simple
planner presented assumes that the robot has a ranging device. The
simplest to model is the LIDAR.

.. raw:: latex

   \centering

.. figure:: slam/discretemap2
   :alt: Discrete object map.[discreteobjmap]

   Discrete object map.[discreteobjmap]

A lidar is a simple device conceptually. The unit is able to sweep or
turn in one direction which for our discussion we assume it is
horizontal. It chops up the angular variable into some number of
discrete angles. At each angle or direction, the lidar unit projects a
laser beam out. It receives the reflected signal and computes the
distance. Naively one simply measures the time of flight, divides by two
(for the round trip) and multiplies by :math:`c` (the speed of light):
:math:`D = RT`. This provides the distance of the nearest obstacle at
the current angle. Record the number and move to the next angle.

A sweep creates an array of values where the array index is a function
of the angle and array values are distances. The unit will return the
array. Angles can be reconstructed if you know the starting angle and
the angular increment: :math:`\theta_i = \theta_0 + i\Delta\theta`. If
you are simulating a given LIDAR unit, then one would use the increment
angle of that unit. If not, then you will decide on the details of
angular increment, maximum range, minimum range and data rate.

How is this done in a discrete environment? Using a two colored image,
let white be free space and red or black indicate occupied space. To
simulate the beam out of the LIDAR, create a virtual line out of the
lidar and follow a straight line along white pixels until you run into a
colored pixel. Stop at the first colored pixel. Using the endpoints of
the line segment (virtual lidar to object pixel), the distance can be
computed. Let :math:`(n,m)` be the start of the line and let
:math:`(i,j)` be the location of the object pixel and recall the
distance is :math:`d = \sqrt{(i-n)^2 + (j-m)^2}`. [4]_

Any actual lidar unit has an effective range, :math:`R`. In simulation
one could certainly compute :math:`d` as you move out along the ray (or
line) and stop when the max range occurred. This approach will work but
it requires computing the distance function within the innermost loop
and will not result in efficient code. A more effective approach is to
just step out in the radial variable. This means you need to represent
the line or ray in polar coordinates. We will assume that :math:`R` is
given in the pixel coordinates and the range would be
:math:`0 \leq r \leq R`. The other issue is increment value for the
lidar simulation. Again, if this value is taken from an actual unit,
then that is the value to use. Otherwise, at the maximum range,
:math:`R`, we would like that an increment in the angle selects the
“next” (adjacent) pixel. So we want :math:`\Delta \theta` to be small
enough to hit all the pixels, but no smaller for performance reasons,
see Figure \ `[inscribedcircle] <#inscribedcircle>`__ (b).The
circumference is :math:`2\pi R`. If a pixel is :math:`1^2` units, then
we select :math:`\Delta\theta \approx 1/(2\pi R)` (or slightly smaller).

.. raw:: latex

   \centering

.. figure:: path/lidarinc
   :alt: Laser angle increments. (a) The first is too small and we
   resample the same pixel. (b) The second increment is too large and we
   miss pixels. [inscribedcircle]

   Laser angle increments. (a) The first is too small and we resample
   the same pixel. (b) The second increment is too large and we miss
   pixels. [inscribedcircle]

The lidar simulation algorithm is given in
Algorithm \ `[lidarsim] <#lidarsim>`__.

:math:`k=0` :math:`\Delta\theta = 1/(2\pi R)`

.. raw:: latex

   \FOR    {$\theta=0$  \TO $2\pi$}

.. raw:: latex

   \FOR      {$r=0$ \TO $R$}

:math:`i= (\text{int}) r \cos \theta`

:math:`j= (\text{int}) r\sin\theta`

.. raw:: latex

   \IF {Map$(i,j)$ is occupied}

break from :math:`r` loop :math:`dist(k) = r`

k++ :math:`\theta += \Delta\theta`

.. raw:: latex

   \ENDFOR

Motion Plannning
----------------

Simple Planning
~~~~~~~~~~~~~~~

When controlling the robot without feedback, open loop control, we
preplan the route and then code up a list of motion instructions. For
differential drive robots, the easiest routes to drive are combinations
of lines and circles,
Figure \ `[fig:simplecurvedpath] <#fig:simplecurvedpath>`__. If you have
a rough idea of the route, place some points along the route, connect
with line and circle segments. Along those segments, the differential
drive has constant wheel speed. In practice this is difficult since one
cannot have instant jumps in wheel velocity. This makes accurate turns
challenging. If stopping and turning in place on the route is
acceptable, paths with just straight lines are the easiest to develop,
Figure \ `[fig:simplecurvedpath] <#fig:simplecurvedpath>`__. Then is is
just a matter of starting with the correct orientation and driving for a
given amount of time.

.. raw:: latex

   \centering

.. raw:: latex

   \centering

.. figure:: sim/simplepath.pdf
   :alt: [fig:simplecurvedpath] Path with arcs

   [fig:simplecurvedpath] Path with arcs

.. raw:: latex

   \hfill

.. raw:: latex

   \centering

.. figure:: sim/simplestraightpath.pdf
   :alt: [fig:simplecurvedpath] Path without arcs

   [fig:simplecurvedpath] Path without arcs

There is a clear problem with open loop control. Any variation in the
physical system can cause drift. This drift accumulates over time and at
some point the robot is not driving the intended course. The other
problem is that the path is tuned to a specific obstacle field. We must
know the obstacles and their locations prior to moving. A more advanced
algorithm would be able to take a goal point and using knowledge of the
current robot location, drive itself to the goal. The basic motion
algorithm attempts this next step. [5]_

Basic Motion Algorithm
~~~~~~~~~~~~~~~~~~~~~~

Assuming we have a simple obstacle map, how should we proceed? Try the
following thought experiment. Pretend that you are in a dark room with
tall boxes. Also pretend that you can hear a phone ringing and you can
tell what direction it is. How would you navigate to the phone? Figuring
that I can feel my way, I would start walking towards the phone. I keep
going as long as there are no obstructions in my way. When I meet an
obstacle, without sight I can’t make any sophisticated routing
decisions. So, I decide to turn right a bit and head that way. If that
is blocked, then I turn right a bit again. I can continue turning right
until the path is clear. Now I should take a few steps in this direction
to pass the obstacle. Hopefully I am clear and I can turn back to my
original heading. I head in this direction until I run into another
obstacle and so I just repeat my simple obstacle avoidance approach.

Set heading towards goal Move forward count = 0 Turn right Move forward
incr count Set heading towards goal

.. raw:: latex

   \centering

.. figure:: turtle/turtleobs
   :alt: The direct path to the goal.[turtlebasicmotion_a]

   The direct path to the goal.[turtlebasicmotion_a]

.. raw:: latex

   \hfill

.. figure:: turtle/turtleobs2
   :alt: Path using the Basic Motion algorithm.[turtlebasicmotion_b]

   Path using the Basic Motion algorithm.[turtlebasicmotion_b]

Figure \ `[turtlebasicmotion] <#turtlebasicmotion>`__ illustrates the
idea. This algorithm is not completely specified. The amount of right
turn and the distance traveled in the move forward steps is not
prescribed above. Assuming values can be determined, will this approach
work? We expect success when faced with convex obstacles but not
necessarily for non-convex obstacles,
Figure \ `[simple1motionproblem] <#simple1motionproblem>`__. Using
Figure \ `[simple1motionproblem] <#simple1motionproblem>`__ as a guide,
we can construct a collection of convex obstacles which still foil the
algorithm; this is expressed in
Figure \ `[simple2motionproblem] <#simple2motionproblem>`__. The robot
bounces from obstacle to obstacle like a pinball and is wrapped around.
Leaving the last obstacle the robot reaches the cutoff distance and then
switches back to the “motion to goal" state. However, this sets up a
cycle. So, the answer to the question “does this work" is not for all
cases.

.. raw:: latex

   \centering

.. raw:: latex

   \centering

.. figure:: planning/simple1
   :alt: Getting trapped in a non-convex solid
   object.[simple1motionproblem]

   Getting trapped in a non-convex solid object.[simple1motionproblem]

.. raw:: latex

   \hfill

.. raw:: latex

   \centering

.. figure:: planning/simple2
   :alt: A collection of convex objects can mimic a non-convex obstacle.
   [simple2motionproblem]

   A collection of convex objects can mimic a non-convex obstacle.
   [simple2motionproblem]

In Chapter \ `[Chap:Planning] <#Chap:Planning>`__, we will fully explore
the challenge of motion planning in an environment with obstacles. It is
easy to see how the thought experiment above can fail and more robust
approaches are needed. Before we jump into motion planning, we want to
understand what view of the world we can get from sensors. This is
necessary so we know what kind of assumptions can be made when
developing our algorithms.

.. raw:: latex

   \FloatBarrier

Problems
--------

.. raw:: latex

   \setcounter{Exc}{0}

]

Why is simulation useful to roboticists?

List some of the advantages and disadvantages of simulating a robot vs.
working with physical robots.

Using ROS and Python, write a program to simulate the motion of a
differential drive robot.

#. Write a program that publishes a sequence of wheel velocities on the
   topic ``/WheelVel`` at 10Hz. Use the multiarray datatype. This node
   should be named ``Control``. This program should also publish on a
   topic named ``/Active`` either 1 or 0 at 1 Hz to say whether or not
   the robot is active (meaning done with wheel velocities and you can
   plot now: active =1, done = 0). Demonstrate the code on
   :math:`\dot{\phi}_1 = 2 + 2e^{-t_n}` and
   :math:`\dot{\phi}_2 = 2+e^{-2t_n}` for :math:`0 \leq t \leq 10`.

#. Write a program that uses the differential drive kinematics to derive
   the robot linear and angular velocities. Publish the velocities using
   the ROS standard twist message and name the topic ``/RobotVel``. This
   node should be named ``ForwardK``. Assume that :math:`D=10`,
   :math:`L=20` and the robot starts at (0,0,0).

#. Write a program that will subscribe to the twist message and plot the
   robot’s path using Python plotting when it gets the signal on the
   Active topic. This node should be named ``RobotPlot``.

Using the above problem, replace the initial pose (0,0,0) in the second
part, (b), with the pose (2,2,45).

Using the forward difference on :math:`x(t) = t^2`, what is the error on
the derivative value for
:math:`\Delta t  = 10^{-1}, 10^{-2}, 10^{-3}, 10^{-4}` at the location
:math:`t=1`.

Let :math:`r=10`, :math:`L=20`, :math:`\Delta t = .1`. Find the discrete
kinematic model if the wheel velocities are
:math:`\dot{\phi}_{1} = 2(1-e^{-t})`,
:math:`\dot{\phi}_{2} = 2(1-e^{-2t})`.[numericalddhw]

Using the discrete model equations in
problem \ `[numericalddhw] <#numericalddhw>`__, plot the path for
:math:`0 \leq t \leq 5`.

For the integral in
example \ `[ddexamplenotworkable] <#ddexamplenotworkable>`__, use a
numerical differential equation solver (with some software package) to
integrate the equations. Compare this to using a Taylor expansion on the
equations to work out the integrals.

What is the smooth (:math:`\dot{x}`, :math:`\dot{y}` are continuous)
parametric form of

#. :math:`y=(3/2)x + 5/2`

#. :math:`y = x^{2/3}`.

#. :math:`(x-3)^2/16 + (y-2)^2/9 = 1`.

Find the analytic wheel velocities and initial pose for a differential
drive robot tasked to follow (:math:`r=1`, :math:`L=4`) the given paths.
Plot the paths and compare to the actual functions to verify.

#. :math:`y=(3/2)x + 5/2`

#. :math:`y = x^{2/3}`

#. :math:`(x-3)^2/16 + (y-2)^2/9 = 1`

Find the wheel velocities and initial pose for a differential drive
robot tasked to drive a square with corners (0,0), (10,0), (10,10),
(0,10). You should stop and turn at a corner. Drive the edges at unit
speed. Plot the paths and compare to the actual functions to verify.

Find the wheel velocities and initial pose for a differential drive
robot in an infinity (:math:`\infty`) shape. Plot the paths and compare
to the actual functions to verify.

Using the STDR simulator, write robot control code to drive the robot
along the triangular path with vertices (0,0), (15,0) , (5,20).

Using the STDR simulator, place two circular obstacles on the canvas.
The first obstacle is a disk centered at (5,5) with radius 2. The second
is a disk centered at (15,15) with radius 3. Write the control code to
drive a figure 8 around the two obstacles. Run at least two loops.

| **a**. Develop the parametric equations to describe the motion of the
  robot:
| Start with a line that oscillates back and forth across the obstacles
  we want to drive around. We need to move along the line between the
  center of the obstacles, :math:`(0,0)` and :math:`(15,15)`. We can
  accomplish this by starting our equations at :math:`(7,7)` and using
  cosine to oscillate both :math:`x` and :math:`y` equally. We multiply
  :math:`\cos(t)` by a large enough constant to clear the obstacles.

  .. math::

     \begin{aligned}
     x&=7+16\cos(t)\\
     y&=7+16\cos(t)\end{aligned}

Next, we need to make the path circle the obstacles. We can add a sine
component to x, which gives us:

.. math::

   \begin{aligned}
   x&=7+16\cos(t)+10\sin(t)\\
   y&=7+16\cos(t)\end{aligned}

Adding a negative sine component to y will widen the loop:

.. math::

   \begin{aligned}
   x&=7+16\cos(t)+10\sin(t)\\
   y&=7+16\cos(t)-10\sin(t)\end{aligned}

Cut the period of the sine components in half and the path will move
through the center of the obstacles.

.. math::

   \begin{aligned}
   x &= 7+16\cos(t)+10\sin(2t)\\
   y &= 7+16\cos(t)-10\sin(2t)\end{aligned}

Finally, divide t of both the :math:`\cos` and :math:`\sin` components
by 10 to slow down the robot:

.. math::

   \begin{aligned}
   x &= 7+16\cos\left(\frac{t}{10}\right)+10\sin\left(\frac{t}{5}\right)\\
   y &= 7+16\cos\left(\frac{t}{10}\right)-10\sin\left(\frac{t}{r}\right)\end{aligned}

| **b**. Implementing the motion control:
| First, take the derivatives of our parametric equations to determine
  velocity:

  .. math::

     \begin{aligned}
     \frac{dx}{dt}&=2\cos\left(\frac{t}{5}\right)-\frac{8\sin\left(\frac{t}{10}\right)}{5}\\
     \frac{dy}{dt}&=-2\cos\left(\frac{t}{5}\right)-\frac{8\sin\left(\frac{t}{10}\right)}{5}\end{aligned}

We can use :math:`\frac{dx}{dt}` and :math:`\frac{dy}{dt}` as the
velocity controls for the robot. This relies on the following robot
starting position:

.. math:: P_0 \approx (23,23)

Therefore, the simulation will move the robot to

.. math:: (x,y)=(23,23)

before spawning the obstacles and driving the figure-8.

| The controller is able to drive the robot along the curve by
  evaluating the velocity functions at discrete time intervals of 0.25
  seconds. The controller publishes the velocity commands and waits for
  0.25 seconds before evaluating the next point. The robot drives for
  :math:`2\pi` radians :math:`\times` :math:`2` loops :math:`\times`
  :math:`10` (because :math:`t` is divided by :math:`10`) for a total of
  :math:`40\pi` seconds. This allows the robot to circle the obstacles
  twice.
| Finally, we bring everything together and run the created scripts with
  PyStage. The code for this algorithm can be seen in
  Listing \ `[lst:3.4] <#lst:3.4>`__

.. raw:: latex

   \centering

.. figure:: solutions/Simulation/p3-4.jpg
   :alt: Pystage: Drawing Figure-8 Around Two Obstacles [fig:3.4]

   Pystage: Drawing Figure-8 Around Two Obstacles [fig:3.4]

.. raw:: latex

   \mylisting[language=Python, firstline=6,basicstyle=\ttfamily\scriptsize, label={lst:3.4}]{../pycode/solutions/Simulation/p3_4.py}

Using STDR Simulator, code the basic motion algorithm. a. Demonstrate
your approach with one obstacle. b. Demonstrate with several obstacles.

| A basic motion algorithm program was developed for ... The completed
  code was tested successfully with world files containing **a) a single
  cluster of obstacles** (Figure `[fig:3.6a] <#fig:3.6a>`__, World
  File \ `[lst:3.6a] <#lst:3.6a>`__) and **b) multiple obstacles**
  (Figure `[fig:3.6b] <#fig:3.6b>`__, World
  File \ `[lst:3.6b] <#lst:3.6b>`__). The code for this algorithm can be
  seen in Listing \ `[lst:3.6] <#lst:3.6>`__.
| Basic idea of the basic motion algorithm!

1

Move toward the goal

If collision: backup, turn right and drive for a few seconds

Repeat until goal is reached

.. raw:: latex

   \centering

.. figure:: solutions/Simulation/p3-6a.jpg
   :alt: Pystage: Basic Motion Algorithm with One Obstacle [fig:3.6a]

   Pystage: Basic Motion Algorithm with One Obstacle [fig:3.6a]

.. raw:: latex

   \mylisting[basicstyle=\ttfamily\scriptsize, label={lst:3.6a}]{../pycode/motion_algorithm/world6a}

.. raw:: latex

   \centering

.. figure:: solutions/Simulation/p3-6b.jpg
   :alt: Pystage: Basic Motion Algorithm with Multiple Obstacles
   [fig:3.6b]

   Pystage: Basic Motion Algorithm with Multiple Obstacles [fig:3.6b]

.. raw:: latex

   \mylisting[basicstyle=\ttfamily\scriptsize, label={lst:3.6b}]{../pycode/motion_algorithm/world6b}

.. raw:: latex

   \mylisting[language=python, firstline=3, breaklines=true, basicstyle=\ttfamily\scriptsize, label={lst:3.6}]{../pycode/motion_algorithm/basic_motion.py}

Using STDR and the basic motion algorithm, place a set of obstacles that
cause the robot to cycle and not find the goal.

| A world file was created for PyStage v2.0 containing several
  obstacles. Using the basic motion program developed for the previous
  problem, these obstacles proved to be successful in causing the basic
  motion algorithm to get stuck in a loop, as seen in
  Figure \ `[fig:3.7] <#fig:3.7>`__. The world file containing the
  cycle-causing obstacles can be seen in
  Listing \ `[lst:3.7] <#lst:3.7>`__.

.. raw:: latex

   \centering

.. figure:: solutions/Simulation/p3-7.jpg
   :alt: Pystage: Basic Motion Algorithm with Cycle [fig:3.7]

   Pystage: Basic Motion Algorithm with Cycle [fig:3.7]

.. raw:: latex

   \mylisting[basicstyle=\ttfamily\scriptsize, label={lst:3.7}]{../pycode/motion_algorithm/world7}

.. raw:: latex

   \Closesolutionfile{Answer}

.. [1]
   that would be a *codeword* for sloppy

.. [2]
   The author would simply call this a remotely operated car, but either
   way, teleoperation does change how one looks at a vehicle.

.. [3]
   Keep in mind that the robot graphics circle method draws from the
   bottom of the circle and so the center for this formula and the one
   for the circle method need to be adjusted by the radius.

.. [4]
   If you wanted an integer array you would cast this as an int.

.. [5]
   This algorithm is slightly more general in that it does not need the
   goal location, but just the direction to the goal during the process.

.. |(a) The two link simulator. (b) Published angle to the simulator.[Fig:twolinksimulator1]| image:: sim/twolinksimulator1.png
.. |(a) The two link simulator. (b) Published angle to the simulator.[Fig:twolinksimulator1]| image:: sim/twolinksimulator2.png
.. |(a) The servo angle control widget and (b) the position control widget.[Fig:tksliderexample]| image:: sim/tksliderexample.png
.. |(a) The servo angle control widget and (b) the position control widget.[Fig:tksliderexample]| image:: sim/tksliderexample2.png
.. |Movement between the points. a) moving both linearly. b) moving the servos sequentially. [Fig:twolinkcoarseexample]| image:: sim/twolinkcoarseexample.png
.. |Movement between the points. a) moving both linearly. b) moving the servos sequentially. [Fig:twolinkcoarseexample]| image:: sim/twolinkcoarseexample2.png
.. |a) Estimating the object boundary. b) Bump sensors can only determine the nature of the boundary at the contact location. c) Using touch sensors to estimate the boundary normal and tangent. [turtleboundary]| image:: turtle/turtlecollide2
.. |a) Estimating the object boundary. b) Bump sensors can only determine the nature of the boundary at the contact location. c) Using touch sensors to estimate the boundary normal and tangent. [turtleboundary]| image:: turtle/turtlecollide3
.. |a) Estimating the object boundary. b) Bump sensors can only determine the nature of the boundary at the contact location. c) Using touch sensors to estimate the boundary normal and tangent. [turtleboundary]| image:: path/tangent

