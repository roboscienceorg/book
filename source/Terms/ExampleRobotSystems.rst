Example Robotic Systems
-----------------------

There are three common examples which we will examine: the serial two
link arm, the parallel two link arm and the differential drive mobile
robot. The serial two link manipulator is a simple robot arm that has
two straight links each driven by an actuator (like a servo). They serve
as basic examples of common robotic systems and are used to introduce
some basic concepts. The parallel two link arm is a two dimensional
version of a common 3D Printer known as the Delta configuration. Last is
the differential drive mobile robot. This design has two drive wheels
and then a drag castor wheel. The two drive wheels can operate
independently like a skid steer “Bobcat”.

Serial Two Link Manipulator
~~~~~~~~~~~~~~~~~~~~~~~~~~~

The last few paragraphs have introduced lots of jargon. To understand
them, it helps to see them in action. The simple :index:`two link manipulator`,
:numref:`intro-two-link` is a good place to
start. Imagine a robotic arm that has two straight links with a rotary
joint at the base and a rotary joint connecting the two links. In
practice, these rotary joints would be run by motors or servos and
probably have some limits, but for now we will assume full
:math:`360^\circ` motion.

.. Owned by Roboscience

.. _`intro-two-link`:
.. figure:: TermsFigures/twolinkalt.*
   :width: 60%
   :align: center

   The two link manipulator.

The workspace that the arm operates inside is a disk,
:numref:`two-link-disk`. This is a two dimensional
workspace. The figure indicates the workspace in gray. It may also be
the case that there is something in the workspace, a workspace obstacle
indicated in red. This unit has two joints which define a two
dimensional configuration space,
:numref:`intro-config-axis`. The dimension of
the configuration space is the degrees of freedom, and so this has two
degrees of freedom. Since the joint is rotary and moving a full
:math:`360^\circ` degrees returns you to the same angle, the two
directions wrap back on themselves. This actually makes the
configuration space a two dimensional torus or “donut”.

.. Owned by Roboscience

.. _`two-link-disk`:
.. figure:: TermsFigures/twolinkconfigobs.*
   :width: 70%
   :align: center

   Two link manipulator: (a) Workspace with equal link lengths and (b) Workspace obstacle.


.. Owned by Roboscience

.. _`intro-config-axis`:
.. figure:: TermsFigures/twolinkconfigdomaintorus.*
   :width: 70%
   :align: center

   Configuration domain and configuration topology which is a torus.



We will illustrate what is meant by kinematics and inverse kinematics
using the two link manipulator. Forward kinematics will identify the
location of the end effector as a function of the joint angles,
:numref:`twolinklabeled`-(a). This is easily done
using a little trigonometry. First we find the location of
:math:`(\xi, \eta)` as a function of :math:`\theta_1` and the link
length :math:`a_1`, :numref:`twolinklabeled`-(b):

.. math:: \xi =  a_1 \cos \theta_1, \quad \eta = a_1 \sin \theta_1

.. Owned by Roboscience

.. _`twolinklabeled`:
.. figure:: TermsFigures/twolink2.*
   :width: 85%
   :align: center

   a) The two link manipulator with the links and joints labeled. b)
   Location of the middle joint.

The next link can be included with

.. math:: \Delta x =  a_2 \cos (\theta_1 + \theta_2), \quad \Delta y = a_2 \sin ( \theta_1 + \theta_2)

Note that :math:`x = \xi + \Delta x` and :math:`y = \eta + \Delta y`.
Combining the expressions, the forward kinematics are:

.. math::
   :label: twolinkforward

   \begin{matrix}
   x = a_2\cos (\theta_1+\theta_2) + a_1 \cos \theta_1 \\
   y = a_2 \sin (\theta_1 +\theta_2) + a_1\sin \theta_1
   \end{matrix}


As you move the servos in the system, you can change the angles
:math:`\theta_1` and :math:`\theta_2`. The
formula :eq:`twolinkforward` gives the location of
the end effector :math:`(x,y)` as a function of
:math:`(\theta_1, \theta_2)`. The values :math:`x`, :math:`y` live in
the workspace. The values :math:`\theta_1`, :math:`\theta_2` live in the
configuration space. This is a holonomic system. A common application is
to move the end effector along some path in the workspace. How does one
find the “path” in configuration space? Meaning how do we find the
values of :math:`\theta_1`, :math:`\theta_2` that give us the correct
:math:`x`, :math:`y` values? This requires inverting the kinematics
equations, hence the term inverse kinematics. The mathematics required
is some algebra and trigonometery for solving :math:`\theta_1`,
:math:`\theta_2` in terms of :math:`x`, :math:`y`.

To find the inverse kinematics formulas we must appeal to some
trigonometry (law of cosines):

.. math::
   :label: eqn:theta2step1

   x^2 + y^2 = a_1^2 + a_2^2 - 2a_1a_2 \cos (\pi - \theta_2).

Using :math:`\cos(\pi - \alpha) = -\cos(\alpha)`, we solve for
:math:`\cos` in Eqn :eq:`eqn:theta2step1`:

.. math:: \cos(\theta_2) = \frac{x^2 + y^2 - a_1^2 - a_2^2}{2a_1a_2 }\equiv D

Using a trig formula:

.. math:: \sin(\theta_2) = \pm \sqrt{1-D^2}

Dividing the sin and cos expressions to get tan and then inverting:

.. math:: \theta_2 = \tan^{-1}\frac{\pm\sqrt{1-D^2}}{D}

The tangent form has the +/- and gives the elbow up and elbow down
solutions.

.. Owned by Roboscience

.. _`twolinklabeled2`:
.. figure:: TermsFigures/twolink3.*
   :width: 40%
   :align: center

   The interior angles for the two link manipulator.

From Figure :numref:`twolinklabeled2`, we have

.. math::
   :label: eqn:theta1step1

   \theta_1 = \phi - \gamma = \tan^{-1}\frac{y}{x} - \gamma .

If you look at the two dotted blue lines you can see that the line
opposite :math:`\gamma` has length :math:`a_2\sin \theta_2`. The segment
adjacent to :math:`\gamma` (blue solid and dotted lines) has length
:math:`a_1 + a_2\cos \theta_2`. Then

.. math:: \tan \gamma =  \frac{\mbox{Opposite}}{\mbox{Adjacent}} = \frac{a_2\sin \theta_2}{a_1 + a_2\cos\theta_2}

which gives us :math:`\gamma`:

.. math:: \gamma = \tan^{-1} \frac{a_2\sin \theta_2}{a_1 + a_2\cos\theta_2}.

Plug :math:`\gamma` into Eqn :eq:`eqn:theta1step1`
and we obtain

.. math:: \theta_1 = \tan^{-1}\frac{y}{x} - \tan^{-1} \frac{a_2\sin \theta_2}{a_1 + a_2\cos\theta_2}

Given the two link manipulator kinematic equations:

.. math::

   \begin{matrix}
   x = a_2\cos (\theta_1+\theta_2) + a_1 \cos \theta_1 \\
   y = a_2 \sin (\theta_1 +\theta_2) + a_1\sin \theta_1
   \end{matrix}

The inverse kinematics (IK) are

.. math:: D = \frac{x^2 + y^2 - a_1^2 - a_2^2}{2a_1a_2 }

.. math::
   :label: IKtwolink

   \theta_1 = \tan^{-1}\frac{y}{x} - \tan^{-1} \frac{a_2\sin \theta_2}{a_1 + a_2\cos\theta_2}, \quad\quad
   \theta_2 = \tan^{-1}\frac{\pm\sqrt{1-D^2}}{D}

Note the kinematic equations only involve the position variables and not
the velocities so they holonomic constraints.

Let :math:`a_1 = 15`, :math:`a_2 = 10`, :math:`x=10`, :math:`y=8`. Find
:math:`\theta_1` and :math:`\theta_2`:

#. :math:`D = (10^2 + 8^2 - 15^2-10^2)/(2*15*10) = -0.53667`

#. :math:`\theta_2 = \tan^{-1}(-\sqrt{1-(-0.53667)^2}/(-0.53667))\approx -2.137278`

#. :math:`\theta_1 = \tan^{-1}(8/10)-\tan^{-1}[(10\sin(-2.137278))/(15+ 10\cos(-2.137278))] \approx 1.394087`

| Check the answer:
| :math:`x = 10*\cos(1.394087-2.137278) + 15*\cos(1.394087) = 10.000`
| :math:`y = 10*\sin(1.394087-2.137278) + 15*\sin(1.394087) = 8.000`

The Python code to do the computations is

::

    In [1]: from math import *
    In [2]: a1,a2 = 15.0,10.0
    In [3]: x,y = 10.0,8.0
    In [4]: d =  (x*x+y*y-a1*a1-a2*a2)/(2*a1*a2)
    In [5]: print d
    -0.536666666667

    In [6]: t2 = atan2(-sqrt(1.0-d*d),d)
    In [7]: t1 = atan2(y,x) - atan2(a2*sin(t2),a1+a2*cos(t2))
    In [8]: print t1,t2
    1.39408671883 -2.13727804092

    In [9]: x1 = a2*cos(t1+t2) + a1*cos(t1)
    In [10]: y1 = a2*sin(t1+t2) + a1*sin(t1)
    In [11]: print x1, y1
    10.0 8.0

[Be careful with Python 2, don’t forget to include the “.0”s.]

Dual Two Link Parallel Manipulator
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The Delta configuration is not just found in *Pick and Place* machines
but has also become popular with the 3D printing community. This style
of printer is fast and accurate. Just to get started, we look at a two
dimensional analog shown in
:numref:`Fig:paralleltwolink`. The top (red)
is fixed and is of length :math:`L_0`. The two links on either side
shown in dark blue are connected by servos (in green). These links are
of length :math:`L_1`. The angles are measured from the dotted line (as
0 degrees) to straight down (90 degrees), see
:numref:`Fig:paralleltwolink2`. At the
other end of the dark blue links is a free rotational joint (pivot).
That connects the two light blue links which are joined together at the
bottom with a rotational joint.

.. Owned by Roboscience

.. _`Fig:paralleltwolink`:
.. figure:: TermsFigures/2dDelta.*
   :width: 30%
   :align: center

   Parallel Two Link Manipulator.

Unlike the previous two link manipulator, it is not completely obvious
what the workspace looks like (although you might guess something
elliptical). The configuration space is the space of all possible
angles. This is limited by the red base in theory and by the servos in
practice. Since 360\ :math:`^\circ` motion for the servos is not
possible, the configuration space is a simple square
:math:`[\theta_m , \theta_M]^2` where :math:`\theta_m`, :math:`\theta_M`
are the minimum and maximum servo angles respectively.

Define the coordinate system as :math:`x` is positive right and
:math:`y` is positive down. The origin is placed in the center of the
red base link. The question is to figure out the position of the end
effector at :math:`(x,y)` as a function of :math:`\theta_1` and
:math:`\theta_2` with fixed link lengths :math:`L_0`, :math:`L_1`,
:math:`L_2`,
Figure :numref:`Fig:paralleltwolink2`. As with the
serial chain manipulator, this is an exercise in trigonometry.

.. Owned by Roboscience

.. _`Fig:paralleltwolink2`:
.. figure:: TermsFigures/2dDeltaCombined.*
   :width: 80%
   :align: center

   Parallel Two Link (a) configuration space (b) with coordinates


The forward kinematics will provide :math:`(x,y)` as a function of
:math:`(\theta_1, \theta_2)`. The derivation is left as an exercise and
so the point :math:`(x,y)` is given by

.. math::
   :label: paralleltwolinkforward

   (x,y) = \left( \frac{a+c}{2} + \frac{v (b-d)}{u} , \frac{b+d}{2} + \frac{v (c-a)}{u} \right)

Where

.. math:: (a,b) = (-L_1 \cos(\theta_1) - L_0/2 , L_1 \sin(\theta_1) )

.. math:: (c,d) = (L_1 \cos(\theta_2) + L_0/2 , L_1 \sin(\theta_2) )

and :math:`u = \sqrt{(a-c)^2 + (b-d)^2}`,
:math:`v  = \sqrt{L_2^2 - u^2/4}`.

If you guessed that the workspace was an ellipse like the author did,
that would be wrong. If you guessed some type of warped rectangle, then
you have great intuition.
:numref:`Fig:paralleltwolinkWS` shows the
workspace for the configuration domain :math:`[0, \pi/2]^2`. The figure
graphs :math:`y` positive going upwards and for the manipulator
:math:`y` positive goes down (so a vertical flip is required to match
up). The workspace can be created by running a program that traces out
all the possible arm angles and plots the resulting end effector
position (not all points, but a dense sample of points will do just fine).
Sample code to plot this workspace is given in :numref:`lst:computeconfigdomain`. It
uses a double loop over :math:`\theta_1` and :math:`\theta_2`, which places
these values in the forward kinematics and then gathers the resulting
:math:`(x,y)` values. Like the serial manipulator, this is a holonomic
robot as well.

.. _`lst:computeconfigdomain`:
.. code-block:: python
   :caption: Configuration Domain Code

    from math import *
    import matplotlib.pyplot as plt

    # Set the link lengths
    L0 = 8
    L1 = 5
    L2 = 10

    # Initialize the arrays
    xlist = []
    ylist = []

    # Loop over the two angles,
    #  stepping about 1.8 degrees each step
    for i in range(100):
        for j in range(100):
            th1 = 0 + 1.57*i/100.0
            th2 = 0 + 1.57*j/100.0

            a = -L1*cos(th1) - L0/2.0
            b = L1*sin(th1)
            c = L1*cos(th2) + L0/2.0
            d = L1*sin(th2)

            dx = c-a
            dy = b-d
            u = sqrt(dx*dx+dy*dy)
            v = sqrt(L2*L2 - 0.25*u*u)

            x = (a+c)/2.0 + v*dy/u
            y = (b+d)/2.0 + v*dx/u

            xlist.append(x)
            ylist.append(y)

    plt.plot(xlist,ylist, 'b.')
    plt.show()

.. Owned by Roboscience

.. _`Fig:paralleltwolinkWS`:
.. figure:: TermsFigures/2dDeltaWS.*
   :width: 40%
   :align: center

   Parallel Two Link Workspace

The inverse kinematics will give you :math:`(\theta_1, \theta_2)` as a
function of :math:`(x,y)`. This is another exercise in trigonometry. For
:math:`(x,y)` given, we obtain

.. math::
   :label: paralleltwolinkIK

   \theta_1  = \pi - \beta - \eta , \quad \quad \theta_2 = \pi - \alpha - \gamma

where

.. math:: \| G \| = \sqrt{(x-L_0/2)^2 + y^2},  \quad\quad \| H\| = \sqrt{(x+L_0/2)^2 + y^2}

.. math:: \alpha = \cos^{-1} \frac{G^2 + L_0^2 - H^2 }{2GL_0}, \quad \quad \beta = \cos^{-1} \frac{H^2 + L_0^2 - G^2 }{2HL_0}

.. math:: \gamma = \cos^{-1} \frac{G^2 + L_1^2 - L_2^2 }{2GL_1},\quad \quad \eta =  \cos^{-1} \frac{H^2 + L_1^2 - L_2^2 }{2HL_1}

:numref:`lst:IKParallelTwoLink` illustrates using the inverse kinematic formulas
for a specific pair of :math:`(x,y)` values.


.. code-block:: python
   :caption: Inverse Kinematics Code for Parallel Two Link
   :name: lst:IKParallelTwoLink


    from math import *
    # Set the link lengths and starting location
    L0 = 8
    L1 = 5
    L2 = 10
    x = 0.2
    y = 0.1*x + 10

    # Compute IK
    G = sqrt((x-L0/2.0)*(x-L0/2.0)+y*y)
    H = sqrt((x+L0/2.0)*(x+L0/2.0)+y*y)

    alpha = acos((G*G + L0*L0 - H*H)/(2.0*G*L0))
    beta = acos((H*H + L0*L0 - G*G)/(2.0*H*L0))
    gamma = acos((G*G + L1*L1 - L2*L2)/(2.0*G*L1))
    eta = acos((H*H + L1*L1 - L2*L2)/(2.0*H*L1))

    th1 = pi - beta - eta
    th2 = pi - alpha - gamma

    print th1, th2

If we want to convert a list of :math:`(x,y)` points like we saw in
previous examples, we needed to embedd our code into a loop. Using NumPy
and SciPy one can leverage existing code considerably, :numref:`lst:IKParallelTwoLinkNP`.
The scalar
(single) operations can be made into array operations (a type of
iterator) with little change in the code. The normal arithmetic
operators are overloaded and the iteration is done elementwise. Although
Python is normally much slower than a C equivalent, numpy is highly
optimized and the code runs close to the speed of C. [#f1]_


.. code-block:: python
   :caption: Inverse Kinematics Code for Parallel Two Link Using Numpy
   :name: lst:IKParallelTwoLinkNP


    import numpy as np
    from math import *
    # Set the link lengths and
    L0 = 8
    L1 = 5
    L2 = 10
    x = np.arange(-3, 3, 0.2)
    y = 0.1*x + 10

    # Work out the IK
    G = np.sqrt((x-L0/2.0)*(x-L0/2.0)+y*y)
    H = np.sqrt((x+L0/2.0)*(x+L0/2.0)+y*y)

    alpha = np.arccos((G*G + L0*L0 - H*H)/(2.0*G*L0))
    beta = np.arccos((H*H + L0*L0 - G*G)/(2.0*H*L0))
    gamma = np.arccos((G*G + L1*L1 - L2*L2)/(2.0*G*L1))
    eta = np.arccos((H*H + L1*L1 - L2*L2)/(2.0*H*L1))

    th1 = pi - beta - eta
    th2 = pi - alpha - gamma

    print th1, th2

The command np.arange generates a range of values starting at -3, ending
at 3 and stepping 0.2. The x array can be manipulated with simple
expressions to yield the y array (four function operator expressions
acting pointwise on the arrays). To gain functions that act pointwise on
the numpy arrays, you need to call them from the numpy library such as
np.sqrt. Very little modification is required to get array operations in
Python. To see how this works, comment out all of the previous code
block. Then uncomment and run (adding a print statement to see the
variable values) line by line. SciPy is very powerful and we will use
many more features in later chapters.

Mobile Disk Robot
~~~~~~~~~~~~~~~~~

The next example a simple mobile ground robot in the shape of a small
disk. The workspace is region in 2D for which the devices can operate,
meaning “drive to”. Mobile robots are not rooted to some point (an
origin) through a chain of links. This is simply not the case for mobile
systems. Configuration space is then taken to be the orientation and
location of the robot. This turns out to be a complicated problem.
Clearly it depends on the underlying drive system. We will later study a
drive system known as :index:`differential drive`. Using differential drive, we
are able to move to any position for which there is a sufficiently clear
path (to be explained below). The differential drive can rotate in place
to orientation is not a problem. The freedom to orient in place is not
something found in automobiles which tend to have a smaller
configuration space than differential drive systems.

For this example, we assume we have something like the differential
drive robot. Assume that you have a simple mobile robot with two driven
wheels and a third free unpowered wheel which can easily pivot or slide,
:numref:`fig:ddriverectangular` or :numref:`fig:ddrivecircular`.
The drive wheels are not
steered but can be spun at different rates which will steer the robot.
This system is known as differential drive and is roughly analogous to
how a tank drive operates. It is necessary to develop equations of
motions for two reasons. The first reason is for simulating the dynamics
or motion of the robot so we can see the results of our robot control
software. The second reason is that the equations will be required in
localization algorithms.

.. _`fig:ddriverectangular`:
.. figure:: TermsFigures/ddrive.*
   :width: 70%
   :align: center

   Rectangular frame.

.. _`fig:ddrivecircular`:
.. figure:: TermsFigures/circular.*
   :width: 50%
   :align: center

   Circular frame.

Reference Frames
^^^^^^^^^^^^^^^^

There are two frames of reference that are used. The coordinate system
used in the environment (without a robot around) is known as the global
or inertial reference frame. It is the predetermined coordinate system
that everyone will use. It is also a static coordinate system which we
assume does not change. In a simulation, we normally take :math:`x` to
be along the horizontal direction with respect to the screen. The
coordinate :math:`y` is taken as the vertical screen direction and
:math:`z` points out of the screen. If we are working with actual
robots, then it is whatever the coordinate system that exists in the
area.

The other coordinate system used is one relative to the robot and is
known as the :index:`local coordinate system`. You can think of it as a mini
coordinate system for an ant living on the robot. We will use the
convention that :math:`x` points forward or in the direction of travel.
:math:`y` is set along the wheel axle and :math:`z` is in the vertical
direction. To remove any ambiguity, we assume that :math:`x`, :math:`y`,
:math:`z` also follow a right hand rule (which in this case sets the
direction of :math:`y`).

.. _`refframe`:
.. figure:: TermsFigures/frames.*
   :width: 55%
   :align: center

   The global and local frames of reference.

The :index:`global coordinate system` already has an origin defined. However, we
can choose the local frame origin. Our choice to simplify the
mathematics by using the center of rotation of the vehicle. Thus when
the robot rotates, the origin of the local coordinate system remains
fixed. For planar motion, we don’t really need to track :math:`z`
movement so we will drop :math:`z` for now. The global or inertial basis
will be identified as :math:`X_I`, :math:`Y_I`, and the local or
relative basis will be identified as :math:`X_R`, :math:`Y_R`.

Any point in the plane can be represented in either coordinate system.
So a particular point :math:`p` can have coordinates :math:`(x_I,y_I)`
and :math:`(x_R, y_R)`. How do these relate? In two dimensions, a
coordinate system can be translated and rotated relative to another. We
can write this translation as the displacement of one origin to another
or in our case, we can just use the location of the robot (local frame)
origin relative to the :index:`global frame`. In other words, the :index:`local frame`
origin position is :math:`(x_I,y_I)`. We then need to track the
orientation of the frame or in our case the robot. The angle,
:math:`\theta`, can be measure from either coordinate system and to be
consistent, we take it as the angle from the global frame to the local
frame. Graphically :math:`\theta` the amount of rotation applied to
:math:`X_I` to line it up with :math:`X_R`.

.. Owned by Roboscience

.. _`refddframe`:
.. figure:: TermsFigures/ddframe.*
   :width: 55%
   :align: center

   The two frames of reference for a mobile robot: the inertial or
   global frame and the relative or local frame.

We can track the robot position by tracking its coordinate system origin
and orientation relative to the global coordinate system, :math:`\xi_I`.
So, we define the object relative to the robot by coordinates
:math:`\xi_R` and rotate into the inertial frame:

.. math:: \xi_I = \begin{pmatrix} x \\ y \\ \theta \end{pmatrix}, \quad \xi_R= \begin{pmatrix} x' \\ y' \\ 0 \end{pmatrix}.

The movement of the robot traces a path, :math:`x(t)`, :math:`y(t)`,
in the global coordinate system which is our motion in the environment
or in the simulation window. It is possible to track this motion through
information obtained in the local frame. In order to do this, we need a
formula to relate global and local frames. Using the standard tools from
Linear Algebra, the relation is done through translation and rotation
matrices. The rotation matrix:

.. math::

   R(\theta) = \begin{bmatrix} \cos \theta & -\sin \theta & 0 \\ \sin \theta &
   \cos \theta & 0 \\
                      0 & 0 & 1
                 \end{bmatrix} .

For example, a 45 degree rotation, :math:`\theta = 45^\circ`, produces a
rotation matrix

.. math::

   R(\theta) =\begin{bmatrix} \sqrt{2}/2 &
   -\sqrt{2}/2 & 0 \\ \sqrt{2}/2 & \sqrt{2}/2 & 0 \\
                      0 & 0 & 1
                 \end{bmatrix}.

The relation depends on the orientation of the robot which changes as
the robot navigates in the plane. However, at a snapshot in time, the
robot does have an orientation so, we can relate orientation at an
instantaneous time:

.. math:: \dot{\xi_I} = R(\theta) \dot{\xi_R}.

We can undo the rotation easily. Since :math:`R` is an orthogonal
matrix, the inverse is easy to
compute, :cite:`strang:1988:linalg`

.. math::

   R(\theta)^{-1} = \begin{bmatrix} \cos \theta & \sin \theta & 0 \\ -\sin \theta
   & \cos \theta & 0 \\
                      0 & 0 & 1
                 \end{bmatrix}

You may have noted that we are not working with a translation. This is
not required for the instantaneous coordinates because the derivative
removes the translation. 


Equations of Motion
^^^^^^^^^^^^^^^^^^^

Working in instantaneous local coordinate enables us to determine the
motion easily. We then use the rotation matrix to relate the robot
position in the global frame. To progress in the modeling process, we
need to know the specifics of the robot, illustrated in
:numref:`robotdimensions`.

-  Wheel size: :math:`D`, so the radius :math:`r = D/2`

-  Axle length: :math:`2L` (:math:`L` is the distance from the origin of
   the coordinate system to a wheel)

-  Origin of local coordinate system: :math:`P` is placed on the
   midpoint of the axle.

.. Owned by Roboscience

.. _`robotdimensions`:
.. figure:: TermsFigures/dddim.*
   :width: 25%
   :align: center

   Robot Dimensions.

Recall that the goal was to compute the motion of the robot based on the
rotational speed of the wheels. Let :math:`\dot{\phi_1}` and
:math:`\dot{\phi_2}` be the right and left wheel rotational speeds
(respectively). Note: :math:`\phi` is an angle and measured in radians,
:math:`\dot{\phi}` is measured in radians per unit time, and
:math:`\dot{\phi}/2\pi` is the “rpm” (or rps, etc).

Next we determine the contribution of each wheel to linear forward
motion. The relation between linear and angular velocities gives us for
the right wheel :math:`\dot{x_1} = r\dot{\phi_1}` and for the left
wheel: :math:`\dot{x_2} = r\dot{\phi_2}`,
:numref:`axlevelocity`. The differential speeds
then produce the rotational motion about the robot center and the
average forward velocity.

.. Owned by Roboscience

.. _`axlevelocity`:
.. figure:: TermsFigures/ddaxle.*
   :width: 70%
   :align: center

   Velocity of axle induced by wheel velocities.



The speed of point :math:`P` is given by the weighted average based on
distances of the wheels to :math:`P`. To see this, we consider a couple
of cases. If the two wheel velocities are the same, then the average
works trivially. If the two velocities are different (but constant),
then the motion of the robot is a circle.
:numref:`axlevelocity` shows the robot motion.
Assuming the outer circle radius is :math:`\rho + 2L` with velocity
:math:`r\dot{\phi}_1` and the inner circle is radius :math:`\rho` with
wheel velocity :math:`r\dot{\phi}_2`, we have that the motion of a
similar wheel at point :math:`P` would be:

.. math:: \displaystyle \frac{\dot{\phi}_2}{\rho} = \frac{\dot{\phi}_1}{\rho +2L} =  \frac{\dot{\phi}_P}{\rho +L} .

Solving for :math:`\rho` with the left two terms:
:math:`\rho  = 2L\dot{\phi}_2/ (\dot{\phi}_1 - \dot{\phi}_2)`. Using the
outer two terms, plug in for this value of :math:`\rho`:

.. math::

   \displaystyle \frac{\dot{\phi}_2}{2L\dot{\phi}_2/ (\dot{\phi}_1 - \dot{\phi}_2)} =  \frac{\dot{\phi}_P}{2L\dot{\phi}_2/ (\dot{\phi}_1 - \dot{\phi}_2)+L}  \Rightarrow
   \displaystyle \frac{\dot{\phi}_1 + \dot{\phi}_2}{2}=  \dot{\phi}_P

This velocity is in the direction of :math:`x_R`.

.. math::

   \dot{x_R} = r\dot{\phi}_P =
   \frac{r}{2} (\dot{\phi_1} + \dot{\phi_2})

For this example, there is no motion parallel to the axle so
:math:`\dot{y_R} = 0`.

Each wheel will act like a lever arm rotating the craft as well as
moving it forward. To determine the amount of rotation, we examine the
contribution of the wheels separately. For example, if the right wheel
moves faster than the left wheel, then we have positive rotation of the
vehicle. The contribution from the right wheel is
:math:`2L\dot{\theta} = r\dot{\phi_1}` or
:math:`\dot{\theta} = r\dot{\phi_1}/(2L)` and the contribution from the
left wheel is :math:`2L\dot{\theta} = -r\dot{\phi_2}` or
:math:`\dot{\theta} = -r\dot{\phi_2}/(2L)`, see
Figure :numref:`diffdriverotation`. The rotation
about :math:`P` is given by adding the individual contributions:

.. math:: \dot{\theta} =  \frac{r}{2L} (\dot{\phi_1} - \dot{\phi_2}).

.. Owned by Roboscience

.. _`diffdriverotation`:
.. figure:: TermsFigures/ddaxlerot.*
   :width: 20%
   :align: center

   The contribution of the two wheels towards rotational
   motion.

In local or robot coordinates we obtain the following equations of
motion

.. math::

   \begin{array}{l}
   \dot{x_R} = \frac{r}{2} (\dot{\phi_1} + \dot{\phi_2}),\\[2mm]
   \dot{y_R} = 0,\\[2mm]
   \dot{\theta} =  \frac{r}{2L} (\dot{\phi_1} - \dot{\phi_2}).
   \end{array}

To get the model in global (or inertial) coordinates we must apply the
transformation (the rotation) to our local coordinate model. This is
done by applying the rotation matrix :math:`R` to the position vector
:math:`\dot{\xi}_R`:

.. math::

   \dot{\xi}_I = R(\theta) \dot{\xi}_R = R(\theta) \begin{bmatrix} \frac{r}{2}
   (\dot{\phi_1}+\dot{\phi_2})\\
   0 \\ \frac{r}{2L} (\dot{\phi_1}-\dot{\phi_2})\end{bmatrix}

.. math::

   = \begin{bmatrix} \cos \theta & -\sin \theta & 0 \\ \sin \theta & \cos \theta
   & 0 \\
                      0 & 0 & 1
                 \end{bmatrix} \begin{bmatrix} \frac{r}{2}
   (\dot{\phi_1}+\dot{\phi_2})\\
   0 \\ \frac{r}{2L} (\dot{\phi_1}-\dot{\phi_2})\end{bmatrix}
   = \begin{bmatrix} \frac{r}{2} (\dot{\phi_1}+\dot{\phi_2})\cos(\theta) \\
   \frac{r}{2} (\dot{\phi_1}+\dot{\phi_2})\sin(\theta)\\
      \frac{r}{2L} (\dot{\phi_1}-\dot{\phi_2})
     \end{bmatrix}

This leads to the following equations of motion in the global reference
frame:

.. math::
   :label: ddkinematicsmodel

   \boxed{
   \begin{array}{l}
   \dot{x} = \frac{r}{2} (\dot{\phi_1}+\dot{\phi_2})\cos(\theta) \\[4mm]
   \dot{y} = \frac{r}{2} (\dot{\phi_1}+\dot{\phi_2})\sin(\theta) \\[4mm]
   \dot{\theta} = \frac{r}{2L} (\dot{\phi_1}-\dot{\phi_2})
   \end{array}}

These are non-holonomic constraints, this is left as a homework exercise.


Assume that you have a differential drive robot. If the drive wheel is
20cm in diameter and turns at 10 rpm (revolutions per minute), what is
the linear speed of the rolling wheel (with no slip or skid)?

We see that distance covered :math:`s = \theta r`

and so :math:`v = ds/dt = r d\theta /dt`. Note that
:math:`d\theta/dt = 2\pi \omega`, where :math:`\omega` is the rpm. So

.. math:: v = 2\pi r \omega = 2\pi *10*10=200\pi.

Let the distance between the wheels be 30cm (axle length). If the right
wheel is turning at 10 rpm (revolutions per minute) and the left is
turning at 10.5 rpm, find a formula for the resulting motion.

As stated earlier, the motion for this robot would be a circle. Thus the
two wheels trace out two concentric
circles :numref:`fig:ddrivecircles`.  The two circles
must be traced out in the same amount of time:

.. math::

   t = \frac{d_1}{v_1} = \frac{d_2}{v_2} \Rightarrow \frac{d_1}{10.5*20\pi} =
   \frac{d_2}{10*20\pi}
   \Rightarrow \frac{2\pi(R+30)}{210\pi} = \frac{2\pi R}{200\pi}

.. math::

   \frac{30}{105} = R\left( \frac{1}{100} - \frac{1}{105}\right) =
   \frac{5R}{100*105}

.. math:: \Rightarrow R = \frac{100*105}{5} \frac{30}{105} = 600

Thus we have :math:`x^2 + y^2 = 600^2` as the basic formula for the
curve of motion.

.. _`fig:ddrivecircles`:
.. figure:: TermsFigures/ddrive_circle.*
   :width: 70%
   :align: center

   A differential drive robot with constant wheel
   velocity drives in straight lines and circles.

| Solve these equations for the given values of
  :math:`\omega_1=\dot{\phi_1}` and :math:`\omega_2=\dot{\phi_2}` below.
  Assume that the wheels are 18cm in diameter and L is 12cm. Find an
  analytic solution and compute the position of the robot starting at
  t=0, x=0, y=0, theta=0, after the following sequence of moves:

+----------------------+--------------------------------------+
| :math:`t=0  \to 5`:  | :math:`\omega_1 = \omega_2 = 3.0`,   |
+----------------------+--------------------------------------+
| :math:`t=5  \to 6`:  | :math:`\omega_1 = - \omega_2 = 2.0`, |
+----------------------+--------------------------------------+
| :math:`t=6  \to 10`: | :math:`\omega_1 = \omega_2 = 3.0`,   |
+----------------------+--------------------------------------+
| :math:`t=10 \to 11`: | :math:`\omega_1 = -\omega_2 = -2.0`, |
+----------------------+--------------------------------------+
| :math:`t=11 \to 16`: | :math:`\omega_1 =  \omega_2 = 3.0`,  |
+----------------------+--------------------------------------+

| Begin at :math:`(x,y,\theta) =(0,0,0)`

+------------------------+--------------------------------------------------------------------------------------------------------------------------------------------+
| :math:`t=0  \to 5`:    | :math:`\omega_1 = \omega_2 = 3.0` , :math:`\Rightarrow` :math:`(0,0,0)+(135,0,0)=(135,0,0)`                                                |
+------------------------+--------------------------------------------------------------------------------------------------------------------------------------------+
|  :math:`t=5  \to 6`:   | :math:`\omega_1 = - \omega_2 = 2.0`, :math:`\Rightarrow` :math:`(135,0,0) + (0,0,3/2) = (1 35,0,3/2)`                                      |
+------------------------+--------------------------------------------------------------------------------------------------------------------------------------------+
|  :math:`t=6  \to 10`:  | :math:`\omega_1 = \omega_2 = 3.0`, :math:`\Rightarrow` :math:`(135,0,3/2)+(108\cos 3/2,108\sin 3/2, 0)` :math:`\approx (142.6, 107.7, 1.5)`|
+------------------------+--------------------------------------------------------------------------------------------------------------------------------------------+
| :math:`t=10 \to 11`:   | :math:`\omega_1 = -\omega_2 = -2.0`, :math:`\Rightarrow` :math:`(142.6, 107.7, 1.5)+(0, 0, -1.5) = (142.6, 107.7, 0)`                      |
+------------------------+--------------------------------------------------------------------------------------------------------------------------------------------+
| :math:`t=11 \to 16`:   | :math:`\omega_1 =  \omega_2 = 3.0`, :math:`\Rightarrow` :math:`(142.6, 107.7, 0)+(135, 0,0) =  (277.6, 107.7, 0)`                          |
+------------------------+--------------------------------------------------------------------------------------------------------------------------------------------+

You may have noticed that these equations related derivatives of the
parameters and variables. Hence these are known as differential
equations. Specifically these are nonlinear differential equations due
to the sine and cosine terms. The standard methods seen in elementary
courses such as Laplace Transforms and Eigenvector Methods do not apply
here. However, there is enough structure to exploit that one can solve
the equations in terms of the wheel rotations. So, if you know
:math:`\phi_1` and :math:`\phi_2`, you can determine position by
integration. They are used to track the position of the middle of the
robot. The derivation of these equations and the inverse kinematics will
be discussed in the motion modeling chapter. We will also hold off on
running some path computations as we did with the manipulators and show
those in the motion chapter as well. The next thing to address is the
workspace and configuration space.

The main difference for our mobile robot is that for the manipulators we
only focused on the end effector position. We tracked the single point
which was at the tip of the end effector. In real situations, however,
we may need to track the entire manipulator. Surgical robots are a fine
example. They have to operate in very narrow corridors to reduce skin
incisions. In those cases a full geometric model may be required and
constraints are placed on all of the intermediate links. For mobile
robots, this problem seems to arise often.

If the mobile robot was extremely small, like a point, it is pretty easy
to deal with. There is only the point to track, no orientation to worry
about and we don’t worry about any manipulator that got it there. A
relatively small robot that can move in any direction can be
approximated by a point robot. In this case, the workspace and
configuration spaces are identical and two dimensional. Although this
seems a bit silly to treat our robot as a point, it can be a useful
simplification when planning routes for the robot. You can also think of
this as tracking the centroid of the robot. If there is no admissible
route for the centroid, then no route exists. The computation for the
point can be much faster than the computation that includes the full
geometry.

Unfortunately, our robots do have size. A circular robot would be the
natural next step to investigate. The question is what is the effect on
the configuration and workspace. If the robot is round, has no
orientation and can move in any direction, then again the configuration
and workspaces are the same. By moving the robot around in the world and
tracking the centroid, we can determine the configuration space. Since
the middle of the robot cannot touch the obstacle boundary, the
interaction between the robot and the obstacle reduces the configuration
space as shown in
:numref:`Fig:RobotSize`, :numref:`Fig:intro-mobile1`.
In this case the size of the robot affects the configuration space,
:numref:`Fig:intro-mobile2`. For a mobile
ground robot that is not a point, orientation will enter as a variable
in the system.

.. _`Fig:RobotSize`:
.. figure:: TermsFigures/circle1.*
   :width: 60%
   :align: center

   Configuration space as a function of robot size.

For a round or disk robot with radius, :math:`r`, the center of the
robot can only get to within distance :math:`r` of an obstacle boundary.
Assume the obstacle is also round with radius, :math:`R` and is the only
one. The configuration space for the robot is all of the points that the
robot centroid can reach. This situation is the same as if the robot was
a point and the obstacle had radius :math:`R+r`. We can study the
configuration space problem by shrinking the robot to a point and
*inflating* the obstacle by the robot’s radius. This can be done for all
the obstacles in the workspace. It is clear that the obstacle does not
need to be round. Move the robot up to the place where it touches the
obstacle. Mark the robot’s center on the workspace. Do this for all
points of contact between the robot and the obstacle. This draws an
outer boundary around the obstacle and makes the obstacle larger. We
have inflated the obstacle.

.. _`Fig:intro-mobile1`:
.. figure:: TermsFigures/mobile.*
   :width: 70%
   :align: center

   Example of the inflation process.

.. _`Fig:intro-mobile2`:
.. figure:: TermsFigures/mobile2.*
   :width: 70%
   :align: center

   Relation between robot size and configuration space.


The previous examples looked at a circular robot. What about a robot
which is a rectangle? What would be the configuration space about some
obstacle? :numref:`shapematters`. The basic shape
of the robot is important as well as its orientation,
:numref:`orientationmatters`. Inflation in
this case depends on the fixed orientation of the robot. One follows the
same process and pushes the robot up until it touches the obstacle.
Doing this for all locations around the obstacle all while keeping the
same orientation will describe the configuration space. Marking the
robot’s centroid at each contact allows us to trace a curve around the
obstacle and thus inflate the obstacle. We then can shrink the robot to
a point. We can then study robot paths through the open space. Of course
in practice this is absurd since the robot orientation is not fixed. But
it does help transition to the general case.

.. _`shapematters`:
.. figure:: TermsFigures/rect.*
   :width: 70%
   :align: center

   Changing robot shape also affects c-space.

.. _`orientationmatters`:
.. figure:: TermsFigures/rect2.*
   :width: 70%
   :align: center

   Changing robot orientation affects c-space as well.

It is helpful to see some examples of the inflation process. A
rectangular object does not just change scale. It changes shape as well.
For a rectangle, he inflated obstacle is a “rectangle” with rounded
corners. It is important to note that each rotation of the rectangle
generates a new and different configuration space,
:numref:`orientationmattersalot`. This
process can be very complicated and often one will want to make
simplifications.

.. _`orientationmattersalot`:
.. figure:: TermsFigures/rect3.*
   :width:  70%
   :align: center

   Two sample rotations and the configuration
   obstacle.

Robot orientation then makes the configuration space question more
complicated since the configuration space is a function of the robot
orientation. A planning algorithm would then need to either fix the
robot orientation or be able to adjust to a changing landscape. To fix
orientation ultimately means that the orientation is independent of
travel direction. This is not the case for the vast majority of
vehicles. The orientation for a car, for example, is pointed in the
direction of travel. [#f2]_ To obtain this independence a holonomic robot
is required. The term holonomic will be carefully defined later, for
now, consider it a mobile robot that can set position and orientation
independently. Independent of the type of motion, it should be clear now
that position and orientation are separate and important variables in
the system which is addressed next.

.. rubric:: Footnotes

.. [#f1] Well, this is true on one Tuesday afternoon a long time ago with one little comparison of some loop/math code.  Your results may be very different.

.. [#f2] Under normal conditions this is true, however, icy roads will allow for much greater freedom of vehicle orientation and travel direction.
