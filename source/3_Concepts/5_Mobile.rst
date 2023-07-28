Mobile Disk Robot
-----------------

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
compute, :cite:`strang:1988:LA`

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
frame, the :index:`differential drive forward kinematics`:

.. math::
   :label: ddkinematicsmodel

   \boxed{
   \begin{array}{l}
   \dot{x} = \frac{r}{2} (\dot{\phi_1}+\dot{\phi_2})\cos(\theta) \\[4mm]
   \dot{y} = \frac{r}{2} (\dot{\phi_1}+\dot{\phi_2})\sin(\theta) \\[4mm]
   \dot{\theta} = \frac{r}{2L} (\dot{\phi_1}-\dot{\phi_2})
   \end{array}}

In Chapter 15 we will define these as non-holonomic constraints which
we will find places restrictions on the type of motion we can expect.
In this case, the position and orientation of this robot are not
independent quantities which may restrict the motions and orientations
we have when moving in obstacle dense regions.


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

What can be said about these equations? Can these be partially solved so
we can run simulations? Our first attempt is to solve the differential
equations by integration. Starting with the third equation, the one for
the angular velocity,
We can attempt to formally integrate the differential drive equations

.. math::

   \dot{\theta} =\frac{d\theta}{dt} = \frac{r}{2L} (\dot{\phi_1}-\dot{\phi_2})

integrate from 0 to t (and be careful about integration variables)

.. math::

   \int_0^t\frac{d\theta}{d\tau}\, d\tau = \int_0^t\frac{r}{2L} (\dot{\phi_1}-\dot{\phi_2})\, d\tau

and we have

.. math::

   \theta(t) - \theta(0) = \int_0^t\frac{r}{2L} (\dot{\phi_1}-\dot{\phi_2})\, d\tau

Assume that you know :math:`\phi_i(t)` (or if you know :math:`\dot{\phi}_i(t)`), then what can you say?

From :math:`\dot{\phi}_i(t)` we can compute :math:`\theta` by integrating the last equation:

.. math::

   \theta(t) = \theta(0) + \int_0^t \frac{r}{2L} \left(\frac{d\phi_1}{d\tau}-\frac{d\phi_2}{d\tau}\right)d\tau

Using this result we can write down formulas for :math:`x` and :math:`y`

.. math::

   x(t)  = x(0)+\displaystyle\int_0^t \frac{r}{2} \left(\frac{d\phi_1}{d\tau}+\frac{d\phi_2}{d\tau}\right)\cos(\theta(\tau))d\tau

.. math::

   y(t)  = y(0) + \displaystyle\int_0^t\frac{r}{2} \left(\frac{d\phi_1}{d\tau}+\frac{d\phi_2}{d\tau}\right)\sin(\theta(\tau))d\tau


These equations are easy to integrate if you know the wheel velocities are constants.  First integrate the :math:`\theta` equation:

.. math:: \theta(t) = (r/2L)(\omega_1 - \omega_2)t + \theta_0

This can be plugged into the x and y equations and then integrated:

.. math:: x(t) = \frac{L(\omega_1 + \omega_2)}{(\omega_1 - \omega_2)} \left[\sin((r/2L)(\omega_1 - \omega_2)t + \theta_0) - \sin(\theta_0)\right] + x(0)

.. math:: y(t) = -\frac{L(\omega_1 + \omega_2)}{(\omega_1 - \omega_2)} \left[\cos((r/2L)(\omega_1 - \omega_2)t + \theta_0) - \cos(\theta_0)\right]+ y(0)



From these solutions (and from the differential equations as well), you can see that there is a problem when :math:`\omega_1=\omega_2`
or when :math:`\omega_1 = -\omega_2`.  These are exactly the cases we are often given.
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


So, you can just
work out the solution from rate-time-distance formulas.
As long as you have piecewise constant angular velocities on the wheels,
you have the robot path made up from circular arcs. A simulation program
can connect these up to produce a path for any sequence of wheel
velocities. The path is made up of combinations of lines and arcs. Note
that a pivot in place is possible so the resulting path need not be
differentiable.
:numref:`fig:piecewisecirculararcs`
shows a sample path.

.. _`fig:piecewisecirculararcs`:
.. figure:: TermsFigures/piecewisecircular.*
   :width: 50%
   :align: center

   Piecewise circular/linear arc paths



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
| :math:`t=0   \to 5`:   | :math:`\omega_1 = \omega_2 = 3.0` ,  :math:`\Rightarrow` :math:`(0,0,0)+(135,0,0)=(135,0,0)`                                               |
+------------------------+--------------------------------------------------------------------------------------------------------------------------------------------+
| :math:`t=5   \to 6`:   | :math:`\omega_1 = - \omega_2 = 2.0`, :math:`\Rightarrow` :math:`(135,0,0) + (0,0,3/2) = (1 35,0,3/2)`                                      |
+------------------------+--------------------------------------------------------------------------------------------------------------------------------------------+
| :math:`t=6  \to 10`:   | :math:`\omega_1 = \omega_2 = 3.0`, :math:`\Rightarrow` :math:`(135,0,3/2)+(108\cos 3/2,108\sin 3/2, 0)` :math:`\approx (142.6, 107.7, 1.5)`|
+------------------------+--------------------------------------------------------------------------------------------------------------------------------------------+
| :math:`t=10 \to 11`:   | :math:`\omega_1 = -\omega_2 = -2.0`, :math:`\Rightarrow` :math:`(142.6, 107.7, 1.5)+(0, 0, -1.5) = (142.6, 107.7, 0)`                      |
+------------------------+--------------------------------------------------------------------------------------------------------------------------------------------+
| :math:`t=11 \to 16`:   | :math:`\omega_1 =  \omega_2 = 3.0`,  :math:`\Rightarrow` :math:`(142.6, 107.7, 0)+(135, 0,0) =  (277.6, 107.7, 0)`                         |
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
robot.


:index:`Differential Drive Inverse Kinematics`
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

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
.. figure:: TermsFigures/tantheta.*
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
we use *+* in :eq:`inverseddequations`. Plugging the
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





Configuration and Workspace
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The next thing to address is the
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

.. [#f2] Under normal conditions this is true, however, icy roads will allow for much greater freedom of vehicle orientation and travel direction.
