Potential Functions
-------------------

A potential function is a differentiable real-valued function
:math:`U: {\Bbb R}^m \to {\Bbb R}`. We may think of it as an energy and thus the
gradient is a force. The gradient

.. math::

   \nabla U(q) = \begin{bmatrix}\displaystyle \frac{\partial U}{\partial q_1} \\[4mm]
   \displaystyle \frac{\partial U}{\partial q_2} \\[2mm] ... \\[2mm]
   \displaystyle \frac{\partial U}{\partial q_n} \end{bmatrix} = \vec{F}

The gradients can be used to act on the robots like forces do on charged
particles.

The vector fields (gradients) may be used to pull a robot to a
particular goal or push a robot away from an obstacle.


.. figure:: PlanningFigures/gradient_navigation.*
   :width: 40%
   :align: center

   Potential function navigation.

Vectors are seen as velocity not forces so this is a first order system.

The robot can move downhill using gradient descent:

.. math:: \dot{c}(t) = -\nabla U(c(t)),

.. math:: \displaystyle \frac{dx}{dt} = -\frac{\partial U}{\partial x}

.. math:: \displaystyle \frac{dy}{dt} = -\frac{\partial U}{\partial y}

:math:`\nabla U(q)`, :math:`q_\text{start}` Sequence
:math:`q_1, q_2, q_3, \dots , q_n` :math:`q(0)=q_\text{start}`
:math:`i=0` :math:`q(i+1) = q(i) - \alpha (i) \nabla U(q(i))`
:math:`i++`

It will stop when it reaches a critical point, :math:`q^*`:
:math:`\nabla U(q^*)=0.` This point is a maximum, minimum or saddle
point. It depends on the eigenvalues of the Hessian

.. math::

   H(U) = \begin{bmatrix}
             \displaystyle\frac{\partial^2 U}{\partial q_1^2} & \dots & \displaystyle\frac{\partial^2 U}{\partial q_1\partial q_n}\\[5mm]
             \displaystyle \vdots & \ddots & \vdots\\[5mm]
             \displaystyle\frac{\partial^2 U}{\partial q_n\partial q_1}  & \dots & \displaystyle\frac{\partial^2 U}{\partial q_n^2}\\[5mm]
            \end{bmatrix}

The Hessian is symmetric so the eigenvalues are real. Thus we get:

.. figure:: PlanningFigures/gradient_figures.png
   :width: 90%
   :align: center

Example Potential Functions
~~~~~~~~~~~~~~~~~~~~~~~~~~~


Provide an example of an attractive potential function to the point
(5,6).

.. math:: U_a = (x-5)^2 +(y-6)^2

The gradient can be found: :math:`\nabla U =<2x-10, 2y-12>`. Does
:math:`-\nabla U` point to (5,6)? Pick a random point, (2,3). The vector
from (2,3) to (5,6) is :math:`<3,3>`. The negative of the gradient,
:math:`-\nabla U` at (2,3) is :math:`<6,6>` which is :math:`2<3,3>`
which works for this point and is easy to show in general. The graph is
shown in :numref:`example2potential_a`.

Next we write down a repulsive potential function for an ellipse. The
general equation of an ellipse is :math:`(x-h)^2/a^2 + (y-k)^2/b^2 = 1`,
and for this example we will select :math:`a=1`, :math:`b=2`,
:math:`h=3`, :math:`k=4`. A repulsive function would be one that the
gradient points away from.



An example of a repulsive potential:

.. math:: U_r = \frac{1}{ (x-3)^2 + (y-4)^2/4 - 1}

The graph of this function is shown in  :numref:`example2potential_b`.

.. _`example2potential_a`:
.. figure:: PlanningFigures/potential1.*
   :width: 40%
   :align: center

   Attractive potential function.

.. _`example2potential_b`:
.. figure:: PlanningFigures/potential2.*
   :width: 40%
   :align: center

   Repulsive potential function.


Constructing Potentials
~~~~~~~~~~~~~~~~~~~~~~~

As suggested above, we will construct the potential functions from two
basic types (:numref:`example2potential_a`, :numref:`example2potential_b`):

-  Attractive Potential, denoted by :math:`U_\text{att}(q)`, and

-  Repulsive Potentials, denoted by :math:`U_\text{rep}(q)`.

The full potential function will then be a combination of the two basic
types. We will begin by just summing the potentials. This is the easiest
approach but as you will see does not scale to multiple objects
effectively. Using just addition, simple potential functions may be
constructed from these:

.. math:: U(q) = U_\text{att}(q) + U_\text{rep}(q)

And more complicated functions may be constructed via

.. math:: U(q) = U_\text{att}(q) + \sum_i U_{\text{rep}\, i}(q)

We also assume that the outer boundary is not critical and so we ignore
outer boundary effects. Later we will be able to include the boundary.

**Attractive Potential** A very simple function to use for the
attractive potential is

.. math:: U_\text{att} = k_0\left[(x-x_0)^2 + (y-y_0)^2\right]

where :math:`(x_0, y_0)` is the location of the goal. The value
:math:`k_0` selects how steep the function walls are and thus changes
the magnitude of the resulting gradient. This is the force pushing the
object to the goal. We will balance :math:`k_0` with the constants of
the other functions to gain an effective potential function surface. See
:numref:`example2potential_a`.

**Repulsive Potential** A single repulsive potential can be formed by
modifying the attractive potential. Assume that you can enclose the
obstacle in a circle :math:`(x-x_0)^2 + (y-y_0)^2 = r^2` for some radius
:math:`r` and some center :math:`(x_0,y_0)`. Let

.. math:: \rho = (x-x_0)^2 + (y-y_0)^2 - r^2

The function :math:`\rho` is zero on the boundary of the circle and is
positive outside the circle. It is a paraboloid that opens up. Then the
repulsive potential is can be formed from :math:`\gamma/\rho` or

.. math:: U_\text{rep} = \frac{\gamma}{(x-x_0)^2 + (y-y_0)^2 - r^2}

:math:`U_\text{rep}` is a function that goes to infinity at you
approach the circle. See :numref:`example2potential_b`. The term
:math:`\gamma` is the strength of the field. It is a parameter which can
be varied to adjust the relative force exerted by the repulsive field.
It can shape the robot path an sometimes avoid local extremals.

Some authors like to shut down the repulsive potential by subtracting
off a constant so it is zero outside a larger circle:

.. math::

   U_\text{rep} = \left\{
   \begin{array}{ll}
   \displaystyle \frac{\gamma}{(x-x_0)^2 + (y-y_0)^2 - r^2} - \sigma & \text{for  } \rho < \frac{\gamma}{\sigma}\\[3mm]
   0 &  \text{for  } \rho \geq \frac{\gamma}{\sigma}.
   \end{array} \right.

Because there are quadratic functions involved, the growth can
excessive. One way to deal with large values is to use conic potentials
instead of quadratic potentials. Let :math:`q=(x,y)` and
:math:`q_\text{goal} = (x,y)_\text{goal}`. The conic potential:

.. math:: U_\text{att} = \gamma d(q, q_\text{goal})

The gradient is then

.. math:: \nabla U(q) = \frac{\gamma}{d(q, q_\text{goal})} (q-q_\text{goal})

This presents numerical issues due to the discontinuity, so normally one
uses :math:`U(q) = \gamma d^2(q, q_\text{goal})`

.. math:: \nabla U(q) = \gamma (q-q_\text{goal})

Velocity is too large far away and will overwhelm other fields. We use a
linear velocity for far field and quadratic velocity for near field. The
switch over point is at distance :math:`d^*_\text{goal}`:

.. math::

   U_\text{att}(q) = \left\{ \begin{array}{ll} (1/2)\gamma d^2(q, q_\text{goal}), & d(q, q_\text{goal})\leq d^*_\text{goal},\\[3mm]
   d^*_\text{goal}\gamma d(q, q_\text{goal}) - (1/2)\gamma (d^*_\text{goal})^2, & d(q, q_\text{goal})> d^*_\text{goal},
   \end{array}\right.

which gives

.. math::

   \nabla U_\text{att}(q) = \left\{ \begin{array}{ll} \gamma (q -q_\text{goal}), & d(q, q_\text{goal})\leq d^*_\text{goal},\\[3mm]
   d^*_\text{goal}\gamma \frac{(q -q_\text{goal})}{d(q, q_\text{goal})}, & d(q, q_\text{goal})> d^*_\text{goal},
   \end{array}\right.

The repulsive potential is the same as the one above. We rewrite the
expression in slightly different notation where the
:math:`\gamma/\sigma` term is replaced by :math:`1/Q^*` which is a
measure of distance away from the obstacle boundary. Essentially
:math:`Q^*` is the cutoff distance for when we no longer express the
repulsive potential field. The formula in the new notation is

.. math::

   U_\text{rep}(q) = \left\{ \begin{array}{ll} (1/2)\eta \left( \frac{1}{D(q)} - \frac{1}{Q^*}\right) , &
   D(q) \leq Q^*,\\[3mm]
   0, & D(q) > Q^*
   \end{array}\right.

This becomes a very complicated formula when the obstacles are no
longer circles. It is very difficult to arrive at a formula for the
closest obstacle. Finding equidistance lines is a whole issue alone. We
will address this when we discuss Voronoi decomposition.

Note that placing repulsive potentials in can change the location of the
minimum that you have setup through the attractive potential. This is
one reason we go to the trouble of placing a cutoff on the obstacle
potentials. A simple one dimensional example can demonstrate. Assume you
want your minimum to be at :math:`x=0`, so you try :math:`U_a = x^2`.
Next you place in a repulsive potential at :math:`x=5`,
:math:`U_r = |x - 5|^{-1}`. Combining we have

.. math:: U = x^2 + \frac{1}{|x - 5|}.

Compute the derivative and set to zero:

.. math:: \frac{dU}{dx} = 2x - \frac{\mbox{sign}(x-5)}{|x - 5|^2} = 0.

For :math:`x<5` we have

.. math:: \frac{dU}{dx} = 2x - \frac{1.0}{|x - 5|^2} = 0

which can solved: :math:`2x(x-5)^2 = 1` or :math:`x\approx 0.02`. No
longer at :math:`x=0`.

**Summary:**

.. math:: U(q) = U_\text{att}(q) + U_\text{rep}(q)

.. math::

   U_\text{att}(q) = \left\{ \begin{array}{ll} (1/2)\gamma d^2(q, q_\text{goal}), & d(q, q_\text{goal})\leq d^*_\text{goal},\\[3mm]
   d^*_\text{goal}\gamma d(q, q_\text{goal}) - (1/2)\gamma (d^*_\text{goal})^2, & d(q, q_\text{goal})> d^*_\text{goal},
   \end{array}\right.

.. math::

   U_\text{rep}(q) = \left\{ \begin{array}{ll} (1/2)\eta \left( \frac{1}{D(q)} - \frac{1}{Q^*}\right) , &
   D(q) \leq Q^*,\\[3mm]
   0, & D(q) > Q^*
   \end{array}\right.

The distance :math:`D(q)` can be determined from a LIDAR sweep if the
robot is located at :math:`q`.


.. figure:: PlanningFigures/range.*
   :width: 35%
   :align: center

   LIDAR Range map.

To compute the potential function, you need to know all of the
distances, not just from a single point :math:`q`.

Often the environment is represented on a grid which can simplify the
planning process in some cases. Our first step is to remove the analytic
repulsive potential and replace it with a discrete method known as the
Brushfire algorithm. This can remove the problems related to finding
repulsive potentials that don’t overwhelm the attractive potential.


.. math:: U = (x-5)^2 +(y-6)^2 +  \frac{\gamma}{ (x-3)^2 + (y-4)^2/4 - 1}

.. figure:: PlanningFigures/potential3.*
   :width: 50%
   :align: center


The equations of motion that generate the path are

.. math::

   \begin{array}{l}
   \displaystyle \frac{dx}{dt} = -\frac{\partial U}{\partial x} = -2(x-5) + \frac{2\gamma(x-3)}{[(x-3)^2 + (y-4)^2/4 - 1]^{2}}\\[10pt]
   \displaystyle \frac{dy}{dt} = -\frac{\partial U}{\partial y} = -2(y-6) + \frac{\gamma(y-4)/2}{[(x-3)^2 + (y-4)^2/4 - 1]^{2}}
   \end{array}

This is solved by using a discrete approach which is known as steepest
descents.

.. math::

   \begin{array}{l}
   \displaystyle x_{n+1} = x_n  - \eta\left\{2(x_n-5) - \frac{2\gamma(x_n-3)}{[(x_n-3)^2 + (y_n-4)^2/4 - 1]^{2}}\right\}\\[10pt]
   \displaystyle y_{n+1} = y_n -   \eta\left\{2(y_n-6) - \frac{\gamma(y_n-4)/2}{[(x_n-3)^2 + (y_n-4)^2/4 - 1]^{2}}\right\}
   \end{array}

Note that :math:`\gamma` is a measure of field strength and
:math:`\eta` is a step size parameter. Moving these two around is useful
to adjust for better computed paths.

::

    import numpy as np
    import scipy as sp
    import pylab as plt
    from matplotlib.patches import Ellipse

    NP = 200
    t = np.arange(0,NP,1)
    x = np.zeros((NP))
    y = np.zeros((NP))
    x[0] = 0.0
    y[0] = 0.0
    gamma = 1.0
    zeta = 0.1

    for i in range(1,NP):
      v = gamma/(((x[i-1]-3.0)**2 + ((y[i-1]-4.0)**2)/4 -1.0)**2)
      vx = 2.0*(x[i-1]-5.0) - 2*(x[i-1]-3)*v
      vy = 2.0*(y[i-1]-6.0) - 0.5*(y[i-1]-4)*v
      vn = np.sqrt(vx*vx+vy*vy)
      vx2 = vx/vn
      vy2 = vy/vn
      print v, -vx2, -vy2
      x[i] = x[i-1] - zeta*vx2
      y[i] = y[i-1] - zeta*vy2

    ell = Ellipse((3.0,4.0),2,4,0)
    a = plt.subplot(111, aspect='equal')
    ell.set_alpha(0.1)
    a.add_artist(ell)

    plt.plot(x,y, 'b.')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Path')
    plt.show()



.. figure:: PlanningFigures/potentialavoid1a.*
   :width: 85%
   :align: center


Let the domain be the square :math:`0\leq x \leq 10`,
:math:`0\leq y \leq 10`.

-  Place the start position at (1,1)

-  Place the goal position at (9,8)

-  Obstacle 1: disk centered at (4,3) of radius 2.5.

-  Obstacle 2: disk centered at (7,8) of radius 1.

What is the potential function?

Obstacles in red...


.. figure:: PlanningFigures/circles.png
   :width: 50%
   :align: center

   Two obstacles and the resulting equal distance line.

What is the attractive potential? Let :math:`q = (x,y)`,

.. math:: U_a(q) =(x -9)^2 + (y -8)^2.

What is the repulsive potential?

.. math:: U_r (q)= \frac{\gamma_1}{(x-4)^2 + (y-3)^2 - 2.5^2} + \frac{\gamma_2}{(x-7)^2 + (y-8)^2 - 1^2}

The resulting potential is the sum:

.. math:: U = U_a(q) + U_r (q)= (x -9)^2 + (y -8)^2 +

.. math:: \frac{\gamma_1}{(x-4)^2 + (y-3)^2 - 2.5^2} + \frac{\gamma_2}{(x-7)^2 + (y-8)^2 - 1^2}

.. figure:: PlanningFigures/potential4.*
   :width: 70%
   :align: center

   Potential function surface.

.. _`fig:Resultingnavigation`:
.. figure:: PlanningFigures/potentialavoid2a.*
   :width: 50%
   :align: center

   Resulting navigation.

These simple functions work well for simple domains. However, when the
obstacles increase, then the simple potentials cease to be effective. A
more methodical approach is needed.

If you looked carefully at the path in
:numref:`fig:Resultingnavigation`, you
will notice that the path appears to oscillate when it gets near the
large obstacle. Indeed this is what is happening. This oscillation is a
direct result of the steepest descent algorithm is appears in many
numerical optimization routines. The numerics will follow the steepest
gradient and will oscillate back and forth along the steep walls. It
will slowly average out traversing the mean path which will trace the
valley floor,
:numref:`fig:numericaloscillation`.


.. _`fig:numericaloscillation`:
.. figure:: PlanningFigures/numericaloscillation.*
   :width: 40%
   :align: center

   Numerical Oscillation near steep
   gradients.

There is nothing particularly special regarding the functions we have
presented. Our goal is to find a potential surface which can “navigate"
a vehicle from start to finish. Getting familiar with the shapes and
level sets of graphs can be very helpful. This can help one in the
construction process. Typically we want our level set to track an
obstacle boundary.



Construct a function which directs the craft onto the line
:math:`y = 2x + 3`. Then :math:`U = (2x+3-y)^2` will suffice. This
function has a minimum along :math:`y = 2x + 3` and increases as you
move away from the line.

Keep in mind that you must be very careful combining the functions since
they can interact in very complex ways. You may have to have cutoff
distances from obstacles to keep them from corrupting each other.

Higher Dimensions
~~~~~~~~~~~~~~~~~

One of the advantages of potential functions is that they scale to
higher dimensions in a very efficient manner. We will start with three
dimensions. The attractive and repulsive potentials follow the same
pattern as we saw in two dimensions.



Construct an attractive potential for the the point :math:`x_0,y_0,z_0`.

.. math:: U_{att} = (x-x_0)^2 + (y-y_0)^2 + (z-z_0)^2,

and

.. math:: \nabla U_{att} = \langle 2(x-x_0) , 2(y-y_0) , 2(z-z_0)\rangle



Construct a repulsive potential for a spherical obstacle centered
:math:`x_0,y_0,z_0` of radius :math:`R`.

.. math:: U_{rep} = \displaystyle \frac{\gamma}{(x-x_0)^2 + (y-y_0)^2 + (z-z_0)^2 - R^2}

and

.. math:: \nabla U_{rep} = \displaystyle \frac{-2\gamma \langle (x-x_0) , (y-y_0) ,  (z-z_0)\rangle}{\left( (x-x_0)^2 + (y-y_0)^2 + (z-z_0)^2 - R^2\right)^2}


Build a function that can direct a drone to a landing pad. Assume the
landing pad is at (0,0,0). We construct a cone centered at the landing
pad which will “pull" the drone in. We can take a simple attractive
function

.. math:: U_{att} =  x^2 + y^2  + \alpha z^2 = r^2 + \alpha z^2, \quad r = \sqrt{x^2 + y^2}

and then a vertical squeeze function

.. math:: U_{att2} = (z -r)^2.

The resulting potential is :math:`U = U_{att}  + \gamma U_{att2}`,

.. math:: U = r^2 + \alpha z^2 +\gamma (z -r)^2  =  (1+\gamma)r^2 + (\gamma + \alpha) z^2 - 2\gamma rz

.. math:: = (1+\gamma)(x^2 + y^2) + (\gamma + \alpha) z^2 - 2\gamma z\sqrt{x^2 + y^2}.

For the attractive function, the parameter :math:`\alpha` can be used
to vary the relative strength in the :math:`z` direction. In the squeeze
function, the parameter :math:`\gamma` can be used to adjust the
strength of that field component.
