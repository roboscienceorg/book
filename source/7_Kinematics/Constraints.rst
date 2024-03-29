

Constraints and Kinematics
~~~~~~~~~~~~~~~~~~~~~~~~~~

Constraints on a mechanical system are the conditions which restrict
possible geometric positions or limits certain motions. The are written
as expressions of the state variables in the system. It is useful to
distinguish geometric and :index:`kinematic constraints`. The term geometric is
concerned only with position where kinematic also includes motion. We
will use the term *Holonomic* instead of geometric since it is standard
usage in robotics.

:index:`Kinematics` describes the geometry of motion. It describes the motion
through a set of constraints on the way the robot will move through
space. For rigid bodies, we focus on displacement and orientation for
which the kinematics restricts in some manner. Assume that you want to
move in the plane from :math:`(x_1,y_1)` to :math:`(x_2,y_2)`. If you
are driving a traditional front wheel steer automobile (Ackerman
Steering), then your final orientation depends on your path. If you
drive straight then your final orientation is in line with the line
between the start and end points. However, you could have made a large
detour and ended up at another
orientation as shown in  :numref:`fig:dependsonpath`.

.. _`fig:dependsonpath`:
.. figure:: KinematicsFigures/dependsonpath.*
   :width: 40%
   :align: center

   Final orientation depends on path.

Assume you decide to replace your auto wheels with caster wheels and
have someone push you. In this case you can travel from point to
point with arbitrary orientation. [#f1]_ This simple example implies that we
have two fundamentally different types of motion. One that depends on
the path and one that does not. The independence of path boils down to
the types of motion constraints given by the system. Our goal here is to
formally describe these two types of constraints. You may notice a
strong similarity between what we are discussing here and the concepts
of independence of path and conservative vector fields taught in
calculus. Indeed these concepts are related. For this section, let
:math:`x_i(t)` be coordinate variables.

Kinematic Constraints
^^^^^^^^^^^^^^^^^^^^^

A constraint is called kinematic if one can express it as

.. math:: f(x_1, x_2, \dots, x_n, \dot{x}_1, \dot{x}_2, \dots , \dot{x}_n, t)=0

:math:`f` is a function in phase space for the system. This constraint
places restrictions on motion through the expression relating velocities
and positions.

Pfaffian Constraints
^^^^^^^^^^^^^^^^^^^^^

Often the constraints appear linear in the velocity terms as

.. math:: \sum_i F_i(x) \dot{x}_i = 0

and are known as Pfaffian constraints.  This can be written as
:math:`F \cdot \dot{x} = 0`.  Which states that the motion of the
system, :math:`\dot{x}`, is orthogonal to the vector field :math:`F`.  For
multiple constraints, these can be bundled as rows into a constraint
matrix :math:`\bf{F}`:

.. math:: {\bf F} \dot{x} = 0

so the motion :math:`\dot{x}` is along the nullspace of :math:`\bf F`.


Holonomic Constraints
^^^^^^^^^^^^^^^^^^^^^

A constraint is called :index:`holonomic` (or geometric) if it is integrable
or one can express it as

.. math::
   :label: eq:holonomicdefn

   h(x_1, x_2, \dots , x_n, t)=0

A holonomic constraint only depends on the coordinates and time and
does not depend on derivatives. If all the system constraints are
holonomic then we say the system is *holonomic*. Otherwise we say the
system is *non-holonomic*. Wikipedia has a nice way of expressing
non-holonomic:

    A :index:`nonholonomic` system in physics and mathematics is a system whose
    state depends on the path taken in order to achieve it. Such a
    system is described by a set of parameters subject to differential
    constraints, such that when the system evolves along a path in its
    parameter space (the parameters varying continuously in values) but
    finally returns to the original set of parameter values at the start
    of the path, the system itself may not have returned to its original
    state.

A holonomic constraint implies a kinematic constraint:

.. math::  \frac{d h(x)}{dt} = \sum_{i=1}^n \frac{\partial h(x)}{\partial x_i} \dot{x}_i
   = \sum_i f_i(x) \dot{x}_i , ~~ \mbox{where} ~~ f_i(x) = \frac{\partial h(x)}{\partial x_i}

But it is not true in general the other way around. It should
be clear that if the expression is not in Pfaffian form, then it cannot integrated.
This integrability
is a special case.  If the Pfaffian expression, :math:`\sum_i f_i(x) \dot{x}_i`
is holonomic, then using a non-zero
integrating factor :math:`\sigma(x)`, we can integrate and express as

.. math:: H(x) = c

This implies that the mechanical system is constrained to a level surface
of :math:`H` which depends on the initial configuration of the system.  This
reduces the degrees of freedom to :math:`n-1`.  Having k holonomic constraints
then reduces the degrees of freedom to :math:`n-k`.

An example of how a holonomic constraint may be used to reduce the number of
degrees of freedom is helpful. If we want to remove :math:`{\displaystyle x_{k}\,\!}` in the
constraint equation :math:`{\displaystyle f_{i}\,\!}` we algebraically
rearrange the expression into the form

.. math:: {\displaystyle x_{k}=g_{i}(x_{1},\ x_{2},\ x_{3},\ \dots ,\ x_{k-1},\ x_{k+1},\ \dots ,\ x_{n},\ t),\,}

and replace every occurrence of :math:`{\displaystyle x_{k}\,\!}` in the
system using the above expression. This can always be done, provided
that :math:`{\displaystyle f_{i}\,\!}` is
:math:`{\displaystyle C^{1}\,\!}` so the expression
:math:`{\displaystyle g_{i}\,}` is given by the implicit function
theorem. Then using this expression it is possible to remove all
occurrences of the dependent variable :math:`{\displaystyle x_{k}\,\!}`.

Assume that a physical system has :math:`{\displaystyle N\,\!}` degrees
of freedom and there are :math:`{\displaystyle h\,\!}` holonomic
constraints. Then, the number of degrees of freedom is reduced to
:math:`{\displaystyle m=N-h\,\!}.` We now may use
:math:`{\displaystyle m\,\!}` independent (generalized) coordinates
:math:`{\displaystyle q_{j}\,\!}` to completely describe the motion of
the system. The transformation equation can be expressed as follows:

.. math:: {\displaystyle x_{i}=x_{i}(q_{1},\ q_{2},\ \ldots ,\ q_{m},\ t)\ ,\qquad  \qquad i=1,\ 2,\ \ldots n.\,}

For our use, it tells us about the maneuverability for the robot. For
holonomic robots, the controllable degrees of freedom is equal to total
degrees of freedom. Kinematic constraints restrict movement of the
robot. Non-holonomic constraints restrict the motion without restricting
the workspace. Holonomic constraints reduce the dimensionality of the
workspace and restricts the motion of the robot.   Having a non-holonomic
constraint means that there are restrictions on velocity but less so on
position.  So local movement is restricted, but global positioning is less
resricted.

Integrability Conditions
^^^^^^^^^^^^^^^^^^^^^^^^^

If the kinematic constraint is holonomic, then it comes from
differentiating some function :math:`f(t,x)`. So, we consider only first order
expressions,

.. math::
   :label: eq:differential

   \frac{df}{dt} = \sum_{i=1}^{n} \frac{\partial f(t,x)}{\partial x_i} \dot{x_i}
   + \frac{\partial f(t,x)}{\partial t}
   = \sum_{i=1}^{n} a_i (x,t) \dot{x_i} + a_t(x,t) =0.

These expressions are Pfaffian (linear in the velocity terms, :math:`\dot{x_i}`).
If your kinematic expression is nonlinear in velocities terms, it did
not come from differentiation of a holonomic constraint. That is enough
to eliminate many expressions as candidates.

Since the terms :math:`a_i` are the partials :math:`\partial f / \partial x_i`,
the mixed partials
are equal

.. math::  \frac{\partial^2 f}{\partial x_i \partial x_j}
   = \frac{\partial^2 f}{\partial x_j \partial x_i} \Rightarrow
   \frac{\partial a_j}{\partial x_i} = \frac{\partial a_i}{\partial x_j}

Because the constraints are set to zero, it is possible that a common
factor has been divided out

.. math::  \sum_{i=1}^{n} a_i (x,t) \dot{x_i} + a_t(x,t)
   = \sum_{i=1}^{n} \sigma(x) b_i (x,t) \dot{x_i} + \sigma(x) b_t(x,t)
   = \sigma(x) \sum_{i=1}^{n} b_i (x,t) \dot{x_i} + b_t(x,t) = 0

.. math::
   \Rightarrow  \sum_{i=1}^{n} b_i (x,t) \dot{x_i} + b_t(x,t) = 0

when :math:`\sigma(x) \neq 0`.

The term :math:`\sigma` is known as an integrating factor and it complicates
the second partial test.   Given a Pfaffian expression,

.. math::
   \sum_{i=1}^{n} b_i (x,t) \dot{x_i} + b_t(x,t) = 0

the second partial test appears as

.. math::
   :label:  holonomycondition

   \frac{\partial \left( \sigma(x)b_j \right)}{\partial x_i}
   = \frac{\partial \left( \sigma(x)b_i \right)}{\partial x_j}

Integration
^^^^^^^^^^^^

To find the antiderivative, one can follow a fixed process.   Assume that you are
given the form :math:`a_1(x_1,x_2) \dot{x_1} + a_2(x_1,x_2)\dot{x_2} = 0`.  Since
:math:`a_1` comes from a partial derivative with respect to :math:`x_1` then we should
integrate with respect to that variable.   This gives us some function :math:`A_1`.
We can do a similar process for :math:`a_2` and gain :math:`A_2`.  We use both
:math:`a_2` and :math:`A_2` to find the correct term.

**Examples**:  are the following holonomic?

#. :math:`\dot{x_1} + \dot{x_2} = 0`.   For this example, you can just integrate
   and see that :math:`x_1 + x_2=c` is the antiderivative.  So it is holonomic.

#. :math:`x_2e^{x_1}\dot{x_1} + e^{x_1}\dot{x_2} = 0`.  Yes. Since

   .. math::  \frac{\partial (x_2e^{x_1})}{\partial x_2} =  \frac{\partial (e^{x_1})}{\partial x_1}

   Integrate the first expression, :math:`x_2e^{x_1}`, wrt to :math:`x_1` and we obtain
   :math:`h(x) = x_2e^{x_1} + c`.  Differentiate wrt to :math:`x_2` to verify no missing terms.


#. :math:`x_2\dot{x_1} + x_1\dot{x_2} = 0`.  Since

   .. math::  \frac{\partial (x_2)}{\partial x_2} =  \frac{\partial (x_1)}{\partial x_1} \Rightarrow 1 = 1

   it is holonomic.  Integrate the first expression, :math:`x_2`, wrt to :math:`x_1` and we obtain
   :math:`h(x) = x_1x_2 + c`.  Differentiate wrt to :math:`x_2` to verify no missing terms.


#. :math:`x_1 \dot{x}_1 + x_2 \dot{x}_2 + x_3 \dot{x}_3 = 0`.
   There are several mixed partials to check.  This
   constraint can be integrated to :math:`x_1^2 + x_2^2 + x_3^2 = c`.
   which means this is a holonomic constraint.

#. :math:`\dot{x}_1/x_2 + \dot{x}_2 / x_1 = 0`
   Note that the mixed partials do not agree.   Multiply the expression
   by :math:`x_1x_2` (a guess) and check

   .. math::  \frac{\partial (x_1)}{\partial x_2} =  \frac{\partial (x_2)}{\partial x_1}

   So the term :math:`x_1x_2` is called the integrating factor and the
   constraint is holonomic.

#. :math:`x_1\dot{x_1} + x_1x_2\dot{x_2} = 0`
   We try guessing a couple of integrating factors but none succeed.   We seek a function :math:`\sigma(x)` so that

   .. math::  \frac{\partial (\sigma (x) x_1)}{\partial x_2} =  \frac{\partial (\sigma(x)x_1 x_2)}{\partial x_1}

   Expand and solve for :math:`\sigma`

   .. math::  \frac{\partial (\sigma (x) x_1)}{\partial x_2} = x_1 \frac{\partial (\sigma (x))}{\partial x_2}

   and

   .. math::  \frac{\partial (\sigma(x)x_1 x_2)}{\partial x_1} = x_1 x_2\frac{\partial (\sigma (x))}{\partial x_1} + \sigma(x) x_2

   We can equate these

   .. math::  x_1\frac{\partial (\sigma (x))}{\partial x_2} = x_1 x_2\frac{\partial (\sigma (x))}{\partial x_1} + \sigma(x) x_2

   We try a simplification by assuming a form on :math:`\sigma(x) = \sigma_1(x_1)\sigma_2(x_2)`.
   Divide the entire expression by :math:`\sigma_1(x_1)\sigma_2(x_2)x_1x_2` and we obtain

   .. math::  \frac{1}{x_2 \sigma_2}\frac{\partial (\sigma_2 )}{\partial x_2} = \frac{1}{\sigma_1}\frac{\partial (\sigma_1)}{\partial x_1} + \frac{1}{x_1}

   The right side is a function of only :math:`x_1` and the left side only of :math:`x_2`.  The only way for them
   to be equal is if they are constant.  Set each side to a constant, :math:`\lambda` and solve the two resulting
   ordinary differential equations.  This gives us both :math:`\sigma`'s.

   .. math::  \sigma_1 = \frac{c_1}{x_1}e^{\lambda x_1} , ~~ \sigma_2 = c_2 e^{\lambda x_2^2/2}
      \Rightarrow  \sigma = \frac{c}{x_1}e^{\lambda (x_1 - x_2^2/2)}

   So we conclude this expression is holonomic.  We also see that this was a very complicated route and there were multiple
   stages in which this process would stall.  The general approach to finding an integrating factor requires finding an analytic
   solution to a quasi-linear first order partial differential equation which in general is not possible.  In our application
   we try a few tricks to solve for the integrating factor and then look to see if we can prove none exists.  The next example
   will illustrate this.

#. :math:`\dot{x_1} + \dot{x_2} + x_1\dot{x_3} = 0`.  Using :eq:`holonomycondition` we gain the following equations

   .. math:: \frac{\partial \sigma}{\partial x_2} = \frac{\partial \sigma}{\partial x_1}

   .. math:: \frac{\partial \sigma}{\partial x_3} = \sigma + x_1\frac{\partial \sigma}{\partial x_1}

   .. math:: \frac{\partial \sigma}{\partial x_3} = x_1\frac{\partial \sigma}{\partial x_2}

   Setting the second two equations equal

   .. math:: \sigma + x_1\frac{\partial \sigma}{\partial x_1} = x_1\frac{\partial \sigma}{\partial x_2}

   Then use the first equation

   .. math:: \sigma + x_1\frac{\partial \sigma}{\partial x_1} = x_1\frac{\partial \sigma}{\partial x_1}

   one concludes that :math:`\sigma \equiv 0` and so this constraint is *non-holonomic*.

#. The vertical rolling wheel produces a constraint of the form :math:`\sin \theta \dot{x} - \cos\theta \dot{y} = 0` where
   :math:`(x,y)` is the location of the wheel (contact point) in the plane and :math:`\theta` is the orientation of the wheel.
   [This will be discussed in detail later.]

   Apply :eq:`holonomycondition` and we have

   .. math:: \sin\theta \frac{\partial \sigma}{\partial y} = -\cos\theta \frac{\partial \sigma}{\partial x}

   .. math:: \cos\theta \frac{\partial \sigma}{\partial \theta} = \sigma \sin\theta

   .. math:: \sin\theta \frac{\partial \sigma}{\partial \theta} = -\sigma \cos\theta

   Squaring the last two equations and adding together, we gain :math:`\partial \sigma / \partial \theta = \pm \sigma`
   and plugging this back in to either gives :math:`\pm (\cos\theta) \sigma = (\sin\theta) \sigma`.  As with the previous example
   we can conclude that :math:`\sigma = 0` so the constraint is non-holonomic.


Systems of Pfaffian constraints are a more complicated matter.  It is possible to have a collection of constraints which are individually
non-holonomic, but the collection turns out to be integrable.   The theory is outside the scope of this text and when we need a result
we will quote the literature.



.. rubric:: Footnotes

.. [#f1] Like the office chair races in the hallway.
