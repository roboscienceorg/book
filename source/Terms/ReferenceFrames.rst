Reference Frames, Workspaces and Configuration Space
----------------------------------------------------

There are several frames of reference which are important to the robot.
The robot operates in the physical world and it can be tracked by an
external frame of reference. This is known as a *world* or *global*
:index:`reference frame`. This is normally the observer’s frame of reference.
Frames of reference on or within the robot are known as *local reference
frames*. Each joint or actuator can have a frame of reference known as
the *joint reference*. These are useful in understanding the
transformations induced by each joint which leads to a kinematic model
of the manipulator. For manipulation, the position and orientation of
the tool needs tracking and will require a frame of reference known as
the *tool reference*. Relating the various frames of reference requires
some knowledge of coordinate systems and transforms which is found in
standard courses in linear algebra.

The operating environment for a robot is known as the *workspace*. It is defined as the
volume or region for which the robot can operate. For robots with
manipulators, traditionally the :index:`workspace` is all points that the tool
end can reach. For mobile robots it is the region that the robot can and
may move into. Any obstacle or constraint will be called a workspace
obstacle or workspace constraint. Motion of a robot or articulator
through the workspace is referred to by the term *workspace path*.
Building a robot from rotary and linear joints means that our effector
end (tool end) can be moved in an angular or linear fashion. This means
that there is a natural coordinate system for the workspace. The three
that are commonly used are Cartesian, Cylindrical or Spherical.

The range of all possible parameter values that the robot can modify is
known as the *configuration space*. It is the span of the machine when
the actuators are run through their different positions. The dimension
of the :index:`configuration space` is the degrees of freedom. The difference
between workspace and configuration space might be confusing at first.
Workspace is the physical one, two or three dimensions, where is robot
operates, whereas configuration space is made up from the different
configurations that the links and servos can define. In the case of
manipulators, it is common to represent configuration space by the joint
variables implicitly knowing the relation between the joint variable and
the configuration. We will use :math:`q` for a configuration and
:math:`{\cal Q}` for configuration space to distinguish it from
workspace variables.

We can denote the configuration space occupied by the robot by
:math:`R(q)`. A configuration space obstacle is
:math:`{\cal Q}{\cal O}_i`,

.. math:: {\cal Q}{\cal O}_i = \left\{ q\in {\cal Q} ~|~ R(q) \bigcap {\cal W}{\cal O}_i \neq \emptyset\right\}.

Free configuration space is then

.. math:: {\cal Q}_\text{free} = {\cal Q}\setminus \left( \bigcup_i {\cal Q}{\cal O}_i\right).

Reaching a point in space requires a particular configuration of the
joints or motors. So there is a point in configuration space that
relates to a point in the workspace. Understanding the relation between
the articulators and the point in space turns out to be a very hard
problem. If you know the position of each joint, you can then generate a
mapping from the set of joint positions to a point in the workspace.
This is known as forward kinematics. Having a target point out in space
and asking what are the joint positions has to do with inverting the
kinematic equations and is known as *inverse kinematics*. The forward
kinematics are expressed as a system of algebraic equations. There is no
general rule that these equations will be invertible. So, *IK*, inverse
kinematics equations may not be available. Later in this text we will
explore numerical approaches .

Just having a relation between the physical workspace and the joint
(wheel, motor, etc) configuration is not the goal in robots. We normally
want to do something. We want to move. We might be welding along a seam
or driving a path. Our controls are working with the actuators and those
are translated over into the workspace through some rather complicated
mathematical expressions. The interesting challenge is to take a desired
path in the workspace, say the welder path, and figure out the motion of
the joints (motors) that produce this path. Then optimize based on
workspace obstacles, machine constraints and other considerations. This
is the subject of *planning* which will be touched on later as well as
in courses on planning.

Constraints and Kinematics
~~~~~~~~~~~~~~~~~~~~~~~~~~

Constraints on a mechanical system are the conditions which restrict
possible geometric positions or limits certain motions. The are written
as expressions of the state variables in the system. It is useful to
distinguish geometric and kinematic constraints. The term geometric is
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
.. figure:: TermsFigures/dependsonpath.*
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

.. math:: F(x_1, x_2, \dots, x_n, \dot{x}_1, \dot{x}_2, \dots , \dot{x}_n, t)=0

:math:`F` is a function in phase space for the system. This constraint
places restrictions on motion through the expression relating velocities
and positions.

Holonomic Constraints
^^^^^^^^^^^^^^^^^^^^^

A constraint is called holonomic (or geometric) if one can express it as

.. math::
   :label: eq:holonomicdefn

   f(x_1, x_2, \dots , x_n, t)=0

A :index:`holonomic` constraint only depends on the coordinates and time and
does not depend on derivatives. If all the system constraints are
holonomic then we say the system is *holonomic*. Otherwise we say the
system is *non-holonomic*. Wikipedia has a nice way of expressing
non-holonomic:

    A nonholonomic system in physics and mathematics is a system whose
    state depends on the path taken in order to achieve it. Such a
    system is described by a set of parameters subject to differential
    constraints, such that when the system evolves along a path in its
    parameter space (the parameters varying continuously in values) but
    finally returns to the original set of parameter values at the start
    of the path, the system itself may not have returned to its original
    state.

Holonomic may be used to reduce the number of degrees of freedom. For
example, if we want to remove :math:`{\displaystyle x_{k}\,\!}` in the
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
workspace and restricts the motion of the robot.

If the kinematic constraint is holonomic, then it comes from
differentiating some function :math:`f(t,x)`. We consider first order
expressions,

.. math::
   :label: eq:differential

   \frac{df}{dt} = \sum_{i=1}^{n} a_i (x,t) \dot{x_i} + a_t(x,t) .

These expressions are linear in the velocity terms, :math:`\dot{x_i}`.
If your kinematic expression is nonlinear in velocities terms, it did
not come from differentiation of a holonomic constraint. That is enough
to eliminate many expressions as candidates. If one is in doubt about an
expression, we can borrow the concepts of independence of path and
conservative vector fields from calculus.
Equation :eq:`eq:differential` is related to the
differential form you studied in line integrals.

.. math::
   :label:  eq:differentialform

   df = \sum_{i=1}^{n} a_i (x,t) d x_i + a_t(x,t) dt .

To be a holonomic constraint, we need that
Equation :eq:`eq:differentialform` to be a total
derivative (exact differential) or that by using an integrating factor
can be made into a total derivative (exact differential). If you are
able to convert an expression to the form in
Equation :eq:`eq:holonomicdefn` then we know that we
have a holonomic constraint.

Maybe the expression is not holonomic or you just don’t see how to
integrate it. Recall that this is related to the independence of path
concept from calculus. There you could integrate over different paths
(same start and end points). If the values differed, then you did not
have independence of path meaning you did not have an exact differential
(stated in Calculus as lacking a potential function). Let :math:`C_1`
and :math:`C_2` be two parameterizations of two different paths with the
same starting and ending points. Then if the path integrals differ:

.. math:: \int_{C_1} F \neq \int_{C_2} F

the expression (constraint) does not have a holonomic representation.

**Example:** Which of the following are holonomic?

#. The constraint
   :math:`x_1 \dot{x}_1 + x_2 \dot{x}_2 + x_3 \dot{x}_3 = 1`? This
   constraint can be integrated to :math:`x_1^2 + x_2^2 + x_3^2 = 2t`.
   This can be expressed as

   .. math:: x_1^2 + x_2^2 + x_3^2 - 2t = 0

   which means this is a holonomic constraint.

#. The constraint :math:`x_1 \dot{x}_1 + \dot{x}_1 \dot{x}_2 = 0`? We
   see that the velocity terms are not expressed linearly so this is not
   non-holonomic. We illustrate the idea of integrating over two paths
   to show how that idea works. Define :math:`C_1` to the the path from
   (0,0) to (1,1) via :math:`x_1(t)=t`, :math:`x_2(t)=t`,
   :math:`0\leq t \leq 1`. Define :math:`C_2` to be the path
   :math:`x_1(t)=t`, :math:`x_2(t)=0`, :math:`0\leq t \leq 1` plus
   :math:`x_1(t)=1`, :math:`x_2(t) = t`, :math:`0\leq t \leq 1`. The
   line integral of the constraint
   :math:`F = x_1 \dot{x}_1 + \dot{x}_1 \dot{x}_2` over the two paths
   gives

   .. math:: \int_{C_1} F \neq \int_{C_2} F

   which implies the constraint cannot be an exact derivative of some
   potential. This confirms that the constraint is not holonomic.

#. The constraint :math:`x_1 \dot{x}_2 + x_2 \dot{x}_1 = 0`? This one
   can be expressed as :math:`d/dt~[ x_1 x_2 ] = 0`. This can be
   integrated to :math:`x_1x_2 = k` and hence is holonomic.

#. The constraint :math:`(x_1 + x_2 )\dot{x}_1 + (2x_1)\dot{x}_2= 0`.
   This is linear. Using the same paths as the example 2, the line
   integral of the constraint
   :math:`F = (x_1 + x_2 )\dot{x}_1 + (2x_1)\dot{x}_2` over the two
   paths gives

   .. math:: \int_{C_1} F \neq \int_{C_2} F.

   \ Thus this is not holonomic.

Forward Position Kinematics
^^^^^^^^^^^^^^^^^^^^^^^^^^^

The :index:`forward position kinematics` (FPK) solves the following problem:
“Given the joint positions, what is the corresponding end effector’s
pose?” If we let :math:`x = (x_1, x_2, x_3)` be the position as a
function of time and :math:`p = (p_1, p_2, \dots , p_n)` the equations
that transform :math:`p` into :math:`x` are the forward kinematic
equations

.. math:: x = F(p).

.. _`fig:threelink`:
.. figure:: TermsFigures/threelink.*
   :width: 50%
   :align: center

   A three link planar manipulator.

.. _`fig:forwardkinematics`:
.. figure:: TermsFigures/forwardkinematics.*
   :width: 70%
   :align: center

   The mapping from configuration space to
   workspace.

Forward Position Kinematics for Serial Chains
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The solution is always unique: one given joint position vector always
corresponds to only one single end effector pose. The FK problem is not
difficult to solve, even for a completely arbitrary kinematic structure.
We may simply use straightforward geometry, use transformation matrices
or the tools developed in standard engineering courses such as statics
and dynamics.

Forward Position Kinematics For Parallel Chains (Stewart-Gough Manipulators)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The solution is not unique: one set of joint coordinates has more
different end effector poses. In case of a Stewart platform there are 40
poses possible which can be real for some design examples. Computation
is intensive but solved in closed form with the help of algebraic
geometry.

Inverse Position Kinematics
^^^^^^^^^^^^^^^^^^^^^^^^^^^

The :index:`inverse position kinematics` (IPK) solves the following problem:
“Given the actual end effector pose, what are the corresponding joint
positions?” In contrast to the forward problem, the solution of the
inverse problem is not always unique: the same end effector pose can be
reached in several configurations, corresponding to distinct joint
position vectors. A 6R manipulator (a serial chain with six revolute
joints) with a completely general geometric structure has sixteen
different inverse kinematics solutions, found as the solutions of a
sixteenth order polynomial.

Forward Velocity Kinematics
^^^^^^^^^^^^^^^^^^^^^^^^^^^

The :index:`forward velocity kinematics` (FVK) solves the following problem:
“Given the vectors of joint positions and joint velocities, what is the
resulting end effector twist?” The solution is always unique: one given
set of joint positions and joint velocities always corresponds to only
one single end effector twist. Using :math:`x` to the the position
vector as a function of time and :math:`p` the joint parameters as a
function of time, let the forward position kinematics be given by
:math:`x = F(p)`. Then the forward velocity kinematics can be derived
from the forward position kinematics by differentiation (and chain
rule). A compact notation uses the Jacobian of the forward kinematics:

.. math:: v = J_F(p) q, \quad  \mbox{ where } \quad v = \frac{dx}{dt}, ~ q = \frac{dp}{dt}.

Inverse Velocity Kinematics
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Assuming that the inverse position kinematics problem has been solved
for the current end effector pose, the :index:`inverse velocity kinematics` (IVK)
then solves the following problem: “Given the end effector twist, what
is the corresponding vector of joint velocities?” Under the assumption
that the Jacobian is invertible (square and full rank) we can find
:math:`J^{-1}` and express

.. math:: q = J_F(p)^{-1} v = J_F\left( F^{-1}(x) \right) v

Forward Force Kinematics
^^^^^^^^^^^^^^^^^^^^^^^^

The :index:`forward force kinematics` (FFK) solves the following problem: “Given
the vectors of joint force/torques, what is the resulting static wrench
that the end effector exerts on the environment?” (If the end effector
is rigidly fixed to a rigid environment.)

Inverse Force Kinematics
^^^^^^^^^^^^^^^^^^^^^^^^

Assuming that the inverse position kinematics problem has been solved
for the current end effector pose, the :index:`inverse force kinematics` (IFK)
then solves the following problem: “Given the wrench that acts on the
end effector, what is the corresponding vector of joint forces/torques?”

We will not treat forward or inverse force kinematics in this text.
These concepts are treated in courses in statics and mechanics.

.. rubric:: Footnotes

.. [#f1] Like the office chair races in the hallway.
