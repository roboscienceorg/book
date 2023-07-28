Workspaces and Configuration Space
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

One may have the exact expression for the forward kinematics.   

.. math::

   \begin{pmatrix} \theta_1(t), ... , \theta_n(t)
              \end{pmatrix}\to p(t)

However it is MUCH harder to find the IPK, i.e. the angle functions (:math:`\theta_k`) as a function
of end effector position.

.. math::

   p(t) \to \begin{pmatrix} \theta_1(t), ... , \theta_n(t)
              \end{pmatrix}

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
