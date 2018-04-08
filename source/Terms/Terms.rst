Terms and Basic Concepts of Robotics[Chap:Terms]
================================================

Getting the language down is the first step. Robotics is like any other
engineering field with lots of jargon and specialized terms. The terms
do convey important concepts which we will introduce here.

Terminology
-----------

In the Introduction, several terms were introduced such as end effector
or actuator. We will round out the common robotics terminology in the
section.

-  *Manipulator*: the movable part of the robot, often this is the
   robotic arm.

-  *Degrees of Freedom*: the number of independently adjustable or
   controllable elements in the robot. It is also the number of
   parameters that are needed to describe the physical state of the
   robot such as positions, angles and velocities.

-  *End Effector*: the end of the manipulator.

-  *Payload*: the amount the robot can manipulate or lift.

-  *Actuator*: the motor, servo or other device which translates
   commands into motion.

-  *Speed*: the linear or angular speed that a robot can achieve.

-  *Accuracy*: how closely a robot can achieve its desired position.

-  *Resolution*: the numerical precision of the device, usually with
   respect to the end-effector. This can also be measured in terms of
   repeatability. Related to accuracy vs precision in general
   measurements.

-  *Sensor*: any device that takes in environmental information and
   translates it to a signal for the computer such as cameras, switches,
   ultrasonic ranges, etc.

-  *Controller*: can refer to the hardware or software system that
   provides low level control of a physical device (mostly meaning
   positioning control), but may also refer to the robot control
   overall.

-  *Processor*: the cpu that controls the system. There may be multiple
   cpus and controllers or just one unit overall.

-  *Software*: all of the code required to make the system operate.

-  

-  *Closed Loop control*: using sensor feedback to improve the control
   accuracy.

Motion is achieved by some device that converts some energy source into
motion. Most often these are electric motors (even non-electric systems
often have electrically controlled components like valves.) However, it
is useful to not focus on the type of equipment, but just the type
motion induced. For simplicity anything that induces motion will be
called actuators for most of the text. Actuators apply forces to the
various robotic components in the system which in turn generates motion.
The connections between actuators are known as *links*. For this work we
will assume they are rigid and fixed in size. Connecting links are
joints. This allows the links to move with respect to each other. There
are two types of common joints, *rotary* and *linear* joints. The name
essentially indicates what it does. A rotary actuator allows the
relative angle between the links to change. A linear actuator changes
the length of a link. Examples of rotary joints are revolute,
cylindrical, helical, universal and spherical
joints \ `[fig:robotjoints] <#fig:robotjoints>`__. Linear joints are
also referred to as prismatic joints.

.. raw:: latex

   \centering

.. figure:: manipulators/robotjoints
   :alt: Some common robot joints.[fig:robotjoints]

   Some common robot joints.[fig:robotjoints]

All of the machines we will study have moving components. The complexity
of the system depends on the number of components and the
interconnections therein. For example, a robotic arm may have three or
four joints that can be moved or varied. A vehicle can have
independently rotated wheels. The number of independently moving
components is referred to as the *degrees of freedom*; the number of
actuators that can induce unique configurations in the system. This
mathematical concept comes from the number of independent variables in
the system. It gives a measure of complexity. Higher degrees of freedom,
just as higher dimensions in an equation, indicate a system of higher
complexity. This concept of degrees of freedom is best understood from
examples.

.. raw:: latex

   \normalfont

Consider a computer-controlled router that can move the tool head freely
in the :math:`x` and :math:`y` directions. This device has *two degrees
of freedom*. It is like a point in the plane which has two parameters to
describe it. Going one step further, consider a 3D printer. These
devices can move the extruder head back and forth in the plane like the
router, but can also move up and down (in :math:`z`). With this we see
three degrees of motion or freedom. While it may seem from these two
examples that the degrees of freedom come from the physical dimensions,
please note that this is not the case. Consider the 3D printer again. If
we added a rotating extruder head, the degrees of freedom would equal to
four (or more, depending on setup), but the physical dimensions would
stay at three.

.. raw:: latex

   \normalfont

Consider a welder that can position its tool head at any point in a
three dimensional space. This implies three degrees of freedom. We
continue and assume that this welder must be able to position its tool
head orthogonal to the surface of any object it works on. This means the
tool must be able to rotate around in space - basically pan and tilt.
This is two degrees of freedom. Now if we attach the rotating tool head
to the welder, we have five degrees of freedom: 5DOF.

Each joint in a robotic arm typically generates a degree of freedom. To
access any point in space from any angle requires five degrees of
freedom (:math:`x,y,z,pan,tilt`). So why would we need more? Additional
degrees of freedom add flexibility when there are obstacles or
constraints in the system. Consider the human arm. The shoulder rotates
with two degrees of freedom. The elbow is a single degree of freedom.
The wrist can rotate (the twisting in the forearm) as well as limited
two degree motion down in the wrist. Thus the wrist can claim three
degrees of freedom. Without the hand, the arm has six degrees of
freedom. So you can approach an object with your hand from many
different directions. You can drive in a screw from any position.

Serial and Parallel Chain Manipulators
--------------------------------------

Manufacturing robots typically work in a predefined and restricted
space. They usually have very precise proprioception (the knowledge of
relative position and forces) within the space. It is common to name the
design class after the coordinate system which the robot naturally
operates in. For example, a cartesian design (similar to gantry systems)
is found with many mills and routers, heavy lift systems, 3D Printers
and so forth, see Figure \ `[gantrysample-a] <#gantrysample-a>`__.
Actuation occurs in the coordinate directions and is described by
variable length linear segments (links) or variable positioning along a
segment. This greatly simplifies the mathematical model of the machine
and allows efficient computation of machine configurations.

In two dimensions, one can rotate a linear actuator about a common
center producing a radial design which would use a polar coordinate
description. Adding a linear actuator on the :math:`z` axis gives a
cylindrical coordinate description,
Figure \ `[gantrysample-b] <#gantrysample-b>`__.

.. raw:: latex

   \centering

.. figure:: robots/cartesian
   :alt: Cartesian design [gantrysample-a]

   Cartesian design [gantrysample-a]

.. raw:: latex

   \centering

.. figure:: robots/cylindrical
   :alt: Cylindrical design [gantrysample-b]

   Cylindrical design [gantrysample-b]

A serial chain manipulator is the most common design in industrial
robots. It is built as a sequence of links connect by actuated joints
(normally seen as a sequence starting from an attached base and
terminating at the end-effector. By relating the links to segments and
joints as nodes, we see that serial link manipulators can be seen as
graphs with no loops or cycles. The classical robot arm is an example of
a serial chain manipulator, Figure \ `[armsample-a] <#armsample-a>`__.
Robot arms normally employ fixed length links and use rotary joints.
This are often called articulated robots or the arm is called an
articulator. Very general tools exist to construct mathematical
descriptions of arm configuration as a function of joint angles. A
formalism developed by Denavit and Hartenberg can be used to obtain the
equations for position.

.. raw:: latex

   \centering

.. figure:: robots/RobotArm
   :alt: Articulated [armsample-a]

   Articulated [armsample-a]

.. raw:: latex

   \centering

.. figure:: robots/deltadesign
   :alt: Delta Design [armsample-b]

   Delta Design [armsample-b]

Another popular approach is the parallel chain manipulator, which uses
multiple serial chains to control the end-effector. An example of one,
called a Delta Robot, can be seen in
Figure \ `[armsample-b] <#armsample-b>`__.

Reference Frames, Workspaces and Configuration Space
----------------------------------------------------

There are several frames of reference which are important to the robot.
The robot operates in the physical world and it can be tracked by an
external frame of reference. This is known as a *world* or *global*
reference frame. This is normally the observer’s frame of reference.
Frames of reference on or within the robot are known as *local reference
frames*. Each joint or actuator can have a frame of reference known as
the *joint reference*. These are useful in understanding the
transformations induced by each joint which leads to a kinematic model
of the manipulator. For manipulation, the position and orientation of
the tool needs tracking and will require a frame of reference known as
the *tool reference*. Relating the various frames of reference requires
some knowledge of coordinate systems and transforms which is found in
standard courses in linear algebra.

The operating environment for a robot is known as the *workspace*,
Figure \ `[intro-two-link] <#intro-two-link>`__. It is defined as the
volume or region for which the robot can operate. For robots with
manipulators, traditionally the workspace is all points that the tool
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
of the configuration space is the degrees of freedom. The difference
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

Kinematics describes the geometry of motion. It describes the motion
through a set of constraints on the way the robot will move through
space. For rigid bodies, we focus on displacement and orientation for
which the kinematics restricts in some manner. Assume that you want to
move in the plane from :math:`(x_1,y_1)` to :math:`(x_2,y_2)`. If you
are driving a traditional front wheel steer automobile (Ackerman
Steering), then your final orientation depends on your path. If you
drive straight then your final orientation is in line with the line
between the start and end points. However, you could have made a large
detour and ended up at another
orientation \ `[fig:dependsonpath] <#fig:dependsonpath>`__.

.. raw:: latex

   \centering

.. figure:: kinematics/dependsonpath
   :alt: Final orientation depends on path[fig:dependsonpath]

   Final orientation depends on path[fig:dependsonpath]

Assume you decide to replace your auto wheels with caster wheels and
have someone push you [1]_. In this case you can travel from point to
point with arbitrary orientation. This simple example implies that we
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

   \label{eq:holonomicdefn}
   f(x_1, x_2, \dots , x_n, t)=0

 A holonomic constraint only depends on the coordinates and time and
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

   \label{eq:differential}
   \frac{df}{dt} = \sum_{i=1}^{n} a_i (x,t) \dot{x_i} + a_t(x,t) .

 These expressions are linear in the velocity terms, :math:`\dot{x_i}`.
If your kinematic expression is nonlinear in velocities terms, it did
not come from differentiation of a holonomic constraint. That is enough
to eliminate many expressions as candidates. If one is in doubt about an
expression, we can borrow the concepts of independence of path and
conservative vector fields from calculus.
Equation \ `[eq:differential] <#eq:differential>`__ is related to the
differential form you studied in line integrals.

.. math::

   \label{eq:differentialform}
   df = \sum_{i=1}^{n} a_i (x,t) d x_i + a_t(x,t) dt .

 To be a holonomic constraint, we need that
Eq \ `[eq:differentialform] <#eq:differentialform>`__ to be a total
derivative (exact differential) or that by using an integrating factor
can be made into a total derivative (exact differential). If you are
able to convert an expression to the form in
Eq \ `[eq:holonomicdefn] <#eq:holonomicdefn>`__ then we know that we
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

   \ which means this is a holonomic constraint.

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

The forward position kinematics (FPK) solves the following problem:
“Given the joint positions, what is the corresponding end effector’s
pose?” If we let :math:`x = (x_1, x_2, x_3)` be the position as a
function of time and :math:`p = (p_1, p_2, \dots , p_n)` the equations
that transform :math:`p` into :math:`x` are the forward kinematic
equations

.. math:: x = F(p).

.. raw:: latex

   \centering

.. figure:: kinematics/threelink
   :alt: [fig:threelink] A three link planar manipulator.

   [fig:threelink] A three link planar manipulator.

.. figure:: kinematics/forwardkinematics
   :alt: [fig:forwardkinematics] The mapping from configuration space to
   workspace.

   [fig:forwardkinematics] The mapping from configuration space to
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

The inverse position kinematics (IPK) solves the following problem:
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

The forward velocity kinematics (FVK) solves the following problem:
“Given the vectors of joint positions and joint velocities, what is the
resulting end effector twist?” The solution is always unique: one given
set of joint positions and joint velocities always corresponds to only
one single end effector twist. Using :math:`x` to the the position
vector as a function of time and :math:`p` the joint parameters as a
function of time, let the forward position kinematics be given by
:math:`x = F(p)`. Then the forward velocity kinematics can be derived
from the forward position kinematics by differentiation (and chain
rule). A compact notation uses the Jacobian of the forward kinematics:

.. math:: v = J_F(p) q, \quad  \mbox{~where~} \quad v = \frac{dx}{dt}, ~ q = \frac{dp}{dt}.

Inverse Velocity Kinematics
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Assuming that the inverse position kinematics problem has been solved
for the current end effector pose, the inverse velocity kinematics (IVK)
then solves the following problem: “Given the end effector twist, what
is the corresponding vector of joint velocities?” Under the assumption
that the Jacobian is invertible (square and full rank) we can find
:math:`J^{-1}` and express

.. math:: q = J_F(p)^{-1} v = J_F\left( F^{-1}(x) \right) v

Forward Force Kinematics
^^^^^^^^^^^^^^^^^^^^^^^^

The forward force kinematics (FFK) solves the following problem: “Given
the vectors of joint force/torques, what is the resulting static wrench
that the end effector exerts on the environment?” (If the end effector
is rigidly fixed to a rigid environment.)

Inverse Force Kinematics
^^^^^^^^^^^^^^^^^^^^^^^^

Assuming that the inverse position kinematics problem has been solved
for the current end effector pose, the inverse force kinematics (IFK)
then solves the following problem: “Given the wrench that acts on the
end effector, what is the corresponding vector of joint forces/torques?”

We will not treat forward or inverse force kinematics in this text.
These concepts are treated in courses in statics and mechanics.

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
them, it helps to see them in action. The simple two link manipulator,
Figure \ `[intro-two-link] <#intro-two-link>`__ is a good place to
start. Imagine a robotic arm that has two straight links with a rotary
joint at the base and a rotary joint connecting the two links. In
practice, these rotary joints would be run by motors or servos and
probably have some limits, but for now we will assume full
:math:`360^\circ` motion.

.. raw:: latex

   \centering

.. figure:: kinematics/twolinkalt
   :alt: The two link manipulator. [intro-two-link]

   The two link manipulator. [intro-two-link]

The workspace that the arm operates inside is a disk,
Figure \ `[two-link-disk] <#two-link-disk>`__. This is a two dimensional
workspace. The figure indicates the workspace in gray. It may also be
the case that there is something in the workspace, a workspace obstacle
indicated in red. This unit has two joints which define a two
dimensional configuration space,
Figure \ `[intro-config-axis] <#intro-config-axis>`__. The dimension of
the configuration space is the degrees of freedom, and so this has two
degrees of freedom. Since the joint is rotary and moving a full
:math:`360^\circ` degrees returns you to the same angle, the two
directions wrap back on themselves. This actually makes the
configuration space a two dimensional torus or “donut”.

.. raw:: latex

   \centering

|a) Workspace for the two link manipulator with equal link lengths. b)
Workspace obstacle for the two link manipulator. [two-link-disk]| |a)
Workspace for the two link manipulator with equal link lengths. b)
Workspace obstacle for the two link manipulator. [two-link-disk]|

.. raw:: latex

   \centering

|Configuration domain and configuration topology which is a torus.
[intro-config-axis]| |Configuration domain and configuration topology
which is a torus. [intro-config-axis]|

We will illustrate what is meant by kinematics and inverse kinematics
using the two link manipulator. Forward kinematics will identify the
location of the end effector as a function of the joint angles,
Figure \ `[twolinklabeled] <#twolinklabeled>`__-(a). This is easily done
using a little trigonometry. First we find the location of
:math:`(\xi, \eta)` as a function of :math:`\theta_1` and the link
length :math:`a_1`, Figure \ `[twolinklabeled] <#twolinklabeled>`__-(b):

.. math:: \xi =  a_1 \cos \theta_1, \quad \eta = a_1 \sin \theta_1

.. raw:: latex

   \centering

.. figure:: kinematics/twolink2
   :alt: a) The two link manipulator with the links and joints labeled.
   b) Location of the middle joint. [twolinklabeled]

   a) The two link manipulator with the links and joints labeled. b)
   Location of the middle joint. [twolinklabeled]

The next link can be included with

.. math:: \Delta x =  a_2 \cos (\theta_1 + \theta_2), \quad \Delta y = a_2 \sin ( \theta_1 + \theta_2)

Note that :math:`x = \xi + \Delta x` and :math:`y = \eta + \Delta y`.
Combining the expressions, the forward kinematics are:

.. math::

   \label{twolinkforward}
   \begin{matrix}
   x = a_2\cos (\theta_1+\theta_2) + a_1 \cos \theta_1 \\ 
   y = a_2 \sin (\theta_1 +\theta_2) + a_1\sin \theta_1 
   \end{matrix}

 As you move the servos in the system, you can change the angles
:math:`\theta_1` and :math:`\theta_2`. The
formula \ `[twolinkforward] <#twolinkforward>`__ gives the location of
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

.. math:: x^2 + y^2 = a_1^2 + a_2^2 - 2a_1a_2 \cos (\pi - \theta_2). \label{eqn:theta2step1}

 Using :math:`\cos(\pi - \alpha) = -\cos(\alpha)`, we solve for
:math:`\cos` in Eqn \ `[eqn:theta2step1] <#eqn:theta2step1>`__:

.. math:: \cos(\theta_2) = \frac{x^2 + y^2 - a_1^2 - a_2^2}{2a_1a_2 }\equiv D

Using a trig formula:

.. math:: \sin(\theta_2) = \pm \sqrt{1-D^2}

Dividing the sin and cos expressions to get tan and then inverting:

.. math:: \theta_2 = \tan^{-1}\frac{\pm\sqrt{1-D^2}}{D}

The tangent form has the +/- and gives the elbow up and elbow down
solutions.

.. raw:: latex

   \centering

.. figure:: kinematics/twolink3
   :alt: The interior angles for the two link manipulator.
   [twolinklabeled2]

   The interior angles for the two link manipulator. [twolinklabeled2]

From Figure \ `[twolinklabeled2] <#twolinklabeled2>`__, we have

.. math:: \theta_1 = \phi - \gamma = \tan^{-1}\frac{y}{x} - \gamma . \label{eqn:theta1step1}

 If you look at the two dotted blue lines you can see that the line
opposite :math:`\gamma` has length :math:`a_2\sin \theta_2`. The segment
adjacent to :math:`\gamma` (blue solid and dotted lines) has length
:math:`a_1 + a_2\cos \theta_2`. Then

.. math:: \tan \gamma =  \frac{\mbox{Opposite}}{\mbox{Adjacent}} = \frac{a_2\sin \theta_2}{a_1 + a_2\cos\theta_2}

which gives us :math:`\gamma`:

.. math:: \gamma = \tan^{-1} \frac{a_2\sin \theta_2}{a_1 + a_2\cos\theta_2}.

Plug :math:`\gamma` into Eqn \ `[eqn:theta1step1] <#eqn:theta1step1>`__
and we obtain

.. math:: \theta_1 = \tan^{-1}\frac{y}{x} - \tan^{-1} \frac{a_2\sin \theta_2}{a_1 + a_2\cos\theta_2}

Given the two link manipulator kinematic equations:

.. math:: x = a_2\cos (\theta_1+\theta_2) + a_1 \cos \theta_1

\ 

.. math:: y = a_2 \sin (\theta_1 +\theta_2) + a_1\sin \theta_1

The inverse kinematics (IK) are

.. math:: D = \frac{x^2 + y^2 - a_1^2 - a_2^2}{2a_1a_2 }

.. math::

   \theta_1 = \tan^{-1}\frac{y}{x} - \tan^{-1} \frac{a_2\sin \theta_2}{a_1 + a_2\cos\theta_2}, \quad\quad
   \theta_2 = \tan^{-1}\frac{\pm\sqrt{1-D^2}}{D}

Note the kinematic equations only involve the position variables and not
the velocities so they holonomic constraints.

Let :math:`a_1 = 15`, :math:`a_2 = 10`, :math:`x=10`, :math:`y=8`. Find
:math:`\theta_1` and :math:`\theta_2`:

#. :math:`D = (10^2 + 8^2 - 15^2-10^2)/(2*15*10) = -0.53667`

#. :math:`\theta_2 = \tan^{-1}(-\sqrt{1-(-0.53667)^2}/(-0.53667))\approx -2.137278`

#. | :math:`\theta_1 = \tan^{-1}(8/10)-\tan^{-1}[(10\sin(-2.137278))/`
   | :math:`(15+ 10\cos(-2.137278))] \approx 1.394087`

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
Figure \ `[Fig:paralleltwolink] <#Fig:paralleltwolink>`__. The top (red)
is fixed and is of length :math:`L_0`. The two links on either side
shown in dark blue are connected by servos (in green). These links are
of length :math:`L_1`. The angles are measured from the dotted line (as
0 degrees) to straight down (90 degrees), see
Figure \ `[Fig:paralleltwolink2] <#Fig:paralleltwolink2>`__. At the
other end of the dark blue links is a free rotational joint (pivot).
That connects the two light blue links which are joined together at the
bottom with a rotational joint.

.. raw:: latex

   \centering

.. figure:: configuration/2dDelta
   :alt: Parallel Two Link Manipulator. [Fig:paralleltwolink]

   Parallel Two Link Manipulator. [Fig:paralleltwolink]

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
Figure \ `[Fig:paralleltwolink2] <#Fig:paralleltwolink2>`__. As with the
serial chain manipulator, this is an exercise in trigonometry.

.. raw:: latex

   \centering

|Parallel Two Link (a) configuration space (b) with coordinates
[Fig:paralleltwolink2]| |Parallel Two Link (a) configuration space (b)
with coordinates [Fig:paralleltwolink2]|

The forward kinematics will provide :math:`(x,y)` as a function of
:math:`(\theta_1, \theta_2)`. The derivation is left as an exercise and
so the point :math:`(x,y)` is given by

.. math::

   \label{paralleltwolinkforward}
   (x,y) = \left( \frac{a+c}{2} + \frac{v (b-d)}{u} , \frac{b+d}{2} + \frac{v (c-a)}{u} \right)

 Where

.. math:: (a,b) = (-L_1 \cos(\theta_1) - L_0/2 , L_1 \sin(\theta_1) )

.. math:: (c,d) = (L_1 \cos(\theta_2) + L_0/2 , L_1 \sin(\theta_2) )

and :math:`u = \sqrt{(a-c)^2 + (b-d)^2}` and
:math:`v  = \sqrt{L_2^2 - u^2/4}`.

If you guessed that the workspace was an ellipse like the author did,
that would be wrong. If you guessed some type of warped rectangle, the
you have great intuition.
Figure \ `[Fig:paralleltwolinkWS] <#Fig:paralleltwolinkWS>`__ shows the
workspace for the configuration domain :math:`[0, \pi/2]^2`. The figure
graphs :math:`y` positive going upwards and for the manipulator
:math:`y` positive goes down (so a vertical flip is required to match
up). The workspace can be created by running a program that traces out
all the possible arm angles and plots the resulting end effector
position [2]_. Sample code to plot this workspace is given below. It
uses a double loop over :math:`\theta_1` and :math:`\theta_2`, places
these values in the forward kinematics and then gathers the resulting
:math:`(x,y)` values. Like the serial manipulator, this is a holonomic
robot as well.

::

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

.. raw:: latex

   \centering

.. figure:: configuration/2dDeltaWS
   :alt: Parallel Two Link Workspace [Fig:paralleltwolinkWS]

   Parallel Two Link Workspace [Fig:paralleltwolinkWS]

The inverse kinematics will give you :math:`(\theta_1, \theta_2)` as a
function of :math:`(x,y)`. This is another exercise in trigonometry. For
:math:`(x,y)` given, we obtain

.. math:: \theta_1  = \pi - \beta - \eta , \quad \quad \theta_2 = \pi - \alpha - \gamma \label{paralleltwolinkIK}

\ where

.. math:: \| G \| = \sqrt{(x-L_0/2)^2 + y^2},  \quad\quad \| H\| = \sqrt{(x+L_0/2)^2 + y^2}

.. math:: \alpha = \cos^{-1} \frac{G^2 + L_0^2 - H^2 }{2GL_0}, \quad \quad \beta = \cos^{-1} \frac{H^2 + L_0^2 - G^2 }{2HL_0}

.. math:: \gamma = \cos^{-1} \frac{G^2 + L_1^2 - L_2^2 }{2GL_1},\quad \quad \eta =  \cos^{-1} \frac{H^2 + L_1^2 - L_2^2 }{2HL_1}

The next code example illustrates using the inverse kinematic formulas
for a specific pair of :math:`(x,y)` values.

::

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
and SciPy one can leverage existing code considerably. The scalar
(single) operations can be made into array operations (a type of
iterator) with little change in the code. The normal arithmetic
operators are overloaded and the iteration is done elementwise. Although
Python is normally much slower than a C equivalent, numpy is highly
optimized and the code runs close to the speed of C. [3]_

::

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
drive system known as differential drive. Using differential drive, we
are able to move to any position for which there is a sufficiently clear
path (to be explained below). The differential drive can rotate in place
to orientation is not a problem. The freedom to orient in place is not
something found in automobiles which tend to have a smaller
configuration space than differential drive systems.

For this example, we assume we have something like the differential
drive robot. Assume that you have a simple mobile robot with two driven
wheels and a third free unpowered wheel which can easily pivot or slide,
Figure \ `[fig:ddrive] <#fig:ddrive>`__. The drive wheels are not
steered but can be spun at different rates which will steer the robot.
This system is known as differential drive and is roughly analogous to
how a tank drive operates. It is necessary to develop equations of
motions for two reasons. The first reason is for simulating the dynamics
or motion of the robot so we can see the results of our robot control
software. The second reason is that the equations will be required in
localization algorithms.

.. raw:: latex

   \centering

.. raw:: latex

   \centering

.. figure:: motion/ddrive
   :alt: Rectangular frame. [fig:ddriverectangular]

   Rectangular frame. [fig:ddriverectangular]

.. raw:: latex

   \centering

.. figure:: motion/circular
   :alt: Circular frame. [fig:ddrivecircular]

   Circular frame. [fig:ddrivecircular]

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
known as the local coordinate system. You can think of it as a mini
coordinate system for an ant living on the robot. We will use the
convention that :math:`x` points forward or in the direction of travel.
:math:`y` is set along the wheel axle and :math:`z` is in the vertical
direction. To remove any ambiguity, we assume that :math:`x`, :math:`y`,
:math:`z` also follow a right hand rule (which in this case sets the
direction of :math:`y`).

.. raw:: latex

   \centering

.. figure:: motion/frames
   :alt: The global and local frames of reference. [refframe]

   The global and local frames of reference. [refframe]

The global coordinate system already has an origin defined. However, we
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
origin relative to the global frame. In other words, the local frame
origin position is :math:`(x_I,y_I)`. We then need to track the
orientation of the frame or in our case the robot. The angle,
:math:`\theta`, can be measure from either coordinate system and to be
consistent, we take it as the angle from the global frame to the local
frame. Graphically :math:`\theta` the amount of rotation applied to
:math:`X_I` to line it up with :math:`X_R`.

.. raw:: latex

   \centering

.. figure:: motion/ddframe
   :alt: The two frames of reference for a mobile robot: the inertial or
   global frame and the relative or local frame.[refddframe]

   The two frames of reference for a mobile robot: the inertial or
   global frame and the relative or local frame.[refddframe]

We can track the robot position by tracking its coordinate system origin
and orientation relative to the global coordinate system, :math:`\xi_I`.
So, we define the object relative to the robot by coordinates
:math:`\xi_R` and rotate into the ineetial frame:

.. math:: \xi_I = \begin{pmatrix} x \\ y \\ \theta \end{pmatrix}, \quad \xi_R= \begin{pmatrix} x' \\ y' \\ 0 \end{pmatrix}.

\ The movement of the robot traces a path, :math:`x(t)`, :math:`y(t)`,
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
compute. :raw-latex:`\cite{strang1988book}`

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

Working in instantenous local coordinate enables us to determine the
motion easily. We then use the rotation matrix to relate the robot
position in the global frame. To progress in the modeling process, we
need to know the specifics of the robot, illustrated in
Figure \ `[robotdimensions] <#robotdimensions>`__.

-  Wheel size: :math:`D`, so the radius :math:`r = D/2`

-  Axle length: :math:`2L` (:math:`L` is the distance from the origin of
   the coordinate system to a wheel)

-  Origin of local coordinate system: :math:`P` is placed on the
   midpoint of the axle.

.. figure:: motion/dddim
   :alt: Robot Dimensions.[robotdimensions]

   Robot Dimensions.[robotdimensions]

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
Figure \ `[axlevelocity] <#axlevelocity>`__. The differential speeds
then produce the rotational motion about the robot center and the
average forward velocity.

.. raw:: latex

   \centering

|Velocity of axle induced by wheel velocities.[axlevelocity]| |Velocity
of axle induced by wheel velocities.[axlevelocity]|

The speed of point :math:`P` is given by the weighted average based on
distances of the wheels to :math:`P`. To see this, we consider a couple
of cases. If the two wheel velocities are the same, then the average
works trivially. If the two velocities are different (but constant),
then the motion of the robot is a circle.
Figure \ `[axlevelocity] <#axlevelocity>`__ shows the robot motion.
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
Figure \ `[diffdriverotation] <#diffdriverotation>`__. The rotation
about :math:`P` is given by adding the individual contributions:

.. math:: \dot{\theta} =  \frac{r}{2L} (\dot{\phi_1} - \dot{\phi_2}).

.. raw:: latex

   \centering

.. figure:: motion/ddaxlerot
   :alt: The contribution of the two wheels towards rotational
   motion.[diffdriverotation]

   The contribution of the two wheels towards rotational
   motion.[diffdriverotation]

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

   \label{ddkinematicsmodel}
   \boxed{
   \begin{array}{l}
    \dot{x} = \frac{r}{2} (\dot{\phi_1}+\dot{\phi_2})\cos(\theta) \\[5mm]
   \dot{y} = \frac{r}{2} (\dot{\phi_1}+\dot{\phi_2})\sin(\theta) \\[5mm]
   \dot{\theta} = \frac{r}{2L} (\dot{\phi_1}-\dot{\phi_2})
   \end{array}}

 These are non-holonomic constraints, see
exercise \ `[DDisnotHolonomic] <#DDisnotHolonomic>`__.

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
circles \ `[fig:ddrivecircles] <#fig:ddrivecircles>`__ The two circles
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

.. raw:: latex

   \centering

.. figure:: motion/ddrive_circle
   :alt: [fig:ddrivecircles]A differential drive robot with constant
   wheel velocity drives in straight lines and circles.

   [fig:ddrivecircles]A differential drive robot with constant wheel
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

+-----------------------------------+-----------------------------------+
| :math:`t=0  \to 5`:               | :math:`\omega_1 = \omega_2 = 3.0` |
|                                   | ,                                 |
|                                   | :math:`\Rightarrow`               |
+-----------------------------------+-----------------------------------+
|                                   | :math:`(0,0,0)+(135,0,0)=(135,0,0 |
|                                   | )`                                |
+-----------------------------------+-----------------------------------+
| [1mm] :math:`t=5  \to 6`:         | :math:`\omega_1 = - \omega_2 = 2. |
|                                   | 0`,                               |
|                                   | :math:`\Rightarrow`               |
+-----------------------------------+-----------------------------------+
|                                   | :math:`(135,0,0) + (0,0,3/2) = (1 |
|                                   | 35,0,3/2)`                        |
+-----------------------------------+-----------------------------------+
| [1mm] :math:`t=6  \to 10`:        | :math:`\omega_1 = \omega_2 = 3.0` |
|                                   | ,                                 |
|                                   | :math:`\Rightarrow`               |
+-----------------------------------+-----------------------------------+
|                                   | :math:`(135,0,3/2)+(108\cos 3/2,  |
|                                   | 108\sin 3/2, 0)`                  |
+-----------------------------------+-----------------------------------+
|                                   | :math:`\approx (142.6, 107.7, 1.5 |
|                                   | )`                                |
+-----------------------------------+-----------------------------------+
| [1mm] :math:`t=10 \to 11`:        | :math:`\omega_1 = -\omega_2 = -2. |
|                                   | 0`,                               |
|                                   | :math:`\Rightarrow`               |
+-----------------------------------+-----------------------------------+
|                                   | :math:`(142.6, 107.7, 1.5)+(0, 0, |
|                                   |  -1.5) = (142.6, 107.7, 0)`       |
+-----------------------------------+-----------------------------------+
| [1mm] :math:`t=11 \to 16`:        | :math:`\omega_1 =  \omega_2 = 3.0 |
|                                   | `,                                |
|                                   | :math:`\Rightarrow`               |
+-----------------------------------+-----------------------------------+
|                                   | :math:`(142.6, 107.7, 0)+(135, 0, |
|                                   |  0) =                             |
|                                   |  (277.6, 107.7, 0)`               |
+-----------------------------------+-----------------------------------+

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
robots, this problem seems to arise more often. We tend to track the
entire machine in the workspace.

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
Figures \ `[Fig:RobotSize] <#Fig:RobotSize>`__, \ `[Fig:intro-mobile1] <#Fig:intro-mobile1>`__.
In this case the size of the robot affects the configuration space,
Figure \ `[Fig:intro-mobile2] <#Fig:intro-mobile2>`__. For a mobile
ground robot that is not a point, orientation will enter as a variable
in the system.

.. raw:: latex

   \centering

|Configuration space as a function of robot size. [Fig:RobotSize]|
|Configuration space as a function of robot size. [Fig:RobotSize]|

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

.. raw:: latex

   \centering

.. figure:: configuration/mobile
   :alt: Example of the inflation process. [Fig:intro-mobile1]

   Example of the inflation process. [Fig:intro-mobile1]

.. raw:: latex

   \centering

.. figure:: configuration/mobile2
   :alt: Relation between robot size and configuration space.
   [Fig:intro-mobile2]

   Relation between robot size and configuration space.
   [Fig:intro-mobile2]

The previous examples looked at a circular robot. What about a robot
which is a rectangle? What would be the configuration space about some
obstacle? Figure \ `[shapematters] <#shapematters>`__. The basic shape
of the robot is important as well as its orientation,
Figure \ `[orientationmatters] <#orientationmatters>`__. Inflation in
this case depends on the fixed orientation of the robot. One follows the
same process and pushes the robot up until it touches the obstacle.
Doing this for all locations around the obstacle all while keeping the
same orientation will describe the configuration space. Marking the
robot’s centroid at each contact allows us to trace a curve around the
obstacle and thus inflate the obstacle. We then can shrink the robot to
a point. We can then study robot paths through the open space. Of course
in practice this is absurd since the robot orientation is not fixed. But
it does help transition to the general case.

.. raw:: latex

   \centering

.. figure:: planning/rect
   :alt: Changing robot shape also affects c-space. [shapematters]

   Changing robot shape also affects c-space. [shapematters]

.. raw:: latex

   \centering

.. figure:: planning/rect2
   :alt: Changing robot orientation affects c-space as well.
   [orientationmatters]

   Changing robot orientation affects c-space as well.
   [orientationmatters]

It is helpful to see some examples of the inflation process. A
rectangular object does not just change scale. It changes shape as well.
For a rectangle, he inflated obstacle is a “rectangle” with rounded
corners. It is important to note that each rotation of the rectangle
generates a new and different configuration space,
Figure \ `[orientationmattersalot] <#orientationmattersalot>`__. This
process can be very complicated and often one will want to make
simplifications.

.. raw:: latex

   \centering

.. figure:: planning/rect3
   :alt: Two sample rotations and the configuration
   obstacle.[orientationmattersalot]

   Two sample rotations and the configuration
   obstacle.[orientationmattersalot]

Robot orientation then makes the configuration space question more
complicated since the configuration space is a function of the robot
orientation. A planning algorithm would then need to either fix the
robot orientation or be able to adjust to a changing landscape. To fix
orientation ultimately means that the orientation is independent of
travel direction. This is not the case for the vast majority of
vehicles. The orientation for a car, for example, is pointed in the
direction of travel. [4]_ To obtain this independence a holonomic robot
is required. The term holonomic will be carefully defined later, for
now, consider it a mobile robot that can set position and orientation
independently. Independent of the type of motion, it should be clear now
that position and orientation are separate and important variables in
the system which is addressed next.

The Piano Movers Problem - Orientation
--------------------------------------

Assume you want to route an object with a complicated shape through a
tight sequence of corridors. Routing a complex shape through a narrow
passage is often referred to as the piano movers problem. Take a simple
example, move the linear robot through the two blocks,
Figure \ `[robotmustrotate] <#robotmustrotate>`__. It is clear to the
human what has to happen. The robot must rotate. For a holonomic robot,
this simply means the controller issues a rotation command while
traveling to the corridor. For a non-holonomic robot, the control system
must change the path so that upon entry and through the corridor the
robot’s orientation will allow for passage. A significant problem arises
if the corridor is curved in a manner that is not supported by the
possible orientations defined by the vehicle dynamics. In plain English,
this is when you get the couch stuck in the stairwell trying to move
into your new flat.

.. raw:: latex

   \centering

.. figure:: planning/obst
   :alt: The object must rotate to fit through the open
   space.[robotmustrotate]

   The object must rotate to fit through the open
   space.[robotmustrotate]

As all of us learned when we were very young, we must turn sideways to
fit through a narrow opening. [5]_ This introduces a new aspect to
routing, that of reconfiguration of the robot. Examine a simple
reconfiguration which is simply a change in orientation. As we saw
above, each rotation of the robot induces a different configuration
space. Figure \ `[robotrotation] <#robotrotation>`__ shows the idea for
three different rotation angles, there are three different configuration
obstacle maps.

.. raw:: latex

   \centering

.. figure:: planning/obst2
   :alt: Different rotations produce different obstacle maps in
   configuration space.[robotrotation]

   Different rotations produce different obstacle maps in configuration
   space.[robotrotation]

Since each rotation generates a two dimensional configuration space,
they can be stacked up in three dimensions. So we have that
configuration space includes the vertical dimension which is the
rotation angle for the robot - the configuration space is three
dimensional. To restate, the configuration space includes all of the
configuration variables :math:`(x,y, \theta)` is now a three dimensional
configuration space which is shown in
Figure \ `[robotrotation3D] <#robotrotation3D>`__. So, although the
workspace is two dimensional, the configuration space is three
dimensional and are different objects.

.. raw:: latex

   \centering

.. figure:: planning/obst3
   :alt: The different rotations can be stacked where the vertical
   dimension is the rotation angle. [robotrotation3D]

   The different rotations can be stacked where the vertical dimension
   is the rotation angle. [robotrotation3D]

For a three dimensional object with a fixed orientation, would have a
three dimensional configuration space. For toolheads, only pitch and yaw
matter. To locate a point on a sphere you need two variables (think
about spherical coordinates): :math:`\theta` the angle in the
:math:`x`-:math:`y` plane and :math:`\phi` the angle from the :math:`z`
axis (or out of the plane if you prefer). For each pair
:math:`(\theta, \phi)` we have a 3D section. This tells us that the
configuration space is five dimensional. When roll, pitch and yaw all
matter then we have a 6 dimensional configuration space. If the robot is
configurable with other elements, then each parameter defining the
configuration would also add a variable to the mix and increase the
dimension of the configuration space.

The construction of configuration space then is built like slices in a
3D printer. Routing or path planning must be done in the full
configuration space. For the current example, we must route in 3D which
will translate to position and orientation routing in the workspace,
Figure \ `[obst4] <#obst4>`__. Path planning or motion planning is
addressed in Chapter \ `[Chap:Planning] <#Chap:Planning>`__.

.. raw:: latex

   \centering

.. figure:: planning/obst4
   :alt: We can see that there is a path that includes the
   rotation.[obst4]

   We can see that there is a path that includes the rotation.[obst4]

Two Link Arm Revisited
----------------------

Articulated (multilink) robot arms also have size and orientation.
Determining which configurations and which physical positions are
actually realizable is more complicated. The size of the robot arm will
affect the regions which the end effector can reach but obstacle
inflation does not give the same workspace. The end effector is designed
to touch an object and from that perspective little inflation is
required. However the base link of the arm might be very wide and does
affect the useable workspace. A simple obstacle inflation approach will
not work with manipulators. The reason is that how you travel affects
your reach. Figure \ `[Fig:pathmatters] <#Fig:pathmatters>`__ shows how
the path matters to access. A more situation can be found in
Figure \ `[Fig:nopaththrough] <#Fig:nopaththrough>`__. Even though the
articulator is small enough to pass through the gap, it cannot due to
the other physical restrictions.

.. raw:: latex

   \centering

.. figure:: ./configuration/pathmatters
   :alt: The elbow down approach is blocked, but not the elbow up
   position. [Fig:pathmatters]

   The elbow down approach is blocked, but not the elbow up position.
   [Fig:pathmatters]

.. figure:: ./configuration/nopaththrough
   :alt: Neither configuration of the robot arm can reach the point.
   [Fig:nopaththrough]

   Neither configuration of the robot arm can reach the point.
   [Fig:nopaththrough]

.. raw:: latex

   \FloatBarrier

Problems
--------

.. raw:: latex

   \setcounter{Exc}{0}

]

What is the difference between *effect* and *affect*?

What is the difference between accuracy and precision?

Define domain and range.

Is the following constraint holonomic:
:math:`\dot{x}_2\sin(x_1) + x_2 \cos(x_1)\dot{x}_1 = 1`.

:math:`x_2\sin(x_1) = t`

Sketch the workspace of a two-link manipulator centered at the origin
with :math:`a_1 = 15` and :math:`a_2 = 10`.

Assume that you have a two link planar manipulator. :math:`\theta_1` is
the angle between the x axis (measured clockwise as positive) and the
first link arm. :math:`\theta_2` is the angle between the second link
arm and the first link arm (again measured clockwise as positive). Given
the length of the first link :math:`a_1 = 12` and the second link
:math:`a_2 = 7` solve the following:

#. If :math:`\theta_1 = 45^\circ`, :math:`\theta_2 = 45^\circ`, find
   :math:`x` and :math:`y`.

#. If :math:`\theta_1 = 45^\circ`, :math:`\theta_2 = 45^\circ`,
   :math:`d\theta_1/dt = 5^\circ s^{-1}`,
   :math:`d\theta_2/dt = 10^\circ s^{-1}` find :math:`dx/dt` and
   :math:`dy/dt`.

#. If :math:`x = 12`, :math:`y = 14`, find :math:`\theta_1` and
   :math:`\theta_2`

Assume that you have a two link planar manipulator. :math:`\theta_1` is
the angle between the x axis (measured clockwise as positive) and the
first link arm. :math:`\theta_2` is the angle between the second link
arm and the first link arm (again measured clockwise as positive). Due
to servo limitations: :math:`-100^\circ < \theta_1 < 100^\circ`,
:math:`-150^\circ < \theta_2 < 150^\circ`. Also assume the first link is
20cm long and the second link is 15cm.

#. What is the configuration space?

#. What is the workspace?

Find the forward velocity kinematics equations for the two link
manipulator.

Assume that you have a two link manipulator that is operating in the
vertical plane :math:`x-z`. Attach the base to a rotational joint so the
arm rotates around the :math:`z` axis. [problem:twolink] See
Figure \ `[fig:two-link] <#fig:two-link>`__.

.. figure:: kinematics/twolinkalt2
   :alt: Two link manipulator in
   problem \ `[problem:twolink] <#problem:twolink>`__.[fig:two-link]

   Two link manipulator in
   problem \ `[problem:twolink] <#problem:twolink>`__.[fig:two-link]

#. Find the position of the end effector as a function of joint angles.

#. Find the inverse kinematic formula.

**A)** To find the position of the end effector as a function of joint
angles we need only to convert equation 2.1 in the text to polar
cylindrical coordinates. That is, y becomes z and x becomes r,
yielding:\ 

.. math::

   \label{two_link_eq}\begin{bmatrix}
       r\\
       z
   \end{bmatrix}
   =
   \begin{bmatrix}
       a_2\cos(\theta_1 + \theta_2) + a_1\cos(\theta_1) \\
       a_2\sin(\theta_1 + \theta_2) + a_1\sin(\theta_1)
   \end{bmatrix}

Math tells us that we may convert between cylindrical coordinates to
Cartesian using the formula:

.. math::

   \begin{bmatrix}
       x\\
       y
   \end{bmatrix}
   =
   \begin{bmatrix}
       r\cos(\theta) \\
       r\sin(\theta)
   \end{bmatrix}

Substitution into equation `[two_link_eq] <#two_link_eq>`__ gives us the
final equation for the 2 link manipulator on a pivot:

.. math::

   \label{two_link_pivot_eq}
   \boxed{
   \begin{bmatrix}
       x\\
       y\\
       z
   \end{bmatrix}
   =
   \begin{bmatrix}
       \cos(\theta_3)[a_2\cos(\theta_1 + \theta_2) + a_1\cos(\theta_1)] \\
       \sin(\theta_3)[a_2\cos(\theta_1 + \theta_2) + a_1\cos(\theta_1)] \\
       a_2\sin(\theta_1 + \theta_2) + a_1\sin(\theta_1)
   \end{bmatrix}
   }

**B)** To compute the inverse kinematics begin with the solutions from
the book while converting from Cartesian to polar cylindrical
coordinates (substituting r and z for x and y respectively):

.. math:: D = \frac{r^2 + z^2 - a_1^2 - a_2^2}{2a_{1}a_{2}}

.. math:: \theta_2 = \arctan\left(\frac{\pm\sqrt{1-D^2}}{D}\right)

.. math:: \theta_1 = \arctan\left(\frac{z}{r}\right) - \arctan\left(\frac{a_2\sin(\theta_2)}{a_1+a_2\cos(\theta_2)}\right)

Assume that you have a two link manipulator with :math:`a_1 = 15`\ cm
and :math:`a_2 = 15`\ cm and that the base of the manipulator is at the
origin of the coordinate system. Write a Python program to take the list
of workspace points and plug them into the inverse kinematics formulas
for the two link manipulator. Plot these points on a graph where
:math:`\theta_1` is the horizontal axis and :math:`\theta_2` is the
vertical axis. You will have to adjust some aspects to get a good
looking plot. (Scale factors etc.) Test your code on the workspace line
(a) :math:`x+y = 25`, :math:`x, y >0` and (b)
:math:`x = 10\cos (t) + 15`, :math:`y = 10\sin (t)` for
:math:`0 \leq t \leq \pi`. The point here is to see what the
configuration space curve looks like.

Using the code from the text as a starting point, we can code a NumPy
solution.

::

    from math import *
    import numpy as np
    import pylab as plt

    a1, a2 = 15, 15
    x = np.arange(0, 25, 0.2)   # a range of x values
    y = 25 - x    # an array of y values using this function
    denom = (2.0*a1*a2)
    numer = a1*a1+a2*a2

    d =  (x*x+y*y-numer)/denom
    t2 = np.arctan2(-np.sqrt(1.0-d*d),d)
    t1 = np.arctan2(y,x) - np.arctan2(a2*np.sin(t2),a1+a2*np.cos(t2))

    plt.plot(x,y,'r-')
    plt.savefig("hw1ch2p7a.pdf")
    plt.show()
    plt.plot(t1,t2,'b-')
    plt.savefig("hw1ch2p7b.pdf")
    plt.show()

| The plot for the line and the two link manipulator angles:
| |image| |image|
| Modify the function above (:math:`y-25-x`) and use the parametric form
  for a half circle.

::

    t = np.arange(0,pi,0.1)
    x = 10.0*np.cos (t) + 15.0
    y = 10.0*np.sin (t)

| and you obtain the following the circle shape and the angle values:
| |image| |image|

Assume that you have a two link manipulator with :math:`a_1 = 15`\ cm
and :math:`a_2 = 15`\ cm and that the base of the manipulator is at the
origin of the coordinate system. Write a two link manipulator location
program (Python). This program will take a list of angles and compute
the location of the end effector. Show how this program works with the
list of angles you generated in the previous problem. If the angle
inputs are generated by a square, the simulated robot arm’s end effector
should trace a square. Plot the end effector points. You need to plot
the input shape and the final shape to see if your code is correct. You
will need to use the previous problem for this problem. Demonstrate your
code to trace out the four segments which form the square with endpoints
(5,0), (5, 15), (20, 15), (20,0).

The text has the forward kinematics formulas and some example code.
Using that code as a starting point, we first need to find the angle
list for the square with endpoints (5,0), (5, 15), (20, 15), (20,0).
This can be done using the code from the last problem. Then we take the
angle list and place into the forward kinematics. The code for the
square is provided (the plot commands are removed for space). The list
of commands from z to zero are a bunch of arrays which will be appended
together to form the list of points for the square.

::

    from math import *
    import numpy as np
    import pylab as plt

    a1, a2 = 15, 15
    z = np.arange(0,15, 0.1)  # range points from 0 to 15
    z5 = z+5  # a range of points from 5 to 20
    z20 = 20-z  # a range of points from 20 to 5 (decreasing)
    z15 = 15-z  # a range of points from 15 to 0 (decreasing)
    c5 = 5*np.ones(z.size)  # an arrray of 5's (same size as z)
    c15 = c5+10.0    # an arrray of 15's (same size as z)
    c20 = c5+15.0    # an arrray of 20's (same size as z)
    zero = np.zeros(z.size)     # an arrray of 0's (same size as z)

    x = np.append(c5, z5)   ##  Glue the arrays above to produce
    x = np.append(x, c20)   ##  a list of points that take us around 
    x = np.append(x,z20)    ##  the square.
    y = np.append(z, c15)   ##  Same for the y values
    y = np.append(y,z15)
    y = np.append(y,zero)

    denom = (2.0*a1*a2)
    numer = a1*a1+a2*a2
    d =  (x*x+y*y-numer)/denom
    t2 = np.arctan2(-np.sqrt(1.0-d*d),d)
    t1 = np.arctan2(y,x) - np.arctan2(a2*np.sin(t2),a1+a2*np.cos(t2))

    xout = a2*np.cos(t1+t2) + a1*np.cos(t1)
    yout = a2*np.sin(t1+t2) + a1*np.sin(t1)

| The plots of the resulting line and half circle:
| |image| |image|
| The plot of the original square, the angles for the square and the
  square drawn by the two link kinematics:
| |image| |image|
| |image|

Typos can creep up in textbooks, papers and reference materials. How
would test the accuracy of the formulas given in equations
`[paralleltwolinkforward] <#paralleltwolinkforward>`__ and
`[paralleltwolinkIK] <#paralleltwolinkIK>`__? Discuss.

The first test would be to see if the forward and inverse kinematics are
actually inverses. This was demonstrated in the previous problems. We
started with (x,y) points (the square for example). The we used the
inverse kinematics to find the theta angles. Those angles were plugged
into the forward kinematics and the original figure (original (x,y)’s)
were recovered. [One should also start with thetas, find (x,y)’s and
then recover the thetas to be complete.] So, have shown that these two
are inverses, but not that they actually related to the problem at hand.
For example, :math:`f(x) = x^2` and :math:`g(x)=\sqrt{x}` are inverses
but not at all related to the manipulator. For this manipulator, both
forward and inverse kinematics were derived from the diagram and not
from one another. So, they are independent derivations which reduces our
concern. A careful diagram with accurate lengths and angles can also
help with our confidence. A careful analysis of the mathematics of the
derivation is the essential ingredient for belief in accuracy. The next
two problems walk through the derivation.

Find the forward velocity kinematics equations for the parallel two link
manipulator.

Derive the formula for
equation \ `[paralleltwolinkforward] <#paralleltwolinkforward>`__:

.. math:: (x,y) = \left( \frac{a+c}{2} + \frac{v (b-d)}{u} , \frac{b+d}{2} + \frac{v (c-a)}{u} \right)

Hint: define the segment from :math:`(a,b)` to :math:`(c,d)` as
:math:`B` (the base of the triangle), and :math:`\vec{A}` as a vector
which is a perpendicular to :math:`B`, see
Figure \ `[Fig:paralleltwolink3] <#Fig:paralleltwolink3>`__.

.. raw:: latex

   \centering

.. figure:: configuration/2dDelta3
   :alt: Extraction of the isosceles triangle. [Fig:paralleltwolink3]

   Extraction of the isosceles triangle. [Fig:paralleltwolink3]

Define the horizontal as the x-axis and the vertical (down) as the
y-axis and the origin be at the center of the base link :math:`L_0` as
shown:

|image|

The location of the point :math:`(a,b)` and :math:`(c,d)` can be found
using the angles

.. math:: (a,b) = \left( -L_0/2 - L_1 \cos(\theta_1) , L_1\sin(\theta_1) \right) , \quad   (c,d) = \left( L_0/2 + L_1 \cos(\theta_2) , L_1\sin(\theta_2) \right)

The distance between the points :math:`(a,b)` and :math:`(c,d)` is
:math:`u = \sqrt{(a-c)^2+(b-d)^2}` and the location of the point B is
given by

.. math:: \left(\frac{a+c}{2}, \frac{b+d}{2}\right).

The vector from :math:`(a,b)` to :math:`(c,d)` is
:math:`\langle c-a, d - b \rangle`. The length of this vector is
:math:`u`. To get from B to :math:`(x,y)` we must travel in the
direction which is perpendicular to :math:`\langle c-a, d - b \rangle`:
:math:`\langle b-d, c-a\rangle`.

|image|

To be a direction vector it should be normalized by dividing by its
length:

.. math:: B^\perp = \left< \frac{b-d}{u}, \frac{c-a}{u}\right> .

\ Lastly we need to determine how far to travel in this direction,
:math:`v`: :math:`A = vB^\perp`. This can be found by the Pythagorean
theorem: :math:`L_2^2 = (u/2)^2 + v^2`. Solving for :math:`v`:
:math:`v = \sqrt{L_2^2-u^2/4}`. :math:`v` is the distance we travel down
and so we gain the displacement vector:

.. math:: A = \left< \frac{v(b-d)}{u}, \frac{v(c-a)}{u}\right> .

Add the displacement vector to B and we gain :math:`(x,y)`:

.. math:: (x,y) = \left( \frac{a+c}{2} + \frac{v (b-d)}{u} , \frac{b+d}{2} + \frac{v (c-a)}{u} \right)

Derive the formulas for the parallel two link manipulator inverse
kinematics given in
equations \ `[paralleltwolinkIK] <#paralleltwolinkIK>`__. Hint:
Figure \ `[Fig:paralleltwolinkIK] <#Fig:paralleltwolinkIK>`__.

.. raw:: latex

   \centering

.. figure:: configuration/2dDelta4
   :alt: Parallel Two Link Inverse Kinematics variables
   [Fig:paralleltwolinkIK]

   Parallel Two Link Inverse Kinematics variables
   [Fig:paralleltwolinkIK]

Using the diagram for labels, we are given :math:`(x,y)`, :math:`L_0`,
:math:`L_1` and :math:`L_2`.

|image|

We also know the attachment points for the arms indicated by the green
dots. These are :math:`(-L_0/2,0)` on the left and :math:`(L_0/2,0)` on
the right. This allows one to compute the distances for the segments
:math:`G` and :math:`H`.

.. math:: \| H \| = \sqrt{(x+L_0/2)^2 + y^2}, \quad \| G \| = \sqrt{(x-L_0/2)^2 + y^2}

Knowing the three sides of a triangle allows one to compute interior
angles. So, we can determine :math:`\alpha`, :math:`\beta`,
:math:`\gamma` and :math:`\eta` using the law of cosines.

.. math:: \alpha = \cos^{-1} \frac{G^2 + L_0^2 - H^2 }{2GL_0}, \quad \quad \beta = \cos^{-1} \frac{H^2 + L_0^2 - G^2 }{2HL_0},

.. math:: \gamma = \cos^{-1} \frac{G^2 + L_1^2 - L_2^2 }{2GL_1},\quad \quad \eta =  \cos^{-1} \frac{H^2 + L_1^2 - L_2^2 }{2HL_1}.

One also recalls that the angles for the line sum to :math:`\pi` and we
obtain:

.. math:: \theta_1  = \pi - \beta - \eta , \quad \quad \theta_2 = \pi - \alpha - \gamma

Assume that you have a parallel two link manipulator with
:math:`L_0 = 10`\ cm, :math:`L_1 = 15`\ cm and :math:`L_2 = 20`\ cm.
Write a Python program to take the list of workspace points given and
plug them into the inverse kinematics formulas for the two link
manipulator. Plot these points on a graph where :math:`\theta_1` is the
horizontal axis and :math:`\theta_2` is the vertical axis. As above, you
will have to adjust some aspects to get a good looking plot. The point
here is to see what the configuration space curve looks like. The
workspace points are the list of points for the rectangle with corners
(-5, 18), (5, 18), (5, 27), (-5,27). Use 10 points in each side of the
rectangle.

For this problem we can generate the square by appending linspace
arrays. Those points are placed into the inverse kinematics formula and
the link arm angles are produced. The bottom of the file plots the data.

::

    from math import *
    import numpy as np
    import pylab as plt

    L0, L1, L2 = 10, 15, 20

    x = np.append(np.linspace(-5,5, 10), 5*np.ones(10))
    x = np.append(x, np.linspace(5,-5, 10))
    x = np.append( x,-5.0*np.ones(10))
    y = np.append(18*np.ones(10), np.linspace(18,27,10))
    y = np.append(y,27*np.ones(10))
    y = np.append(y,np.linspace(27,18,10))

    G = np.sqrt((x-L0/2.0)*(x-L0/2.0)+y*y)
    H = np.sqrt((x+L0/2.0)*(x+L0/2.0)+y*y)
    alpha = np.arccos((G*G + L0*L0 - H*H)/(2.0*G*L0))
    beta = np.arccos((H*H + L0*L0 - G*G)/(2.0*H*L0))
    gamma = np.arccos((G*G + L1*L1 - L2*L2)/(2.0*G*L1))
    eta = np.arccos((H*H + L1*L1 - L2*L2)/(2.0*H*L1))
    th1 = pi - beta - eta
    th2 = pi - alpha - gamma

    plt.gca().set_ylim([-1,max(y)+3])
    plt.gca().set_xlim([min(x)-11,max(x)+11])
    plt.gca().invert_yaxis()
    plt.scatter(x,y, color='#ADD8E6', marker='o')
    plt.axvline(linewidth=1.3, color = 'green')
    plt.axhline(linewidth=1.3, color = 'green')
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.show()

    plt.scatter(th1,th2, color= '#ADD8E6', marker='o')
    plt.axvline(linewidth=1.3, color = 'green')
    plt.axhline(linewidth=1.3, color = 'green')
    plt.xlabel("Theta 1")
    plt.ylabel("Theta 2")
    plt.xticks([0, pi/6, pi/4, pi/3, pi/2],['$0$', r'$\frac{\pi}{6}$', r'$\frac{\pi}{4}$', r'$\frac{\pi}{3}$', r'$\frac{\pi}{2}$'])
    plt.yticks([0, pi/6, pi/4, pi/3, pi/2],['$0$', r'$\frac{\pi}{6}$', r'$\frac{\pi}{4}$', r'$\frac{\pi}{3}$', r'$\frac{\pi}{2}$'])
    plt.grid()
    plt.show()

| The curves:
| |image|
| |image|

Assume that you have a parallel two link manipulator with
:math:`L_0 = 10`\ cm, :math:`L_1 = 15`\ cm and :math:`L_2 = 20`\ cm.
Write a Python program that will take a list of angles and compute the
location of the end effector. Show how this program works with the list
of angles you generated in the previous problem. [If the angle inputs
are generated by a rectangle, the simulated robot arm’s end effector
should trace a rectangle.] Plot the end effector points. You will need
to use the previous problem for this problem.

Adding to the solution above we can append the code and simulate the
manipulator drawing the rectangle.

::

    a = -L1*np.cos(th1) - L0/2.0
    b = L1*np.sin(th1)
    c = L1*np.cos(th2) + L0/2.0
    d = L1*np.sin(th2)

    dx = c-a
    dy = b-d
    u = np.sqrt(dx*dx+dy*dy)
    v = np.sqrt(L2*L2 - 0.25*u*u)
    xout = (a+c)/2.0 + v*dy/u
    yout = (b+d)/2.0 + v*dx/u

    plt.gca().set_ylim([-1,max(yout)+3])
    plt.gca().set_xlim([min(xout)-11,max(xout)+11])
    plt.gca().invert_yaxis()
    plt.scatter(x,y, color='#ADD8E6', marker='o')
    plt.axvline(linewidth=1.3, color = 'green')
    plt.axhline(linewidth=1.3, color = 'green')
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.show()

| The curve appears as:
| |image|

[Ex:HalfDisk] Using Numpy and the linspace command, build an array of
points for Figure \ `[Fig:shapeforhw] <#Fig:shapeforhw>`__. The top is
given by :math:`(x-10)^2 + (y-8)^2 = 25` and the bottom is the line
segment along :math:`y=8`. Traverse the figure starting at the right
corner, going counter clockwise (circle first) and ending on the line
segment. Check this with the Python plot command. Show the result.

.. raw:: latex

   \centering

.. figure:: configuration/halfcircle
   :alt: Half disk for Ex \ `[Ex:HalfDisk] <#Ex:HalfDisk>`__
   [Fig:shapeforhw]

   Half disk for Ex \ `[Ex:HalfDisk] <#Ex:HalfDisk>`__ [Fig:shapeforhw]

We will use the same “array appending” approach as in previous problems.

::

    from math import *
    import numpy as np
    import pylab as plt

    a1, a2 = 15, 15
    t = np.arange(0,pi,0.1)
    x1 = 5.0*np.cos (t) + 10.0
    y1 = 5.0*np.sin (t) + 8.0
    x2 = np.arange(5,15,0.1)
    y2 = 8.0*np.ones(x2.size)
    x = np.append(x1,x2)
    y = np.append(y1,y2)
    plt.xlim([0,20])
    plt.ylim([0,20])
    plt.plot(x,y,'g-')
    plt.savefig("hw1ch2p14.pdf")
    plt.show()

|image|

Is the differential drive motion model given by
Equations \ `[eqn:DDequationsTerms] <#eqn:DDequationsTerms>`__
holonomic? Why or why not?

When inflating an obstacle, how much do you inflate it by?

Find the rotation matrix that will rotate clockwise by :math:`30^\circ`.

Vector review

#. Given the vector :math:`<1, 2>`, rotate this by 37 degrees
   (positive),

#. If an axle is rotated off of the x-axis by 64 degrees, what is the
   vector that is in-line (parallel) the the axle?

#. What is the projection of :math:`<3,1>` onto the axle direction in
   the previous part?

Show that the inverse rotation matrix is the same matrix as replacing
:math:`\theta` by :math:`-\theta`.

The rotation matrix is

.. math::

   R(\theta) = 
   \begin{pmatrix}
   \cos\theta & -\sin\theta \\[2mm]
   \sin\theta & \cos\theta 
   \end{pmatrix}

Replacing :math:`\theta` by :math:`-\theta` and then using that
:math:`\cos` is even and :math:`\sin` is odd we have

.. math::

   R(-\theta) = 
   \begin{pmatrix}
   \cos(-\theta) & -\sin(-\theta) \\[2mm]
   \sin(-\theta) & \cos(-\theta)
   \end{pmatrix}
   = 
   \begin{pmatrix}
   \cos(\theta) & \sin(\theta) \\[2mm]
   -\sin(\theta) & \cos(\theta)
   \end{pmatrix}

Multiply :math:`R(\theta)` times :math:`R(-\theta)`:

.. math::

   R(\theta)R(-\theta)=
   \begin{pmatrix}
   \cos\theta & -\sin\theta \\[2mm]
   \sin\theta & \cos\theta 
   \end{pmatrix}
   \begin{pmatrix}
   \cos(\theta) & \sin(\theta) \\[2mm]
   -\sin(\theta) & \cos(\theta)
   \end{pmatrix}

.. math::

   = 
   \begin{pmatrix}
   \cos^2(\theta) + \sin^2(\theta) & \cos(\theta)\sin(\theta) - \cos(\theta)\sin(\theta)\\[2mm]
   \cos(\theta)\sin(\theta) - \cos(\theta)\sin(\theta) & \cos^1(\theta) + \sin^2(\theta)
   \end{pmatrix}
   = 
   \begin{pmatrix}
   1 & 0 \\[2mm]
   0 & 1
   \end{pmatrix}

We have the same result for :math:`R(-\theta)R(\theta)`, so this shows
that :math:`R(\theta)^{-1} = R(-\theta)`.

Assume that your differential drive robot has 10 cm diameter wheels and
a total axle length (wheel to wheel) of 20 cms. If both wheels are
turning at 0.8 revolutions per second, what is the speed of the robot.
[basicddhw]

The orientation does not matter here so we can assume the robot is
pointed in the direction of the :math:`x`-axis. We use the equations in
the text, \ `[ddkinematicsmodel] <#ddkinematicsmodel>`__,

.. math::

   \begin{array}{l}
    \dot{x} = \frac{r}{2} (\dot{\phi_1}+\dot{\phi_2})\cos(\theta) \\[5mm]
   \dot{y} = \frac{r}{2} (\dot{\phi_1}+\dot{\phi_2})\sin(\theta) \\[5mm]
   \dot{\theta} = \frac{r}{2L} (\dot{\phi_1}-\dot{\phi_2}).
   \end{array}

With :math:`\theta = 0` we see that :math:`\dot{y} = 0` and
:math:`\dot{\theta}=0`. One should note that we are given 0.8
revolutions per second and this converts to :math:`0.8(2\pi) = 1.6\pi`
radians per second. Using the remaining (first) equation

.. math:: \dot{x} = \frac{r}{2} (\dot{\phi_1}+\dot{\phi_2})\cos(\theta) = (5/2) (2* 1.6\pi) \approx 25.13274 ~\mbox{cm/s}

Using the same robot as problem \ `[basicddhw] <#basicddhw>`__, but
where the left wheel is turning at 1.5 radians per second and the right
wheel is turning at 1.8 radians per second. Determine the linear
velocity and path of the robot. You may assume the initial pose is
(0,0,0) at :math:`t=0`.

The velocity

.. math:: v = \sqrt{ \dot{x}^2 + \dot{y}^2} = \frac{r}{2} |\dot{\phi_1}+\dot{\phi_2}| = (5/2) |1.5 + 1.8| = 8.25 ~\mbox{cm/s}

The path can be determined from first solving the :math:`\theta`
equation:
:math:`\dot{\theta} = \frac{r}{2L} (\dot{\phi_1}-\dot{\phi_2}) = (5/20)(0.3) = 0.075`,
:math:`\theta = 0.075 t`. Then plug this into the first two equations:

.. math::

   \begin{array}{l}
    \dot{x} = \frac{r}{2} (\dot{\phi_1}+\dot{\phi_2})\cos(\theta)  = 8.25 \cos( 0.075 t)  \\[5mm]
   \dot{y} = \frac{r}{2} (\dot{\phi_1}+\dot{\phi_2})\sin(\theta)  = 8.25 \sin( 0.075 t) .
   \end{array}

Integrating gives us

.. math::

   \begin{array}{l}
    x = 110\sin( 0.0375 t) + x_0\\[5mm]
   y = 110 \cos( 0.0375 t) + y_0 .
   \end{array}

At :math:`t=0`, we have :math:`x = y = 0`. So :math:`x_0 = 0` and
:math:`y_0 = -110`.

.. math:: x^2 + (y+110)^2 = (110\sin( 0.075 t))^2 + (110 \cos( 0.075 t) )^2 = 110^2

The path is a circle of radius 110 centered at (0,-110).

For the differential drive robot, let :math:`r=10`, :math:`L=15`,
:math:`\dot{\phi_1} = 0.9` :math:`\dot{\phi_2}= 1.2`.

#. What is the angular velocity of the robot?

#. What is the velocity vector for the robot when
   :math:`\theta = 45^\circ`?

**1** Solve for the angular velocity :math:`\dot{\theta}` of the
robot:\ 

.. math::

   \begin{aligned}
   \dot{\theta} &= \frac{r}{2L}\left(\dot{\phi_1}-\dot{\phi_2}\right)\\[15pt]
                        &= \frac{\SI{10}{\centi\meter}}{2(\SI{15}{\centi\meter})}\left(\SI{0.9}{\radian \per\second}-\SI{1.2}{\radian\per\second}\right)\\[15pt]
                        &= (0.33)(\SI{-0.3}{\radian\per\second})\\[15pt]
                        &= \SI{-0.099}{\radian\per\second}\end{aligned}

**2** Solve for the velocity vector of the robot given
:math:`\theta = 45^\circ`:

.. math::

   \begin{aligned}
   \dot{x} &= \frac{r}{2} (\dot{\phi_1}+\dot{\phi_2})\cos(\theta)\\[15pt]
                &= \frac{\SI{10}{\centi\meter}}{2} (\SI{0.9}{\radian\per\second}+\SI{1.2}{\radian \per\second})\cos\left(\frac{pi}{4}\right)\\[15pt]
                &= \SI{7.42}{\centi\meter\per\second}\end{aligned}

.. math::

   \begin{aligned}
   \dot{y} &= \frac{r}{2} (\dot{\phi_1}+\dot{\phi_2})\cos(\theta)\\[15pt]
                &= \frac{\SI{10}{\centi\meter}}{\SI{2}{\radian}} (\SI{0.9}{\radian \per\second}+\SI{1.2}{\radian \per\second})\sin\left(\frac{pi}{4}\right)\\[15pt]
                &= \SI{7.42}{\centi\meter \per\second}\end{aligned}

.. math::

   \begin{aligned}
   \begin{bmatrix} \dot{x}\\ \dot{y}\\ \end{bmatrix} &=  \begin{bmatrix} 7.42\\ 7.42\\ \end{bmatrix}\end{aligned}

Let :math:`r=10`, :math:`L=15`. If you program the robot to drive
straight and the robot traces out a circle of diameter 3 meters while
traveling 1 m/s, what are the two wheel speeds?

Say you have a differential drive robot that has an axle length of 30cm
and wheel diameter of 10cm. Find the angular velocity for the left and
right wheel if the robot is going to

#. Spin in place at a rate of 6 rpm (revolutions per min),

#. Drive a circle of radius 1 meter (measured center of circle to middle
   of axle) at 3 rpm,

#. Drive a straight line at 1 meter / min.

#. *Spin in place at a rate of 6 rpm (revolutions per min).* We can
   approach this problem using the general equations or track wheel
   motion. Using the general equations, to spin in place
   :math:`\dot{x} = \dot{y}=0`. It also must be the case that
   :math:`\dot{\phi_1} = -\dot{\phi_2}`. 6 revolutions per minute will
   be :math:`12\pi` radians per minute. Using the :math:`\dot{\theta}`
   equation we have
   :math:`\dot{\theta} = \frac{r}{2L} (\dot{\phi_1}-\dot{\phi_2}) = (5/30)(2\dot{\phi_1}) =
   (1/3) \dot{\phi_1}`. This gives :math:`12\pi = (1/3) \dot{\phi_1}` or
   :math:`\dot{\phi_1} = 36\pi`, :math:`\dot{\phi_2} =-36\pi`.

#. *Drive a circle of radius 1 meter (measured center of circle to
   middle of axle) at 3 rpm*. The outer wheel will trace a circle of
   radius 1.15m and the inner wheel of radius 0.85m. This gives a
   distance of :math:`2.3\pi`\ m and :math:`1.7\pi`\ m. At 3rpm, we have
   :math:`6.9\pi`\ m/min and :math:`5.1\pi`\ m/min. Converting to wheel
   rotation :math:`6.9\pi /(0.1 \pi) = 69` rev/min,
   :math:`5.1\pi/(0.1\pi) = 51`\ revs/min. These can be converted to
   rads/min: outside wheel is 433.5 rad/min, inside wheel is 320.4
   rads/min.

#. *Drive a straight line at 1 meter / min.* Just use the angular to
   linear velocity conversion (:math:`v = r\omega`):
   :math:`\dot{\phi} = 1/r = 1/0.05 = 20`.

| Given a differential drive robot starting from (0,0,0) find the final
  position when wheel velocities are given by:
| t=0 to t=5: :math:`\omega_1` = 2, :math:`\omega_2` = 2
| t=5 to t=6: :math:`\omega_1` = 3, :math:`\omega_2` = 4
| t=6 to t=10: :math:`\omega_1` = 1, :math:`\omega_2` = 2
| where D=10, L=16.

List the variables in the configuration space of a circular ground robot
that can drive around and use a telescopic arm with a rotational base,
lifting servo and elbow joint servo.

Although a circular robot would appear to be rotationally symmetric, the
robotic arm breaks that symmetry. The vehicle has three degrees of
freedom :math:`x`, :math:`y`, :math:`\theta`. The arm has the base and
two servos. So the arm has three degrees of freedom. The total is six
degrees of freedom.

[DDisnotHolonomic] Show that the differential drive kinematic equations
are non-holonomic constraints.

.. raw:: latex

   \Closesolutionfile{Answer}

.. [1]
   Like the office chair races in the hallway.

.. [2]
   Well, not all points, but a dense sample of points will do just fine.

.. [3]
   Well, this is true on one Tuesday afternoon a long time ago with one
   little comparison of some loop/math code. Your results may be very
   different.

.. [4]
   Under normal conditions this is true, however, icy roads will allow
   for much greater freedom of vehicle orientation and travel direction.

.. [5]
   Cavers will tell you that you can crawl through a vertical gap
   spanned by the distance of your thumb and your fifth (pinky) finger.
   For the average American, this is a very small gap.

.. |a) Workspace for the two link manipulator with equal link lengths. b) Workspace obstacle for the two link manipulator. [two-link-disk]| image:: configuration/twolinkconfig
.. |a) Workspace for the two link manipulator with equal link lengths. b) Workspace obstacle for the two link manipulator. [two-link-disk]| image:: configuration/twolinkobs
.. |Configuration domain and configuration topology which is a torus. [intro-config-axis]| image:: configuration/twolinkconfigdomain
.. |Configuration domain and configuration topology which is a torus. [intro-config-axis]| image:: configuration/twolinktorus
.. |Parallel Two Link (a) configuration space (b) with coordinates [Fig:paralleltwolink2]| image:: configuration/2dtwolinkconfigdomain
.. |Parallel Two Link (a) configuration space (b) with coordinates [Fig:paralleltwolink2]| image:: configuration/2dDelta2
.. |Velocity of axle induced by wheel velocities.[axlevelocity]| image:: motion/ddaxle
.. |Velocity of axle induced by wheel velocities.[axlevelocity]| image:: motion/ddforward
.. |Configuration space as a function of robot size. [Fig:RobotSize]| image:: planning/circle1
.. |Configuration space as a function of robot size. [Fig:RobotSize]| image:: planning/circle2
.. |image| image:: solutions/Terms/hw1ch2p7a
.. |image| image:: solutions/Terms/hw1ch2p7b
.. |image| image:: solutions/Terms/hw1ch2p7c
.. |image| image:: solutions/Terms/hw1ch2p7d
.. |image| image:: solutions/Terms/hw1ch2p8c
.. |image| image:: solutions/Terms/hw1ch2p8d
.. |image| image:: solutions/Terms/hw1ch2p8a
.. |image| image:: solutions/Terms/hw1ch2p8b
.. |image| image:: solutions/Terms/hw1ch2p8e
.. |image| image:: configuration/2dDelta2
.. |image| image:: configuration/2dDelta3
.. |image| image:: configuration/2dDelta4
.. |image| image:: solutions/Terms/hw1ch2p12a
.. |image| image:: solutions/Terms/hw1ch2p12b
.. |image| image:: solutions/Terms/hw1ch2p13
.. |image| image:: solutions/Terms/hw1ch2p14

