Exploration and Navigation
--------------------------

Motion planning is complicated. Even when we restrict ourselves to two
dimensions and have a polygonal constraint set, the algorithms for
successful motion are much harder than one would expect. We will use the
term navigation to mean the process of guiding the robot from one point
in the workspace (or configuration space) to another. Implicitly we mean
that we look for an acceptable path from one point to another that
avoids collisions and respects machine constraints. To navigate, it is
necessary to know the starting point of the process, the ending point of
the process and the current location. Determining the current location
of the robot relative to the landscape turns out to be a very difficult
problem in general. This is known as localization.

In many applications, a map of the environment is necessary. The map
might be given apriori. The map might be the desired result of the robot
exploration. In the latter case, the process of generating a map, simply
known as mapping is another significant challenge in the exploration
process. There is often a “chicken and egg” problem that arises. If you
don’t have a map, then it is not possible to localize. Without
localization, one can’t build a map from sensor data. It least this is
how it seems at first. The process known as SLAM, Simultaneous
Localization and Mapping, addresses this problem by building the map
dynamically and tracking robot location. This is discussed in a later
chapter.

There are many approaches to navigation. We will separate them based on
algorithmic properties as well as computation difficulty. Does the
algorithm provide the optimal result (such as the shortest path) or does
it return a satisfactory solution. Will the algorithm always return a
solution if one exists (meaning the algorithm is complete) or can it
fail without definitive answer? Is the algorithm deterministic or
stochastic?

Some algorithms are fast (and this is relative to the current hardware)
and some are slow. Algorithms that construct a solution as the robot
navigates are said to be online. Those that construct a path apriori, in
a batch mode, are said to be offline. Sometimes offline and online are
used as a speed issue and this is a function of the computing power on
the robot. Does the algorithm increase in memory or computational
requirements as a function of runtime or obstacles encountered? If so,
is the growth polynomial or exponential? These questions and more arise
in the study of motion planning as the do in any course on algorithms.
We will start with some relatively simple problems.

The wonderful text *Principles of Robot
Motion* :raw-latex:`\cite{Choset:2005:PRM}` begins the study of planning
with three basic navigation algorithms or planners that are similar to
the maze routines presented in the last section. These are adaptations
of basic planners by Lumelsky and
Stepanov :raw-latex:`\cite{lumelsky:1987}`. Using very simple models one
can strip out the non-essential elements and focus on the core issues
related to path planning. Even so, as Choset points out “even a simple
planner can present interesting and difficult issues." The bug
algorithms of Lumelsky and Stepanov can illustrate the adaptations of
depth first and greedy search approachs to more general domains.

| Choset’s notation for the bug algorithms is standard usage and we will
  be consistent with their choice. The real line or any one dimensional
  quantity will be indicated by :math:`\RR`; the two dimensional plane
  by :math:`\RR^2`; and three dimensional space will be denoted by
  :math:`\RR^3`. Higher dimensional spaces will be denoted in the same
  manner by :math:`\RR^n`. The robot workspace will be denoted by
  :math:`{\cal W} \subset \RR^n` and the workspace obstacles
  (specifically the :math:`i^{th}` obstacle) by
  :math:`{\cal W}{\cal O}_i`. Free space is then the workspace minus the
  obstacles: :math:`{\cal W}\setminus \bigcup_i {\cal W}{\cal O}_i`. A
  point in space has the usual notation :math:`x = (x_1, x_2, x_3)` and
  we distinguish vectors by using a bracket: :math:`\vec{v} \in \RR^n`
  or more often :math:`v = [v_1, v_2, \dots , v_n]^T`. A workspace is
  said to be bounded if
  :math:`{\cal W} \subset B_r(x) \equiv \{ y \in \RR^n | d(x,y) < r\}`
| for some :math:`0 < r < \infty`.


.. _`bug_obstacle`:
.. figure:: PlanningFigures/obstacle.*
   :width: 40%
   :align: center

   The bot’s direction and the obstacle. How does the bot arrive at the
   desired destination?

We will make several assumptions for this section:

-  The robot is a single point.
   Thus we can ignore the boundary-obstacle intersection problem.

-  The robot is able to detect an obstacle by touching it.

-  Robot knows its pose (location and orientation): :math:`(x,y,\theta)`
   and it knows the direction to the goal.

-  The robot is able to measure distance between any two points:
   :math:`d[(x_1,y_1),(x_2,y_2)]`.

Planning or routing problems are often more than just navigating a path
around obstacles that does not violate vehicle constraints. There are
additional issues. We might require the algorithm to produce the minimal
distance path or the minimum travel time path. [1]_ A very common
problem that humans must resolve is moving obstacles. Driving is a fine
example of moving the vehicle along an obstacle free path within the
vehicle contraints and dealing with other moving vehicles.

Driving is also an example of another type of constraint. We normally
resolve safe paths. These may be defined as paths which maximize
distance from obstacles or have some other relation to the landscape.
Information may be incomplete when planning and so we require that the
algorithm can run in an interactive manner which can monotonically
improve the solution as additional information or computation is
provided.

As we did earlier, we will make some simplifying assumptions to get
started. We assume we have a point (mass) robot. Essentially this is
done by assuming the robot is rigid and we can reduce the robot to the
center of mass where we compensate by inflating the obstacles. In
addition, we will assume that the domain boundary is smooth and there
are a finite number of obstacles all with piecewise smooth boundary.

When designing an algorithm, we must keep in mind issues of the
environment and the robot, robot geometry and capability. We must
concern ourselves with the soundness of the path, optimality of path as
well as the computation resources which are available, The algorithm
must balance the needs for a fast robust solution with the time
available to obtain a solution.

Our first foray into planners develops several very simple planners
which emulate insects. These will be used to illustrate the issues
involved with motion planning in unstructured domains. These are also
local planners in that they don’t need to know the entire obstacle
domain.

The Bug Algorithms
------------------

Bug 1
^^^^^

The Bug 1 algorithm is a very simple planner. In the absence of an
obstacle, it makes sense to head towards the goal, and if an obstacle is
met, then it makes sense to go around the obstacle. So, Bug 1 follows
our basic intuition for how the robot should move. This robot is blind -
although it knows where the goal is (as a direction). For example if you
are walking on a very dark night and cannot see your surroundings, but
can see the north star. This provides a direction, but does not
illuminate the landscape.

Adding on an exit strategy completes the algorithm. As the robot
circumnavigates the obstacle, it computes the distance from itself to
the goal. After circumnavigation, the robot will continue on the
boundary until it finds the closest point to the goal along the
boundary. This point will be the exit point for the obstacle. The idea
behind this is, the longer the traverse from the boundary to the goal,
the higher chance we encounter another obstacle, so we slide along the
boundary until this distance is at a minimum.

A point robot with a tactile sensor A path to the
:math:`q_{\text{goal}}` or a conclusion no such path exists. From
:math:`q^L_{i-1}` move toward :math:`q_{\text{goal}}` Exit Follow the
obstacle boundary Determine the point :math:`q^L_{i}` on the perimeter
that has the shortest distance to the goal Go to :math:`q^L_{i}`
Conclude :math:`q_{\text{goal}}` is not reachable and exit

By assumption, Bug 1 has contact sensors so will determine the obstacle
by direct contact. The contact point will be labeled :math:`q^H_i`
(where :math:`i` indicates the :math:`i`-th contact point). After
contact with the obstacle, the robot switches to boundary following
mode. Similarly, point of departure will be denoted :math:`q^L_i`. In
terms of a state machine, we have moved from the movement to goal state
to the boundary following state. We will use Choset’s terminology here
and call the point of contact, the *hit point*. When the bug departs
from the object, we call it the *leave point*. This point is the closest
point on the boundary to the goal, but does not mean the line of sight
(later defined as the :math:`m`-line) is obstacle free.

Bug 1 completely investigates each obstacle. It is exhaustive in terms
of the boundary search. By looking at the paths in
:numref:`bug1path`, it is appears that Bug 1 is not the
most efficient path planner. It does not, nor does it claim to, find the
shortest valid path from the start to the finish. Not all problems are
even solvable. The planning problem shown in
:numref:`unreachable` does not have a solution, so
Bug 1 will exit without success on this one.

.. _`bug1path`:
.. figure:: PlanningFigures/bug1.*
   :width: = 50%
   :align: center

   An example of a path using the Bug 1 algorithm.

.. _`unreachable`:
.. figure:: PlanningFigures/bug1_a.*
   :align: center

   An example of an unreachable goal.

Bug 2
^^^^^

The path that Bug 1 takes is clearly not the shortest path from start to
goal, as shown in :numref:`bug12bug2`. The first thing
you might ask, is “why go all the way around the obstacle"? Once you go
around the obstacle and you can resume your original path. Define the
line between the start point and the goal point as the :math:`m`-line
(motion to goal line).

.. _`bug12bug2`:
.. figure:: PlanningFigures/bug1tobug2.*
   :width: 40%
   :align: center

   Shortening the path by eliminating the circum-navigation used in
   Bug1. Thus we no longer have an exhaustive search process.


For the Bug 2 algorithm, motion begins along the :math:`m`-line in the
direction of the goal. When an obstacle is encountered, motion switches
to boundary following mode. It is customary to select boundary traversal
direction to be “in the direction of travel”. [If the direction of
travel is :math:`\vec{v}` and the boundary direction or boundary tangent
is :math:`\vec{a}`, then :math:`\vec{v}\cdot\vec{a} > 0`. In the case
where :math:`\vec{v}\cdot\vec{a} = 0`, then pick a convention like “go
left”.] During boundary following mode continue until the :math:`m`-line
is re-emcountered. If the bug can depart in the direction of the goal,
it proceeds along the :math:`m`-line towards the goal or the next
obstacle. If the bug cannot depart, then conclude that there is no path
to the goal.

A point robot with a tactile sensor A path to the
:math:`q_{\text{goal}}` or a conclusion no such path exists. Turn Left
(or right) Let :math:`q^L_{i+1} = m` increment :math:`i`

.. _`bug2path`:
.. figure:: PlanningFigures/bug2
   :width: 50%
   :align: center

   An example of a path using the Bug 2 algorithm.[]

If free space between the start and goal are not path-wise connected,
then we have no hope of finding a path between the two points. In other
words, Bug2 will fail to find a path. This is shown in
:numref: `unreachable2`



.. figure:: PlanningFigures/bug2_a
   :alt: An example of an unreachable goal for Bug 2.[unreachable2]

   An example of an unreachable goal for Bug 2.[unreachable2]

From :numref:`bug2path`, it appears that the length of
Bug 2’s path would be shorter than the length of Bug 1’s path. This
seems obvious since we don’t circumnavigate the obstacle, leaving
roughly have of the obstacle’s perimeter untraversed.
:numref:`complicatedobstacle` shows that Bug
1 can indeed have a shorter path than Bug 2. The basic shape is given in
:numref:`complicatedobstacle`-(a). The
vertical obstacle can be made arbitrarily long. This means that
traversing around it can have an arbrarily long path. Alternatively in
:numref:`complicatedobstacle`-(b), we can
increase the number of vertical bars. What are the path lengths for Bug
1 and Bug 2 when they encounter
:numref:`complicatedobstacle`-(b)?



a) |a) A more disceptive obstacle. This provides the basic obstacle
shape and relative pose. b) Extending the difference in the obstacle
shape to increase the path difference between Bug 1 and Bug2.
[complicatedobstacle]| b) |a) A more disceptive obstacle. This provides
the basic obstacle shape and relative pose. b) Extending the difference
in the obstacle shape to increase the path difference between Bug 1 and
Bug2. [complicatedobstacle]|


.. figure:: PlanningFigures/complicated_obst_dim
   :alt: Some dimensions for this obstacle. [complicatedobstacledim]

   Some dimensions for this obstacle. [complicatedobstacledim]

To make the analysis easier, actual numbers are used,
Figure \ `[complicatedobstacledim] <#complicatedobstacledim>`__. The
units are not really important, but included for those who like it to
seem real. The path for Bug 1 is given in
Figure \ `[bug1vsbug2] <#bug1vsbug2>`__-(a) and the path for Bug 2 is
given in Figure \ `[bug1vsbug2] <#bug1vsbug2>`__-(b).

.. raw:: latex

   \centering

| a) |Bug1 can outperform Bug2. [bug1vsbug2]|
| b) |Bug1 can outperform Bug2. [bug1vsbug2]|

Following Bug 1 we accumulate the distance is 76. [2]_ For Bug 2, we
obtain the distance is :math:`7.5+26.5n` where :math:`n` is the number
of vertical obstacles. The figure shows the case where :math:`n=6` which
provides a distance of 166 (rounding down). For the specific horizontal
length of 17 cm and the current spacing used, we can replace the dots by
one additional vertical obstacle, making :math:`n=7`. Beyond that, we
need to increase the horizontal length. The horizontal length then
scales roughly by :math:`3n` and the path then would scale by
:math:`9n`. Beyond :math:`n=3`, the path length for Bug 2 is larger than
for Bug 1.

Lumelsky and Stepanov has illustrated is two basic approaches to
searching - exhaustive and greedy. Bug 1 is an exhaustive search where
Bug 2 is a greedy search. For simple domains, the greedy approach works
well and thus Bug 2 is the better performer. In complicated domains, an
exhaustive search may work better (not assured) and so Bug 1 may
outperform Bug 2. If you look at Choset’s text, you will see another
example of a domain for which Bug 1 outperforms Bug 2. It is a spiral
(or G shaped) domain. Although it is not hard to find domains, start and
end points, which give this result; geometrically classifying them is a
much more difficult problem which we leave for the reader.

There is one additional modification to the bug path that can
intuitively decrease path length. The idea is that when the obstacle no
longer blocks the goal during the boundary following state, leave the
obstacle and head for the goal. This is shown in
Figure \ `[bug1tobug2] <#bug1tobug2>`__. This modification has the bug
leave the obstacle when the obstacle becomes visible.

.. raw:: latex

   \centering

.. figure:: PlanningFigures/bug2tobug3
   :alt: What about reducing the path even more?[bug1tobug2]

   What about reducing the path even more?[bug1tobug2]

Bug 3
^^^^^

[h!]

A point robot with a tactile ring sensor A path to the
:math:`q_{\text{goal}}` or a conclusion no such path exists. Turn Left
(or right) Let :math:`q^L_{i+1} = m` increment :math:`i`

Bug 3 appears to effectively equivalent to Bug 2. It will suffer from
many of the same types of problems as Bug 2 suffers from and get trapped
in the same types of domains. The advantage often is the possible use of
direct routes which can shorten travel distances.

.. raw:: latex

   \centering

.. figure:: PlanningFigures/bug3
   :alt: An example of a path using the Bug 3 algorithm.[bug3path]

   An example of a path using the Bug 3 algorithm.[bug3path]

However, note that for Figure \ `[bugmaze] <#bugmaze>`__, Bug 2 will
difficulties reaching the goal where Bug 1 and 3 succeed.

.. raw:: latex

   \centering

.. figure:: PlanningFigures/bugmaze
   :alt: Trace this with the different bug algorithms: bug 1 and 3
   succeed and bug 2 fails. [bugmaze]

   Trace this with the different bug algorithms: bug 1 and 3 succeed and
   bug 2 fails. [bugmaze]
