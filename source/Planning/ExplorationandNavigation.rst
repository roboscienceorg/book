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

The wonderful text *Principles of Robot Motion* 
:cite:`Choset:2005:PRM` begins the study of planning
with three basic navigation algorithms or planners that are similar to
the maze routines presented in the last section. These are adaptations
of basic planners by Lumelsky and
Stepanov :cite:`lumelsky:1987`. Using very simple models one
can strip out the non-essential elements and focus on the core issues
related to path planning. Even so, as Choset points out “even a simple
planner can present interesting and difficult issues." The bug
algorithms of Lumelsky and Stepanov can illustrate the adaptations of
depth first and greedy search approachs to more general domains.

Choset’s notation for the bug algorithms is standard usage and we will
be consistent with their choice. The real line or any one dimensional
quantity will be indicated by :math:`\Bbb R`; the two dimensional plane
by :math:`\Bbb R^2`; and three dimensional space will be denoted by
:math:`\Bbb R^3`. Higher dimensional spaces will be denoted in the same
manner by :math:`\Bbb R^n`. The robot workspace will be denoted by
:math:`{\cal W} \subset \Bbb R^n` and the workspace obstacles
(specifically the :math:`i^{th}` obstacle) by
:math:`{\cal W}{\cal O}_i`. Free space is then the workspace minus the
obstacles: :math:`{\cal W}\setminus \bigcup_i {\cal W}{\cal O}_i`. A
point in space has the usual notation :math:`x = (x_1, x_2, x_3)` and
we distinguish vectors by using a bracket: :math:`\vec{v} \in \Bbb R^n`
or more often :math:`v = [v_1, v_2, \dots , v_n]^T`. A workspace is
said to be bounded if
:math:`{\cal W} \subset B_r(x) \equiv \{ y \in \Bbb R^n | d(x,y) < r\}`
for some :math:`0 < r < \infty`.


.. _`bug_obstacle`:
.. figure:: PlanningFigures/obstacle.*
   :width: 70%
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
distance path or the minimum travel time path. [#f1]_ A very common
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


.. _`alg:bug1`:
.. topic::  The bug 1 algorithm :cite:`Choset:2005:PRM`

   | **Input** A point robot with a tactile sensor
   | **Output** A path to the :math:`q_{\text{goal}}` or a conclusion no such path exists.
   | **while** True **do**
   |   **repeat**
   |     From :math:`q^L_{i-1}` move toward :math:`q_{\text{goal}}`
   |   **until**  :math:`q_{\text{goal}}` is reached *or*  obstacle is encountered at hit point :math:`q^H_{i}`
   |   **if** Goal is reached **then**  Exit  **endif**
   |   **repeat**
   |     Follow obstacle boundary
   |   **until** :math:`q_{\text{goal}}` is reached or :math:`q^H_{i}` is re-encountered.
   |   Determine the point :math:`q^L_{i}` on the perimeter that has the shortest distance to the goal
   |   Go to :math:`q^L_{i}`
   |   **if** the robot were to move toward the goal **then**
   |   Conclude :math:`q_{\text{goal}}` is not reachable and exit
   |   **endif**
   | **end while**


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
   :width: 60%
   :align: center

   An example of a path using the Bug 1 algorithm.

.. _`unreachable`:
.. figure:: PlanningFigures/bug1_a.*
   :width: 60%
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
   :width: 80%
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


.. _`alg:bug2`:
.. topic::  The bug 2 algorithm  :cite:`Choset:2005:PRM`

   | **Input** A point robot with a tactile sensor
   | **Output** A path to the :math:`q_{\text{goal}}` or a conclusion no such path exists.
   | **while** True **do**
   |   **repeat**
   |     From :math:`q^L_{i-1}` move toward :math:`q_{\text{goal}}` along :math:`m`-line
   |   **until**  :math:`q_{\text{goal}}` is reached *or*  obstacle is encountered at hit point :math:`q^H_{i}`
   |   **if** Goal is reached **then**  Exit  **endif**
   |   **repeat**
   |     Follow obstacle boundary
   |   **until** :math:`q_{\text{goal}}` is reached or :math:`q^H_{i}` is re-encountered
   |     or m-line is re-encountered at a point m, such that :math:`m\neq q^H_{i}` (robot did not reach hit point),
   |     and :math:`d(m,q_{\text{goal}}) < d(m, q^H_{i})` (robot is closer), and if robot moves toward goal, it would not hit obstacle.
   |   Let $q^L_{i+1} = m$,  increment i
   |   **if** the robot were to move toward the goal **then**
   |     Conclude :math:`q_{\text{goal}}` is not reachable and exit
   |   **endif**
   | **end while**



.. _`bug2path`:
.. figure:: PlanningFigures/bug2.*
   :width: 50%
   :align: center

   An example of a path using the Bug 2 algorithm.

If free space between the start and goal are not path-wise connected,
then we have no hope of finding a path between the two points. In other
words, Bug2 will fail to find a path. This is shown in
:numref:`unreachable2`.


.. _`unreachable2`:
.. figure:: PlanningFigures/bug2_a.*
   :width: 50%
   :align: center

   An example of an unreachable goal for Bug 2.

From :numref:`bug2path`, it appears that the length of
Bug 2’s path would be shorter than the length of Bug 1’s path. This
seems obvious since we don’t circumnavigate the obstacle, leaving
roughly have of the obstacle’s perimeter untraversed.
:numref:`complicatedobstacle_a` and :numref:`complicatedobstacle_b` shows that Bug
1 can indeed have a shorter path than Bug 2. The basic shape is given in
:numref:`complicatedobstacle_a`. The
vertical obstacle can be made arbitrarily long. This means that
traversing around it can have an arbrarily long path. Alternatively in
:numref:`complicatedobstacle_b`, we can
increase the number of vertical bars. What are the path lengths for Bug
1 and Bug 2 when they encounter
:numref:`complicatedobstacle_b`?

.. _`complicatedobstacle_a`:
.. figure:: PlanningFigures/complicated_obst0.*
   :width: 15%
   :align: center

   A more disceptive obstacle.  This provides the basic obstacle shape and relative pose.

.. _`complicatedobstacle_b`:
.. figure:: PlanningFigures/complicated_obst.*
   :width: 45%
   :align: center

   Extending the difference in the obstacle shape to increase the path difference between Bug 1 and Bug2.


To make the analysis easier, actual numbers are used,
:numref:`complicatedobstacledim`. The
units are not really important, but included for those who like it to
seem real. The path for Bug 1 is given in
:numref:`bug1vsbug2_a` and the path for Bug 2 is
given in :numref:`bug1vsbug2_b`.

.. _`complicatedobstacledim`:
.. figure:: PlanningFigures/complicated_obst_dim.*
   :width: 50%
   :align: center

   Some dimensions for this obstacle.

.. _`bug1vsbug2_a`:
.. figure:: PlanningFigures/complicated_obst_b1.*
   :width: 50%
   :align: center

   Bug2's path.


.. _`bug1vsbug2_b`:
.. figure:: PlanningFigures/complicated_obst_b2.*
   :width: 50%
   :align: center

   Bug1 can outperform Bug2.




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
:numref:`bug1tobug2`. This modification has the bug
leave the obstacle when the obstacle becomes visible.


.. _`bug1tobug2`:
.. figure:: PlanningFigures/bug2tobug3.*
   :width: 40%
   :align: center

   What about reducing the path even more?

Bug 3
^^^^^


.. _`alg:bug3`:
.. topic::  The bug 3 algorithm

   | **Input** A point robot with a tactile sensor
   | **Output** A path to the :math:`q_{\text{goal}}` or a conclusion no such path exists.
   | **while** True **do**
   |   **repeat**
   |     From :math:`q^L_{i-1}` move toward :math:`q_{\text{goal}}` along :math:`m`-line
   |   **until**  :math:`q_{\text{goal}}` is reached *or*  obstacle is encountered at hit point :math:`q^H_{i}`
   |   **if** Goal is reached **then**  Exit  **endif**
   |   Turn left (or right)
   |   **repeat**
   |     Follow obstacle boundary
   |   **until** :math:`q_{\text{goal}}` is reached or :math:`q^H_{i}` is re-encountered or
   |   tangent line at a point m points towards the goal, such that :math:`m\neq q^H_{i}` (robot did not reach hit point),
   |   and :math:`d(m,q_{\text{goal}}) < d(m, q^H_{i})` (robot is closer), and if robot moves toward goal, it would not hit obstacle.
   |   Let $q^L_{i+1} = m$,  increment i
   |   **if** the robot were to move toward the goal **then**
   |     Conclude :math:`q_{\text{goal}}` is not reachable and exit
   |   **endif**
   | **end while**



Bug 3 appears to effectively equivalent to Bug 2. It will suffer from
many of the same types of problems as Bug 2 suffers from and get trapped
in the same types of domains. The advantage often is the possible use of
direct routes which can shorten travel distances.

.. _`bug3path`:
.. figure:: PlanningFigures/bug3.*
   :align: center
   :width: 50%

   An example of a path using the Bug 3 algorithm.

However, note that for :numref:`bugmaze`, Bug 2 will
difficulties reaching the goal where Bug 1 and 3 succeed.


.. _`bugmaze`:
.. figure:: PlanningFigures/bugmaze.*
   :width: 60%
   :align: center

   Trace this with the different bug algorithms: bug 1 and 3 succeed and
   bug 2 fails.


Tangent Bug
^^^^^^^^^^^

The tangent bug algorithm will follow the basic idea in Bug 3, with the
addition of a range sensor to the bug. As before our bug will have
motion to goal in the absence of obstacles in the path. When an obstacle
is encountered, the bug will switch to a boundary following mode. With a
range sensor, there is more than one way to address the transition to
boundary following mode which we will see. For simplicity, we assume
that the range sensor has 360 degree infinite orientation resolution:
:math:`\rho:  \Bbb R^2 \times S^1 \to \Bbb R`

.. math:: \rho (x,\theta) = \min_{\lambda\in [0,\infty]} d(x,x+\lambda [\cos\theta , \sin\theta ]^T),
   :label: LidarRangeEq

such that

.. math:: \quad x+\lambda [\cos\theta , \sin\theta ]^T \in \bigcup_i {\cal W}{\cal O}_i
   :label: ObsConstrEq

and a finite range:

.. math::
   :label: lidarFiniteRange

   \rho_R(x,\theta) = \left\{ \begin{array}{ll} \rho(x,\theta), & \text{ if } \rho(x,\theta) < R \\
                                 \infty, & \text{ otherwise}.
                                \end{array}\right.

.. _`Fig:lidar_ray`:
.. figure:: PlanningFigures/lidar_ray.*
   :width: 50%
   :align: center

   The LIDAR ray from the location :math:`x` at the angle :math:`\theta`.

:eq:`LidarRangeEq` and
:eq:`ObsConstrEq` find the shortest distance
between the point :math:`x` and all of the points in the obstacle which
intersect the ray eminating from :math:`x`. A real sensor has a finite
range. :eq:`lidarFiniteRange` truncates
the result at some maximum range :math:`R`.

The range sensor returns a polar map, meaning a function
:math:`\rho = \rho_R(x,\theta)`. This function will be be piecewise
continuous. Discontinuities will occur by occlusion of one object by
another or by reaching the maximum range,
:numref:`discontrange` and :numref:`discontrangefn`. Having a discrete function
makes finding discontinuities a bit subtle.


.. _`discontrange`:
.. figure:: PlanningFigures/range.*
   :width: 40%
   :align: center

   Obstacles producing discontituities in the range map. Assume that one
   can determine discontinuities in the distance function
   :math:`\rho_R`.

.. _`discontrangefn`:
.. figure:: PlanningFigures/rangefunction.*
   :width: 50%
   :align: center

   Range map for the obstacle above.


Normally one uses

.. math:: \rho_R(x,\theta_{k+1}) - \rho_R(x,\theta_k) > \delta \geq 1

for some :math:`\delta` as the criterion.

.. _`discontinuitypoints`:
.. figure:: PlanningFigures/discont.*
   :width: 35%
   :align: center

   Points of discontinuity: :math:`O_1`, :math:`O_2`, ..., :math:`O_n`

.. _`discontinuitypoints_b`:
.. figure:: PlanningFigures/singleVSdouble.*
   :width: 80%
   :align: center

   Object ambiguity.



Using this idea, we obtain some number of discontinuities, call them
:math:`O_1`, :math:`O_2`, ..., :math:`O_n`. It is not possible in
general to tell if :math:`O_1`, :math:`O_2`, ..., :math:`O_n` indicate
boundaries of separate obstacles,
:numref:`discontinuitypoints`. Since we are
only concerned about obstacles that prevent us from moving to the goal,
we will only focus on those,
:numref:`discontpathblock` (left).


.. _`discontpathblock`:
.. figure:: PlanningFigures/discont2.*
   :width: 80%
   :align: center

   Sensing an object does not mean it is  a problem, only if it blocks the path.
   The robot will then move toward the discontinuity point $O_i$ which most decreases the distance
   :math:`d(x, O_i) + d(O_i,q_{\text{goal}})`


If the goal is obscured by an obstacle, then the robot moves towards the
:math:`O_i` that minimizes the heuristic distance:
:math:`d(x, O_i) + d(O_i,q_{\text{goal}})`. In
:numref:`discontpathblock`, two variations are
shown. The middle figure shows that :math:`d(x,O_2) + d(O_2,y)` is less
than :math:`d(x,O_1) + d(O_1,y)`, so :math:`O_2` is the first target for
motion. In the right figure where the goal :math:`y` has moved,
:math:`d(x,O_1) + d(O_1,y)` is less than :math:`d(x,O_2) + d(O_2,y)`.
Thus the target in that case is :math:`O_1`. The points :math:`O_i` are
continuously updated as the robot moves. New points may enter the list
and some points may leave.

We have seen two types of motion to goal. One is the free space motion
where the robot moves towards the goal without an obstacle. The other is
the motion towards a boundary point which is the minimizing
discontinuity point discussed above. These two can be merged into just
motion towards goal where goal is selected from :math:`n = \{ T, O_i\}`,
:math:`i=1 \dots k` where :math:`T` is defined as the intersection of
the circle of radius :math:`R` centered at :math:`x` with the line
segment from :math:`x` to the goal, :numref:`defnT`.

The robot will continue with the motion to goal until it can no longer
decrease the heuristic distance, then it switches to boundary following.
The robot follows the same direction in boundary following mode as it
did in motion to goal mode. As the robot approaches the boundary, the
direction will change due to pursuit of temporary goal :math:`n`. The
distance :math:`d(x,n)+d(n,\text{goal})` will start to increase. If you
are far from the boundary, you are heading roughly in the direction of
the goal. Once close enough and with the direction strongly affected by
the obstacle boundary, it makes sense to just switch to boundary
following mode. :numref:`transitionboundary`
shows the three states. The left figure indicates the robot motion to
goal in free space. In the middle figure, the robot has sensed the
obstacle and computed that the lower boundary discontinuity is the one
to set as the temporary goal.

.. _`defnT`:
.. figure:: PlanningFigures/defnT.*
   :width: 70%
   :align: center

   The free space point :math:`T` (left). :math:`T` and :math:`O_1`
   (right). [defnT]


.. _`transitionboundary`:
.. figure:: PlanningFigures/discont4.*
   :width: 85%
   :align: center

   Motion to goal (left), motion to boundary discontinuity point
   (middle) and boundary following (right).

We define the point :math:`M` which is the closest point on the sensed
boundary to the goal, :numref:`Mdefinition`. This is
used in the computation of the departure point.

.. _`Mdefinition`:
.. figure:: PlanningFigures/discont3.*
   :width: 70%
   :align: center

   M - the closest point on the sensed boundary to the goal. Can be one
   of the discontinuity points from the ranger or simply a boundary
   point.

Boundary following mode can get you around the obstacle. The next
question is when to release and return to motion to goal (or to the next
obstacle). We define :math:`d_{\text{followed}}` as the shortest
distance between boundary that has been sensed and the goal,
:numref:`Fig:Dfollowed`.

.. _`Fig:Dfollowed`:
.. figure:: PlanningFigures/d_followed.*
   :width: 70%
   :align: center

   The value :math:`d_{\text{followed}}`.

Define :math:`\Lambda` as all of the points between the robot, :math:`x`
and the boundary of the obstacle, :math:`\partial WO` which are visible
to the robot and within range :math:`R` (the range of the sensor).
Precisely this is
:math:`\Lambda = \{ y \in \partial WO: \lambda x + (1-\lambda )y \in Q_{\mbox{free}} \quad \forall \lambda \in [0,1]`,
:numref:`Fig:Dlambda`.  We define
:math:`d_{\text{reach}}` as the minimum distance point in
:math:`\Lambda` to the goal:
:math:`d_{\mbox{reach}} = \mbox{min}_{c\in\Lambda} d(c,q_{\mbox{goal}})`.
See :numref:`Fig:Dreach`, :numref:`Fig:Dreach2`
for a description of this distance.

.. _`Fig:Dlambda`:
.. figure:: PlanningFigures/d_lambda.*
   :width: 30%
   :align: center

   The region :math:`\Lambda`.

.. _`Fig:Dreach`:
.. figure:: PlanningFigures/d_reach.*
   :width: 80%
   :align: center

   The value :math:`d_{\text{reach}}`.

.. _`Fig:Dreach2`:
.. figure:: PlanningFigures/d_reach2.*
   :width: 80%
   :align: center

   The value :math:`d_{\text{reach}}` with a different
   domain.

These values are continuously updated as the robot traverses the
boundary. When :math:`d_{\text{reach}} < d_{\text{followed}}` then we
terminate the boundary following and return to motion to goal.
:numref:`Fig:DreachFollowed2` shows when the
values become equal.
:numref:`Fig:DreachFollowed3` shows when the
boundary following termination condition is satisfied. The planner is
summarized in Algorithm [TangentBugAlg]_.

.. _`Fig:DreachFollowed2`:
.. figure:: PlanningFigures/d_reach_followed2.*
   :width: 80%
   :align: center

   The process and location where
   :math:`d_{\text{reach}} = d_{\text{followed}}`.

.. _`Fig:DreachFollowed3`:
.. figure:: PlanningFigures/d_reach_followed3.*
   :width: 80%
   :align: center

   The process and location where
   :math:`d_{\text{reach}} < d_{\text{followed}}`.

The bug algorithms are biased towards motion along the original direct
route. This last algorithm stayed in boundary following mode longer than
did the Bug 3 algorithm. This behavior, however, depends on the max
range of the range sensor and is thus “tunable”. An interesting
experiment would modify the Tangent Bug to have the boundary exit
behavior the same as Bug 3 and compare paths.


.. _`alg:tangentbug`:
.. topic::  The tangent bug algorithm

   | **Input** A point robot with a tactile sensor
   | **Output** A path to the :math:`q_{\text{goal}}` or a conclusion no such path exists.
   | **while** True **do**
   |   **repeat**
   |     Continuously move from the point :math:`n\in \{ T, O_i\}` which minimizes :eq:`d(x,n)+d(n,q_{\text{goal}})`.
   |   **until**  :math:`q_{\text{goal}}` is reached or the direction that minimizes :math:`d(x,n)+d(n,q_{\text{goal}})` begins to increase :math:`d(n,q_{\text{goal}})`
   |   **if** Goal is reached **then**  Exit  **endif**
   |   Choose a boundary following direction which continues in the same direction as the most recent motion-to-goal direction.
   |   **repeat**
   |     Continuously update :math:`d_\text{reached}`, :math:`d_\text{followed}` and :math:`\{O_i\}`.
   |     Continuously moves toward :math:`n\in O_i` that is in the chosen boundary direction.
   |   **until** :math:`q_{\text{goal}}` is reached or the robot completes a full cycle around the obstacle or :math:`d_\text{reached} < d_\text{followed}`.
   |   **if** the robot were to move toward the goal **then**
   |     Conclude :math:`q_{\text{goal}}` is not reachable and exit
   |   **endif**
   | **end while**


.. _`finitesensorrange`:
.. figure:: PlanningFigures/finite_range.*
   :width: 70%
   :align: center

   Finite Sensor Range


.. _`infinitesensorrange`:
.. figure:: PlanningFigures/infinite_range.*
   :width: 70%
   :align: center

   Infinite Sensor Range.



Bug Comparison
^^^^^^^^^^^^^^

The best paths we have seen from the bug algorithms have been the
Tangent Bug paths with infinite sensor range. A sufficiently large
sensor range would effectively be an infinite range sensor, so we just
assume we have infinite range. We can reexamine the complicated obstacle
with the tangent bug.

.. _`bug1vstb`:
.. figure:: PlanningFigures/complicated_obst_tb.*
   :width: 40%
   :align: center

   The path of the tangent bug on the difficult obstacle field.


.. _`bug1vstb2`:
.. figure:: PlanningFigures/complicated_obst_tb2.*
   :width: 90%
   :align: center

   (left) Bug 1 and Bug 2 suceed. (right) Tangent Bug does not.

From :numref:`bug1vstb`, we see that Tangent Bug
performs well on the obstacle field that caused so much headache for Bug
2. :numref:`bug1vstb2` (left) shows a obstacle domain
for which the path for Bug 1 and Bug2 are equivalent and arrive at the
goal. The bugs begin at the start position and head to goal. Upon
arrival they turn left and head up over point a. Heading down the back
side of the ellipse, Bug 2 will split off when it crosses the
:math:`M`-line. Bug 2 will then head straight for the goal. Bug 1 will
continue to circumnavigate the ellipse. After return to the
:math:`M`-line it too will head to the goal. By construction Bug 1’s
leave point is the :math:`M`-line as well. Both arrive at the goal.

The right figure shows how the Tangent Bug does not arrive at the goal
and cycles around the outside. [Had the left side of the figure dropped
lower, this would have been an example of a longer path, but the Tangent
Bug would have arrvied at the goal.] In this case, T-Bug leaves the
start location and heads towards the goal. Although rather subtle, the
line from the start to goal is slight above the vertical symmetry axis.
This means that the top of the ellipse, location a, will be the closest
point of discontinuity for the ranger. Thus it will minimize the
heuristic traveling to a. The points b and c are the next two
discontinuities to choose. By construction, the point b minimizes the
heuristic over the location c. After arriving at c, the robot will
shortly transition to boundary following mode. This will carry the robot
around the obstacle back to a location above the starting point. The
robot will head to the discontinuity a. Any implementation that stores
locations will note that we have done a cycle and exit or needs to
switch algorithms.

.. rubric:: Footnotes

.. [#f1] These need not be the same. For example certain paths may be traversed
   at different speeds depending on location and path geometry.
