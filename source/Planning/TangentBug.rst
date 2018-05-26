Tangent Bug
-----------

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
   :width: 50%
   :align: center

   The free space point :math:`T` (left). :math:`T` and :math:`O_1`
   (right). [defnT]


.. _`transitionboundary`:
.. figure:: PlanningFigures/discont4.*
   :width: 50%

   Motion to goal (left), motion to boundary discontinuity point
   (middle) and boundary following (right). 

We define the point :math:`M` which is the closest point on the sensed
boundary to the goal, Figure \ `[Mdefinition] <#Mdefinition>`__. This is
used in the computation of the departure point.

.. raw:: latex

   \centering

.. figure:: path/discont3
   :alt: M - the closest point on the sensed boundary to the goal. Can
   be one of the discontinuity points from the ranger or simply a
   boundary point. [Mdefinition]

   M - the closest point on the sensed boundary to the goal. Can be one
   of the discontinuity points from the ranger or simply a boundary
   point. [Mdefinition]

Boundary following mode can get you around the obstacle. The next
question is when to release and return to motion to goal (or to the next
obstacle). We define :math:`d_{\text{followed}}` as the shortest
distance between boundary that has been sensed and the goal,
Figure \ `[Fig:Dfollowed] <#Fig:Dfollowed>`__.

.. raw:: latex

   \centering

.. figure:: planning/d_followed
   :alt: The value :math:`d_{\text{followed}}`. [Fig:Dfollowed]

   The value :math:`d_{\text{followed}}`. [Fig:Dfollowed]

Define :math:`\Lambda` as all of the points between the robot, :math:`x`
and the boundary of the obstacle, :math:`\partial WO` which are visible
to the robot and within range :math:`R` (the range of the sensor).
Precisely this is
:math:`\Lambda = \{ y \in \partial WO: \lambda x + (1-\lambda )y \in Q_{\mbox{free}} \quad \forall \lambda \in [0,1]`,
Figure \ `[Fig:Dlambda] <#Fig:Dlambda>`__. We define
:math:`d_{\text{reach}}` as the minimum distance point in
:math:`\Lambda` to the goal:
:math:`d_{\mbox{reach}} = \mbox{min}_{c\in\Lambda} d(c,q_{\mbox{goal}})`.
See
Figures \ `[Fig:Dreach] <#Fig:Dreach>`__, \ `[Fig:Dreach2] <#Fig:Dreach2>`__
for a description of this distance.

.. raw:: latex

   \centering

.. figure:: planning/d_lambda
   :alt: The region :math:`\Lambda`.[Fig:Dlambda]

   The region :math:`\Lambda`.[Fig:Dlambda]

.. raw:: latex

   \centering

.. figure:: planning/d_reach
   :alt: The value :math:`d_{\text{reach}}`.[Fig:Dreach]

   The value :math:`d_{\text{reach}}`.[Fig:Dreach]

.. raw:: latex

   \centering

.. figure:: planning/d_reach2
   :alt: The value :math:`d_{\text{reach}}` with a different
   domain.[Fig:Dreach2]

   The value :math:`d_{\text{reach}}` with a different
   domain.[Fig:Dreach2]

These values are continuously updated as the robot traverses the
boundary. When :math:`d_{\text{reach}} < d_{\text{followed}}` then we
terminate the boundary following and return to motion to goal.
Figure \ `[Fig:DreachFollowed2] <#Fig:DreachFollowed2>`__ shows when the
values become equal.
Figure \ `[Fig:DreachFollowed3] <#Fig:DreachFollowed3>`__ shows when the
boundary following termination condition is satisfied. The planner is
summarized in Algorithm \ `[TangentBugAlg] <#TangentBugAlg>`__.

.. raw:: latex

   \centering

.. figure:: planning/d_reach_followed2
   :alt: The process and location where
   :math:`d_{\text{reach}} = d_{\text{followed}}` .[Fig:DreachFollowed2]

   The process and location where
   :math:`d_{\text{reach}} = d_{\text{followed}}` .[Fig:DreachFollowed2]

.. raw:: latex

   \centering

.. figure:: planning/d_reach_followed3
   :alt: The process and location where
   :math:`d_{\text{reach}} < d_{\text{followed}}`.[Fig:DreachFollowed3]

   The process and location where
   :math:`d_{\text{reach}} < d_{\text{followed}}`.[Fig:DreachFollowed3]

The bug algorithms are biased towards motion along the original direct
route. This last algorithm stayed in boundary following mode longer than
did the Bug 3 algorithm. This behavior, however, depends on the max
range of the range sensor and is thus “tunable”. An interesting
experiment would modify the Tangent Bug to have the boundary exit
behavior the same as Bug 3 and compare paths.

A point robot with a range sensor. A path to the :math:`q_{\text{goal}}`
or a conclusion no such path exists. Choose a boundary following
direction which continues in the same direction as the most recent
motion-to-goal direction. Continuously update :math:`d_\text{reached}`,
:math:`d_\text{followed}` and :math:`\{O_i\}`. Continuously moves toward
:math:`n\in O_i` that is in the chosen boundary direction.

.. raw:: latex

   \centering

.. figure:: path/finite_range
   :alt: Finite Sensor Range [finitesensorrange]

   Finite Sensor Range [finitesensorrange]

.. raw:: latex

   \centering

.. figure:: path/infinite_range
   :alt: Infinite Sensor Range. [infinitesensorrange]

   Infinite Sensor Range. [infinitesensorrange]

Implementation
^^^^^^^^^^^^^^

The algorithms presented above have two basic modes. One is motion to
goal. This behavior assumes that the robot knows the target location or
at least knows the direction to head. This is done in practice using a
type of localization system. In a simulated environment, it is of course
very easy since you always have absolute knowledge of the robot and
goal’s location. The more challenging problem is boundary following.
Unless you have very accurate maps to start with or apriori knowledge of
the objects in the environment, the boundaries of the obstacles are
unknown. This means they must be discovered during the planning process.
How does the robot move around the boundary? What information is
required? What information is provided by the sensors and so what
information needs to be computed? How is the path determined?

We will assume that object boundaries are smooth curves and would be
locally a function, :math:`y-f(x)`. If this is the case, we can compute
the tangent and normal directions as shown in
Figure \ `[offsetcurve] <#offsetcurve>`__-(a). An offset curve is a
curve that follows the boundary at some fixed distance from the
boundary. It looks like a level set curve. We can compute the tangents
and normals for offsets as well,
Figure \ `[offsetcurve] <#offsetcurve>`__-(b).

.. raw:: latex

   \centering

(a) |a) We assume that the boundary is a smooth function. b) The normal
and tangent directions to the offset curve.[offsetcurve]| (b) |a) We
assume that the boundary is a smooth function. b) The normal and tangent
directions to the offset curve.[offsetcurve]|

An offset curve can be found analytically using only the Tangent
direction vector :math:`v(t)` [where :math:`v` is a basis vector in
:math:`(n(c(t)))^\perp`]. Assume that the curve is given in parametric
form :math:`\{c_1(t), c_2(t)\}`. Solving the differential equations
:math:`\dot{c}(t) = v`, :math:`\{c_1(0), c_2(0)\} = c_0` provides the
offset curve.

**Example:** If the tangent to an offset curve is :math:`v = <-y, 2x>`,
find the offset curve :math:`\dot{c}(t) = v` when :math:`c_0 = (1,2)`.

.. math:: \dot{c}(t)=dc/dt = <dx/dt , dy/dt> = <-y,2x>

\ so (1) :math:`dx/dt = -y` and (2) :math:`dy/dt = 2x`. Differentiate
the first equation to get :math:`d^2x/dt^2 = -dy/dt` and then plug into
the second equation: :math:`d^2x/dt^2 = -2x`. We can solve this equation
to obtain

.. math:: x(t) = A\cos\sqrt{2}t + B\sin\sqrt{2}t.

 The condition :math:`x(0) = 1` means :math:`x(0) = A = 1`. From the
first equation we obtain

.. math:: y(t) = \sqrt{2}\sin\sqrt{2}t - B\sqrt{2}\cos\sqrt{2}t

 Using the second condition, :math:`y(0)=2`, we see that
:math:`B = -\sqrt{2}`.

We have already discussed computing an obstacle boundary normal and
tangent, Figure \ `[turtleboundary] <#turtleboundary>`__, using a ring
of touch sensors. In a real application, you may stop once the tangent
has been determined. The robot can be steered in that direction. The act
of driving the robot continuously in the direction of :math:`v` is the
same as solving the differential equations (other than the different
errors that arise).

Simple boundary following using a range sensor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If a range sensor is available, it is a better choice for determining
the boundary normal (avoids contact with the obstacle). Assume that you
are looking to follow the boundary of obstacle 2 in
Figure \ `[rangeinfo] <#rangeinfo>`__. Let :math:`D(x)` be the distance
from :math:`x` to the followed obstacle:

.. math:: D(x) = \min_{c\in{\cal W}{\cal O}_i} d(x,c)

\ Look for global minimum to find the point on the followed obstacle.
The gradient of distance is given by

.. math::

   \nabla D(x) = \begin{bmatrix} \displaystyle \frac{\partial D(x)}{\partial x_1}\\[5mm]
   \displaystyle\frac{\partial D(x)}{\partial x_2}\end{bmatrix}

 The closest point by definition is the point that is a minimum of the
distance function between the ranging device, :math:`x`, and the
obstacle boundary, :math:`y`. This means that the tangent must be
orthogonal to the line segment connecting :math:`x` and :math:`y`. Once
the direction to :math:`y` is determined then the travel direction can
be computed. Assume the direction to :math:`y` is given by
:math:`\nabla D(x) = <a_1,a_2>`. The travel direction is
:math:`\pm <a_2, -a_1>` which is orthogonal to :math:`\nabla D`.

.. raw:: latex

   \centering

.. figure:: path/range2
   :alt: Obtaining information from range data.[rangeinfo]

   Obtaining information from range data.[rangeinfo]

A ranging device in practice returns discrete data. You can detect the
approximate nearest point on the obstacle boundary, say at index k in
the range array data: d[]. You can convert (k-1, d[k-1]),(k, d[k]),(k+1,
d[k+1]) into (x,y) points in the robots coordinates:
:math:`(x_{k-1}, y_{k-1})`, :math:`(x_{k}, y_{k})`,
:math:`(x_{k+1}, y_{k+1})`:

.. math:: (x_k,y_k) = \left(d[k] \cos (\Delta \theta k + \theta_0), d[k] \sin (\Delta \theta k + \theta_0)\right)

 where :math:`\theta_0` is the angle for the start of the sweep. Knowing
the closest point on the boundary to the robot is again sufficient to
compute the tangent direction. We can smooth out the boundary motion
using a boundary motion
algorithm \ `[alg:boundarymotion] <#alg:boundarymotion>`__

List all neighbor cells adjacent to occupied cells. Select neighbor
according to policy (right or left hand travel): (m,n). Mark (i,j) as
visited. Set current cell: (m,n) :math:`\to` (i,j). List unvisited
neighbor cells adjacent to occupied cells. Select neighbor: (m,n) Mark
(i,j) as visited. Set current cell: (m,n) :math:`\to` (i,j).
