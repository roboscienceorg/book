Bug Comparison
--------------

The best paths we have seen from the bug algorithms have been the
Tangent Bug paths with infinite sensor range. A sufficiently large
sensor range would effectively be an infinite range sensor, so we just
assume we have infinite range. We can reexamine the complicated obstacle
with the tangent bug.

.. raw:: latex

   \centering

.. figure:: path/complicated_obst_tb
   :alt: The path of the tangent bug on the difficult obstacle field.
   [bug1vstb]

   The path of the tangent bug on the difficult obstacle field.
   [bug1vstb]

.. raw:: latex

   \centering

.. figure:: path/complicated_obst_tb2
   :alt: (left) Bug 1 and Bug 2 suceed. (right) Tangent Bug does not.
   [bug1vstb2]

   (left) Bug 1 and Bug 2 suceed. (right) Tangent Bug does not.
   [bug1vstb2]

From Figure \ `[bug1vstb] <#bug1vstb>`__, we see that Tangent Bug
performs well on the obstacle field that caused so much headache for Bug
2. Figure \ `[bug1vstb2] <#bug1vstb2>`__ (left) shows a obstacle domain
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
