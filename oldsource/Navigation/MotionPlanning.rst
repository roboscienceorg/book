Basic Motion Planning
---------------------

Simple Planning
~~~~~~~~~~~~~~~

When controlling the robot without feedback, open loop control, we
preplan the route and then code up a list of motion instructions. For
differential drive robots, the easiest routes to drive are combinations
of lines and circles,
:numref:`fig:simplecurvedpath`. If you have
a rough idea of the route, place some points along the route and connect
with line (or circle) segments. Along those segments, the differential
drive has constant wheel speed. In practice this is difficult since one
cannot have instant jumps in wheel velocity. This makes accurate turns
challenging. If stopping and turning in place on the route is
acceptable, paths with just straight lines are the easiest to develop,
:numref:`fig:simplestraightpath`. Then is is
just a matter of starting with the correct orientation and driving for a
given amount of time.

.. _`fig:simplecurvedpath`:
.. figure:: NavigationFigures/simplepath.*
   :width: 50%
   :align: center

   Path with arcs

.. _`fig:simplestraightpath`:
.. figure:: NavigationFigures/simplestraightpath.*
   :width: 50%
   :align: center

   Path *without* arcs

There is a clear problem with open loop control (preprogrammed on any path without
sensor feedback). Any variation in the
physical system can cause drift. This drift accumulates over time and at
some point the robot is not driving the intended course. The other
problem is that the path is tuned to a specific obstacle field. We must
know the obstacles and their locations prior to moving. A more advanced
algorithm would be able to take a goal point and using knowledge of the
current robot location, drive itself to the goal. The basic motion
algorithm attempts this next step. [#f5]_

:index:`Basic Motion Algorithm`
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Assuming we have a simple obstacle map, how should we proceed? Try the
following thought experiment. Pretend that you are in a dark room with
tall boxes. Also pretend that you can hear a phone ringing and you can
tell what direction it is. How would you navigate to the phone? Figuring
that I can feel my way, I would start walking towards the phone. I keep
going as long as there are no obstructions in my way. When I meet an
obstacle, without sight (or a map) I can’t make any sophisticated routing
decisions. So, I decide to turn right a bit and head that way. If that
is blocked, then I turn right a bit again. I can continue turning right
until the path is clear. Now I should take a few steps in this direction
to pass the obstacle. Hopefully I am clear and I can turn back to my
original heading. I head in this direction until I run into another
obstacle and so I just repeat my simple obstacle avoidance approach.

.. _`alg:basicmotion`:
.. topic::  Basic Motion Algorithm

   | Set heading towards goal
   | **while** Not arrived at goal **do**
   |   **while** No obstacle in front **do**
   |     Move forward
   |   end while
   |   count = 0
   |   **while** count <= N **do**
   |     **while** Obstacle in front **do**
   |       Turn right
   |     **end while**
   |     Move forward
   |     incr count
   |   **end while**
   |   Set heading towards goal
   | **end while**



.. _`turtlebasicmotion_a`:
.. figure:: NavigationFigures/turtleobs.*
   :width: 50%
   :align: center

   The direct path to the goal.

.. _`turtlebasicmotion_b`:
.. figure:: NavigationFigures/turtleobs2.*
   :width: 50%
   :align: center

   Path using the Basic Motion algorithm.


:numref:`turtlebasicmotion_b` illustrates the
idea. This algorithm is not completely specified. The amount of right
turn and the distance traveled in the move forward steps is not
prescribed above. Assuming values can be determined, will this approach
work? We expect success when faced with convex obstacles but not
necessarily for non-convex obstacles,
:numref:`simple1motionproblem`. Using
:numref:`simple1motionproblem` as a guide,
we can construct a collection of convex obstacles which still foil the
algorithm; this is expressed in
:numref:`simple2motionproblem`. The robot
bounces from obstacle to obstacle like a pinball and is wrapped around.
Leaving the last obstacle the robot reaches the cutoff distance and then
switches back to the “motion to goal" state. However, this sets up a
cycle. So, the answer to the question “does this work" is not for all
cases.

.. _`simple1motionproblem`:
.. figure:: NavigationFigures/simple1.*
   :width: 30%
   :align: center

   Getting trapped in a non-convex solid object.

.. _`simple2motionproblem`:
.. figure:: NavigationFigures/simple2.*
   :width: 65%
   :align: center

   A collection of convex objects can mimic a non-convex obstacle.


In the Chapter on Motion Planning, we will fully explore
the challenge of motion planning in an environment with obstacles. It is
easy to see how the thought experiment above can fail and more robust
approaches are needed. Before we jump into motion planning, we want to
understand what view of the world we can get from sensors. This is
necessary so we know what kind of assumptions can be made when
developing our algorithms.

.. rubric:: Footnotes

.. [#f5] This algorithm is slightly more general in that it does not need the goal location, but just the direction to the goal during the process.
