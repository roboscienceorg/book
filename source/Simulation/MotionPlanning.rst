Motion Planning
----------------

Simple Planning
~~~~~~~~~~~~~~~

When controlling the robot without feedback, open loop control, we
preplan the route and then code up a list of motion instructions. For
differential drive robots, the easiest routes to drive are combinations
of lines and circles,
Figure \ `[fig:simplecurvedpath] <#fig:simplecurvedpath>`__. If you have
a rough idea of the route, place some points along the route, connect
with line and circle segments. Along those segments, the differential
drive has constant wheel speed. In practice this is difficult since one
cannot have instant jumps in wheel velocity. This makes accurate turns
challenging. If stopping and turning in place on the route is
acceptable, paths with just straight lines are the easiest to develop,
Figure \ `[fig:simplecurvedpath] <#fig:simplecurvedpath>`__. Then is is
just a matter of starting with the correct orientation and driving for a
given amount of time.

.. raw:: latex

   \centering

.. raw:: latex

   \centering

.. figure:: sim/simplepath.pdf
   :alt: [fig:simplecurvedpath] Path with arcs

   [fig:simplecurvedpath] Path with arcs

.. raw:: latex

   \hfill

.. raw:: latex

   \centering

.. figure:: sim/simplestraightpath.pdf
   :alt: [fig:simplecurvedpath] Path without arcs

   [fig:simplecurvedpath] Path without arcs

There is a clear problem with open loop control. Any variation in the
physical system can cause drift. This drift accumulates over time and at
some point the robot is not driving the intended course. The other
problem is that the path is tuned to a specific obstacle field. We must
know the obstacles and their locations prior to moving. A more advanced
algorithm would be able to take a goal point and using knowledge of the
current robot location, drive itself to the goal. The basic motion
algorithm attempts this next step. [5]_

Basic Motion Algorithm
~~~~~~~~~~~~~~~~~~~~~~

Assuming we have a simple obstacle map, how should we proceed? Try the
following thought experiment. Pretend that you are in a dark room with
tall boxes. Also pretend that you can hear a phone ringing and you can
tell what direction it is. How would you navigate to the phone? Figuring
that I can feel my way, I would start walking towards the phone. I keep
going as long as there are no obstructions in my way. When I meet an
obstacle, without sight I can’t make any sophisticated routing
decisions. So, I decide to turn right a bit and head that way. If that
is blocked, then I turn right a bit again. I can continue turning right
until the path is clear. Now I should take a few steps in this direction
to pass the obstacle. Hopefully I am clear and I can turn back to my
original heading. I head in this direction until I run into another
obstacle and so I just repeat my simple obstacle avoidance approach.

Set heading towards goal Move forward count = 0 Turn right Move forward
incr count Set heading towards goal

.. raw:: latex

   \centering

.. figure:: turtle/turtleobs
   :alt: The direct path to the goal.[turtlebasicmotion_a]

   The direct path to the goal.[turtlebasicmotion_a]

.. raw:: latex

   \hfill

.. figure:: turtle/turtleobs2
   :alt: Path using the Basic Motion algorithm.[turtlebasicmotion_b]

   Path using the Basic Motion algorithm.[turtlebasicmotion_b]

Figure \ `[turtlebasicmotion] <#turtlebasicmotion>`__ illustrates the
idea. This algorithm is not completely specified. The amount of right
turn and the distance traveled in the move forward steps is not
prescribed above. Assuming values can be determined, will this approach
work? We expect success when faced with convex obstacles but not
necessarily for non-convex obstacles,
Figure \ `[simple1motionproblem] <#simple1motionproblem>`__. Using
Figure \ `[simple1motionproblem] <#simple1motionproblem>`__ as a guide,
we can construct a collection of convex obstacles which still foil the
algorithm; this is expressed in
Figure \ `[simple2motionproblem] <#simple2motionproblem>`__. The robot
bounces from obstacle to obstacle like a pinball and is wrapped around.
Leaving the last obstacle the robot reaches the cutoff distance and then
switches back to the “motion to goal" state. However, this sets up a
cycle. So, the answer to the question “does this work" is not for all
cases.

.. raw:: latex

   \centering

.. raw:: latex

   \centering

.. figure:: planning/simple1
   :alt: Getting trapped in a non-convex solid
   object.[simple1motionproblem]

   Getting trapped in a non-convex solid object.[simple1motionproblem]

.. raw:: latex

   \hfill

.. raw:: latex

   \centering

.. figure:: planning/simple2
   :alt: A collection of convex objects can mimic a non-convex obstacle.
   [simple2motionproblem]

   A collection of convex objects can mimic a non-convex obstacle.
   [simple2motionproblem]

In Chapter \ `[Chap:Planning] <#Chap:Planning>`__, we will fully explore
the challenge of motion planning in an environment with obstacles. It is
easy to see how the thought experiment above can fail and more robust
approaches are needed. Before we jump into motion planning, we want to
understand what view of the world we can get from sensors. This is
necessary so we know what kind of assumptions can be made when
developing our algorithms.

.. raw:: latex

   \FloatBarrier
