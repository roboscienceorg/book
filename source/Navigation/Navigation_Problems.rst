Problems
--------

#. Using Veranda Simulator, code the basic motion algorithm.

   a. Demonstrate your approach with one obstacle.

   #. Demonstrate with several obstacles.


#. Using Veranda and the basic motion algorithm, place a set of obstacles that
   cause the robot to cycle and not find the goal.  In other words, build a
   robot trap.

#. Write a Python algorithm to perform boundary following on a grid domain.

#. Write a boundary following routine for the DD robot in Gazebo using the
   Lidar. Use a video screen capture program to record the results.

#. Assume that you have a finite number of convex solid obstacles (solid
   means you are not starting inside). Prove or provide a counter-example.

   #. Will Bug 1 succeed in navigating from any start point to any goal
      point?

   #. Will Bug 2 succeed in navigating from any start point to any goal
      point?

   #. Will Tangent Bug succeed in navigating from any start point to any
      goal point?

#. Sketch equations :eq:`LidarRangeEq` and :eq:`ObsConstrEq`.

#. Assume that you have a grid map of the type found in the left image of
   :numref:`coarsemap` which was stored in an array. If
   the start point was an interior cell, implement the Bug 1 algorithm to
   find the sequence of cells which describe an escape path if one exists.

#. Is it possible to have a single non-convex obstacle trap Bug 1 or Bug 2?

#. Assume that you have a grid domain and the obstacles are represented in
the grid map. Write a Python program to implement:

   #. Bug 1

   #. Bug 2

   #. Bug 3

   #. Tangent Bug

#. Implement the following in Veranda:

   #. Bug 1

   #. Bug 2

   #. Bug 3

   #. Tangent Bug

   Use a video screen capture program to record the results.
