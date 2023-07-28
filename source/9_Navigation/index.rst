.. _`Chap:Navigation`:

**********
Navigation
**********

It can be argued that the single most important aspect of a robot is its ability to move.
Roughly motion is a necessary condition but not a sufficient condition.
Motion itself is not complicated. Requiring only servos and motors,
motion is easily accomplished. The complexity arises through the
interaction of the environment. In this chapter we explore how robots
move in the plane and navigate around simple landscapes. Although ground
robots have to address three dimensional environments, the restriction
to the plane simplifies the mathematics and the algorithms allowing us
to focus on concepts and not the complexities of the extra dimension.

Motion planning is an entire field of study, we will highlight some
aspects here. The solution to the planning problem routes from an
initial configuration, start location and pose, to a final
configuration, end location and pose or goal.

The basic path planning problem refers determining a
path in configuration space such that the robot does not collide with
any obstacles and the path is consistent with the vehicle
constraints.

.. toctree::
   :maxdepth: 1

   MotionPlanning
   ExplorationandNavigation
   Implementation
   Mazes
   Wavefront
   Guidance
   Navigation_Problems
