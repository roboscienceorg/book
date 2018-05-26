Bug simulations
---------------

Even though the bug algorithms are intended to illustrate particular
ideas, they are valid algorithms for path planning. In addition,
roboticists get some odd sense of pleasure out of watching planners
succeed and fail. We will restrict our work here to discrete (bitmap)
domains only because it makes the headache of determining collision go
away. A collision requires the robot to move. How is this done?

There are two ways initially you can approach controlling the robot:
through position or velocity. A position control sets the location
directly. The other method, velocity, control adjusts the velocities and
the position is updated indirectly. Position control is easy to start
with when you directly work with the simulation window or canvas. It is
not really how a robot is controlled since it models stop and go motion.
Velocity control follows how most mobile robots are actually controlled.
The movement must be consistent with vehicle kinematics which can vary
depending on robot design. For example a differential drive robot can
set forward velocity and rotational (turn) velocity. Using the kinematic
equations (presented later), position and orientation can be computed.

In the Chapter `simulationchapter`_, we
discussed how to determine obstacle impact or collision for a circular
robot. For a general robot shape, impact is more difficult to compute.
Collision can be defined as when the distance between the robot and
obstacle becomes zero. In the continuous world, the distance is

.. math:: D(t) = \min \sqrt{(x_R(t)-x_O)^2 + (y_R(t)-y_O)^2}

 where :math:`(x_R,y_R)` lies on the robot boundary and
:math:`(x_O,y_O)` lies on the object boundary. Setting :math:`D=0` and
solving for :math:`t` then provides the time and location of the
collision. However, the robot and the obstacle will have some shape
defined by a bitmap. Collision between the robot and the obstacle boils
down to determining if a pixel or pixels will overlap after a robot
position update. You might notice that this is not quite correct. It is
possible for the time step to be large enough that the robot appears to
jump over obstacles. In that case, we need to select a smaller time
update so that motion begins to appear continuous and teleporting robots
are avoided.

Assume that you have :math:`n` objects in the landscape which have a
general discrete (bitmap) shape. This means each obstacle is a union of
squares. After the motion step, one needs to check if any pixel occupied
by the robot is also occupied by an obstacle. After the move, all of the
pixels associated with the robot location are checked. If any pixel is
painted black, then the robot has driven into an obstacle. This should
generate an event associated with a crash and signal both the graphics
window as well as the control program.

This same approach may be used to simulate a bump or touch sensor. The
boundary pixels of the robot can be used as the sensors. If those pixels
are adjacent to an object, then a touch is registered. This can be done
via a direct adjacency test - looking at neighbor pixels. Or this can be
done by a ghost pixel method. Inflate the objects by one row of pixels.
If the robot overlaps a ghost pixel then the corresponding robot
pixel/sensor would register a touch. Inflation is essentially one step
of a flood fill. Flood fill is normally implemented as a type of DFS
algorithm, but does not need this machinery here. A common error here is
to write the adjacent pixel into the current array, but if you are
sweeping through the image row by row, you might use this new pixel as
an obstacle pixel. Then this new sweep will mark the neighbor. This can
continue and cause lines to grow down with the sweep.

An array used for the image data which stores multiple values can be
very handy. Pixels are marked according to type. For example, zero for
open pixel, one for a robot occupied pixel, two for an obstacle, three
for the inflated set. Later on having data stored per pixel (or cell) is
essential for our motion planners.

Getting a Map
~~~~~~~~~~~~~

There are a variety of approaches for getting map data created. Various
range sensors and cameras have been used to create viable maps. Pixel
based occupancy grids are relatively easy to create. For mazes, we might
only be interested in the topology of occupied or free space. A
skeletonization of free space returns a graph that can be used for graph
based search algorithms.
