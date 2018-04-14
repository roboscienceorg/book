The Ground Robot World
----------------------

One of the main differences many see between a vehicle and a robotic
vehicle is whether or not a person is “onboard". If you are driving a
car, then we would not call this a robot. But if your car was remotely
operated, then some would call it a robotic car.  [2]_ Can we make the
robot simulation remotely operated? In this case we mean, *can this be
controlled from an external program?* The answer is yes.

The previous robot code examples allow the user to move a simulated
device around an open rectangle. The world has obstacles and a
simulation should reflect this. So, how should we include obstacles? The
simulation is in two dimensions and so the obstacle will also be in 2D.
The obstacle is then represented as a 2D shape as viewed from above. The
presentation of the simulation is in a window which means at some point
the robot and obstacles are presented on a grid or in a discrete
fashion. This means we have some choices on how to represent the world,
obstacles and other objects,
Figure \ `[fig:enviromodel] <#fig:enviromodel>`__.

The environment can be represented in three different manners:
continuous, discrete and topological. Continuous is how we tend to think
about the world. All of the locations and distances for objects,
ourselves and the robots use floating point values. For example, the
center of the robot would be located by a pair of floating point values
and exact information about the robot shape stored in a database,
Figure \ `[fig:metricmap] <#fig:metricmap>`__.

For a discrete representation, the world is discretized and objects are
located using integer values,
Figure \ `[fig:discretemap] <#fig:discretemap>`__. The world is then a
large checkerboard with a square (pixel) either occupied or not
occupied. Simple two or three color bitmaps then suffice (two for object
maps and optionally a third to track the robot). Painting a pixel white
will indicate that pixel or location is unoccupied. Painting it colored
indicates the pixel is occupied. This approach is known as an occupancy
grid. The obstacle is simply the collection of black pixels on the
occupancy map. A B/W image file can then be used to generate obstacle
maps. [One handy way to accomplish this task is to use a paint program
(or image editing tool) which can export the image into a format that is
easy to read. ]

.. raw:: latex

   \centering

.. raw:: latex

   \centering

.. figure:: planning/envrep
   :alt: How one should represent the environment.[fig:enviromodel]

   How one should represent the environment.[fig:enviromodel]

.. raw:: latex

   \centering

.. figure:: slam/metricmap
   :alt: Continuous environmental representation.[fig:metricmap]

   Continuous environmental representation.[fig:metricmap]

.. raw:: latex

   \centering

.. raw:: latex

   \centering

.. figure:: slam/discretemap
   :alt: Discrete environmental representation. [fig:discretemap]

   Discrete environmental representation. [fig:discretemap]

.. raw:: latex

   \centering

.. raw:: latex

   \centering

.. figure:: slam/topomap
   :alt: Topological representation. [fig:topomap]

   Topological representation. [fig:topomap]

Topological representations do not include metric information like the
other two, Figure \ `[fig:topomap] <#fig:topomap>`__. Relationships are
through graphs that indicate two things are connected via a path. How
they are connected is another issue. This is very much how humans store
maps. You probably know that to get to your favorite restaurant, you
have to pass the Home Depot and take the next right. Then you keep going
until you pass the Whole Foods market. Then a quick left and there you
are. In this description, no distances were provided and even the notion
of left and right are flexible since we don’t require the streets
intersect at right angles.

For the case of the robot simulation, the choice has been partially
made. The robot’s world appears as an image which is a discretization or
a grid. Thus we have a discrete environment. We might decide to go with
an obstacle map. Each obstacle is just written into the map and then
disappears in to the large collection of filled pixels. Or we may elect
to keep our obstacles in a continuous representation. However, this
means that translations between the continuous and discrete forms must
happen often.

Continuous and discrete forms each have strengths and weaknesses. We
have very precise information in the continuous form. To increase
precision in the discrete world, we must decrease pixel size which
increases the array storage dramatically or forces a more sophisticated
data format over a simple 2D array. Although storage has increased, many
operations in the discrete world are much easier.

Consider the problem of simulating a robot impact on a object. Say that
the object has an irregular shape. This shape can be approximated by the
pixelized version in the discrete world or by a cubic spline
approximation using a continuous approximation. True that you have much
better accuracy with the cubic spline. The problem is in determining
intersection of the robot boundary with the object boundary. In the
continuous world, we need to take both of the functions and look for
intersecting boundaries at each time step. This requires a complex
nonlinear equation solving routine. [Just work out the algebra for two
circles intersecting.] For the bitmap version we just check that the
front of the robot is on an occupied pixel or cell (if cell[i][j] == 1
then ....).

The continuous version will keep objects as objects. For example, if you
have disks that touch, the continuous representation will track the
centers and radii of the two disks. You always know you have multiple
objects. Once converted to a bit map, it could be two adjacent objects
or one connected object or multiple partial objects, etc. It is the
difference between high and low level representations. A topological
representation takes this approach to the next level by removing metric
information and just keeping object description in a connectivity graph.
Many factors enter into the choice of representation. It is always a
trade off between speed, accuracy and simplicity.

Simple Obstacles
~~~~~~~~~~~~~~~~

The simplest object to study is a disk. It is simple not only in
geometry, but in the more difficult task of determining collision. We
know that if any part of our robot is within a radius of the center, we
have collided. Our robots are round, so collision is just checking the
distance between centers minus the radii. It makes a good stage for a
first path planning exercise. We assume for the moment that our robot
can move freely around the plane (in the open space) and that the plane
is covered with disk shaped obstacles. We also assume that the robot
knows its coordinate location and heading. For a given obstacle map, can
we find a path connecting two points in the plane?

The Python code to check if two disks intersect is fairly
straightforward:

::

    def collide(center1, r1, center2, r2):
        x1 = center1[0]
        y1 = center1[1]
        x2 = center2[0]
        y2 = center2[1]
        d = sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2))-r1-r2
        return d

Where center is a list and r is the radius.  [3]_

.. raw:: latex

   \centering

.. figure:: turtle/collision
   :alt: Collision detection with circular robots. [circlecollide]

   Collision detection with circular robots. [circlecollide]

To check for intersection, we only need to check that :math:`d` is
small. Using this we may build a method for a contact sensor. You can
treat a contact sensor as a disk of zero radius and use the formula
above (adjusting for the relation between the center of the robot and
the sensor). Many early robots had sensors placed in a ring around the
body of the robot, Figure \ `[turtlesensors] <#turtlesensors>`__. For
this example, they will be contact or touch sensors, but in experimental
units often low cost ultrasonic ranging sensors would be used.

.. raw:: latex

   \centering

.. figure:: turtle/turtlesensors
   :alt: A circular robot (like a Create) with touch sensors mounted
   around the body. [turtlesensors]

   A circular robot (like a Create) with touch sensors mounted around
   the body. [turtlesensors]

Assume that you have a circular robot with a ring of touch or bump
sensors around the body. Knowing the direction of travel, it is possible
to estimate the boundary of the obstacle relative to the robot,
Figure \ `[turtleboundary] <#turtleboundary>`__. The boundary normal can
be estimated from the vector created by the sensor location to the robot
center. This is a local estimate only as
Figure \ `[turtleboundary] <#turtleboundary>`__ shows. Being able to
estimate the boundary means that a robot can follow the boundary. The
tangent to the boundary is required for this task.

.. raw:: latex

   \centering

|a) Estimating the object boundary. b) Bump sensors can only determine
the nature of the boundary at the contact location. c) Using touch
sensors to estimate the boundary normal and tangent. [turtleboundary]|
|a) Estimating the object boundary. b) Bump sensors can only determine
the nature of the boundary at the contact location. c) Using touch
sensors to estimate the boundary normal and tangent. [turtleboundary]|
|a) Estimating the object boundary. b) Bump sensors can only determine
the nature of the boundary at the contact location. c) Using touch
sensors to estimate the boundary normal and tangent. [turtleboundary]|

Using the normal vector, :math:`\hat{n} = <n_1, n_2>`, the tangent to
the boundary is computed via

.. math:: T = \pm <n_2, -n_1>

where the sign is taken so that motion is to the right (right hand
rule). This tangent direction will provide the motion direction for a
boundary following approach. Estimation of the tangent or the direction
of travel can be done with a ring of touch sensors,
Figure \ `[turtleboundary] <#turtleboundary>`__.

Using a range sensor
^^^^^^^^^^^^^^^^^^^^

Recall the components in
Figure \ `[intro-components] <#intro-components>`__. There was not a
touch or impact sensor listed. However, there are two types of range
sensors shown. One is a LIDAR and the other is a Kinect. The next simple
planner presented assumes that the robot has a ranging device. The
simplest to model is the LIDAR.

.. raw:: latex

   \centering

.. figure:: slam/discretemap2
   :alt: Discrete object map.[discreteobjmap]

   Discrete object map.[discreteobjmap]

A lidar is a simple device conceptually. The unit is able to sweep or
turn in one direction which for our discussion we assume it is
horizontal. It chops up the angular variable into some number of
discrete angles. At each angle or direction, the lidar unit projects a
laser beam out. It receives the reflected signal and computes the
distance. Naively one simply measures the time of flight, divides by two
(for the round trip) and multiplies by :math:`c` (the speed of light):
:math:`D = RT`. This provides the distance of the nearest obstacle at
the current angle. Record the number and move to the next angle.

A sweep creates an array of values where the array index is a function
of the angle and array values are distances. The unit will return the
array. Angles can be reconstructed if you know the starting angle and
the angular increment: :math:`\theta_i = \theta_0 + i\Delta\theta`. If
you are simulating a given LIDAR unit, then one would use the increment
angle of that unit. If not, then you will decide on the details of
angular increment, maximum range, minimum range and data rate.

How is this done in a discrete environment? Using a two colored image,
let white be free space and red or black indicate occupied space. To
simulate the beam out of the LIDAR, create a virtual line out of the
lidar and follow a straight line along white pixels until you run into a
colored pixel. Stop at the first colored pixel. Using the endpoints of
the line segment (virtual lidar to object pixel), the distance can be
computed. Let :math:`(n,m)` be the start of the line and let
:math:`(i,j)` be the location of the object pixel and recall the
distance is :math:`d = \sqrt{(i-n)^2 + (j-m)^2}`. [4]_

Any actual lidar unit has an effective range, :math:`R`. In simulation
one could certainly compute :math:`d` as you move out along the ray (or
line) and stop when the max range occurred. This approach will work but
it requires computing the distance function within the innermost loop
and will not result in efficient code. A more effective approach is to
just step out in the radial variable. This means you need to represent
the line or ray in polar coordinates. We will assume that :math:`R` is
given in the pixel coordinates and the range would be
:math:`0 \leq r \leq R`. The other issue is increment value for the
lidar simulation. Again, if this value is taken from an actual unit,
then that is the value to use. Otherwise, at the maximum range,
:math:`R`, we would like that an increment in the angle selects the
“next” (adjacent) pixel. So we want :math:`\Delta \theta` to be small
enough to hit all the pixels, but no smaller for performance reasons,
see Figure \ `[inscribedcircle] <#inscribedcircle>`__ (b).The
circumference is :math:`2\pi R`. If a pixel is :math:`1^2` units, then
we select :math:`\Delta\theta \approx 1/(2\pi R)` (or slightly smaller).

.. raw:: latex

   \centering

.. figure:: path/lidarinc
   :alt: Laser angle increments. (a) The first is too small and we
   resample the same pixel. (b) The second increment is too large and we
   miss pixels. [inscribedcircle]

   Laser angle increments. (a) The first is too small and we resample
   the same pixel. (b) The second increment is too large and we miss
   pixels. [inscribedcircle]

The lidar simulation algorithm is given in
Algorithm \ `[lidarsim] <#lidarsim>`__.

:math:`k=0` :math:`\Delta\theta = 1/(2\pi R)`

.. raw:: latex

   \FOR    {$\theta=0$  \TO $2\pi$}

.. raw:: latex

   \FOR      {$r=0$ \TO $R$}

:math:`i= (\text{int}) r \cos \theta`

:math:`j= (\text{int}) r\sin\theta`

.. raw:: latex

   \IF {Map$(i,j)$ is occupied}

break from :math:`r` loop :math:`dist(k) = r`

k++ :math:`\theta += \Delta\theta`

.. raw:: latex

   \ENDFOR
