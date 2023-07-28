Configuration space and reach for simple vehicles
-------------------------------------------------

In this section, we determine the reach (and time limited reach) of the
robot from a given point and the possible paths between two points. Does
the reach cover the plane or are there some points in the plane which
cannot be reached? First, we make precise what is meant by reach
:cite:`lavalle2006`. Let :math:`X` be the state space,
:math:`{\cal U} \subset X` be the set of all permissible trajectories on
:math:`[0,\infty)` and :math:`R(q_0,{\cal U} )` denote the reachable set
from :math:`x_0`.

We define the reachable set as

.. math:: R(x_0,{\cal U} ) = \left\{  x_1 \in X | \exists \tilde{u}\in {\cal U} \mbox{ and } \exists t \in [0,\infty) \mbox{ s.t. } x(t) = x_1 \right\}

Let :math:`R(q_0,{\cal U} ,t)` denote the time-limited reachable set
from :math:`x_0`.

We define the time-limited reachable set as

.. math:: R(x_0,{\cal U},t ) = \left\{ x_1 \in X | \exists \tilde{u}\in {\cal U} \mbox{ and } \exists \tau \in [0,t] \mbox{ s.t. } x(\tau) = x_1 \right\}

The Dubins Car, :cite:`dubins`, is a vehicle that can move
straight forward or turn at any curvature up to some maximum curvature.
This vehicle provides a geometric motion model for automobiles and can
be used to understand basic optimal path planning. The Reeds-Shepps Car,
:cite:`reeds`, extends the Dubins vehicle to include reverse
motion. This greatly enhances maneuverability. Small back and forth
motions can realign a vehicle to a new orientation. This means if the
robot arrives at a destination point with the wrong orientation, it can
be corrected locally (assuming sufficient room about the point).

Dubins showed that a vehicle which can go only forward and turn at any
curvature up to some maximum curvature can reach any point in the plane
in the absence of obstacles :cite:`dubins`. Optimality of
solutions is discussed in
:cite:`kelly2013mobile`, :cite:`lavalle2006`. A slight
generalization is given in :cite:`reeds` for a car that can
go forwards and backwards. In
:cite:`reeds`, :cite:`sussman`, :cite:`lavalle2006`, it is shown that
optimal solutions are piecewise collections of line segments and maximum
curvature circles. Since the DDD (dual differential drive) and FWS (four
wheel steer) designs have less restrictive motion, we can answer the
reach question. The entire plane can be covered. The question of optimal
paths will be left for a future study. The FWS system we have built is
targeted for an environment filled with obstacles. Our main concern is
reach in the presence of obstacles, for which the reach and the optimal
path results for Dubins and Reeds-Shepps are no longer valid.

Both the DDD and FWS designs are more maneuverable than the Dubins
vehicle, and so we expect more flexibility in dealing with obstacles.
The time limited reach of the Dubins Car is the forward fan seen in
:numref:`fig:fmotion` and the time limited reach of
the Reeds-Shepps car is the open set about the initial point
:cite:`lavalle2006`. Since both the DDD and FWS systems
include the motion patterns found in the Reeds-Shepps car, the time
limited reach for these two designs is an open set about the initial
point: there exists a set :math:`U`, open, such that
:math:`U \subset R(x_0,{\cal U},t )`. This is possible due to the
ability to perform back and forth maneuvers like that found in parallel
parking.

Rigid Motion
~~~~~~~~~~~~

The FWS can move from point to point and then adjust orientation as
required. If there exists a path between two points, the FWS axle can
traverse the path via the waypoints, re-orient at each point and reach
the goal location. Thus it can follow a piecewise linear path between
two configuration space locations. A smooth path can be found by using a
b-spline and if curvature exceeds the maximum bound, the vehicle can
stop, re-orient and then continue. Traversal is possible if the start
and goal locations are path connected and that path locations with
curvature above :math:`R` have a disk of radius :math:`r` centered at
the path point which does not intersect any obstacle.

The DDD design has additional constraints compared to the FWS design.
The solution that :cite:`reeds`, :cite:`sussman`, :cite:`lavalle2006`
suggest is to perform a series of short adjustment maneuvers as seen in
:numref:`fig:deltatheta`. Although the results
for re-orientation can be applied to arbitrarily small robots and
adjustment regions, in practice for a given robot or vehicle, the region
has some minimum size. Assume that the adjustment maneuvers falls in a
circle of radius :math:`r`. Let :math:`W` be a bounded domain in
:math:`{\Bbb R}^2`, the obstacles be :math:`{\cal O}_i` and the free
space be given by :math:`\Omega = W\setminus \cup_{i}{\cal O}_i`.

.. _`fig:deltatheta`:
.. figure:: MotionFigures/deltatheta.*
   :width: 20%
   :align: center

   A series of short adjustment maneuvers to re-orient the vehicle.


For simplicity here, we assume the domain satisfies a traversability
condition. Let :math:`D(x,r)` be the disk of radius :math:`r` centered
at :math:`x`. :math:`\Omega` is said to be disk traversable if for any
two points :math:`x_0,x_1 \in \Omega`, there exists a continuous
function :math:`p(t)\in{\Bbb R}^2` and :math:`\epsilon >0` such that
:math:`D(p(t),\epsilon)\subset\Omega` for :math:`t\in [0,1]` and
:math:`x_0=p(0)`, :math:`x_1=p(1)`. Note that :math:`p(t)` generates the
curve :math:`C` which is a path in :math:`\Omega` and the path is a
closed and bounded subset of :math:`\Omega`. Navigation along jeep
trails, bike trails and large animal trails (in our case, Cattle and
Bison) produces small corridors though the forest. Along these tracks
there is a corridor produced which we describe as disk traversable.

.. _`disktraverseDDD`:

**Traversability Theorem:**  If :math:`\Omega` is disk traversable, then the DDD
and FWS vehicles can navigate to the goal ending with the correct
orientation.  **Proof:** See :ref:`Appendix <appendix>`.


The Piano Movers Problem - Orientation
--------------------------------------

Assume you want to route an object with a complicated shape through a
tight sequence of corridors. Routing a complex shape through a narrow
passage is often referred to as the :index:`piano movers problem`. Take a simple
example, move the linear robot through the two blocks,
:numref:`robotmustrotate`. It is clear to the
human what has to happen. The robot must rotate. For a holonomic robot,
this simply means the controller issues a rotation command while
traveling to the corridor. For a non-holonomic robot, the control system
must change the path so that upon entry and through the corridor the
robot’s orientation will allow for passage. A significant problem arises
if the corridor is curved in a manner that is not supported by the
possible orientations defined by the vehicle dynamics. In plain English,
this is when you get the couch stuck in the stairwell trying to move
into your new flat.

.. _`robotmustrotate`:
.. figure:: MotionFigures/obst.*
   :width: 40%
   :align: center

   The object must rotate to fit through the open
   space.

As all of us learned when we were very young, we must turn sideways to
fit through a narrow opening. [#f3]_   This introduces a new aspect to
routing, that of reconfiguration of the robot. Examine a simple
reconfiguration which is simply a change in orientation. As we saw
above, each rotation of the robot induces a different configuration
space. :numref:`robotrotation` shows the idea for
three different rotation angles, there are three different configuration
obstacle maps.

.. _`robotrotation`:
.. figure:: MotionFigures/obst2.*
   :width: 70%
   :align: center

   Different rotations produce different obstacle maps in configuration
   space.

Since each rotation generates a two dimensional configuration space,
they can be stacked up in three dimensions. So we have that
configuration space includes the vertical dimension which is the
rotation angle for the robot - the configuration space is three
dimensional. To restate, the configuration space includes all of the
configuration variables :math:`(x,y, \theta)` is now a three dimensional
configuration space which is shown in
:numref:`robotrotation3D`.   So, although the
workspace is two dimensional, the configuration space is three
dimensional and are different objects.

.. _`robotrotation3D`:
.. figure:: MotionFigures/obst3.*
   :width: 70%
   :align: center

   The different rotations can be stacked where the vertical dimension
   is the rotation angle.

For a three dimensional object with a fixed orientation, would have a
three dimensional configuration space. For toolheads, only pitch and yaw
matter. To locate a point on a sphere you need two variables (think
about spherical coordinates): :math:`\theta` the angle in the
:math:`x`-:math:`y` plane and :math:`\phi` the angle from the :math:`z`
axis (or out of the plane if you prefer). For each pair
:math:`(\theta, \phi)` we have a 3D section. This tells us that the
configuration space is five dimensional. When roll, pitch and yaw all
matter then we have a 6 dimensional configuration space. If the robot is
configurable with other elements, then each parameter defining the
configuration would also add a variable to the mix and increase the
dimension of the configuration space.

The construction of configuration space then is built like slices in a
3D printer. Routing or path planning must be done in the full
configuration space. For the current example, we must route in 3D which
will translate to position and orientation routing in the workspace,
:numref:`obst4`. 

.. _`obst4`:
.. figure:: MotionFigures/obst4.*
   :width: 50%
   :align: center

   We can see that there is a path that includes the rotation.



Two Link Arm Revisited
~~~~~~~~~~~~~~~~~~~~~~~

Articulated (multilink) robot arms also have size and orientation.
Determining which configurations and which physical positions are
actually realizable is more complicated. The size of the robot arm will
affect the regions which the end effector can reach but obstacle
inflation does not give the same workspace. The end effector is designed
to touch an object and from that perspective little inflation is
required. However the base link of the arm might be very wide and does
affect the useable workspace. A simple obstacle inflation approach will
not work with manipulators. The reason is that how you travel affects
your reach.  :numref:`Fig:pathmatters` shows how
the path matters to access. A more situation can be found in
:numref:`Fig:nopaththrough`. Even though the
articulator is small enough to pass through the gap, it cannot due to
the other physical restrictions.

.. _`Fig:pathmatters`:
.. figure:: MotionFigures/pathmatters.*
   :width: 50%
   :align: center

   The elbow down approach is blocked, but not the elbow up position.

.. _`Fig:nopaththrough`:
.. figure:: MotionFigures/nopaththrough.*
   :width: 50%
   :align: center

   Neither configuration of the robot arm can reach the point.


.. _`appendix`:

Appendix
--------

The proof for the :ref:`Traversability Theorem <disktraverseDDD>`,
statement reproduced below, is given here.

If :math:`\Omega` is disk traversable, then the DDD and FWS vehicles can
navigate to the goal ending with the correct orientation.

**Proof:** Let :math:`C` be the path from :math:`x_0` to :math:`x_1`. At
each point of the path there exists an open disk of radius
:math:`\epsilon` which does not intersect an obstacle. The intersection
of the curve :math:`C` with the open disk induces an open set in
:math:`C`. The collection of open sets is an open cover of the curve
:math:`C`. Since the curve is a closed and bounded set, and thus
compact, there is a finite subcover of open intervals
:cite:`munkres2000topology`. These correspond to a finite
set of open disks which cover the path. The vehicle may travel a
straight line from disk center to disk center. At each center the
vehicle may reorient if required. The time limited reach for the DDD
drive is a proper subset of the FWS reach, and follows from the DDD
result.


.. rubric::  Footnotes

.. [#f3] Cavers will tell you that you can crawl through a vertical gap spanned by the distance of your thumb and your fifth (pinky) finger.  For the average American, this is a very small gap.
