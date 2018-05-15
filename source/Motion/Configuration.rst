Configuration space and reach for simple vehicles
-------------------------------------------------

In this section, we determine the reach (and time limited reach) of the
robot from a given point and the possible paths between two points. Does
the reach cover the plane or are there some points in the plane which
cannot be reached? First, we make precise what is meant by reach
:raw-latex:`\cite{lavalle2006}`. Let :math:`X` be the state space,
:math:`{\cal U} \subset X` be the set of all permissible trajectories on
:math:`[0,\infty)` and :math:`R(q_0,{\cal U} )` denote the reachable set
from :math:`x_0`.

We define the reachable set as

.. math:: R(x_0,{\cal U} ) = \left\{  x_1 \in X | \exists \tilde{u}\in {\cal U} \mbox{ and } \exists t \in [0,\infty) \mbox{ s.t. } x(t) = x_1 \right\}

Let :math:`R(q_0,{\cal U} ,t)` denote the time-limited reachable set
from :math:`x_0`.

We define the time-limited reachable set as

.. math:: R(x_0,{\cal U},t ) = \left\{ x_1 \in X | \exists \tilde{u}\in {\cal U} \mbox{ and } \exists \tau \in [0,t] \mbox{ s.t. } x(\tau) = x_1 \right\}

The Dubins Car, :raw-latex:`\cite{dubins}`, is a vehicle that can move
straight forward or turn at any curvature up to some maximum curvature.
This vehicle provides a geometric motion model for automobiles and can
be used to understand basic optimal path planning. The Reeds-Shepps Car,
:raw-latex:`\cite{reeds}`, extends the Dubins vehicle to include reverse
motion. This greatly enhances maneuverability. Small back and forth
motions can realign a vehicle to a new orientation. This means if the
robot arrives at a destination point with the wrong orientation, it can
be corrected locally (assuming sufficient room about the point).

Dubins showed that a vehicle which can go only forward and turn at any
curvature up to some maximum curvature can reach any point in the plane
in the absence of obstacles :raw-latex:`\cite{dubins}`. Optimality of
solutions is discussed in
:raw-latex:`\cite{kelly2013mobile, lavalle2006}`. A slight
generalization is given in :raw-latex:`\cite{reeds}` for a car that can
go forwards and backwards. In
:raw-latex:`\cite{reeds, sussman, lavalle2006}`, it is shown that
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
Figure \ `[fig:fmotion] <#fig:fmotion>`__ and the time limited reach of
the Reeds-Shepps car is the open set about the initial point
:raw-latex:`\cite{lavalle2006}`. Since both the DDD and FWS systems
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
The solution that :raw-latex:`\cite{reeds, sussman, lavalle2006}`
suggest is to perform a series of short adjustment maneuvers as seen in
Figure \ `[fig:deltatheta] <#fig:deltatheta>`__. Although the results
for re-orientation can be applied to arbitrarily small robots and
adjustment regions, in practice for a given robot or vehicle, the region
has some minimum size. Assume that the adjustment maneuvers falls in a
circle of radius :math:`r`. Let :math:`W` be a bounded domain in
:math:`{\Bbb R}^2`, the obstacles be :math:`{\cal O}_i` and the free
space be given by :math:`\Omega = W\setminus \cup_{i}{\cal O}_i`.


.. figure:: MotionFigures/deltatheta.*
   :width: 20%
   :align: center

   A series of short adjustment maneuvers to re-orient the vehicle.
   [fig:deltatheta]

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

[disktraverseDDD] If :math:`\Omega` is disk traversable, then the DDD
and FWS vehicles can navigate to the goal ending with the correct
orientation.

**Proof:** See Chapter Appendix.
