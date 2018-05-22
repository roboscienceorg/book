The Piano Movers Problem - Orientation
--------------------------------------

Assume you want to route an object with a complicated shape through a
tight sequence of corridors. Routing a complex shape through a narrow
passage is often referred to as the :index:`piano movers problem`. Take a simple
example, move the linear robot through the two blocks,
Figure :numref:`robotmustrotate`. It is clear to the
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
.. figure:: TermsFigures/obst.*
   :width: 40%
   :align: center

   The object must rotate to fit through the open
   space.

As all of us learned when we were very young, we must turn sideways to
fit through a narrow opening. [#f3]_   This introduces a new aspect to
routing, that of reconfiguration of the robot. Examine a simple
reconfiguration which is simply a change in orientation. As we saw
above, each rotation of the robot induces a different configuration
space. Figure :numref:`robotrotation` shows the idea for
three different rotation angles, there are three different configuration
obstacle maps.

.. _`robotrotation`:
.. figure:: TermsFigures/obst2.*
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
Figure :numref:`robotrotation3D`.   So, although the
workspace is two dimensional, the configuration space is three
dimensional and are different objects.

.. _`robotrotation3D`:
.. figure:: TermsFigures/obst3.*
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
Figure :numref:`obst4`. Path planning or motion planning is
addressed in Planning Chapter.

.. _`obst4`:
.. figure:: TermsFigures/obst4.*
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
your reach. Figure :numref:`Fig:pathmatters` shows how
the path matters to access. A more situation can be found in
Figure :numref:`Fig:nopaththrough`. Even though the
articulator is small enough to pass through the gap, it cannot due to
the other physical restrictions.

.. _`Fig:pathmatters`:
.. figure:: TermsFigures/pathmatters.*
   :width: 50%
   :align: center

   The elbow down approach is blocked, but not the elbow up position.

.. _`Fig:nopaththrough`:
.. figure:: TermsFigures/nopaththrough.*
   :width: 50%
   :align: center

   Neither configuration of the robot arm can reach the point.

.. rubric::  Footnotes

.. [#f3] Cavers will tell you that you can crawl through a vertical gap spanned by the distance of your thumb and your fifth (pinky) finger.  For the average American, this is a very small gap.
