Two Link Arm Revisited
----------------------

Articulated (multilink) robot arms also have size and orientation.
Determining which configurations and which physical positions are
actually realizable is more complicated. The size of the robot arm will
affect the regions which the end effector can reach but obstacle
inflation does not give the same workspace. The end effector is designed
to touch an object and from that perspective little inflation is
required. However the base link of the arm might be very wide and does
affect the useable workspace. A simple obstacle inflation approach will
not work with manipulators. The reason is that how you travel affects
your reach. Figure \ `[Fig:pathmatters] <#Fig:pathmatters>`__ shows how
the path matters to access. A more situation can be found in
Figure \ `[Fig:nopaththrough] <#Fig:nopaththrough>`__. Even though the
articulator is small enough to pass through the gap, it cannot due to
the other physical restrictions.

.. raw:: latex

   \centering

.. figure:: ./configuration/pathmatters
   :alt: The elbow down approach is blocked, but not the elbow up
   position. [Fig:pathmatters]

   The elbow down approach is blocked, but not the elbow up position.
   [Fig:pathmatters]

.. figure:: ./configuration/nopaththrough
   :alt: Neither configuration of the robot arm can reach the point.
   [Fig:nopaththrough]

   Neither configuration of the robot arm can reach the point.
   [Fig:nopaththrough]

.. raw:: latex

   \FloatBarrier
