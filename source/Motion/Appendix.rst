Appendix
--------

The proof for Theorem \ `[disktraverseDDD] <#disktraverseDDD>`__,
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
:raw-latex:`\cite{munkres2000topology}`. These correspond to a finite
set of open disks which cover the path. The vehicle may travel a
straight line from disk center to disk center. At each center the
vehicle may reorient if required. The time limited reach for the DDD
drive is a proper subset of the FWS reach, and follows from the DDD
result.

.. |Turn geometry for the DDD (left) and FWS (right) designs. [fig:turngeo]| image:: motion/curvature
.. |Turn geometry for the DDD (left) and FWS (right) designs. [fig:turngeo]| image:: motion/curvature2
.. |image| image:: /motion/mecanumpath
.. |image| image:: solutions/MotionModel/p6-14exact
.. |image| image:: solutions/MotionModel/p6-14noise
.. |image| image:: solutions/MotionModel/p6-14ellipse
.. |image| image:: motion/omniwheelmounting

