Animation of the Two Link Manipulator
-------------------------------------

[example_twolinkmanipulator] For the arm in the two link example,
determine the joint angles to trace out a circle centered at (10,8) of
radius 5. The circle can be parametrized by
:math:`x(t) = 5\cos (t) + 8`, :math:`y(t) = 3 \sin(t) + 10`,
:math:`-\pi \leq t \leq \pi`. Generate an array of points on the circle
and plug them into the inverse kinematics.

Bring up the two link simulator. Then run the following code in Python.
You should see an animation of the two link arm drawing a circle. The
final position is given in
:numref:`Fig:twolinkcircleexample`.

::

    # Bring in libraries
    import rospy
    from std_msgs.msg import String
    import numpy as np
    import time
    from math import *

::

    #Setup Arrays
    step = 0.1
    t = np.arange(-pi, pi, step)
    x = 5.0*np.cos(t) + 8.0
    y = 3.0*np.sin(t) + 10.0

::

    #Initialize variables
    a1 = 10.0
    a2 = 10.0
    d = (x*x + y*y - a1*a1 - a2*a2)/(2*a1*a2)
    t2 = np.arctan2(-np.sqrt(1.0-d*d),d)
    t1 = np.arctan2(y,x) - np.arctan2(a2*np.sin(t2),a1+a2*np.cos(t2))

::

    # Setup ROS and publish joint data
    pub = rospy.Publisher('TwoLinkTheta', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)

    for i in range(t.size):
       print t1[i], "  ", t2[i]
       m = str(180*t1[i]/np.pi) + ":" + str(180*t2[i]/np.pi) + ":" + str(1)
       time.sleep(0.25)
       pub.publish(m)

.. _`Fig:twolinkcircleexample`:
.. figure:: SimulationFigures/twolinkcircleexample.png
   :width: 60%
   :align: center

   The output of the circle inverse kinematics
   code.

In this example, we generate an array named t which is used for the
parametric equations of the circle to generate the x and y arrays. We
may use the inverse kinematic formulas to determine the arrays for
:math:`\theta_1` and :math:`\theta_2` called t1 and t2. The
:math:`\theta_1` and :math:`\theta_2` would be the values sent to the
joint actuators.
:numref:`Fig:twolinkcircleexample`
shows the results.

You can modify the data arrays to plot a line:

::

    #Setup Arrays
    t = np.arange(-5, 8, step)
    x = t
    y = x + 5

The inverse kinematics can be placed into a separate ROS node. The
driving program follows (same headers as before). To connect to the
simulation program, we use the inverse kinematics node as before

::

    #Setup Arrays
    a1 = 10
    a2 = 10
    step = 0.1
    t = np.arange(-pi, pi, step)
    x = 5.0*np.cos(t) + 8.0
    y = 3.0*np.sin(t) + 10.0

    pub = rospy.Publisher('TwoLinkCoords', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)

    for i in range(t.size):
       locs = str(x[i]) + ":" + str(y[i]) + ":" + str(10) + ":" + str(10)
                       +":" + str(1)
       time.sleep(0.25)
       pub.publish(locs)


.. figure:: SimulationFigures/twolinkcoarseexample.png
   :width: 60%
   :align: center

    Movement between the points - moving both linearly.

.. _`Fig:twolinkcoarseexample`:
.. figure:: SimulationFigures/twolinkcoarseexample2.png
   :width: 60%
   :align: center

    Movement between the points - moving the servos sequentially.

This simulation gives an idea about how to move the robotic arm and the
path is correct. The motion however is not smooth. This is because we
are moving the arm from position to position. This is known as position
control. If you look at the curve produced, it is not a smooth curve but
is a curve made of of connected segments like a polygon,
:numref:`Fig:twolinkcoarseexample`.
Note that the output is not actually a polygon; the sides are not
straight line segments.

In between the control points, the system moves according to how the
controllers are programmed. They will move the joint angles in a linear
fashion. If they are moved together you will see
:numref:`Fig:twolinkcoarseexample`(a).
If they are moved one at a time you will see
:numref:`Fig:twolinkcoarseexample`(b).
