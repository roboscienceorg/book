Simulating Motion
-----------------

As stated before, producing motion is not difficult. Deciding on the
correct actions and controlling the system are the more challenging
aspects. The first is known as *Motion Planning* or just *Planning* and
the second is called *Controls*. To get started we will borrow
algorithms from nature since we see so many successful autonomous agents
in the biological world. Worms and insects are very successful animals.
They can sense the world and move around in it. We can borrow from
notions in physics and chemistry when we see simple systems moving in
constrained manners. The simplest solution is the best solution. It is
best to use no more components or technology than necessary. Beyond
basic elegance is the fact that the more components something has, the
greater probability the system will fail. This is true in our
simulations as well.

We return to the two examples in the previous section, the Two Link
Manipulator and the Mobile Disk Robot. Using these two systems, we will
introduce methods to simulate motion. These very basic systems can be
used as the prototypes for developing a simulation and for the simple
motion planning algorithms.

You can download the simulators by following the links on D2L. To get
started, again you need to be in your Ubuntu session and run the ROS
Master:

::

    >  roscore

You can run the Two Link Manipulator simulator we will use by typing

::

    >  python twolinksimple.py

and you should see what is indicated in
Figure \ `[Fig:twolinksimulator1] <#Fig:twolinksimulator1>`__-(a). In
another terminal, run Python and type

::

    >>> import rospy
    >>> from std_msgs.msg import String
    >>> pub = rospy.Publisher('TwoLinkTheta', String, queue_size=10)
    >>> rospy.init_node('talker', anonymous=True)
    >>> message = "20:10:0"
    >>> pub.publish(message)

.. raw:: latex

   \centering

|(a) The two link simulator. (b) Published angle to the
simulator.[Fig:twolinksimulator1]| |(a) The two link simulator. (b)
Published angle to the simulator.[Fig:twolinksimulator1]|

You should see the link arm move as shown in
Figure \ `[Fig:twolinksimulator1] <#Fig:twolinksimulator1>`__-(b). The
API is very simple. You need to publish a string formatted as
"theta1:theta2:pen". The values theta1 and theta2 are in degrees (int or
float), and pen is an int. Pen is set to 1 to draw and 0 to not draw.
The program DialCntrl.py is an example of a Tk widget that uses two
sliders to set the angle,
Figure \ `[Fig:tksliderexample] <#Fig:tksliderexample>`__\ (a). To gain
an understanding of the ROS Node structure, one may list out the ROS
nodes (example, your numbers will vary):

::

    rosnode list
    /DialController_5943_1473004072330
    /TwoLinkSimulation_5785_1473004028541
    /rosout

To view the resulting node graph we can use the ROS tool rqt_graph:

::

    rosrun rqt_graph rqt_graph

In this case it produces Figure \ `[Fig:rosgraph0] <#Fig:rosgraph0>`__.

.. raw:: latex

   \centering

.. figure:: sim/rosgraph0.png
   :alt: The ROS Node Graph Tool rqt_graph. [Fig:rosgraph0]

   The ROS Node Graph Tool rqt_graph. [Fig:rosgraph0]

If you are curious about the messages flowing on a topic, recall ROS can
echo those to a terminal for debugging purposes. In a free terminal,
type

::

    rostopic echo /TwoLinkTheta
     

The move one of the sliders. You will see the message on the
TwoLinkTheta topic echoed. If you have source code you can clearly print
out the messages. It is nice to see what is actually going across. If
you don’t have source code, then this tool is very handy.

A Tk control that can set position is given in the next example
PositionCntrl.py and shown in
Figure \ `[Fig:tksliderexample] <#Fig:tksliderexample>`__\ (b). The
widget PositionCntrl.py publishes :math:`(x,y)` coordinates. An
intermediate node IK.py is used to convert the :math:`(x,y)` values to
:math:`(\theta_1, \theta_2)` and these values are published to the Two
Link Simulator.

.. raw:: latex

   \centering

|(a) The servo angle control widget and (b) the position control
widget.[Fig:tksliderexample]| |(a) The servo angle control widget and
(b) the position control widget.[Fig:tksliderexample]|

::

    # Libraries
    from math import *
    import rospy
    from std_msgs.msg import String

::

    # Call back function
    def capture(data):
        var = data.data.split(":")
        x = float(var[0])
        y = float(var[1])
        a1 = float(var[2])
        a2 = float(var[3])
        pen = int(var[4])
        inverse(x,y,a1,a2,pen)

::

    # Compute IK and send to simulator    
    def inverse(x,y,a1,a2,pen):
        if (sqrt(x*x+y*y) > a1+a2):
          print "(x,y) out of reach for links"
        else:
          d =  (x*x+y*y-a1*a1-a2*a2)/(2.0*a1*a2)
          t2 = atan2(-sqrt(1.0-d*d),d)
          t1 = atan2(y,x) - atan2(a2*sin(t2),a1+a2*cos(t2))
          dt1 = (180.0*t1/pi)
          dt2 = (180.0*t2/pi)
          print x,y, dt1, dt2
          sliders = str(dt1) + ':' + str(dt2) + ':' + str(pen)
          pub.publish(sliders)

::

    # ROS management
    pub = rospy.Publisher('TwoLinkTheta', String, queue_size=10)
    rospy.init_node('Converter', anonymous=True)
    rospy.Subscriber("TwoLinkCoords", String, capture)
    rospy.spin()

.. raw:: latex

   \centering

.. figure:: sim/rosgraph1.png
   :alt: The ROS Node Graph Tool rqt_graph. [Fig:rosgraph1]

   The ROS Node Graph Tool rqt_graph. [Fig:rosgraph1]
