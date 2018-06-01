ROS Communication
-----------------

OSRF provides tutorials on ROS, http://wiki.ros.org/ROS/Tutorials. Some
of that material is repeated here and much greater detail can be found
in the texts referenced earlier. After installing ROS, you need to setup
the environment, create and build a basic ROS package, see
:numref:`fig:ros_install`. These commands are
covered in the Beginner Level steps 1, 3, 4 (in detail). Our goal for
this section is to illustrate basic ROS communications which requires
some infrastructure. We will return to the administrative side of ROS
after some simple coding examples. Some experience with Linux and the
command line is useful here. A command line window can be brought up by
a right click and "

The terminal or command window brings up the shell or command
interpreter. For those not familiar with linux, this is like DOS. The
shell program is called bash. There are good online references for bash.
The appendix has a brief introduction.

::

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main"  > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116
    sudo apt-get update
    sudo apt-get install ros-kinetic-desktop-full
    sudo rosdep init
    rosdep update
    echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    sudo apt-get install python-rosinstall

As mentioned above the basic form of ROS communication is the
:index:`Publish-Subscribe` mechanism. To see this in action, you need to do three
things: (1) get ROS running, (2) run a subscriber, (3) run a publisher.
Step (1) is easy, bring up a terminal window [#f2]_ and type:

::

    roscore

Simple Publisher-Subscriber Example
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Our first example is going to send a single text message from one
program to another. Bring up two more terminal windows and type python
in each:

::

    jmcgough@ubuntu:~$ python
    Python 2.7.12 (default, Jul  1 2016, 15:12:24)
    [GCC 5.4.0 20160609] on linux2
    Type "help", "copyright", "credits" or "license" for more information.
    >>>

In one window type:

::

    >>> import rospy
    >>> from std_msgs.msg import String
    >>> rospy.init_node('talker', anonymous=True)
    >>> pub = rospy.Publisher('chatter', String, queue_size=10)

The first step imports ROS. The second step brings in the ROS message
type *String*. Next we establish ourself as a publisher. We are
publishing on the topic named ’chatter’ and the data will be the ROS
standard message type ’String’. The last (fourth) line names your node
’talker’ and sets up the communication with the ROS master. So, your
python shell is now a ROS node that can publish on the established
topic.

In the second window, type:

::

    >>> import rospy
    >>> from std_msgs.msg import String
    >>> def callback(data):
    ...    print data.data
    ...
    >>> rospy.init_node('listener', anonymous=True)
    >>> rospy.Subscriber("chatter", String, callback)

First two steps are the same as above. The third line defines the
callback function. This function is called when a message is published
on the topic that our node has subscribed to. Following the callback
function, we initialize the node with node name listener and the last
line has our node subscribe to the “chatter” topic and lists the
callback function.


.. _`Fig:simplePubSub`:
.. figure:: ROSFigures/pubsub1.*
   :width: 40%
   :align: center

   Simple PubSub example

Now the fun step. In the first python window (the one that has the
Publisher line), type:

::

    >>> pub.publish("This is a message")

You should see on the Subscriber window:

::

    >>> This is a message

You have successfully sent a message from one process (program) to
another. There is a similarity between writing to a topic and writing to
a file. The line

::

    pub = rospy.Publisher('chatter', String, queue_size=10)

is similar to opening a file named chatter and returning the file
descriptor pub. The full power of Python is available; a simple
extension can produce multiple messages. He is a sample of a loop
containing a publish.

::

    >>> for i in range(5):
    ...   message = "Message number " + str(i)
    ...   pub.publish(message)
    ...
    >>>

This results with the text in the other window:

::

    Message number 0
    Message number 1
    Message number 2
    Message number 3
    Message number 4

We can extend this example so that our talker is talking to two
listening programs. First we modify our talker to “talk” on two topics,
by adding the line:

::

    pub2 = rospy.Publisher('chatter2', String, queue_size=10)

Next we create a new program. Create a new terminal window and enter:

.. code-block:: python

    import rospy
    from std_msgs.msg import String
    def callback(data):
        print data.data

    rospy.init_node('listener2', anonymous=True)
    rospy.Subscriber("chatter2", String, callback)

.. _`Fig:simplePubSub2`:
.. figure:: ROSFigures/pubsub2.*
   :width: 40%
   :align: center

   Simple PubSub example cont.

From the “talker” python process you have the two options for
communication

::

    pub.publish("On the chatter topic")
    pub2.publish("On the chatter2 topic")

You should see the output on the two separate listener programs. One
more modification will illustrate these ideas. On the talker process,
add the following two lines

::

    from std_msgs.msg import Int16
    pub3 = rospy.Publisher('chatter3', Int16, queue_size=10)

and on one of the listeners add

::

    from std_msgs.msg import Int16
    rospy.Subscriber("chatter3", Int16, callback2)

Then on the talker type:

::

    pub3.publish(42)

.. _`Fig:simplePubSub3`:
.. figure:: ROSFigures/pubsub3.*
   :width: 40%
   :align: center

   Simple PubSub example cont.

You should see the number appear on the listener. You now have a fairly
complicated connection between three processes. We can express the data
communication in a data flow graph. The processes are the nodes in the
graph and the topics are the edges. ROS can generate this for you using:

::

    rqt_graph

.. _`fig:rosgraph`:
.. figure:: ROSFigures/rosgraph.png
   :width: 75%
   :align: center

   The graph of nodes and topics for the current ROS
   session.

:numref:`fig:rosgraph` shows the resulting graph.
ROS’s Publish/Subscribe architecture is a many-to-many communication
protocol. This means that a publisher can talk to many different
subscribers. Multiple publishers can be on a single topic. It can get
complicated and ``rqt_graph`` might not resolve it well graphically as
you see that it did not show the multiple topics between the publisher
(talker) and the subscriber (listener2).

A list of the topics currently managed by ROS can be produced using the
rostopic command.

::

    jmcgough@ubuntu:~$ rostopic list
    /chatter
    /chatter2
    /chatter3
    /rosout
    /rosout_agg

You can get information on one of the topics:

::

    jmcgough@ubuntu:~$ rostopic info /chatter
    Type: std_msgs/String

    Publishers:
     * /talker_25024_1505313174390 (http://ubuntu:36647/)

    Subscribers:
     * /listener_25288_1505313198989 (http://ubuntu:41441/)

You can even listen in on a topic using the rostopic command.

::

    jmcgough@ubuntu:~$ rostopic echo /chatter

Into the talker python window type:

::

    pub.publish("Did this echo??")

and you will see in the rostopic command window:

::

    data: Did this echo??
    ---

.. list-table:: Data Types
   :widths:  20 20 20
   :align: center

   * - 3 Bool
     - Byte
     - ByteMultiArray
   * - Char
     - ColorRGBA
     - Duration
   * - Empty
     - Float32
     - Float32MultiArray
   * - Float64
     - Float64MultiArray
     - Header
   * - Int16
     - Int16MultiArray
     - Int32
   * - Int32MultiArray
     - Int64
     - Int64MultiArray
   * - Int8
     - Int8MultiArray
     - MultiArrayDimension
   * - MultiArrayLayout
     - String
     - Time
   * - UInt16
     - UInt16MultiArray
     - UInt32
   * - UInt32MultiArray
     - UInt64
     - UInt64MultiArray
   * - UInt8
     - UInt8MultiArray
     - ...


Often we need to publish a message on a periodic basis. To do that you
need some control over delays and timing. The examples that follow will
use these functions. The first example is a simple sleep command. The
argument is a float in seconds.

::

    # sleep for 10 seconds
    rospy.sleep(10.)

The variation in using sleep is the Duration function. The first
argument is seconds and the second field is nanoseconds. Both are
integers.

::

    # sleep for duration
    d = rospy.Duration(10, 0)
    rospy.sleep(d)

One issue with placing a delay is that the other functions consume some
CPU time. It is hard to account for that and your effective publish
frequency might be off some. ROS has a solution using interrupts (best
effort to maintain correct frequency) that can publish at a prescribed
frequency. This is done by calling the rate function as shown below.

::

    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub.publish("hello")
        r.sleep()

Python ROS Programs
~~~~~~~~~~~~~~~~~~~

The Python interpreter is very handy for developing code and
experimenting with parameters. However, as the code base grows it makes
sense to move over to placing the code in a file and running it from the
bash terminal. Place the code in a file and at the top of the file enter

::

    #!/usr/bin/env python

The ``#!`` (called shebang) in the first two bytes tells the operating
system to use the python interpreter for the file. One new issue is that
the process will terminate after the last command. We did not need to
worry about this when we were running in the interpreter since it was
running an event loop (waiting for our input). So we need to have
something to keep the process going. A simple open loop has been added
to the publisher for the demonstration. On the subscriber side, we also
need a way to keep the process running. ROS provides a handy command ``rospy.spin()``
which is an infinite loop and waits for an event like a
message published on a topic.

Based on the couple of modifications above, the simple publisher and
subscriber example can be written as the following Python programs,
:numref:`lst:publishercode`, :numref:`lst:subscribercode`.

.. _`lst:publishercode`:
.. code-block:: python
   :caption: Publisher Code

    #!/usr/bin/env python
    import rospy
    from std_msgs.msg import String
    rospy.init_node('talker', anonymous=True)
    pub = rospy.Publisher('chatter', String, queue_size=10)
    n = 1
    while(n > 0):
        message = raw_input("Message:  ")
        n = len(message)
        pub.publish(message)

.. _`lst:subscribercode`:
.. code-block:: python
   :caption: Subscriber Code

    #!/usr/bin/env python
    import rospy
    from std_msgs.msg import String
    def callback(data):
        print data.data

    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("chatter", String, callback)
    rospy.spin()

.. _`Fig:simplePubSubProg`:
.. figure:: ROSFigures/pubsubprog.*
   :width: 40%
   :align: center

   Simple PubSub Program example

Don’t forget to make the two files executable by

::

    chmod +x <filename>

Publisher - Subscriber for the Two Link Kinematics
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Assume that you want to control a two link manipulator using ROS. To do
this you will need to describe the path you want to travel in the
workspace. So, the first step is to produce the workspace domain points.
The you want to ship those points to the inverse kinematics to find the
corresponding angles that set the manipulator end effector in the
workspace points you desire.

For this example, we are going to create the workspace data, and then
publish it with the first node. The next node will subscribe and convert
:math:`(x,y)` data to angle data. That node will then publish to a node
that will run the forward kinematics to check the answer. To make this
look like a stream of points, a delay is placed

The node that creates the workspace points is given in
:numref:`lst:workspacepathcode`. We
illustrate with the curve :math:`x(t) = 5\cos(t)+8`,
:math:`y(t) = 3\sin(t)+10`. The interval :math:`[-\pi , \pi]` is
discretized into intervals of :math:`0.1`. The :math:`(x,y)` points are
published on the topic named /WorkspacePath.

.. _`lst:workspacepathcode`:
.. code-block:: python
   :caption: Workspace Points


    #!/usr/bin/env python
    import rospy
    from std_msgs.msg import Float32
    from std_msgs.msg import Int8
    import numpy as np
    import math
    rospy.init_node('Workspace', anonymous=True)
    pub = rospy.Publisher('WorkspacePath', Float32, queue_size=10)
    flag = rospy.Publisher('Control', Int8, queue_size=10)

    def createdata():
        #Setup Arrays
        step = 0.1
        t = np.arange(-math.pi, math.pi+step, step)
        x = 5.0*np.cos(t) + 8.0
        y = 3.0*np.sin(t) + 10.0
        foo = raw_input("Hit enter to publish")
        #publish data
        for i  in range(t.size):
            pub.publish(x[i])
            pub.publish(y[i])
            rospy.sleep(0.25)

        flag.publish(127)
        rospy.sleep(3)


    if __name__ == '__main__':
        createdata()

The next stage of the process is to convert the points from the
workspace to the configuration space using the inverse kinematic
equations. The program performs the inverse kinematics and then
publishes the results on the topic /ConfigspacePath. The code is given
in :numref:`lst:inversekinematicscode`.

.. _`lst:inversekinematicscode`:
.. code-block:: python
   :caption: Inverse Kinematics Code

    #!/usr/bin/env python
    import rospy
    from std_msgs.msg import Float32
    import math

    def callback(data):
        global i, x, y
        if (i%2 == 0):
            x = data.data
        else:
             y = data.data
             convert(x,y)
        i = i+1

    def convert(x,y):
        global pub, a1, a2
        d = (x*x + y*y - a1*a1 - a2*a2)/(2*a1*a2)
        t2 = math.atan2(-math.sqrt(1.0-d*d),d)
        t1 = math.atan2(y,x) - math.atan2(a2*math.sin(t2),a1+a2*math.cos(t2))
        # print (t1, t2)
        pub.publish(t1)
        pub.publish(t2)

    def processdata():
        global i, x, y, a1, a2, pub
        rospy.init_node('InverseK', anonymous=True)
        rospy.Subscriber("WorkspacePath", Float32, callback)
        pub = rospy.Publisher('ConfigspacePath', Float32, queue_size=10)

        #Initialize global variables
        a1, a2 = 10.0, 10.0
        i = 0
        x, y = 0.0, 0.0
        rospy.spin()

    if __name__ == '__main__':
        processdata()

Finally we would like to check our answer. The angle values from the
last node are evaluated by the forward kinematics producing
:math:`(\tilde{x},\tilde{y})` values. These values are compared to the
original :math:`(x,y)` values. The two sets of values should agree
closely. The code for the verification is given in
:numref:`lst:checkinversekinematics`.

.. _`lst:checkinversekinematics`:
.. code-block:: python
   :caption: Inverse Kinematics Verification

    #!/usr/bin/env python
    import rospy
    import numpy as np
    import pylab as plt
    from std_msgs.msg import Float32
    from std_msgs.msg import Int8
    import math

    def callback(data):
        global i, t1, t2
        if (i%2 == 0):
            t1 = data.data
        else:
            t2 = data.data
            convert(t1,t2)
        i = i+1

    def cbctrl(data):
        global flag, u, v
        flag = data.data
        if (flag == 127):
            plt.xlim(0,15)
            plt.ylim(0,15)
            plt.plot(u,v,'b-')
            plt.show()

    def convert(t1,t2):
        global pub, a1, a2, u, v
        x = a1*math.cos(t1) + a2*math.cos(t1+t2)
        y = a1*math.sin(t1) + a2*math.sin(t1+t2)
        u = np.append(u,x)
        v = np.append(v,y)
        # print (x, y)

    def consumedata():
        global a1, a2, flag, i, t1, t2, u, v
        rospy.init_node('ForwardK', anonymous=True)
        rospy.Subscriber("ConfigspacePath", Float32, callback)
        rospy.Subscriber("Control", Int8, cbctrl)

        #Initialize global variables
        a1, a2 = 10.0, 10.0
        flag = 0
        i = 0
        t1, t2 = 0.0, 0.0
        u = np.array([])
        v = np.array([])
        rospy.spin()

    if __name__ == '__main__':
        consumedata()


.. _`Fig:twolinkrosexample`:
.. figure:: ROSFigures/twolinkrosexample.*
   :width: 75%
   :align: center

   Two Link Manipulator ROS example.

Although many devices produce data in a sequential manner, there are
times when you have blocks of data. ROS provides a number of datatypes
in both scalar and array form as well as some specialized messages for
sending common data blocks such as position and pose updates. When it is
possible, one can often get better performance out of sending arrays.
This next example demonstrates how to send arrays. For this example we
will send a block of 32bit integers which is the datatype ``Int32MultiArray``.

::

    #!/usr/bin/env python
    import rospy
    from std_msgs.msg import Int32MultiArray
    rospy.init_node('talker', anonymous=True)
    pub = rospy.Publisher('chatter', Int32MultiArray, queue_size=10)
    a=[1,2,3,4,5]
    myarray = Int32MultiArray(data=a)
    pub.publish(myarray)

::

    #!/usr/bin/env python
    import rospy
    from std_msgs.msg import Int32MultiArray

    def callback(data):
        print data.data
        var = data.data
        n = len(var)
        for i in range(n):
            print var[i]


    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("chatter", Int32MultiArray, callback)
    rospy.spin()


.. rubric:: Footnotes

.. [#f2] the same type you used above in the installation process
