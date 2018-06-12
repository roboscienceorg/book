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


As mentioned above the basic form of ROS communication is the
:index:`Publish-Subscribe` mechanism. To see this in action, you need to do two
things: (1) run a subscriber, (2) run a publisher.   The "pubsub"
communication will be shown in Python.  ROS2 is only available for
Python3, so if you don't have Python3, please load it now.

Simple Publisher-Subscriber Example
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Our first example is going to send a single text message from one
program to another. This material is adapted from the basic examples
on the ROS2 Github site.  Bring up two more terminal windows and type python
in each:

::

   alta:~ jmcgough$ python3
   Python 3.6.5 (default, Apr 25 2018, 14:26:36)
   [GCC 4.2.1 Compatible Apple LLVM 9.0.0 (clang-900.0.39.2)] on darwin
   Type "help", "copyright", "credits" or "license" for more information.
   >>>

In one window type:

::

   >>> import rclpy
   >>> from std_msgs.msg import String
   >>> rclpy.init(args=None)
   >>> pnode = rclpy.create_node('minimal_publisher')
   >>> publisher = pnode.create_publisher(String, 'topic')

The first step imports ROS library. The second step brings in the ROS message
type *String*.   Next we start up ROS2.  The fourth line creates a ROS node
and names it.   The fifth line establishes ourself as a publisher and
sets the topic name and datatype.
In this example, the topic name is `topic` and the topic datatype is
the ROS standard message
type `String`. So, your
python shell is now a ROS node that can publish on the established
topic.

In the second window, type:

::

   >>> import rclpy
   >>> from std_msgs.msg import String
   >>> rclpy.init(args=None)
   >>> node = rclpy.create_node('min_sub')
   >>> def chatter_callback(msg):
   ...   global node
   ...   node.get_logger().info('This is what I heard: "%s"' % msg.data)
   ...
   >>> subscription = node.create_subscription(String, 'topic', chatter_callback)
   >>> while rclpy.ok():
   ...   rclpy.spin_once(node)


The first four steps are the same as above.  The fifth line defines the
callback function. This function is called when a message is published
on the topic that our node has subscribed to. Following the callback
function,  we subscribe to the topic and define the
callback function to handle the message that has arrived.  The while
at the end places the node into an event loop to capture the message.


.. _`Fig:simplePubSub`:
.. figure:: ROSFigures/pubsub1.*
   :width: 50%
   :align: center

   Simple PubSub example

Now the fun step. In the first python window (the one that has the
Publisher line), type:

::

   >>> msg = String()
   >>> msg.data = "Hello"
   >>> publisher.publish(msg)

You should see on the Subscriber window:

::

   [INFO] [min_sub]: This is what I heard: "Hello"

You have successfully sent a message from one process (program) to
another. There is a similarity between writing to a topic and writing to
a file. The line

::

   publisher = pnode.create_publisher(String, 'topic')

is similar to opening a file named topic and returning the file
descriptor ``publisher``. The full power of Python is available; a simple
extension can produce multiple messages. He is a sample of a loop
containing a publish.

::

   >>> for i in range(5):
   ...   msg.data = "Message number " + str(i)
   ...   publisher.publish(msg)
   ...
   >>>

This results with the text in the other window:

::

   [INFO] [min_sub]: This is what I heard: "Message number 0"
   [INFO] [min_sub]: This is what I heard: "Message number 1"
   [INFO] [min_sub]: This is what I heard: "Message number 2"
   [INFO] [min_sub]: This is what I heard: "Message number 3"
   [INFO] [min_sub]: This is what I heard: "Message number 4"

We can extend this example so that our talker is talking to two
listening programs. First we modify our talker to `talk` on two topics,
by adding the line:

::

   >>> publisher2 = pnode.create_publisher(String, 'topic2')

Next we create a new program to listen to the new
optic. Create a new terminal window and enter:

.. code-block:: python

    import rclpy
    from std_msgs.msg import String
    rclpy.init(args=None)
    node = rclpy.create_node('min_sub2')
    def chatter_callback(msg):
    ...   global node
    ...   node.get_logger().info('This is what I heard: "%s"' % msg.data)
    ...

    subscription = node.create_subscription(String, 'topic2', chatter_callback)
    >>> while rclpy.ok():
    ...   rclpy.spin_once(node)
    ...


.. _`Fig:simplePubSub2`:
.. figure:: ROSFigures/pubsub2.*
   :width: 50%
   :align: center

   Simple PubSub example cont.

From the publisher python process,  setup the new topic

::

   >>> publisher2 = pnode.create_publisher(String, 'topic2')

and now you can send to the new node:

::

   >>> msg.data = "Second topic Hello"
   >>> publisher2.publish(msg)

or you can send to the old node:

::

   >>> msg.data = "First topic Hello"
   >>> publisher.publish(msg)


You should see the output on the two separate listener programs. One
more modification will illustrate these ideas.  The previous examples
got us up and running.  At this point, it is easy to make small
changes and run brief experiments in the command interpreter.

Python ROS Programs
~~~~~~~~~~~~~~~~~~~

There is a limit to how convenient it is using
the interpreter directly. The Python interpreter is very handy for developing code and
experimenting with parameters. However, as the code base grows it makes
sense to move over to placing the code in a file and running it from the
bash terminal.  For the rest of the examples, we switch to
a more traditional programming style.  This means the code is in a file
which will be executed as a script and not as individual commands.  A bit
more like what you do with C, Java or normal Python usage.

The main difference it makes at this stage is that you no longer have
the event loop which the Python command interpreter gave you.  You will need
to supply some type of event loop or have all the commands entered and timed
as needed.  We will focus on the former.
So the last example above will be modified with a small loop added and
the three programs will be listed below.  If you are reading this from an
electronic version, you can then cut and paste into your editor.  Otherwise
the code can be obtained from CODE REPO LINK HERE!!!

Place the code in a file and at the top of the file enter

::

    #!/usr/bin/env python3

The ``#!`` (called shebang) in the first two bytes tells the operating
system to use the python interpreter for the file. One new issue is that
the process will terminate after the last command. We did not need to
worry about this when we were running in the interpreter since it was
running an event loop (waiting for our input). So we need to have
something to keep the process going. A simple open loop has been added
to the publisher for the demonstration. On the subscriber side, we also
need a way to keep the process running. ROS provides some commands that allow
us to set up the event loop.  We will combine a while loop with
 ``rclpy.spin_once(node)``
which gives us an infinite loop and waits for an event like a
message published on a topic.

Based on the couple of modifications above, the simple publisher and
subscriber example can be written as the following Python programs,
:numref:`lst:publishercode`, :numref:`lst:subscribercode`.

.. _`lst:publishercode`:
.. code-block:: python
   :caption: Two topic publisher example

   #!/usr/bin/env python3
   import rclpy
   from std_msgs.msg import String

   rclpy.init(args=None)

   node = rclpy.create_node('publisher')
   pub1 = node.create_publisher(String, 'topic1')
   pub2 = node.create_publisher(String, 'topic2')
   msg = String()


   while True:
     message = input("> ")
     if message == 'exit':
        break
     msgarr = message.split(',')
     ch = int(msgarr[1])
     msg.data = msgarr[0]
     if ch == 1:
        pub1.publish(msg)
     if ch == 2:
        pub2.publish(msg)


   node.destroy_node()
   rclpy.shutdown()


.. _`lst:subscribercode`:
.. code-block:: python
   :caption: Subscriber 1

   #!/usr/bin/env python3
   import rclpy
   from std_msgs.msg import String

   def chatter_callback(msg):
      global node
      node.get_logger().info('This is what I heard: "%s"' % msg.data)

   rclpy.init(args=None)
   node = rclpy.create_node('min_sub1')
   subscription = node.create_subscription(String, 'topic1', chatter_callback)
   while rclpy.ok():
      rclpy.spin_once(node)


.. code-block:: python
   :caption: Subscriber 2

   #!/usr/bin/env python3
   import rclpy
   from std_msgs.msg import String

   def chatter_callback(msg):
      global node
      node.get_logger().info('This is what I heard: "%s"' % msg.data)

   rclpy.init(args=None)
   node = rclpy.create_node('min_sub2')
   subscription = node.create_subscription(String, 'topic2', chatter_callback)
   while rclpy.ok():
      rclpy.spin_once(node)


Cut and paste these into three different files, pub.py, sub1.py and sub2.py,
and run in three different terminals.   In pub.py one can type your message, then
comma, then the topic number (1 or 2):  `message, number` .

.. _`Fig:simplePubSubProg`:
.. figure:: ROSFigures/pubsubprog.*
   :width: 50%
   :align: center

   Simple PubSub Program example.  Computing the wheel velocties in one
   program and sending the commmands to another program to implement.


Don’t forget to make the two files executable by

::

    chmod +x <filename>


One can have multiple communication lines between nodes.  We will add
a third topic to the publisher and have sub1 subscribe to it.   The new versions
of the publisher and sub1 are given below.

.. code-block:: python
   :caption: Multi-topic publisher

   #!/usr/bin/env python3
   import rclpy
   from std_msgs.msg import String
   from std_msgs.msg import Int16

   rclpy.init(args=None)

   node = rclpy.create_node('publisher')
   pub1 = node.create_publisher(String, 'topic1')
   pub2 = node.create_publisher(String, 'topic2')
   pub3 = node.create_publisher(Int16, 'topic3')
   msg = String()
   var = Int16()

   while True:
     message = input("> ")
     if message == 'exit':
        break
     msgarr = message.split(',')
     ch = int(msgarr[1])
     msg.data = msgarr[0]
     if ch == 1:
        pub1.publish(msg)
     if ch == 2:
        pub2.publish(msg)
     if ch == 3:
        var.data = int(msgarr[0])
        pub3.publish(var)


   node.destroy_node()
   rclpy.shutdown()

and for sub1.py we modify

.. code-block:: python
   :caption: Multi-topic subscriber

   #!/usr/bin/env python3
   import rclpy
   from std_msgs.msg import String
   from std_msgs.msg import Int16

   def chatter_callback(msg):
      global node
      node.get_logger().info('This is what I heard: "%s"' % msg.data)

   def chatter_callback2(msg):
      global node
      node.get_logger().info('This is what I heard: "%s"' % msg.data)


   rclpy.init(args=None)
   node = rclpy.create_node('min_sub1')
   subscription = node.create_subscription(String, 'topic1', chatter_callback)
   subscription = node.create_subscription(Int16, 'topic3', chatter_callback2)

   while rclpy.ok():
      rclpy.spin_once(node)



Then on the publisher enter:  `42, 3` .   You should see the number 42 echoed
on the terminal running sub1.

.. _`Fig:simplePubSub3`:
.. figure:: ROSFigures/pubsub3.*
   :width: 50%
   :align: center

   Simple PubSub example cont.

To see what topics are defined, you can get a list of them:

::

   alta:Desktop jmcgough$ ros2 topic list
   /topic1
   /topic2
   /topic3

As of early 2018, the topic list command was under development.  This
tool is only accurate for nodes and topics on a single computer.  Current
development by OSRF is to make the topic list work on distributed nodes.


You can listen in on a topic using the rostopic command.

::

   alta:Desktop jmcgough$ ros2 topic echo /topic1

Into the publisher python window type:

::

    > Hello, 1

and you will see in the rostopic command window:

::

    data: Hello

.. list-table:: Data Types
   :widths:  20 20 20
   :align: center

   * - Bool
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


Often we need to publish a message on a periodic basis.  It is possible
to place a delay via python sleep in the publishing loop:

::

   while True:
     message = input("> ")
     if message == 'exit':
        break
     time.sleep(delay)

The sleep command will introduce a delay.  This approach will enforce
at least that time interval, but not exactly that time interval.  The
process shares the cpu and longer delays can arise when other processes
slow down the system.   Some robotics applications require that
the time interval is accurate within some constraint.

To increase the timing accuracy, ROS supports an interrupt based method.
This approach sets a timer which raises an interrupt.  That interrupt
causes a function to be called, known as an interrupt handler.  Sample
code is provided below (adapted from the ROS2 example programs).

.. code-block:: python

   import rclpy
   from std_msgs.msg import String
   def timer_callback():
       global i
       msg.data = 'Hello World: %d' % i
       i += 1
       node.get_logger().info('Publishing: "%s"' % msg.data)
       publisher.publish(msg)


   rclpy.init(args=None)
   node = rclpy.create_node('publisher')
   publisher = node.create_publisher(String, 'topic1')

   msg = String()
   i = 0
   timer_period = 0.5  # seconds
   timer = node.create_timer(timer_period, timer_callback)

   rclpy.spin(node)

   node.destroy_timer(timer)
   node.destroy_node()
   rclpy.shutdown()



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
:math:`y(t) = 3\sin(t)+10`. The interval is
discretized into intervals of :math:`0.1`. The :math:`(x,y)` points are
published on the topic named /WorkspacePath.

.. _`lst:workspacepathcode`:
.. code-block:: python
   :caption: Workspace Points

   #!/usr/bin/env python
   import rclpy
   from std_msgs.msg import Float32
   from std_msgs.msg import Int8
   import math

   def timer_callback():
     global t, pubx, puby
     x = 5.0*math.cos(t) + 8.0
     y = 3.0*math.sin(t) + 10.0
     xval.data = x
     yval.data = y
     node.get_logger().info('Publishing: "%f" , "%f" ' % (x,y) )
     pubx.publish(xval)
     puby.publish(yval)
     t = t+step

   rclpy.init(args=None)
   node = rclpy.create_node('Workspace')
   pubx = node.create_publisher(Float32, 'WorkspacePathX')
   puby = node.create_publisher(Float32, 'WorkspacePathY')
   step = 0.1
   t = 0.0
   xval = Float32()
   yval = Float32()

   timer_period = 0.5  # seconds
   timer = node.create_timer(timer_period, timer_callback)

   rclpy.spin(node)

   node.destroy_timer(timer)
   node.destroy_node()
   rclpy.shutdown()



The next stage of the process is to convert the points from the
workspace to the configuration space using the inverse kinematic
equations. The program performs the inverse kinematics and then
publishes the results on the topic /ConfigspacePath. The code is given
in :numref:`lst:inversekinematicscode`.

.. _`lst:inversekinematicscode`:
.. code-block:: python
   :caption: Inverse Kinematics Code

   #!/usr/bin/env python
   import rclpy
   from std_msgs.msg import Float32
   from std_msgs.msg import Int8
   import math

   def callbackX(data):
       global x, y
       x = data.data

   def callbackY(data):
       global x, y
       y = data.data
       convert(x,y)

   def convert(x,y):
       global pub, a1, a2
       d = (x*x + y*y - a1*a1 - a2*a2)/(2*a1*a2)
       t2 = math.atan2(-math.sqrt(1.0-d*d),d)
       t1 = math.atan2(y,x) - math.atan2(a2*math.sin(t2),a1+a2*math.cos(t2))
       xval.data = t1
       yval.data = t2
       node.get_logger().info('Publishing: "%f" , "%f" ' % (t1,t2) )
       pubcx.publish(xval)
       pubcy.publish(yval)


   global x, y, a1, a2, pub
   rclpy.init(args=None)
   node = rclpy.create_node('InverseK')
   subx = node.create_subscription(Float32, 'WorkspacePathX', callbackX)
   suby = node.create_subscription(Float32, 'WorkspacePathY', callbackY)
   pubcx = node.create_publisher(Float32, 'ConfigspacePathX')
   pubcy = node.create_publisher(Float32, 'ConfigspacePathY')
   xval = Float32()
   yval = Float32()


   #Initialize global variables
   a1, a2 = 10.0, 10.0
   x, y = 0.0, 0.0
   while rclpy.ok():
      rclpy.spin_once(node)


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
   import rclpy
   from std_msgs.msg import Float32
   from std_msgs.msg import Int8
   import math


   def callbackX(data):
       global t1, t2
       t1 = data.data

   def callbackY(data):
       global t1, t2
       t2 = data.data
       convert(t1,t2)


    def convert(t1,t2):
        global a1, a2
        x = a1*math.cos(t1) + a2*math.cos(t1+t2)
        y = a1*math.sin(t1) + a2*math.sin(t1+t2)
        print (x, y)

   global a1, a2
   rclpy.init(args=None)
   node = rclpy.create_node('InverseKcheck')
   subx = node.create_subscription(Float32, 'ConfigspacePathX', callbackX)
   suby = node.create_subscription(Float32, 'ConfigspacePathY', callbackY)

   #Initialize global variables
   a1, a2 = 10.0, 10.0
   t1, t2 = 0.0, 0.0

   while rclpy.ok():
      rclpy.spin_once(node)




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
   import rclpy
   from std_msgs.msg import Int32MultiArray
   rclpy.init(args=None)
   node = rclpy.create_node('Talker')
   pub = node.create_publisher(Int32MultiArray, 'Chatter')

   a=[1,2,3,4,5]
   myarray = Int32MultiArray(data=a)
   pub.publish(myarray)

::

   #!/usr/bin/env python
   import rclpy
   from std_msgs.msg import Int32MultiArray

   def callback(data):
      print(data.data)
      var = data.data
      n = len(var)
      for i in range(n):
        print(var[i])


   rclpy.init(args=None)
   node = rclpy.create_node('InverseKcheck')
   sub = node.create_subscription(Int32MultiArray, 'Chatter', callback)

   while rclpy.ok():
      rclpy.spin_once(node)
