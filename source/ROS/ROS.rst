ROS - The Robot Operating System [ROSChapter]
=============================================

| The official ROS website defines ROS as follows:
| *ROS (Robot Operating System) provides libraries and tools to help
  software developers create robot applications. It provides hardware
  abstraction, device drivers, libraries, visualizers, message-passing,
  package management, and more.*

We present a brief summary of ROS. It should be noted that it fails to
convey the sheer power and complexity of the tasks that it performs, and
the ways in which it goes about doing so. The reader is strongly
encouraged to look at some of the very good recent texts on
ROS:raw-latex:`\cite{okane:2014:GIR, quigley:2015:PRR}`.

It is slightly misleading that ROS includes the phrase “operating
system” in the title. ROS itself is not an operating system in the
traditional sense, but it is much more than just a piece of software.
The many components combine to form an ecosystem of software which
warrants its title but is best thought of as middleware. While on the
“not” topic, ROS is not a programming language, not an IDE (integrated
development environment) and not just a library of robotics codes.

As mentioned, package management and hardware abstraction are just a
couple of features under the ROS umbrella, which support the
communication framework around which ROS is based. The intent is to
create a universal system which promotes code reuse and sharing among
multiple robotic platforms, operating systems, and applications as well
as small program footprints and efficient scaling. These pillars form
the core goals of ROS as a whole.

Origins
-------

Player
~~~~~~

The beginnings of ROS date back to the Player project, which was founded
in 2000 by Brian Gerkey. This model included a hardware-abstracted
robotic system known as the player, which interfaced with its simulated
environment, known as the stage.

.. raw:: latex

   \hspace*{1mm}

|image|

Switchyard
~~~~~~~~~~

The common API used by player was a major part of the next step on the
road to ROS, the Stanford project known as “Switchyard.” Switchyard was
developed by Morgan Quigley in 2007 under the Stanford Artificial
Intelligence Robot (STAIR) project. Development of the system was
shifted to a Stanford robotics start-up known as Willow Garage in 2008.
The platform matured for about 2 years, and in 2010, Willow Garage
released the first version of ROS.

.. raw:: latex

   \hspace*{1mm}

|image|

OSRF
~~~~

In 2012, development of ROS began to shift from Willow Garage to the
newly formed OSRF also oversees development of the Gazebo robot
simulator, as well as the annual ROSCon, where ROS developers meet and
discuss various ROS-related topics. Development using ROS still
continues at Willow Garage, but the framework as a whole is developed at
OSRF.

.. raw:: latex

   \hspace*{1mm}

|image|

ROS Installation and Setup
--------------------------

ROS ’s native habitat is Ubuntu. Although there are current efforts to
port ROS to Windows or OSX, we assume (and strongly advise) that you are
using Ubuntu. [1]_ There are several ways to approach getting ROS on
your system. A standalone Linux system is the easiest. The author has
had good success with a virtual machine (Parallels on OSX). Whatever you
select, the next step is to install ROS. For this chapter we assume you
are running the Kinetic Kame version of ROS. Kinetic Kame is available
for Ubuntu Wily (15.10) and Ubuntu Xenial (16.04 LTS).

Installation instructions can be found at
`ros.org <http://wiki.ros.org/kinetic/Installation/Ubuntu>`__. Please do
this now if not already completed. We will review the instructions here.
The final authority on ROS installation is `OSRF <ros.org>`__. The
instructions below can and will become out of date. They are included
here for completeness.

Install ROS Kinetic:
~~~~~~~~~~~~~~~~~~~~

[Enter the following commands in a terminal window]

Setup your computer to accept software from packages.ros.org
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

::

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

Set up your keys
^^^^^^^^^^^^^^^^

Before you can install ROS packages, you must get and add their package
authentication key.

::

    sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

Update all of your repositories (not just those for ROS).

::

    sudo apt-get update

Install ROS (Desktop Full version)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

For this text, we strongly suggest the Full Desktop version.

::

    sudo apt-get install ros-kinetic-desktop-full

User Account
^^^^^^^^^^^^

Before use, you need to setup the user account. Note that the first
command requires root and the second is done as the user.

::

    sudo rosdep init
    rosdep update 

Set up your environment
^^^^^^^^^^^^^^^^^^^^^^^

ROS uses the shell environment in order to run properly on a system. The
shell is an interface for a user to access operating system services and
programs. In order to start ROS programs in the terminal, the user needs
to source the setup.bash file to add ROS to the environment. The
environment variables allow the OS to find the executable programs and
for ROS programs to find other ROS progams installed on the system.

::

    echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
    source ~/.bashrc

Create a Catkin Workspace:
~~~~~~~~~~~~~~~~~~~~~~~~~~

Catkin is the build system that ROS uses. It is an extension of CMake
and some Python code. Cmake and Python allow for portability to any
system that supports them. A catkin workspace is a folder where the
programmer will modify, build, and install catkin packages. You will do
all of your programming for ROS inside of this workspace.

Create a new directory for the workspace:
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

::

    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws
    catkin_make
    echo $ROS_PACKAGE_PATH

| This should build the workspace and then echo out
| ``/home/youruser/catkin_ws/src:/opt/ros/kinetic/share``.

STDR Simulator Installation
~~~~~~~~~~~~~~~~~~~~~~~~~~~

Everything is up in the Bitbucket repo it should be public to view.
https://bitbucket.org/stdr_simulator/stdr_simulator So here are the
instructions.

Install git (if not already installed):
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

::

    sudo apt-get install git

.. _create-a-catkin-workspace-1:

Create a catkin workspace
^^^^^^^^^^^^^^^^^^^^^^^^^

::

    cd <your_catkin_ws>/src
    git clone https://bitbucket.org/stdr_simulator/stdr_simulator.git
    cd ..
    rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO
    catkin_make
    source devel/setup.bash

Test Simulator Install
^^^^^^^^^^^^^^^^^^^^^^

| Now everything should be installed that is required for the launch
  files as well. For the python example, see the code in
| ``stdr_simulator/stdr_samples/scripts.speedcntl.py``

#. In one terminal

   ::

       roslaunch stdr_launchers server_with_map_and_gui_plus_robot.launch

#. Then in another terminal

   ::

       python stdr_simulator/stdr_samples/scripts/speedcntl.py

The control GUI should pop up. The left slider is the forward velocity
and the right slider is robot orientation. Try driving the robot through
the map for a short distance. If this works, you have succeeded in
showing that ROS and the STDR Simulator are working. The inner workings
will be described in detail later. For those who want to jump in, the
``sim_with_joy`` launch file has some comments in it which are useful in
getting up to speed.

Joystick Control Example
^^^^^^^^^^^^^^^^^^^^^^^^

For a joystick controller, here is an example: (ps3 controller)

::

    sudo chmod a+rw /dev/input/js0
    roslaunch stdr_launchers sim_with_joy

Note the select button has to be held and left joystick controls.

Fundamental ROS Entities
------------------------

Most of the concepts in ROS are based around a set of fundamental
entities, which will be discussed in this section. Understanding ROS,
the challenges which ROS attempts to overcome and the challenges of
using ROS are not possible without a firm understanding of these
materials.

Package
~~~~~~~

Pieces of software are, as the name suggests, known as packages in ROS.
Each package carries out a single or group of tightly related functions.
Packages need not include code at all; some packages are simply
metapackages for the purpose of ensuring that other packages are present
on the system, while other packages include startup routines for robots
or 3-D physical definitions which are used to render robots in
simulation.

Packages may be placed in different ROS environments, and a number of
these environments may be exposed simultaneously, allowing developers to
switch between different groups of packages with ease.

Node
~~~~

The node is quite possibly the single most important concept to
understand when discussing the use of ROS. Nodes are essentially
vertices in the computation graph that is implemented in ROS, and all of
the computation in ROS occurs in a node.

It is considered to be the best form in ROS for a single node to carry
out a single task. This helps to promote code reuse, as the node could
then be used to perform the same task as part of a completely different
system, ideally without any modification of the code.

It is quite common to see hundreds of nodes as part of a single ROS
environment, and it is also common of many of them to be active
simultaneously. For instance:

Master
~~~~~~

The ROS master provides a registration system for the nodes on a ROS
system, among other services. Think of it as the operator of a phone
network. When a node requests information, it asks the ROS master to
connect it to someone who can provide that information. The ROS master
doesn’t actually give the information to the node, it simply tells it
where it may be found. This communication happens over an XMLRPC
protocol.

A node does not typically communicate with the master once it has
finished initializing and is sending and receiving data. It does,
however, talk to the master whenever it needs a new data stream or
parameter information.

Worth noting is that while communication between the nodes and the
master is sparse, loss of communication with the master can be
devastating to a ROS system. If the master were to crash or become
otherwise unavailable, the entire ROS system would likely fail if any
master communication were to be attempted. The node which tried to
communicate with the master would fail, likely causing a domino effect
in nodes trying to request data streams that are sequentially becoming
unavailable.

In general, every ROS system must have exactly one master. There exist
methods of inter-master communication, but there is no built-in
methodology for this.

Message
~~~~~~~

Any data or information that is exchanged between nodes is known as a
message, which is defined as a combination of primitive data types or
other messages. Some messages include a common header, which includes a
sequence number, time stamp and a physical origin known as a frame ID.
For example, a “Twist” message contains 6 Float64 values; a 3-D vector
of linear velocities as well as a 3-D vector of angular velocities. This
message is widely used to describe the velocity of a body in ROS.

Any message defined in ROS is available in any of the supported language
in ROS. Once a node sends a message over ROS, the message can be
interpreted by another node even if the nodes are not written in the
same language or are running on the same operating system. On that note,
messages could be considered to be “data contracts” among nodes.

Topic
~~~~~

While a node may request a certain type of data from the master, it is
possible that multiple nodes could provide data of that type. The use of
“topics” is necessary to uniquely identify a data source to other nodes.
Therefore, when a node notifies the master of available data, it must
provide a topic name for that data. A connection between nodes is only
ever established if the nodes agree on a data type and a topic.

Topics can be thought of as being similar to a telephone number. When a
node registers its “number” with the master (a process known as
advertising), the master notes the message type that the “number”
corresponds to as well as the network address of the node that is
providing it. When another node “calls” that number (a process known as
subscribing), the master looks to see if there is a registered node
providing the requested message type, and tells the node what the
address of the other node is. The direct connection between the nodes is
then established and the data transfer begins.

It should be noted that while this seems to indicate that a topic
corresponds to a single server-client relationship, the topic system
allows for multiple subscribers as well as multiple publishers.
Therefore the relationship is generally referred to as
publisher-subscriber, or “pub-sub.”

Service
~~~~~~~

The publisher-subscriber (pub-sub) model is not always appropriate for
all types of data, and the service system exists in ROS to fill in the
gap. Services are, like pub-sub messages, exposed in ROS over topics.
The group of topic names is separate from the pub-sub topics, but the
structure remains.

Services are unique in that they are based on a call-and-response model
instead of pub-sub. A node can not only request information from another
node, but it can include a message to the other node containing
information about the request. The remote node then responds with a
single message back to the node that initiated the service call.

Services are useful in many ways, but should not be over-used. Each time
a service call is made, the node must request the address of the service
provider from the master. If service calls are made frequently, a
bottleneck could form in the computation graph at the master.

ROS Communication
-----------------

OSRF provides tutorials on ROS, http://wiki.ros.org/ROS/Tutorials. Some
of that material is repeated here and much greater detail can be found
in the texts referenced earlier. After installing ROS, you need to setup
the environment, create and build a basic ROS package, see
Figure \ `[fig:ros_install] <#fig:ros_install>`__. These commands are
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
Publish-Subscribe mechanism. To see this in action, you need to do three
things: (1) get ROS running, (2) run a subscriber, (3) run a publisher.
Step (1) is easy, bring up a terminal window [2]_ and type:

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

.. raw:: latex

   \centering

.. figure:: ROS/pubsub1
   :alt: Simple PubSub example[Fig:simplePubSub]

   Simple PubSub example[Fig:simplePubSub]

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

::

    import rospy
    from std_msgs.msg import String
    def callback(data):
        print data.data

    rospy.init_node('listener2', anonymous=True)
    rospy.Subscriber("chatter2", String, callback)

.. raw:: latex

   \centering

.. figure:: ROS/pubsub2
   :alt: Simple PubSub example cont.[Fig:simplePubSub2]

   Simple PubSub example cont.[Fig:simplePubSub2]

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

.. raw:: latex

   \centering

.. figure:: ROS/pubsub3
   :alt: Simple PubSub example cont.[Fig:simplePubSub3]

   Simple PubSub example cont.[Fig:simplePubSub3]

You should see the number appear on the listener. You now have a fairly
complicated connection between three processes. We can express the data
communication in a data flow graph. The processes are the nodes in the
graph and the topics are the edges. ROS can generate this for you using:

::

    rqt_graph

.. raw:: latex

   \centering

.. figure:: ROS/rosgraph.png
   :alt: [fig:rosgraph] The graph of nodes and topics for the current
   ROS session.

   [fig:rosgraph] The graph of nodes and topics for the current ROS
   session.

Figure \ `[fig:rosgraph] <#fig:rosgraph>`__ shows the resulting graph.
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

.. raw:: latex

   \vspace*{-5mm}

| 3 Bool
| Byte
| ByteMultiArray
| Char
| ColorRGBA
| Duration
| Empty
| Float32
| Float32MultiArray
| Float64
| Float64MultiArray
| Header
| Int16
| Int16MultiArray
| Int32
| Int32MultiArray
| Int64
| Int64MultiArray
| Int8
| Int8MultiArray
| MultiArrayDimension
| MultiArrayLayout
| String
| Time
| UInt16
| UInt16MultiArray
| UInt32
| UInt32MultiArray
| UInt64
| UInt64MultiArray
| UInt8
| UInt8MultiArray

.. raw:: latex

   \vspace*{-2mm}

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
need a way to keep the process running. ROS provides a handy command
``rospy.spin()`` which is an infinite loop and waits for an event like a
message published on a topic.

Based on the couple of modifications above, the simple publisher and
subscriber example can be written as the following Python programs,
Listings \ `[lst:publishercode] <#lst:publishercode>`__, \ `[lst:subscribercode] <#lst:subscribercode>`__.

::

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

::

    #!/usr/bin/env python
    import rospy
    from std_msgs.msg import String
    def callback(data):
        print data.data

    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("chatter", String, callback)
    rospy.spin()

.. raw:: latex

   \centering

.. figure:: ROS/pubsubprog
   :alt: Simple PubSub Progam example[Fig:simplePubSubProg]

   Simple PubSub Progam example[Fig:simplePubSubProg]

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
Listing \ `[lst:workspacepathcode] <#lst:workspacepathcode>`__. We
illustrate with the curve :math:`x(t) = 5\cos(t)+8`,
:math:`y(t) = 3\sin(t)+10`. The interval :math:`[-\pi , \pi]` is
discretized into intervals of :math:`0.1`. The :math:`(x,y)` points are
published on the topic named /WorkspacePath.

::

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
in
Listing \ `[lst:inversekinematicscode] <#lst:inversekinematicscode>`__.

::

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
Listing \ `[lst:checkinversekinematics] <#lst:checkinversekinematics>`__.

::

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

.. raw:: latex

   \centering

.. figure:: ROS/twolinkrosexample
   :alt: Two Link Manipulator ROS example. [Fig:twolinkrosexample]

   Two Link Manipulator ROS example. [Fig:twolinkrosexample]

Although many devices produce data in a sequential manner, there are
times when you have blocks of data. ROS provides a number of datatypes
in both scalar and array form as well as some specialized messages for
sending common data blocks such as position and pose updates. When it is
possible, one can often get better performance out of sending arrays.
This next example demonstrates how to send arrays. For this example we
will send a block of 32bit integers which is the datatype
``Int32MultiArray``.

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

.. raw:: latex

   \FloatBarrier

Problems
--------

.. raw:: latex

   \setcounter{Exc}{0}

]

Using ROS and Python, write a chat program (call it *chat.py*). First
prompt the user for their name. Write to all members in the chat group
that this person has entered the chat. In a loop, grab user inputs and
broadcast to the chat with format: name: <user input> . Echo to the
terminal all strings sent to the chat.

A single python program will do what is required:

::

    import rospy
    from std_msgs.msg import String

    def callback(data):
        print data.data

    input_flag = 1
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.Subscriber("chatter", String, callback)
    rospy.init_node('chat', anonymous=True)

    person = str(raw_input('Enter your username: '))
    msg = person + " has entered the chat"
    pub.publish(msg)

    while(input_flag == 1):
      stuff = str(raw_input('> '))
      if(stuff != "q"):
        msg = person + ":  " + stuff
        pub.publish(msg)
      else:
        input_flag = 0

Using ROS and Python, modify the example programs in the text on the
kinematics of the two link manipulator.

#. Write a program that creates a list of 100 equally spaced points
   along the path :math:`y = 15 -  x` for :math:`0 \leq x \leq 10` and
   publishes those points on the topic /physData using a multiarray
   floating data type, i.e. values x and y are floats. Publish the data
   at 5Hz.

#. Write a program that subscribes to topic /physData, plugs the values
   in, computes the serial two link inverse kinematics to gain the servo
   angles (pick one of the +/-) and publishes the angles to the topic
   /thetaData. You may assume the link arms are :math:`a_1=a_2 = 10`.
   Format will be the same as the previous topic.

#. Write a program that subscribes to both /physData and /thetaData. The
   program should plug the angles into the forward kinematics and check
   against the data in /physData. It should plot the original curve in
   green and the “check” in blue.

Assume that you have a parallel two link manipulator with
:math:`L_0 = 10`\ cm, :math:`L_1 = 15`\ cm and :math:`L_2 = 20`\ cm.

#. Write a ROS program that creates a list of 100 equally spaced points
   along the path :math:`x = 7\cos(t)+10`, :math:`y = 5\sin(t) + 15` and
   publishes those points on the topic /physData using a multiarray
   floating data type, i.e. values x and y are floats. Publish the data
   at 5Hz.

#. Write a ROS program that subscribes to topic /physData, plugs the
   values in, computes the serial two link inverse kinematics to gain
   the servo angles and publishes the angles to the topic /thetaData.
   Format will be the same as the previous topic.

#. Write a ROS program that subscribes to both /physData and /thetaData.
   The program should plug the angles into the forward kinematics and
   check against the data in /physData. It should plot the original
   curve in green and the “check” in blue.

Using ROS and python write a program that will add padding to obstacles
while shrinking the footprint of the robot to a point. Assume that you
have a circular robot with radius 10 and starting pose (15,15,90).

#. Write a program that will publish the pose of the robot on the topic
   ``/robot/pose`` and the footprint type of the robot on
   ``/robot/footprint`` as a string (For example circle or polygon).
   Also publish the radius of the robot on ``/robot/radius`` as a uint16
   message type.

#. Write a program that will publish a list of obstacles as polygons on
   the topic ``/obstacles``. For this program let the obstacles be the
   following:

   #. Rectangle with the vertices (40,30), (50,5), (50, 30) (40,30).

   #. Rectangle with the vertices (40,5), (50,5), (50,0), (40,5).

#. Write a program that subscribes to ``/robot/pose``,
   ``/robot/footprint``, and ``/obstacles``. Based on the footprint
   string, this program should be able to subscribe to either the robot
   radius or dimension topics for circular and rectangular robots. This
   program will reduce the robot footprint to a point, add padding to
   the obstacles, and plot the robot as a point and padded obstacles
   with the maximum x and y values being 70 and 30.

Rework the previous problem assuming that you have a rectangular robot
with :math:`width=10` and :math:`length=20` and initial pose (0,10,0).

Plot the padding of obstacles using the ros nodes in the previous
problem with the initial pose of the robot being (a) (5,10,30), (b)
(5,10,70), and (c) (5,25,-90).

Write a program that will publish the changing poses of a rectangular
robot over time from to in increments of . Assume the inital pose is
(5,10, -90) and :math:`width=10` and :math:`length=20`. Use the programs
from problem 6 to publish the obstacles and display the padding.

.. raw:: latex

   \Closesolutionfile{Answer}

.. [1]
   And we mean Ubuntu not just Linux. Much but not all of the ROS
   packages are ported to the other Linux distros.

.. [2]
   the same type you used above in the installation process

.. |image| image:: ROS/player_button_v3.png
.. |image| image:: ROS/willow_garage.jpg
.. |image| image:: ROS/osrf_masthead.png

