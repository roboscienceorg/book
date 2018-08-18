Tutorial: Simulating with Veranda
---------------------------------

This section of the book will walk you through the entire process
of designing your own robot and programming it to drive and produce
sensor feedback.

Part 0: Install and Run Veranda
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Veranda can be installed from `Roboscience.org <http://www.roboscience.org/veranda/>`_. Download
and run the install script to set it up. The default install location will be ``~/veranda``.
This installer script will add the command ``veranda`` as a bash alias that will start Veranda
using ROS.

.. code:: bash

    > veranda

.. NOTE::
    The default RMW Implementation on linux is FastRTPS; however, the version of it included with ROS Ardent appears to have some issues,
    so this script will automatically switch to OpenSplice by doing ``export RMW_IMPLEMENTATION=rmw_opensplice_cpp`` before running the application.

Part 1: Build a Turtle
^^^^^^^^^^^^^^^^^^^^^^

The first step to simulating robots is having a robot to simulate. When you are greeted with the Veranda
application, select the 'Editor' button to open the editor.

.. figure:: TutorialFigures/editorbutton.*
    :figwidth: 90%
    :width: 50%
    :align: center

    The editor button

In the editor, you can place robot components together to build your very own robot! Let's start by adding
a circular body...

.. figure:: TutorialFigures/addcircle.*
    :figwidth: 90%
    :width: 50%
    :align: center

    To select the circle from the shapes tab, and press the green plus to add it

Next, we'll add a couple of wheels...

.. figure:: TutorialFigures/addwheels.*
    :figwidth: 90%
    :width: 75%
    :align: center

    Completed turtle bot

Now, you may be noticing that my robot looks much more square than yours; if you want to make sure the wheels
are exactly where you want them, you can set their position properties to the exact coordinates you want. I made the wheels
be exactly 0.6m to the left/right of the center, and 0m above it.

.. figure:: TutorialFigures/wheelproperties.*
    :figwidth: 90%
    :width: 50%
    :align: center

    With a wheel selected, you can set properties for it

Now that you have a robot built, we need to load it into the simulation. Choose the 'save' button, and save your robot as ``Turtle.json``. Don't
forget the ``.json``! It will not be added automatically if you forget it.

.. figure:: TutorialFigures/saverobot.*
    :figwidth: 90%
    :width: 50%
    :align: center

    Save your robot by pressing the save button in the editor mode

Next we have to switch back to simulator mode.

.. figure:: TutorialFigures/simulatorbutton.*
    :figwidth: 90%
    :width: 50%
    :align: center

    The simulator button

Now, we can load the robot into our toolbox on the right.

.. figure:: TutorialFigures/simulatorloadrobot.*
    :figwidth: 90%
    :width: 50%
    :align: center

    Press the load button on the simulator toolbox to load a robot file

And once your robot is in the toolbox, you can add it to the simulation and position it wherever you want!

.. figure:: TutorialFigures/simulatoraddrobot.*
    :figwidth: 90%
    :width: 75%
    :align: center

    Add robots from the toolbox by selecting them and pressing the green plus

Finally, we can start the simulation with the 'play' button on the left.

.. figure:: TutorialFigures/simulatorplaybutton.*
    :figwidth: 90%
    :width: 50%
    :align: center

    The play button to start a simulation

Congratuations! You just simulated your first robot; it sat there, and did nothing. Next, we're going to write some code to make it move.

.. TIP::
    If you don't want to go through the trouble of saving your robot in a file and then loading it again, you can use the 'quick-add' button
    on the editor to put it directly in the toolbox, but beware, if you close Veranda, the robot will be lost forever!

    .. figure:: TutorialFigures/designerquickadd.*
        :figwidth: 90%
        :width: 50%
        :align: center

        The designer quick-add button

Part 2: Drive your robot in a circle
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Now that we have a robot designed, we need to write some code to control it and then connect that code to the simulation using ROS.
First, we will pick names for the ROS topics we want to use. Select your turtle robot in the simulator, and then search through its properties
for the topic settings for the wheels. Since I left my wheels named 'Fixed Wheel', I am looking for the properties called 'Fixed Wheel1/channels/input_speed', and
'Fixed Wheel2/channels/input_speed'. In my turtle, 'Fixed Wheel1' is on the left, and 'Fixed Wheel2' is on the right, so I named the ROS topics 'robot0/left_wheel' and 'robot0/right_wheel', respectively.

.. figure:: TutorialFigures/wheelchannels.*
    :figwidth: 90%
    :width: 75%
    :align: center

    Setting the wheel control topics

We also need to indicate that the wheels can be driven. Find the properties 'Fixed Wheel1/is_driven' and Fixed Wheel2/is_driven' and set them both
to be 'true'

.. TIP::
    Having issues telling your wheels apart? They have a 'Name' property that can be changed in the editor to differentiate them better.

.. TIP::
    Don't want to have to set properties every time you start Veranda? You can set many properties in the editor and save their values
    along with the rest of the robot.

Now that the channels are set, we need to write some code to start driving the robot. To drive a differential robot in a circle, 
all we need to do is send a different speed command to each wheel; then they will drive that speed forever.

First, we need our python to import the ``rclpy`` module, and the Node type from that module

.. code:: python

    import rclpy
    from rclpy.node import Node

Next, we need to import the message type that should be used to communicate to the wheels.

.. code:: python

    from std_msgs.msg import Float32

Now, we can initialize ROS and create a Node to publish from

.. code:: python

    rclpy.init()
    node = Node("circle")

Once the node is created, we can create two publishers; one for each of the wheel topics

.. code:: python

    publeft = node.create_publisher(Float32, 'robot0/left_wheel')
    pubright = node.create_publisher(Float32, 'robot0/right_wheel')

Finally we can send a command to each of the wheels. Let's create a Float32 message, and send it with different values to each wheel.

.. code:: python

    msg = Float32()

    msg.data = 5.0
    publeft.publish(msg)

    msg.data = 10.0
    pubright.publish(msg)

.. NOTE::
    This will command the wheels to drive 5 radians/second and 10 radians/second respectively.

However, if we run the code right now, the messages will not be sent; they have only been queued for publishing.
To send them out of the application, we need to 'spin' the ROS node. Once we spin it, ROS will enter an infinite loop
which sends queued messages and receives incoming ones.

.. code:: python

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

And there we have it! One python program to start driving a robot in a circle. Let's call it 'circle.py'

.. code:: python

    import rclpy
    from rclpy.node import Node

    from std_msgs.msg import Float32

    rclpy.init()
    node = Node("circle")

    publeft = node.create_publisher(Float32, 'robot0/left_wheel')
    pubright = node.create_publisher(Float32, 'robot0/right_wheel')

    msg = Float32()

    msg.data = 5.0
    publeft.publish(msg)

    msg.data = 10.0
    pubright.publish(msg)

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

Now, all that's left is to run it. First, we need to start the simulation in Veranda because messages are not published or received while the simulation is stopped.
Once the simulation is running, we can run our script to send a command to the wheels to start driving. This is a three-command 
step, because we need to set up the ROS environment first.

.. code:: bash

    > source ~/veranda/install/setup.bash
    > export RMW_IMPLEMENTATION=rmw_opensplice_cpp
    > python3 circle.py

If all has gone well, the robot in your simulation will now be driving in a circle! Your code will be in an infinite loop waiting
to send and receive messages, you can stop it with ``Ctrl-C``

.. TIP::

    You don't need to do ``source ~/veranda/install/setup.bash`` and ``export RMW_IMPLEMENTATION=rmw_opensplice_cpp`` every time you run your code, just the first time. After you have
    sourced the environment for a specific terminal, those environment variables will stay set up!

.. IMPORTANT::

    Your robot might look a little goofy driving this circle. That's because of the way the simulation handles relative mass; the 
    body of the robot is much larger than the wheels, so the wheels have a difficult time moving it. Both wheels have a `density` property
    that you can use to give them more oomph; I've found that setting the density of the wheels in this demo robot to 5 works well. When
    you are building your own robot, this is something you will have to adjust so that it drives correctly.

.. TIP::

    Want to reset the simulation? Instead of removing the robot and putting it in again, you can use the quicksave before starting the simulation
    and quickload to reset to the saved version.

    .. figure:: TutorialFigures/quicksaveload.*
        :figwidth: 90%
        :width: 25%
        :align: center

        Quicksave (left) and Quickload (right)

Part 3: Drive a more complex path
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Driving in a circle is easy, but what if we want to make the robot drive along some path that requires changing
wheel speeds? Lets make it drive a wiggle; first driving one wheel, then the other.

Once we call ``rclpy.spin()``, our program goes into a loop, so how do we send more commands? We use Timers with callbacks. A Timer in ROS
can be created to call a specific function every X seconds.

This is done with the function ``node.create_timer(seconds, callback)``. The call returns a Timer Handle, which can be used
later to cancel the timer with ``node.destroy_timer(handle)``.

So, let's set up some functions to drive a wiggle, they will both work the same way, but one will drive the left wheel,
and the other will drive the right.

After we have created our ``publeft`` and ``pubright`` publishers, we'll define our function

.. code:: python

    def wiggle_left():
        msg = Float32()

        msg.data = 5.0
        publeft.publish(msg)

        msg.data = 0.0
        pubright.publish(msg)

This will stop the right wheel, and start the left wheel. Once we do that, we need to start a timer. When the timer ends,
we should call ``wiggle_right`` to stop the left wheel and start the right one.

.. code:: python

    def wiggle_left():
        msg = Float32()

        msg.data = 5.0
        publeft.publish(msg)

        msg.data = 0.0
        pubright.publish(msg)

        node.create_timer(1, wiggle_right)

This will have a 1 second gap between commands. But wait! Timers in ROS go for forever, so if we do this, we'll end up with
a bunch of timers starting and stopping the wheels, so we need to save the timer handle, and be able to destroy the timer after it
goes off.

.. code:: python

    def wiggle_left():
        global timer_handle
        node.destroy_timer(timer_handle)

        msg = Float32()

        msg.data = 5.0
        publeft.publish(msg)

        msg.data = 0.0
        pubright.publish(msg)

        timer_handle = node.create_timer(1, wiggle_right)

If we do the same thing in the ``wiggle_right`` function, then they can share the timer handle and pass it between themselves.
Finally, we need to start the first timer before we spin the node.

.. code:: python

    timer_handle = node.create_timer(0.1, wiggle_left)
    rclpy.spin(node)

And there we have it! Now ``wiggle.py`` will drive the wheels alternately. Go ahead and run it to see what it looks like.

.. code:: python

    import rclpy
    from rclpy.node import Node

    from std_msgs.msg import Float32

    rclpy.init()
    node = Node("wiggle")

    publeft = node.create_publisher(Float32, 'robot0/left_wheel')
    pubright = node.create_publisher(Float32, 'robot0/right_wheel')

    def wiggle_left():
        global timer_handle
        node.destroy_timer(timer_handle)

        msg = Float32()

        msg.data = 5.0
        publeft.publish(msg)

        msg.data = 0.0
        pubright.publish(msg)

        timer_handle = node.create_timer(1, wiggle_right)

    def wiggle_right():
        global timer_handle
        node.destroy_timer(timer_handle)

        msg = Float32()

        msg.data = 0.0
        publeft.publish(msg)

        msg.data = 5.0
        pubright.publish(msg)

        timer_handle = node.create_timer(1, wiggle_left)

    timer_handle = node.create_timer(0.1, wiggle_left)
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

Part 4: Hooking into the Simulation Clock
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Now that you have a couple of scripts running, let's take a look at what happens when we use the time-warp capabilities of Veranda.
Click the time-warp button while your ``wiggle.py`` is driving a robot.

.. figure:: TutorialFigures/timewarpbutton.*
    :figwidth: 90%
    :width: 50%
    :align: center

    The time warp button; press it multiple times to cycle through 2x, 3x, and 0.5x speeds

That probably didn't do what you expected, did it? The issue here is that, in the simulation, time started moving faster, but the clock
in your control script didn't! So for every 1 second of wiggling that the control code thought it was doing, the simulator was actually
driving the robot for more than 1 second.

This can be accounted for by using the Veranda SimTimer. The SimTimer listens to the clock message coming from Veranda, and 
uses those to determine how much time has passed, instead of the sytem clock.

First, we need to include the SimTimer module

.. code:: python

    from veranda.SimTimer import SimTimer

Next, after we create our ROS node, we create a timer object which uses that node.

.. code:: python
    
    simTime = SimTimer(True, "veranda/timestamp", node)

.. NOTE::

    The parameters for the SimTimer are
        * Boolean - Should it use the Simulation Timer? If False, the regular system clock is used
        * String - ROS Topic that the timestamp is published to. This is currently always the same
        * Node - The ROS Node that should be used to listen for time messages

Now, everywhere that we have ``node.create_timer`` and ``node.destroy_timer``, we can replace with ``simTime.create_timer`` and ``simTime.destroy_timer``.
It's that easy! Go ahead and run your new wiggle code, and test out how it works with the time-warp feature.

.. IMPORTANT::

    While the create and destroy functions behave similarly, the SimTimer does not return the same dataType as the ROS Node. If the SimTimer
    is using the timestamp message, it will return integer values as the timer handles, but if it is using the regular ROS timer functionality, (Param 1 is False),
    it will return the Timer type that ``Node.create_timer()`` yields.

.. code:: python

    import rclpy
    from rclpy.node import Node

    from std_msgs.msg import Float32

    from veranda.SimTimer import SimTimer

    rclpy.init()
    node = Node("wiggle")

    simTime = SimTimer(True, "veranda/timestamp", node)

    publeft = node.create_publisher(Float32, 'robot0/left_wheel')
    pubright = node.create_publisher(Float32, 'robot0/right_wheel')

    def wiggle_left():
        global timer_handle
        simTime.destroy_timer(timer_handle)

        msg = Float32()

        msg.data = 5.0
        publeft.publish(msg)

        msg.data = 0.0
        pubright.publish(msg)

        timer_handle = simTime.create_timer(1, wiggle_right)

    def wiggle_right():
        global timer_handle
        simTime.destroy_timer(timer_handle)

        msg = Float32()

        msg.data = 0.0
        publeft.publish(msg)

        msg.data = 5.0
        pubright.publish(msg)

        timer_handle = simTime.create_timer(1, wiggle_left)

    timer_handle = simTime.create_timer(0.1, wiggle_left)
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

Part 5: Making obstacles
^^^^^^^^^^^^^^^^^^^^^^^^

Having a robot that can drive around is fun, but eventually, you may want to try to write code to make the robot avoid things it might run into.
The first step to doing that is having things for the robot to hit. Veranda can load image files as a simulation, and turn them into obstacles that robots
can hit. To do this, choose the 'load simulation' button, and find your image file. My image is just a black square; we're going to put the robot
in a box.

.. figure:: TutorialFigures/loadimage.*
    :figwidth: 90%
    :width: 75%
    :align: center

    Use the 'Load Simulation' button to to load images as obstacles

.. TIP::

    Loading images in Veranda works best if they contain only black and white pixels, with no other colors (including grey).
    If you do try to load other images, you can play with the black/white threshold to get it to turn out better.


.. IMPORTANT::

    Veranda can load a number of different files as full simulations, make sure you pick the correct file type in the file-choose dialog so that you are able to select the file you want.

Once you choose an image, you will be presented with some import options. The most important will be the size options, followed
by the threshold options. Veranda will report the size of the image, in pixels, and you will have the option to set the pixel/m ratio, or
and the image size (in meters). Our little roomba has a radius of 2m, so if we make the 5m x 5m, then the robot will have very little room to move about.

.. figure:: TutorialFigures/importoptions.*
    :figwidth: 90%
    :width: 50%
    :align: center

    The image importing options

Part 6: Getting feedback
^^^^^^^^^^^^^^^^^^^^^^^^

Finally, we want to know when the robot runs into the wall. To do this, we'll add a touch sensor to the robot; it will send messages
to the control script whenever it touches something.

In the editor, add a Touch Ring to your turtle bot. If you kept your robot at the default size, you will not be able to see any difference,
because the touch ring is also a circle, and it defaults to 1m radius.

.. figure:: TutorialFigures/touchring.*
    :figwidth: 90%
    :width: 50%
    :align: center

    Touch Ring is found under the sensors tab of the editor toolbox

The touch ring represents a ring of bump sensors evenly spaced around the robot; by default, the 'angle_start' and 'angle_end' properties, which
specify which part of the robot has the sensors, encompass the entire chassis. Let's make
there be 20 buttons them by setting the property 'sensor_count' to 20. Don't forget to set the ROS topic property 'channels/output_touches' to 'robot0/touches'.

.. figure:: TutorialFigures/touchringproperties.*
    :figwidth: 90%
    :width: 50%
    :align: center

    The properties we want to set for the touch ring

.. TIP::

    Don't have your robot loaded in the editor anymore? You can load it into the editor from file!

    .. figure:: TutorialFigures/editorloadbutton.*
        :figwidth: 90%
        :width: 50%
        :align: center

        The load button in the editor

Now, when your robot runs into a wall, you'll see a little circle appear on the simulation representing the location of the touch
sensor that was triggered. 

.. figure:: TutorialFigures/collisioncircles.*
    :figwidth: 90%
    :width: 50%
    :align: center

    The indicators that your touch ring is sensing something

The last step is to set up a callback in your script to respond to this stimulus. Let's modify ``circle.py`` for
this one.

First, we have to import the message type that the touch ring publishes: ByteMultiArray

.. code:: python

    from std_msgs.msg import ByteMultiArray

Next, we create our callback function to handle this data. ROS callbacks always have 1 parameter by default, and that is
the message that was sent. In the ROS std_msg messages, each message has a ``.data`` element which contains the actual information
sent. Let's make a callback that outputs the indexes of the buttons that were touched. Because of how ROS handles the ByteMultiArray
type in python, we have to use the ``struct::unpack()`` function to get the data as a char type.

.. code:: python

    from struct import *
    def get_hit(message):
        hits = message.data

        for i in range(len(hits)):
            hit = unpack('b', hits[i])[0]

            if hit != 0: 
                print("Touched on", i)
        print("----------------")

Lastly, we set up a subscriber on the node which will listen to the ``robot0/touches`` topic for ByteMultiArray messages and call the callback function
whenever a message comes in.

.. code:: python

    subtouches = node.create_subscription(ByteMultiArray, 'robot0/touches', get_hit)

Now, if you load your little robot into that box and run this code, it will hit the wall, and you'll see something like the following output

.. code:: python

    Touched on 1
    ----------------
    Touched on 0
    ----------------
    Touched on 0
    Touched on 3
    ----------------

.. code:: python

    import rclpy
    from rclpy.node import Node

    from std_msgs.msg import Float32

    from std_msgs.msg import ByteMultiArray
    from struct import *

    def get_hit(message):
        hits = message.data

        for i in range(len(hits)):
            hit = unpack('b', hits[i])[0]

            if hit != 0: 
                print("Touched on", i)
        print("^----------------")

    rclpy.init()
    node = Node("circle")

    publeft = node.create_publisher(Float32, 'robot0/left_wheel')
    pubright = node.create_publisher(Float32, 'robot0/right_wheel')

    subtouches = node.create_subscription(ByteMultiArray, 'robot0/touches', get_hit)

    msg = Float32()

    msg.data = 5.0
    publeft.publish(msg)

    msg.data = 10.0
    pubright.publish(msg)

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
