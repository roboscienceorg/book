Tutorial: Placing Obstacles and Sensing in Veranda
--------------------------------------------------

Assuming you've finished :ref:`Tutorial 1<tutorial-1>`, you have a little robot that can wander around aimlessly in an empty world. But one of the 
most important parts of robotics is being able to take input from sensors on your robot and react to your environment. This tutorial will show you how
to put some obstacles in the world with your robot and some ways the sensors provided with Veranda can be used.

Part 1: Making obstacles
^^^^^^^^^^^^^^^^^^^^^^^^

Having a robot that can drive around is fun, but eventually, you may want to try to write code to make the robot avoid things it might run into.
The first step to doing that is having things for the robot to hit. Veranda can load image files as a simulation, and turn them into obstacles that robots
can hit. To do this, choose the 'load simulation' button, and find your image file. Make sure that all the obstacles you want are in the image, because
loading an image will clear your simulation. This example image has a square that we can keep the robot in, and some shapes in the middle for it to avoid.

.. figure:: TutorialFigures/loadimage_button.*
    :figwidth: 90%
    :width: 75%
    :align: center

    Use the 'Load Simulation' button to to load images as obstacles

.. figure:: TutorialFigures/loadimage.*
    :figwidth: 90%
    :width: 75%
    :align: center

    Example of the kind of image you might load. Make sure to get all your obstacles in one picture!

.. TIP::

    Loading images in Veranda works best if they contain only black and white pixels, with no other colors (including grey).
    If you do try to load other images, you can play with the black/white threshold to get it to turn out better.


.. IMPORTANT::

    Veranda can load a number of different files as full simulations, make sure you pick the correct file type in the file-choose dialog so that you are able to select the file you want.

Once you choose an image, you will be presented with some import options. The most important will be the size options, followed
by the threshold options. Veranda will report the size of the image, in pixels, and you will have the option to set the pixel/m ratio, or
and the image size (in meters). Our little roomba has a radius of 2m, and the circle obstacle in our image is 60 px in diameter, so if we set 30 px/m, the robot will
be the same size as that circle. Let's make it 10 px/m so the robot is smaller than the circle.

.. figure:: TutorialFigures/importoptions.*
    :figwidth: 90%
    :width: 50%
    :align: center

    The image importing options

.. figure:: TutorialFigures/loadimage_scales.*
    :figwidth: 90%
    :width: 100%
    :align: center

    Scaling the image to 30x30 px/m (left) and 10x10 px/m (right)

Part 2: Did it crash?
^^^^^^^^^^^^^^^^^^^^^

Now that there's something to hit, we want to know when the robot hits it. To do this, we'll add a touch sensor to the robot; it will send messages
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
        print("----------------")

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

Part 3: Where is it?
^^^^^^^^^^^^^^^^^^^^

One of the most valuable pieces of information you can get is the location of your robot. If you don't have a GPS, or some other positioning system
available, your robot will have to estimate it's location based on what it sees. Fortunately, Veranda comes equipped with a GPS sensor
that you can use to get the absolute location of your robot.

For this example, I added a GPS to my turtle robot, and set its output channel to be robot0/gps.

.. figure:: TutorialFigures/turtle_gps.*
    :figwidth: 90%
    :width: 80%
    :align: center

    Turtle bot upgraded with a gps.

.. _Pose2D: http://docs.ros.org/lunar/api/geometry_msgs/html/msg/Pose2D.html

Now all we need to do to start listening to the robot locations is subscribe to that topic and write a function to handle
the ROS `Pose2D`_ message

.. NOTE::

    This link goes to the original ROS documentation; that's ok, a lot of the built-in messages are the same as they were in ROS 1, just placed
    under a different header directory

The Pose2D message contains 3 pieces of information: x, y, and theta - the robot's location in the world and direction. Let's observe the turtle's location
as it drives in a circle.

First, we need to change our import statement to get the Pose2D message, then we need to change our subscription to use that message.

.. code:: python

    from geometry_msgs.msg import Pose2D
    ...
    gps = node.create_subscription(Pose2D, 'robot0/gps', get_position)

We also need to update our callback to handle the message. I set it up to print the angle in degrees. Make sure you modulus the angle
to get it into the range you want, because it will just count up or down forever if your robot spins.

.. code:: python

    import math
    def get_position(message):
        print("Robot is at (" + str(message.x) + "," + str(message.y) + ") facing " + str((message.theta*180/math.pi) % 360) + " degrees")
        print("----------------")

Other than those changes, our code is exactly the same as the code used to print when the robot ran into something. This is what
it outputs.

.. code:: python

    Robot is at (-3.6408586502075195,-0.10235483199357986) facing 163.32569095773033 degrees
    ----------------
    Robot is at (-3.8731796741485596,-0.5048621296882629) facing 179.23729964819177 degrees
    ----------------
    Robot is at (-3.9862496852874756,-0.9556393027305603) facing 195.1489083386532 degrees
    ----------------
    Robot is at (-3.971405267715454,-1.4201442003250122) facing 211.06051702911464 degrees
    ----------------
    Robot is at (-3.8297839164733887,-1.8627822399139404) facing 226.97212571957607 degrees
    ----------------
    Robot is at (-3.572237491607666,-2.2496349811553955) facing 242.88373441003932 degrees
    ----------------
    Robot is at (-3.2185018062591553,-2.551058292388916) facing 258.79534310050076 degrees
    ----------------
    Robot is at (-2.795682191848755,-2.743954658508301) facing 274.7069517909622 degrees
    ----------------
    Robot is at (-2.336179733276367,-2.8135430812835693) facing 290.61856048142363 degrees

.. code:: python

    import rclpy
    from rclpy.node import Node

    from std_msgs.msg import Float32

    from geometry_msgs.msg import Pose2D
    from struct import *

    import math

    def get_position(message):
        print("Robot is at (" + str(message.x) + "," + str(message.y) + ") facing " + str((message.theta*180/math.pi) % 360) + " degrees")
        print("----------------")

    rclpy.init()
    node = Node("circle")

    publeft = node.create_publisher(Float32, 'robot0/left_wheel')
    pubright = node.create_publisher(Float32, 'robot0/right_wheel')

    gps = node.create_subscription(Pose2D, 'robot0/gps', get_position)

    msg = Float32()

    msg.data = 5.0
    publeft.publish(msg)

    msg.data = 10.0
    pubright.publish(msg)

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

.. NOTE:: 

    The GPS seems like a simple sensor, but it has a lot of options. In the gps properties, you can properties for x, y, and theta to specify...
    
    - Drift: How much error can accumulate on each time step
    - Noise: How far away from the drifted position can the reported position be
    - Probability: What is the probability [0, 1] that the value will not be invalid

    For both Drift and Noise, you can specify the Sigma and Mu of the Gaussian distribution used to pick values.

Part 4: What's nearby?
^^^^^^^^^^^^^^^^^^^^^^

.. _LaserScan: http://docs.ros.org/lunar/api/sensor_msgs/html/msg/LaserScan.html

It's great that we can use a bump sensor to know when we hit something, but wouldn't it be great if we could avoid crashing in the first place?
The LIDAR sensor allows for just that! It can simulate bouncing rays of light across a range of angles to report how far away things
are from your robot. The message that the lidar publishes is the `LaserScan`_ message.

Let's upgrade our turtle again, and put it somewhere that the lidar will sense something.

.. figure:: TutorialFigures/turtle_lidar.*
    :figwidth: 90%
    :width: 80%
    :align: center

    Turtle bot upgraded with a lidar, sensing some obstacles. I set my lidar to report on the robot0/lidar channel. It is sensing 180 degrees in front of it, with
    50 rays that go 3 meters at max.

.. NOTE::

    In that image, the lines for the lidar had been updated during simulation. Right after you place the robot, they won't change to reflect
    what's around them until you press 'play'.

Once again, changing our existing code to use the new message is pretty easy; the hard part is understanding the LaserScan message.
This code will make the robot spin slowly in place, and it will print the message as-is when it arrives. 

.. code:: python

    import rclpy
    from rclpy.node import Node

    from std_msgs.msg import Float32

    from sensor_msgs.msg import LaserScan
    from struct import *

    import math

    def get_position(message):
        print(message)
        print("----------------")

    rclpy.init()
    node = Node("circle")

    publeft = node.create_publisher(Float32, 'robot0/left_wheel')
    pubright = node.create_publisher(Float32, 'robot0/right_wheel')

    gps = node.create_subscription(LaserScan, 'robot0/lidar', get_position)

    msg = Float32()

    msg.data = 0.5
    publeft.publish(msg)

    msg.data = -0.5
    pubright.publish(msg)

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

Let's take a look at one of the LaserScan messages

.. code:: python

    sensor_msgs.msg.LaserScan(
        header=std_msgs.msg.Header(
            stamp=builtin_interfaces.msg.Time(sec=0, nanosec=0), 
            frame_id=''), 
        angle_min=-1.5707963705062866, 
        angle_max=1.5707963705062866, 
        angle_increment=0.06411413848400116, 
        time_increment=0.0, 
        scan_time=0.10000000149011612, 
        range_min=2.1359217166900635, 
        range_max=2.844834089279175, 
        ranges=[inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, 
                2.2153570652008057, 2.213566303253174, 2.2417705059051514, 2.280193567276001, 
                2.3296966552734375, 2.391442060470581, 2.4669628143310547, 2.5582637786865234, 
                2.7111518383026123, 2.844834089279175, inf, inf, inf, inf, inf, inf, inf, inf, 
                inf, inf, inf, inf, inf, inf, 2.3573288917541504, 2.236565351486206, 2.1359217166900635, 
                inf, inf, inf, inf, inf, inf, inf, inf, 2.6468541622161865, 2.440544605255127, 
                2.3114850521087646, 2.2470221519470215, 2.1885221004486084], 
        intensities=[])

There's a lot here to unpack, so let's go one item at a time

- header: Every ROS message has a header stating the message time and the message's id. These are not populated by the Veranda Lidar.
- angle_min/maximum_angle: Bounding range (radians) of the scan, relative to the lidar. Our lidar has a range of 180 degrees, so it goes from -90 to +90, or -pi to +pi.
- angle_increment: Number of radians between each scan point
- time_increment: Time taken between each scan point. Since Veranda is a simulation, we can pause the world and scan it, resulting in instantaneous information
- scan_time: Total time taken to do the scan. This lidar is set to output at 10hz, so that's what it reports.
- range_min/range_max: Minimum and Maximum distance (meters) seen by the lidar.
- ranges: The actual distances seen by the lidar, 1 per scan point. They are reported from minimum angle to maximum. Locations where nothing was seen report infinity.
- intensities: Some lidars (not Veranda's simulation) report the intensity of the light at each point

Part 5: How fast is it going?
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The last sensor we're going to discuss here is the encoder. Encoders are devices that can be used to measure the angular velocity of an axle. While real encoders
might report frequency of a spinning stripe in front of a sensor, the encoders included in Veranda just report angular velocity. They are attached
by default to both the fixed wheel type and Ackermann steering weels type. Just add a wheel to get an encoder. However, until you set the output topic
for an encoder, it will do nothing.

Encoders return a single value, the angular velocity of the wheel in radians/second. If we set the output channels for our encoders,
and add a little bit of noise, we can see how the noise affects the output while the robot drives in a circle.

.. code:: python

    import rclpy
    from rclpy.node import Node

    from std_msgs.msg import Float32

    from struct import *

    import math

    left_speed, right_speed = 0, 0

    def output():
        print("Wheel speeds: " + str(left_speed) + " - " + str(right_speed))
        print("----------------")

    def get_left(message):
        global left_speed

        left_speed = message.data
        output()

    def get_right(message):
        global right_speed

        right_speed = message.data
        output()

    rclpy.init()
    node = Node("circle")

    publeft = node.create_publisher(Float32, 'robot0/left_wheel')
    pubright = node.create_publisher(Float32, 'robot0/right_wheel')

    subleft = node.create_subscription(Float32, 'robot0/left_encoder', get_left)
    subright = node.create_subscription(Float32, 'robot0/right_encoder', get_right)

    msg = Float32()

    msg.data = 5.0
    publeft.publish(msg)

    msg.data = 10.0
    pubright.publish(msg)

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


.. code:: python

    ----------------
    Wheel speeds: 2.6641697883605957 - 7.652131080627441
    ----------------
    Wheel speeds: 2.6641697883605957 - 10.325849533081055
    ----------------
    Wheel speeds: 5.337887287139893 - 10.325849533081055
    ----------------
    Wheel speeds: 3.404207706451416 - 10.325849533081055
    ----------------
    Wheel speeds: 3.404207706451416 - 8.392169952392578
    ----------------
