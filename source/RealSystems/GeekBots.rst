==================
Trossen GeekBots
==================

In the course of this text we have seen several different robot topologies but have focused a significant amount of time on the simple differential-drive robot. To accompany the text a basic differential-drive robot has been developed and deployed. The donor chassis for this specific design is the (now defunct) RobotGeeks GeekBot, from which we will appropriate a name. The GeekBot as constructed for use with this text has been developed with cost in mind: total parts cost comes in at just below $300. A 3D printer was used to print one mount for the SBC as well as a mount for the infrared sensor. All files will be made available online for the aspiring roboticist.

The purpose of the SDSMT GeekBot is to teach the basics of robot control using ROS on physical hardware. To accomplish this task the familiar RobotGeeks GeekBot platform was used as a mobile base and extra hardware added as needed.

Hardware:
---------
Locomotion
~~~~~~~~~~
	The GeekBot comes stock with two drive wheels, powered by 6v continuous-rotation hobby servos. Two caster balls keep the bot vertical with a bit of wobble. These motors are not encoded, meaning they only have relative speed adjustment. The actual angular velocity of the wheels is unknown at any point other than stopped. Important: these motors are run directly off the unregulated input power supply. These motors will cook if supplied with more than ~8.5v.

Electrical
~~~~~~~~~~
	The GeekBot is supplied by an 8.4v 2.2Ah LiPo battery. This is 8.4v is passed straight to the drive motors, the onboard Arduino, and a 5v regulator for supplying the Odroid XU4. The harness switches power to the system, and a disconnect is located on the back to decouple battery power and allow the use of another <9v supply for testing.

Sensors
~~~~~~~
	The GeekBot has two sensors: a front-mounted webcam and a servo-actuated IR distance sensor. The IR sensor is mounted under the front of the robot and centered forward; the full range of motion encompasses 0-180 degrees. This sensor returns a non-linear analog voltage corresponding to perceived distance that is passed to the onboard Arduino and processed. The mounted webcam is is both powered by and communicates over one of the Odroid's USB ports. This particular webcam is manual focus and captures at 640x480px, 30fps.

Compute and control
~~~~~~~~~~~~~~~~~~~
	The GeekBot follows a standard distributed-control topology: a high-level controller issues commands to low-level controllers which handle command implementation for specific subsystems. In this case, the high-level controller is an Odroid XU4 running a minimal version of Ubuntu 16.04 LTS. All hardware interfacing tasks (driving servos, reading voltages, turning lights on) are controlled by the Arduino, taking commands from the Odroid via UART.

Software:
---------
ROS Kinetic
~~~~~~~~~~~
	Despite the rest of the text using ROS2 Ardent, the GeekBot runs ROS1 Kinetic. The reasoning behind this decision pertains mostly to the lack of documentation, compressed image transport, networking tools, and debugging tools in ROS2 as of publishing time. The concepts presented in this text for ROS2 are directly transferable to ROS Kinetic. Nodes run independently of each other and communicate via passing messages over topics, exactly like ROS2. ROS1, however, does have a supervisory piece of software called ``roscore`` which handles node connections and message direction.

Software architecture
~~~~~~~~~~~~~~~~~~~~~
	The GeekBot's software setup is best understood through an image: PICTURE HERE
	As can be readily seen, the GeekBot starts up two nodes on boot: one to handle webcam things (webcam), and the other to handle communication with the Arduino (geekbot_node). Both exist under the /geekbot namespace.

	Bringup on boot is handled by a system service through systemd that points to a script contained in the ``geekbot_pkg`` package that resides within the ``geekbot_ws`` directory in the ``/root`` directory of the Odroid's filesystem. Handling bringup through systemd allows for synchronization with the networking stack as well as an easy start-stop-restart interface so the GeekBot's ROS system can be restarted with the robot powered up and online. Linking to a systemd service in a repository-held package gives the option for updates to bringup handling without requiring extensive filesystem rework.

Published topics
~~~~~~~~~~~~~~~~
	After being powered on and being assigned an IP address in the 10.42.0.X range the GeekBot will publish to several topics. The topics we care about are as follows:

	**/geekbot/ir_cm**
  	 - Publishes: Int32
  	 - Distance to nearest object in centimeters as seen by the IR sensor

	**/geekbot/ir_pos**
  	 - Subscribes: Int32
  	 - Angle (0-180) at which to set the IR sensor's servo

	**/geekbot/left_wheel**
  	 - Subscribes: Int32
  	 - Relative speed (-100-100) at which to set the left wheel. Negative for reverse

	**/geekbot/right_wheel**
  	 - Subscribes: Int32
  	 - Relative speed (-100-100) at which to set the right wheel. Negative for reverse

	**/geekbot/webcam/image_raw**
  	 - Publishes: Image
  	 - Raw image from the camera, with zero compression of any kind. Very large message

	**/geekbot/webcam/image_raw/compressed**
  	 - Publishes: CompressedImage
  	 - Compressed frame from the camera, 85% JPEG quality. MUCH smaller than raw image

Using your GeekBot:
-------------------
Initial setup
~~~~~~~~~~~~~
On a Ubuntu 16.04 LTS installation install ROS Kinetic alongside your ROS2 Ardent installation. Follow the `instructions
<http://wiki.ros.org/kinetic/Installation/Ubuntu/>`_ to install ``ros-kinetic-desktop``. *HOWEVER*, do **not** add the excerpt as specified in step 1.6. Doing so will cause conflicts with your ROS2 Ardent installation. A few other packages will have to be installed to meet setup script dependencies and break out some image tools:

``sudo apt install nmap ros-kinetic-image-view ros-kinetic-image-common ros-kinetic-image-transport-plugins ros-kinetic-cv-bridge``

Next, clone the ``geekbot_resources`` repository found `here
<https://github.com/sdsmt-robotics/geekbot_resources/>`_ to somewhere in your filesystem:

``git clone https://github.com/sdsmt-robotics/geekbot_resources``

You should now have a ``geekbot_resources`` directory. This repository contains all client-pc information pertaining to GeekBot operation. Inside you'll find notes and handy examples as well as the Arduino code running on the GeekBot's onboard controller.

In Ubuntu's system settings, navigate to the 'Networking' section. You should see a list of network connections on the left side. Select the wired network and in the lower-righthand side of the pane click 'Options'. Here we can change specific settings for how Ubuntu handles the Ethernet port of your computer. Click on the IPv6 tab. In the drop down, select 'Ignore'. We won't be using IPv6 to connect to the GeekBots. Now select the IPv4 tab and choose 'Share to other computers' from the dropdown menu. In the lower right hand corner click 'Save'. The Ethernet port on your computer is now set to automatically assign an IP on 10.42.0.X spectrum to anything connected to it and requesting an IP address. This is the default state of the GeekBot, so if the GeekBot is connected to your computer then it will request and be assigned an IP in the 10.42.0.X range.

Connecting to the GeekBot
~~~~~~~~~~~~~~~~~~~~~~~~~
1. Connect an Ethernet cable between your computer and the GeekBot's Odroid.
2. Power on the GeekBot by flipping the switch in the left-rear of the bot outwards. The Odroid and Arduino should start flashing lights.
3. Wait patiently for the Odroid to boot. This should take ~30 seconds. When the Odroid has finished the booting process and has grabbed an IP from your computer, it will launch its ROS system and initiate communications with the onboard Arduino. If a successful connection is made you will hear two beeps from the robot.
4. Navigate to the geekbot_resources folder you cloned in the initial setup. Source the ``geekbot_connect.source`` file. This will use ``nmap`` to scrape the 10.42.0.X subnet looking for your bot, set the necessary environment variables, and automatically load in ROS Kinetic to this specific terminal instance.
5. If you see a list of topics print out to your terminal you have successfully connected! You will have to follow step #4 for each terminal instance you would like to connect to the GeekBot.


Shutting down the GeekBot
~~~~~~~~~~~~~~~~~~~~~~~~~
1. Flip the power switch in the left-rear of the bot forward, into the robot. If the power is off no lights should be on.

Charging the GeekBot
~~~~~~~~~~~~~~~~~~~~
1. Locate your GeekBot battery charger. This is a wall-wart supply that has a ribbed back section, a little LED in the bottom left corner, a yellow tip, and an 8.4v 2A output.
2. Plug the charger into an outlet.
3. Locate the battery charging port on the front of the robot. This should be zip-tied down to the lower platform and will run directly into the battery.
4. Plug in the charger to the charging port. The light on the charger should become red. When fully charged, the light will turn green. These batteries have automatic over-voltage protection, so the charger can be left on the battery indefinitely.

Running the GeekBot from external power
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
1. Make sure the GeekBot is powered off.
2. Disconnect the battery from the 2x5.5mm splitter zip-tied to the rear-right vertical support on the bot. Connect a power supply from 7v-8.5v, or the battery charger provided with the robot.
3. If the charger for the battery is used be aware: this supply does not provide enough power to run both motors as well as intense computation on the Odroid. If your robot is intermittently losing connection when operating the motors with this supply you are most likely browning out the Odroid.
