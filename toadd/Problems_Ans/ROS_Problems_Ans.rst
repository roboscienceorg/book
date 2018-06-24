Problems
--------


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
