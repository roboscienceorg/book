Problems
--------


#. Why is simulation useful to roboticists?

#. List some of the advantages and disadvantages of simulating a robot
   vs. working with physical robots.


#. Using Python and ZeroMQ, write a chat program (call it *chat.py*). First
   prompt the user for their name. Write to all members in the chat group
   that this person has entered the chat. In a loop, grab user inputs and
   broadcast to the chat with format: name: <user input> . Echo to the
   terminal all strings sent to the chat.


#. Using Python and ZeroMQ, modify the example programs in the text on the
   kinematics of the two link manipulator.

   #. Write a program that creates a list of 100 equally spaced points
      along the path :math:`y = 15 -  x` for :math:`0 \leq x \leq 10` and
      publishes those points on the topic `physData` using a multiarray
      floating data type, i.e. values x and y are floats. Publish the data
      at 5Hz.

   #. Write a program that subscribes to topic `physData`, plugs the values
      in, computes the serial two link inverse kinematics to gain the servo
      angles (pick one of the +/-) and publishes the angles to the topic
      /thetaData. You may assume the link arms are :math:`a_1=a_2 = 10`.
      Format will be the same as the previous topic.

   #. Write a program that subscribes to both `physData` and `thetaData`. The
      program should plug the angles into the forward kinematics and check
      against the data in `physData`. It should plot the original curve in
      green and the “check” in blue.

#. Assume that you have a parallel two link manipulator with
   :math:`L_0 = 10`\ cm, :math:`L_1 = 15`\ cm and :math:`L_2 = 20`\ cm.

   #. Write a ZeroMQ program that creates a list of 100 equally spaced points
      along the path :math:`x = 7\cos(t)+10`, :math:`y = 5\sin(t) + 15` and
      publishes those points on the topic `physData` using a multiarray
      floating data type, i.e. values x and y are floats. Publish the data
      at 5Hz.

   #. Write a ZeroMQ program that subscribes to topic `physData`, plugs the
      values in, computes the serial two link inverse kinematics to gain
      the servo angles and publishes the angles to the topic `thetaData`.
      Format will be the same as the previous topic.

   #. Write a ZeroMQ program that subscribes to both `physData` and `thetaData`.
      The program should plug the angles into the forward kinematics and
      check against the data in `physData`. It should plot the original
      curve in green and the “check” in blue.

#. Using Python and 0MQ write a program that will add padding to obstacles
   while shrinking the footprint of the robot to a point. Assume that you
   have a circular robot with radius 10 and starting pose (15,15,90).

   #. Write a program that will publish the pose of the robot on the topic
      `robot_pose` and the footprint type of the robot on
      `robot_footprint` as a string (For example circle or polygon).
      Also publish the radius of the robot on `robot_radius`.

   #. Write a program that will publish a list of obstacles as polygons on
      the topic `obstacles`. For this program let the obstacles be the
      following:

      #. Rectangle with the vertices (40,30), (50,5), (50, 30) (40,30).

      #. Rectangle with the vertices (40,5), (50,5), (50,0), (40,5).

   #. Write a program that subscribes to `robot_pose`,
      `robot_footprint`, and `obstacles`. Based on the footprint
      string, this program should be able to subscribe to either the robot
      radius or dimension topics for circular and rectangular robots. This
      program will reduce the robot footprint to a point, add padding to
      the obstacles, and plot the robot as a point and padded obstacles
      with the maximum x and y values being 70 and 30.

#. Rework the previous problem assuming that you have a rectangular robot
   with :math:`width=10` and :math:`length=20` and initial pose (0,10,0).



#. Using Python and ZeroMQ, write a program to calculate the motion of a
   differential drive robot.

   a. Write a program that publishes a sequence of wheel velocities on the
      topic `WheelVel` at 10Hz. Use the multiarray datatype. This node
      should be named `Control`. This program should also publish on a
      topic named `Active` either 1 or 0 at 1 Hz to say whether or not
      the robot is active (meaning done with wheel velocities and you can
      plot now: active =1, done = 0). Demonstrate the code on
      :math:`\dot{\phi}_1 = 2 + 2e^{-t_n}` and
      :math:`\dot{\phi}_2 = 2+e^{-2t_n}` for :math:`0 \leq t \leq 10`.

   #. Write a program that uses the differential drive kinematics to derive
      the robot linear and angular velocities. Publish the velocities using
      a message and name the topic `RobotVel`. This
      node should be named `ForwardK`. Assume that :math:`D=10`,
      :math:`L=20` and the robot starts at (0,0,0).

   #. Write a program that will subscribe to the  message and plot the
      robot’s path using Python plotting when it gets the signal on the
      Active topic. This node should be named `RobotPlot`.

#. Using the above problem, replace the initial pose (0,0,0) in the second
   part, (b), with the pose (2,2,45).

#. Using the forward difference on :math:`x(t) = t^2`, what is the error on
   the derivative value for
   :math:`\Delta t  = 10^{-1}, 10^{-2}, 10^{-3}, 10^{-4}` at the location
   :math:`t=1`.

#. Let :math:`r=10`, :math:`L=20`, :math:`\Delta t = .1`. Find the discrete
   kinematic model if the wheel velocities are
   :math:`\dot{\phi}_{1} = 2(1-e^{-t})`,
   :math:`\dot{\phi}_{2} = 2(1-e^{-2t})`.

#. Using the discrete model equations in the previous, plot the path for
   :math:`0 \leq t \leq 5`.

#. For the integral in :eq:`ddexamplenotworkable`, use a
   numerical differential equation solver (with some software package) to
   integrate the equations. Compare this to using a Taylor expansion on the
   equations to work out the integrals.

#. What is the smooth (:math:`\dot{x}`, :math:`\dot{y}` are continuous)
   parametric form of

   a. :math:`y=(3/2)x + 5/2`

   #. :math:`y = x^{2/3}`.

   #. :math:`(x-3)^2/16 + (y-2)^2/9 = 1`.

#. Find the analytic wheel velocities and initial pose for a differential
   drive robot tasked to follow (:math:`r=1`, :math:`L=4`) the given paths.
   Plot the paths and compare to the actual functions to verify.

   a. :math:`y=(3/2)x + 5/2`

   #. :math:`y = x^{2/3}`

   #. :math:`(x-3)^2/16 + (y-2)^2/9 = 1`

#. Find the wheel velocities and initial pose for a differential drive
   robot tasked to drive a square with corners (0,0), (10,0), (10,10),
   (0,10). You should stop and turn at a corner. Drive the edges at unit
   speed. Plot the paths and compare to the actual functions to verify.

#. Find the wheel velocities and initial pose for a differential drive
   robot in an infinity (:math:`\infty`) shape. Plot the paths and compare
   to the actual functions to verify.

#. Write robot control code to drive the robot
   along the indicated paths and plot the obstacles and paths in matplotlib.

   a. the triangular path with vertices (0,0), (15,0) , (5,20),
   #. the square path with corners (0,0), (10,0), (10,10), (0,10),
   #. the circular path centered at the origin and radius is 15.

#. Create two circular obstacles.
   The first obstacle is a disk centered at (5,5) with radius 2. The second
   is a disk centered at (15,15) with radius 3. Write the control code to
   drive a figure 8 around the two obstacles. Run at least two loops.  Plot
   the obstacles and path in matplotlib.
