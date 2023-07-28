Problems
--------


#. Why is simulation useful to roboticists?

#. List some of the advantages and disadvantages of simulating a robot
   vs. working with physical robots.

#. Using ROS and Python, write a program to calculate the motion of a
   differential drive robot.

   a. Write a program that publishes a sequence of wheel velocities on the
      topic ``/WheelVel`` at 10Hz. Use the multiarray datatype. This node
      should be named ``Control``. This program should also publish on a
      topic named ``/Active`` either 1 or 0 at 1 Hz to say whether or not
      the robot is active (meaning done with wheel velocities and you can
      plot now: active =1, done = 0). Demonstrate the code on
      :math:`\dot{\phi}_1 = 2 + 2e^{-t_n}` and
      :math:`\dot{\phi}_2 = 2+e^{-2t_n}` for :math:`0 \leq t \leq 10`.

   #. Write a program that uses the differential drive kinematics to derive
      the robot linear and angular velocities. Publish the velocities using
      the ROS message and name the topic ``/RobotVel``. This
      node should be named ``ForwardK``. Assume that :math:`D=10`,
      :math:`L=20` and the robot starts at (0,0,0).

   #. Write a program that will subscribe to the twist message and plot the
      robot’s path using Python plotting when it gets the signal on the
      Active topic. This node should be named ``RobotPlot``.

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

#. Using the Veranda simulator, write robot control code to drive the robot
   along:

   a. the triangular path with vertices (0,0), (15,0) , (5,20),
   #. the square path with corners (0,0), (10,0), (10,10), (0,10),
   #. the circular path centered at the origin and radius is 15.

#. Using the Veranda simulator, place two circular obstacles on the canvas.
   The first obstacle is a disk centered at (5,5) with radius 2. The second
   is a disk centered at (15,15) with radius 3. Write the control code to
   drive a figure 8 around the two obstacles. Run at least two loops.
