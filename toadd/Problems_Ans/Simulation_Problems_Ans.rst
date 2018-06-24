Problems
--------

.. raw:: latex

   \setcounter{Exc}{0}

]

Why is simulation useful to roboticists?

List some of the advantages and disadvantages of simulating a robot vs.
working with physical robots.

Using ROS and Python, write a program to simulate the motion of a
differential drive robot.

#. Write a program that publishes a sequence of wheel velocities on the
   topic ``/WheelVel`` at 10Hz. Use the multiarray datatype. This node
   should be named ``Control``. This program should also publish on a
   topic named ``/Active`` either 1 or 0 at 1 Hz to say whether or not
   the robot is active (meaning done with wheel velocities and you can
   plot now: active =1, done = 0). Demonstrate the code on
   :math:`\dot{\phi}_1 = 2 + 2e^{-t_n}` and
   :math:`\dot{\phi}_2 = 2+e^{-2t_n}` for :math:`0 \leq t \leq 10`.

#. Write a program that uses the differential drive kinematics to derive
   the robot linear and angular velocities. Publish the velocities using
   the ROS standard twist message and name the topic ``/RobotVel``. This
   node should be named ``ForwardK``. Assume that :math:`D=10`,
   :math:`L=20` and the robot starts at (0,0,0).

#. Write a program that will subscribe to the twist message and plot the
   robot’s path using Python plotting when it gets the signal on the
   Active topic. This node should be named ``RobotPlot``.

Using the above problem, replace the initial pose (0,0,0) in the second
part, (b), with the pose (2,2,45).

Using the forward difference on :math:`x(t) = t^2`, what is the error on
the derivative value for
:math:`\Delta t  = 10^{-1}, 10^{-2}, 10^{-3}, 10^{-4}` at the location
:math:`t=1`.

Let :math:`r=10`, :math:`L=20`, :math:`\Delta t = .1`. Find the discrete
kinematic model if the wheel velocities are
:math:`\dot{\phi}_{1} = 2(1-e^{-t})`,
:math:`\dot{\phi}_{2} = 2(1-e^{-2t})`.[numericalddhw]

Using the discrete model equations in
problem \ `[numericalddhw] <#numericalddhw>`__, plot the path for
:math:`0 \leq t \leq 5`.

For the integral in
example \ `[ddexamplenotworkable] <#ddexamplenotworkable>`__, use a
numerical differential equation solver (with some software package) to
integrate the equations. Compare this to using a Taylor expansion on the
equations to work out the integrals.

What is the smooth (:math:`\dot{x}`, :math:`\dot{y}` are continuous)
parametric form of

#. :math:`y=(3/2)x + 5/2`

#. :math:`y = x^{2/3}`.

#. :math:`(x-3)^2/16 + (y-2)^2/9 = 1`.

Find the analytic wheel velocities and initial pose for a differential
drive robot tasked to follow (:math:`r=1`, :math:`L=4`) the given paths.
Plot the paths and compare to the actual functions to verify.

#. :math:`y=(3/2)x + 5/2`

#. :math:`y = x^{2/3}`

#. :math:`(x-3)^2/16 + (y-2)^2/9 = 1`

Find the wheel velocities and initial pose for a differential drive
robot tasked to drive a square with corners (0,0), (10,0), (10,10),
(0,10). You should stop and turn at a corner. Drive the edges at unit
speed. Plot the paths and compare to the actual functions to verify.

Find the wheel velocities and initial pose for a differential drive
robot in an infinity (:math:`\infty`) shape. Plot the paths and compare
to the actual functions to verify.

Using the STDR simulator, write robot control code to drive the robot
along the triangular path with vertices (0,0), (15,0) , (5,20).

Using the STDR simulator, place two circular obstacles on the canvas.
The first obstacle is a disk centered at (5,5) with radius 2. The second
is a disk centered at (15,15) with radius 3. Write the control code to
drive a figure 8 around the two obstacles. Run at least two loops.

| **a**. Develop the parametric equations to describe the motion of the
  robot:
| Start with a line that oscillates back and forth across the obstacles
  we want to drive around. We need to move along the line between the
  center of the obstacles, :math:`(0,0)` and :math:`(15,15)`. We can
  accomplish this by starting our equations at :math:`(7,7)` and using
  cosine to oscillate both :math:`x` and :math:`y` equally. We multiply
  :math:`\cos(t)` by a large enough constant to clear the obstacles.

  .. math::

     \begin{aligned}
     x&=7+16\cos(t)\\
     y&=7+16\cos(t)\end{aligned}

Next, we need to make the path circle the obstacles. We can add a sine
component to x, which gives us:

.. math::

   \begin{aligned}
   x&=7+16\cos(t)+10\sin(t)\\
   y&=7+16\cos(t)\end{aligned}

Adding a negative sine component to y will widen the loop:

.. math::

   \begin{aligned}
   x&=7+16\cos(t)+10\sin(t)\\
   y&=7+16\cos(t)-10\sin(t)\end{aligned}

Cut the period of the sine components in half and the path will move
through the center of the obstacles.

.. math::

   \begin{aligned}
   x &= 7+16\cos(t)+10\sin(2t)\\
   y &= 7+16\cos(t)-10\sin(2t)\end{aligned}

Finally, divide t of both the :math:`\cos` and :math:`\sin` components
by 10 to slow down the robot:

.. math::

   \begin{aligned}
   x &= 7+16\cos\left(\frac{t}{10}\right)+10\sin\left(\frac{t}{5}\right)\\
   y &= 7+16\cos\left(\frac{t}{10}\right)-10\sin\left(\frac{t}{r}\right)\end{aligned}

| **b**. Implementing the motion control:
| First, take the derivatives of our parametric equations to determine
  velocity:

  .. math::

     \begin{aligned}
     \frac{dx}{dt}&=2\cos\left(\frac{t}{5}\right)-\frac{8\sin\left(\frac{t}{10}\right)}{5}\\
     \frac{dy}{dt}&=-2\cos\left(\frac{t}{5}\right)-\frac{8\sin\left(\frac{t}{10}\right)}{5}\end{aligned}

We can use :math:`\frac{dx}{dt}` and :math:`\frac{dy}{dt}` as the
velocity controls for the robot. This relies on the following robot
starting position:

.. math:: P_0 \approx (23,23)

Therefore, the simulation will move the robot to

.. math:: (x,y)=(23,23)

before spawning the obstacles and driving the figure-8.

| The controller is able to drive the robot along the curve by
  evaluating the velocity functions at discrete time intervals of 0.25
  seconds. The controller publishes the velocity commands and waits for
  0.25 seconds before evaluating the next point. The robot drives for
  :math:`2\pi` radians :math:`\times` :math:`2` loops :math:`\times`
  :math:`10` (because :math:`t` is divided by :math:`10`) for a total of
  :math:`40\pi` seconds. This allows the robot to circle the obstacles
  twice.
| Finally, we bring everything together and run the created scripts with
  PyStage. The code for this algorithm can be seen in
  Listing \ `[lst:3.4] <#lst:3.4>`__

.. raw:: latex

   \centering

.. figure:: solutions/Simulation/p3-4.jpg
   :alt: Pystage: Drawing Figure-8 Around Two Obstacles [fig:3.4]

   Pystage: Drawing Figure-8 Around Two Obstacles [fig:3.4]

.. raw:: latex

   \mylisting[language=Python, firstline=6,basicstyle=\ttfamily\scriptsize, label={lst:3.4}]{../pycode/solutions/Simulation/p3_4.py}

Using STDR Simulator, code the basic motion algorithm. a. Demonstrate
your approach with one obstacle. b. Demonstrate with several obstacles.

| A basic motion algorithm program was developed for ... The completed
  code was tested successfully with world files containing **a) a single
  cluster of obstacles** (Figure `[fig:3.6a] <#fig:3.6a>`__, World
  File \ `[lst:3.6a] <#lst:3.6a>`__) and **b) multiple obstacles**
  (Figure `[fig:3.6b] <#fig:3.6b>`__, World
  File \ `[lst:3.6b] <#lst:3.6b>`__). The code for this algorithm can be
  seen in Listing \ `[lst:3.6] <#lst:3.6>`__.
| Basic idea of the basic motion algorithm!

1

Move toward the goal

If collision: backup, turn right and drive for a few seconds

Repeat until goal is reached

.. raw:: latex

   \centering

.. figure:: solutions/Simulation/p3-6a.jpg
   :alt: Pystage: Basic Motion Algorithm with One Obstacle [fig:3.6a]

   Pystage: Basic Motion Algorithm with One Obstacle [fig:3.6a]

.. raw:: latex

   \mylisting[basicstyle=\ttfamily\scriptsize, label={lst:3.6a}]{../pycode/motion_algorithm/world6a}

.. raw:: latex

   \centering

.. figure:: solutions/Simulation/p3-6b.jpg
   :alt: Pystage: Basic Motion Algorithm with Multiple Obstacles
   [fig:3.6b]

   Pystage: Basic Motion Algorithm with Multiple Obstacles [fig:3.6b]

.. raw:: latex

   \mylisting[basicstyle=\ttfamily\scriptsize, label={lst:3.6b}]{../pycode/motion_algorithm/world6b}

.. raw:: latex

   \mylisting[language=python, firstline=3, breaklines=true, basicstyle=\ttfamily\scriptsize, label={lst:3.6}]{../pycode/motion_algorithm/basic_motion.py}

Using STDR and the basic motion algorithm, place a set of obstacles that
cause the robot to cycle and not find the goal.

| A world file was created for PyStage v2.0 containing several
  obstacles. Using the basic motion program developed for the previous
  problem, these obstacles proved to be successful in causing the basic
  motion algorithm to get stuck in a loop, as seen in
  Figure \ `[fig:3.7] <#fig:3.7>`__. The world file containing the
  cycle-causing obstacles can be seen in
  Listing \ `[lst:3.7] <#lst:3.7>`__.

.. raw:: latex

   \centering

.. figure:: solutions/Simulation/p3-7.jpg
   :alt: Pystage: Basic Motion Algorithm with Cycle [fig:3.7]

   Pystage: Basic Motion Algorithm with Cycle [fig:3.7]

.. raw:: latex

   \mylisting[basicstyle=\ttfamily\scriptsize, label={lst:3.7}]{../pycode/motion_algorithm/world7}

.. raw:: latex

   \Closesolutionfile{Answer}

.. [1]
   that would be a *codeword* for sloppy

.. [2]
   The author would simply call this a remotely operated car, but either
   way, teleoperation does change how one looks at a vehicle.

.. [3]
   Keep in mind that the robot graphics circle method draws from the
   bottom of the circle and so the center for this formula and the one
   for the circle method need to be adjusted by the radius.

.. [4]
   If you wanted an integer array you would cast this as an int.

.. [5]
   This algorithm is slightly more general in that it does not need the
   goal location, but just the direction to the goal during the process.

.. |(a) The two link simulator. (b) Published angle to the simulator.[Fig:twolinksimulator1]| image:: sim/twolinksimulator1.png
.. |(a) The two link simulator. (b) Published angle to the simulator.[Fig:twolinksimulator1]| image:: sim/twolinksimulator2.png
.. |(a) The servo angle control widget and (b) the position control widget.[Fig:tksliderexample]| image:: sim/tksliderexample.png
.. |(a) The servo angle control widget and (b) the position control widget.[Fig:tksliderexample]| image:: sim/tksliderexample2.png
.. |Movement between the points. a) moving both linearly. b) moving the servos sequentially. [Fig:twolinkcoarseexample]| image:: sim/twolinkcoarseexample.png
.. |Movement between the points. a) moving both linearly. b) moving the servos sequentially. [Fig:twolinkcoarseexample]| image:: sim/twolinkcoarseexample2.png
.. |a) Estimating the object boundary. b) Bump sensors can only determine the nature of the boundary at the contact location. c) Using touch sensors to estimate the boundary normal and tangent. [turtleboundary]| image:: turtle/turtlecollide2
.. |a) Estimating the object boundary. b) Bump sensors can only determine the nature of the boundary at the contact location. c) Using touch sensors to estimate the boundary normal and tangent. [turtleboundary]| image:: turtle/turtlecollide3
.. |a) Estimating the object boundary. b) Bump sensors can only determine the nature of the boundary at the contact location. c) Using touch sensors to estimate the boundary normal and tangent. [turtleboundary]| image:: path/tangent

