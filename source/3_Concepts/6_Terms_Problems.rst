Problems
--------

#.  What is the difference between accuracy and resolution?

#. What is meant by serial chain manipulator and how is this different from a parallel chain manipulator?

#. What is meant by cartesian and cylindrical robot designs?  Why would you select one over the otehr?

#. What mechanical property do the ball screw, threaded rod and rack \& pinon share?

#. Define configuration space and workspace.  What mathematical object relates them?

#.  What is meant by forward position kinematics?  What is meant by inverse velocity kinematics?

#. Graph the workspace of a two-link manipulator centered at the origin with :math:`a_1 = 15` and :math:`a_2 = 10`.

#. Assume that you have a two link planar manipulator. :math:`\theta_1` is
   the angle between the x axis (measured counter-clockwise as positive) and the
   first link arm. :math:`\theta_2` is the angle between the second link
   arm and the first link arm (again measured counter-clockwise as positive).  The link lengths are :math:`a_1, a_2`.
   Derive the formulas for :math:`dx/dt` and :math:`dy/dt` as a function of :math:`d\theta_1/dt` and :math:`d\theta_2/dt`.

#. Using the two link planar manipulator described in the previous problem.
   Derive the formulas for :math:`d\theta_1/dt` and :math:`d\theta_2/dt`  as a function of :math:`x, y`, :math:`dx/dt` and :math:`dy/dt`.

#. Given the two link manipulator as described in the previous problems with
   the length of the first link :math:`a_1 = 10` and the second link
   :math:`a_2 = 10`.

   a. If :math:`\theta_1 = 45^\circ`, :math:`\theta_2 = 45^\circ`, find
      :math:`x` and :math:`y`.

   #. If :math:`\theta_1 = 45^\circ`, :math:`\theta_2 = 45^\circ`,
      :math:`d\theta_1/dt = 5^\circ s^{-1}`,
      :math:`d\theta_2/dt = 10^\circ s^{-1}` find :math:`dx/dt` and
      :math:`dy/dt`.

#. Given the two link manipulator as above with
   the length of the first link :math:`a_1 = 10` and the second link
   :math:`a_2 = 10`.

   #. If :math:`x = 12`, :math:`y = 14`, find :math:`\theta_1` and :math:`\theta_2`

   #. If :math:`x = 12`, :math:`y = 14`, :math:`dx/dt = -0.25`, :math:`dy/dt = 0.5`, find :math:`d\theta_1/dt` and :math:`d\theta_2/dt`

#. Assume that you have a two link planar manipulator. :math:`\theta_1` is
   the angle between the x axis (measured clockwise as positive) and the
   first link arm. :math:`\theta_2` is the angle between the second link
   arm and the first link arm (again measured clockwise as positive). Due
   to servo limitations: :math:`-100^\circ < \theta_1 < 100^\circ`,
   :math:`-150^\circ < \theta_2 < 150^\circ`. Also assume the first link is
   20cm long and the second link is 15cm.

   a. What is the configuration space?

   #. What is the workspace?

#. Find the forward velocity kinematics equations for the two link
   manipulator.

#. Assume that you have a two link manipulator that is operating in the
   vertical plane, meaning the two arms lie in the :math:`x-z`plane. Attach the base to a rotational joint so the
   arm rotates around the :math:`z` axis.  Assume the base joint does not induce an offset distance *d*.  See
   :numref:`fig:two-link-z` .

   .. _`fig:two-link-z`:
   .. figure:: TermsFigures/twolinkalt2.*
      :width: 50%
      :align: center

      Two link manipulator.

   a. Find the position of the end effector as a function of joint angles.

   #. Find the inverse kinematic formula.



#. Typos can creep up in textbooks, papers and reference materials. How
   would test the accuracy of the formulas given in equations
   :eq:`paralleltwolinkforward` and :eq:`paralleltwolinkIK`? Discuss.

#. Find the forward velocity kinematics equations for the parallel two link manipulator.

#. Derive the formula for :eq:`paralleltwolinkforward`:

   .. math:: (x,y) = \left( \frac{a+c}{2} + \frac{v (b-d)}{u} , \frac{b+d}{2} + \frac{v (c-a)}{u} \right)

   Hint: define the segment from :math:`(a,b)` to :math:`(c,d)` as
   :math:`B` (the base of the triangle), and :math:`\vec{A}` as a vector
   which is a perpendicular to :math:`B`, see :numref:`Fig:paralleltwolink3` .

   .. _`Fig:paralleltwolink3`:
   .. figure:: TermsFigures/2dDelta3.png
      :width: 40%
      :align: center

      Extraction of the isosceles triangle.

#. Derive the formulas for the parallel two link manipulator inverse
   kinematics given in
   :eq:`paralleltwolinkIK`. Hint: :numref:`Fig:paralleltwolinkIK`.

   .. _`Fig:paralleltwolinkIK`:
   .. figure:: TermsFigures/2dDelta4.png
      :width: 40%
      :align: center

      Parallel Two Link Inverse Kinematics variables


#. The Denavit-Hartenberg link definition contains rotation matrices, :math:`R`, and translation matrices, :math:`T`, in homogeneous coordinates.   Which pairs of matrices commute and which do not.  For example, is :math:`RT = TR`?.  Is :math:`T_1T_2 = T_2T_1`?


#.  Translate the following DH parameters into a DH convention transformation matrix.


    +------+------------------+-----------+-------------+------------------+
    | Link | :math:`\theta`   | :math:`d` | :math:`a`   | :math:`\alpha`   |
    +======+==================+===========+=============+==================+
    | 1    | :math:`\theta_1` | 1.0       | :math:`a_1` | 0                |
    +------+------------------+-----------+-------------+------------------+
    | 2    | 0                | 0         | :math:`a_2` | :math:`45^\circ` |
    +------+------------------+-----------+-------------+------------------+


#. Modify the robot arm found in :numref:`fig:two-link-z` where the base joint raises the arm by :math:`d_1` in the *z* direction.   When the base joint angle is zero, :math:`\theta_1=0`, the two link arms indicated by :math:`a_2, a_3` lie in the x-z plane.  The latter two joints would have an axis of rotation parallel to the y axis (again when :math:`\theta_1=0`).

   #.  What is the DH parameter table?
   #.  Write out the DH transformation matrices.
   #.  Find the forward kinematics.


   .. _`Fig:offsetplanar1`:
   .. figure:: TermsFigures/offsetplanar1.*
      :width: 50%
      :align: center

      Offset Two Link


#. Modify the robot arm found in :numref:`fig:two-link-z` where the base joint raises the arm by :math:`d_1` in the *z* direction. In adition, the joint mounts the arm along the rotation direction by :math:`a_1`.    When the base joint angle is zero, :math:`\theta_1=0`, the two link arms indicated by :math:`a_2, a_3` lie in the x-z plane.  The latter two joints would have an axis of rotation parallel to the y axis (again when :math:`\theta_1=0`).

   #.  What is the DH parameter table?
   #.  Write out the DH transformation matrices.
   #.  Find the forward kinematics.


   .. _`Fig:offsetplanar2`:
   .. figure:: TermsFigures/offsetplanar2.*
      :width: 50%
      :align: center

      Offset  Two Link with side mount


#. For the robotic arm found in :numref:`Fig:offsetplanar3`,  the distance :math:`d_1` is constant where :math:`a_2, a_3` can vary.

   #.  What is the DH parameter table?
   #.  Write out the DH transformation matrices.
   #.  Find the forward kinematics.
   #.  Compare your DH convention approach to a direct trigonometric method.


   .. _`Fig:offsetplanar3`:
   .. figure:: TermsFigures/offsetplanar3.*
      :width: 50%
      :align: center

      Cylindrical robot.


#. Assume that your differential drive robot has 10 cm diameter wheels and
   a total axle length (wheel to wheel) of 20 cms. If both wheels are
   turning at 0.8 revolutions per second, what is the speed of the robot.

#. Using the same robot as previous problem,  but
   where the left wheel is turning at 1.5 radians per second and the right
   wheel is turning at 1.8 radians per second. Determine the linear
   velocity and path of the robot. You may assume the initial pose is
   (0,0,0) at :math:`t=0`.

#. For the differential drive robot, let :math:`r=10`, :math:`L=15`,
   :math:`\dot{\phi_1} = 0.9` :math:`\dot{\phi_2}= 1.2`.

   a. What is the angular velocity of the robot?

   #. What is the velocity vector for the robot when
      :math:`\theta = 45^\circ`?

#. Let :math:`r=10`, :math:`L=15`. If you program the robot to drive
   straight and the robot traces out a circle of diameter 3 meters while
   traveling 1 m/s, what are the two wheel speeds?

#. Say you have a differential drive robot that has an axle length of 30cm
   and wheel diameter of 10cm. Find the angular velocity for the left and
   right wheel if the robot is going to

   a. Spin in place at a rate of 6 rpm (revolutions per min),

   #. Drive a circle of radius 1 meter (measured center of circle to middle
      of axle) at 3 rpm,

   #. Drive a straight line at 1 meter / min.

#. | Given a differential drive robot starting from (0,0,0) find the final position when wheel velocities are given by:
   | t=0 to t=5: :math:`\omega_1` = 2, :math:`\omega_2` = 2
   | t=5 to t=6: :math:`\omega_1` = 3, :math:`\omega_2` = 4
   | t=6 to t=10: :math:`\omega_1` = 1, :math:`\omega_2` = 2
   | where D=10, L=16.

#. List the variables in the configuration space of a circular ground robot
   that can drive around and use a telescopic arm with a rotational base,
   lifting servo and elbow joint servo.
