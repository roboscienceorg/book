Problems
--------

#. Using python, drive the DD robot along the following points at uniform speed with
   a p-controller: (0,0), (1,1), (2,0), (3,-1).  Ramp up at the first point and ramp down
   to stop at the last point.    Plot the points and the robot’s path.

#. Create a list of 12 points that trace a figure 8.
   Using python, drive the DD robot along the list of points.
   Ramp up at the first point and ramp down
   to stop at the last point.    Plot the points and the robot’s path.


#. Write a Python program to navigate a robot using the Wavefront algorithm.
   Demonstrate on a map with multiple obstacles.


#. Using Veranda, drive the DD robot along the following points at uniform speed with
   a p-controller: (0,0), (1,1), (2,0), (3,-1).  Ramp up at the first point and ramp down
   to stop at the last point.    Plot the points and the robot’s path.

#. Create a list of 12 points that trace a figure 8.
   Using Veranda, drive the DD robot along the list of points.
   Ramp up at the first point and ramp down
   to stop at the last point.    Plot the points and the robot’s path.


#. Using Veranda, navigate a robot via the Wavefront algorithm.
   Demonstrate on a map with multiple obstacles.



#. In Veranda, drive along :math:`x(t) = 2t-1` and :math:`y(t) = 3t +4` with
   unit speed (:math:`r=1`, :math:`L=4`) with

   a. Basic bot

   #. DD Robot

   #. Mecanum robot

   Use a video screen capture program to record the results.

#. Assume that your path planner provided 20 points for a path. The
   :math:`x` and :math:`y` arrays are given below.

   ::

      x =
      [ -0.01899877   0.50988231   0.39195992   0.49792957   1.19094274
       0.95169032   0.9641402    1.66612687   1.61850455   1.75073195
       1.82176635   2.32907279   2.76304305   2.31714      2.67598521
       3.02594109   3.17585141   3.5780992    3.49887423   4.27148019
       3.8037559    3.75547343   4.67667882   4.37852488   5.03050105
       4.68437205   5.3628283    5.69527227   5.5529013    5.65019384
       6.06986463   6.6702379    6.71739313   6.97436902   7.18467038
       7.26795319   7.07714903   7.67207152   7.90740038   7.77081972
       8.03395401   8.22962525   8.43945312   8.9500287    8.83487244
       9.02244307   9.82322882   9.753483     9.79235576  10.49574545]
      y =
      [ -6.53957063  -8.40356597  -7.48865588  -6.78476431  -6.78271663
      -7.92142123  -6.57019995  -6.68484716  -5.79862184  -5.9067885
      -5.23993105  -4.28376187  -2.74857672  -2.92599682  -1.44630301
      -0.55009994   0.42216262   1.5820503    1.85426302   4.54071375
       4.93526941   6.84360249   7.56566459   7.93381515   8.88892991
      10.11966925  10.8261301   12.03920962  11.37421679  11.94089295
      13.0999794   13.06199908  12.75615952  13.33620909  12.57081916
      12.56314669  12.50733857  11.7055243   10.1871132   10.41736635
       9.22141768   8.18619806   5.76331083   3.81212651   1.08743568
      -1.32495576  -3.92211759  -7.07911689 -10.18134814 -13.76897354]

   Let :math:`r=10`, :math:`L=20`. Use a linear control approach to
   direct a DD robot to follow the path.

#. Drive the DD robot along the following points at uniform speed using a
   cubic spline: (0,0), (1,1), (2,0), (3,-1). Plot the points and the
   robot’s path.

#. Assume that you have the four wheel Mecanum system with the
   :math:`45^{\circ}` roller wheels. Also assume the wheel radius is 5cm,
   axle distance between wheels (left to right) is 30cm and distance
   between axles (front to back) is 40cm. What should the wheel velocities
   be if

   a. you want to drive at an angle of 20 degrees to the right of the
      forward line for the vehicle at 10 cm/sec (but not change
      orientation),

   #. you want to head in a straight line at 10cm/sec but rotate the
      vehicle about the centroid at 1 radian per minute.

#. Assume you have a planner that has provided the following points (2,9),
   (12,13), (23, 40). Also asume that you start with zero derivative at
   (2,9) and pass through (12, 13) with slope :math:`m = 2.5` and have slope
   :math:`m = 0` at (23,40).   Find the spline.

#. Assume you have a planner that has provided the following points (7,13),
   (10,20), (25,5). Assume that your previous point was (2,0), you want to
   pass through (10, 20), with slope :math:`m = -2`, and have slope
   :math:`m = 3` at (25,5).   Find the spline.
