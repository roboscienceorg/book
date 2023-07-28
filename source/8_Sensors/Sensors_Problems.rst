Problems
--------

#. Assume that you are working in a large event center which has beacons
   located around the facility. Estimate the location of a robot,
   :math:`(a,b,c)`, if the :math:`(x,y,z)` location of the beacon and the
   distance from the beacon to the robot, :math:`d`, are given in the table
   below.

   +-----+-----+-----+-----+
   | x   | y   | z   | d   |
   +=====+=====+=====+=====+
   | 884 | 554 | 713 | 222 |
   +-----+-----+-----+-----+
   | 120 | 703 | 771 | 843 |
   +-----+-----+-----+-----+
   | 938 | 871 | 583 | 436 |
   +-----+-----+-----+-----+
   | 967 | 653 | 46  | 529 |
   +-----+-----+-----+-----+
   | 593 | 186 | 989 | 610 |
   +-----+-----+-----+-----+


#. If you are using a laser diode to build a distance sensor, you need some
   method to determine the travel time. Instead of using pulses and a
   clock, try using phase shifts. What is the wavelength of the modulated
   frequency of 10MHz? If you measure a 10 degree phase shift, this value
   corresponds to what distances? What if the phase shift measurement has
   noise: zero mean with standard deviation 0.1? How does one get a good
   estimate of position if the ranges to be measured are from 20 meters to
   250 meters?

#. Write a Python function to simulate a LIDAR. The simulated LIDAR will
   scan a map and return the distance array. We assume that the obstacle
   map is stored in a two dimensional gridmap, call it map. You can use a
   simple gridmap which uses 0 for a free space cell and 1 for an occupied
   cell. The robot pose (location and orientation) will be stored in a list
   called pose which will hold x, y, theta (where these are in map
   cordinates). Place LIDAR parameters into a list which has total sweep
   angle, sweep increment angle and range. The function call will be data =
   lidar(pose, objmap, params) in which data is the 1D array of distance
   values to obstacles as a function of angle. Test this on a map with more
   than one obstacle.
   Appendix A shows how one
   may generate a map in a bit map editor like GIMP and then export in a
   plain text format which is easily read into a Python (Numpy) array.
   [Although you can fill the grid by a python function which sets the
   values, using the bit map editor will be much easier in the long run.]

   
   
   


#. Assume you have a laser triangulation system as shown in
   :numref:`fig:lasertriangulation2` given
   by :eq:`industrialvision` and
   that :math:`f  = 8`\ mm, :math:`b = 30`\ cm. What are the ranges for
   :math:`\alpha` and :math:`u` if we need to measure target distances in a
   region (in cm) :math:`20 < z < 100` and :math:`10 < x < 30`?


#. Assume you have two cameras that are calibrated into a stereo pair with
   a baseline of 10cm, and focal depth of 7mm. If the error is 10% on
   :math:`v_1` and :math:`v_2`, :math:`v_1 =  2`\ mm and
   :math:`v_2 = 3`\ mm, what is the error on the depth measurement
   :math:`z`? Your answer should be a percentage relative to the error free
   number. Hint: If :math:`v_1 = 2` then a 10% error ranges from 1.8 to
   2.2. [Although not required, another way to approach this problem is the
   total differential from calculus.]


#. Assume you have two cameras that are calibrated into a stereo pair with
   an estimated baseline of 10cm, and focal depth of 10mm. If the error is
   10% on the baseline, what is error on the depth measurement :math:`z`
   with :math:`v_1 = 2`\ mm and :math:`v_2 = 3`\ mm? Your answer should be
   a percentage relative to the error free number. See the hint above.


#. With a single camera, explain how a straight line (produced by a laser)
   can resolve depth information.
