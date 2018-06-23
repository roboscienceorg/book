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
   Appendix :numref:`section:imagemaps` shows how one
   may generate a map in a bit map editor like GIMP and then export in a
   plain text format which is easily read into a Python (Numpy) array.
   [Although you can fill the grid by a python function which sets the
   values, using the bit map editor will be much easier in the long run.]
