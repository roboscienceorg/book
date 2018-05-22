Problems
--------

.. raw:: latex

   \setcounter{Exc}{0}

]

Assume that you are working in a large event center which has beacons
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

Each individual error term is given by

.. math:: E = \left(\sqrt{(a-x)^2 + (b-y)^2 + (c-z)^2} - distance \right)^2

From the individual errors, we can form the total error function by
summing up the individual error terms.

.. math::

   \begin{array}{ll}
   E &= \left(\sqrt{(a-884)^2 + (b-554)^2 + (c-713)^2} - 222\right)^2    \\
      &+ \left(\sqrt{(a-120)^2 + (b-703)^2 + (c-771)^2} - 843\right)^2   \\
      &+ \left(\sqrt{(a-938)^2 + (b-871)^2 + (c-583)^2} - 436\right)^2   \\
      &+ \left(\sqrt{(a-967)^2 + (b-653)^2 + (c-46)^2 }   - 529\right)^2   \\
      &+ \left(\sqrt{(a-593)^2 + (b-186)^2 + (c-989)^2} - 610\right)^2 
   \end{array}

If :math:`E=0`, then the :math:`(x,y)` point matches all five distances.
While we may not be able to find an error value :math:`0`, we can
minimize the error using a hill climbing algorithm, as shown in
Listing \ `[lst:4.1] <#lst:4.1>`__. The result of the error minimization
and the corresponding robot coordinates can be seen in
Figure \ `[fig:4.1] <#fig:4.1>`__.

.. math::

   \begin{aligned}
   (a, b, c) &= (883.1, 443.0, 521.4)   
   E &\approx 14.83\end{aligned}

.. raw:: latex

   \centering

.. figure:: solutions/Sensors/p4-1.jpg
   :alt: Hill Climbing Error Minimization Result [fig:4.1]

   Hill Climbing Error Minimization Result [fig:4.1]

.. raw:: latex

   \mylisting[language=Python, firstline=4, basicstyle=\ttfamily\scriptsize, label={lst:4.1}]{../pycode/p4-1.py}

If you are using a laser diode to build a distance sensor, you need some
method to determine the travel time. Instead of using pulses and a
clock, try using phase shifts. What is the wavelength of the modulated
frequency of 10MHz? If you measure a 10 degree phase shift, this value
corresponds to what distances? What if the phase shift measurement has
noise: zero mean with standard deviation 0.1? How does one get a good
estimate of position if the ranges to be measured are from 20 meters to
250 meters?

**a.** *What is the wavelength of the modulated frequency of 10MHz?*
[15pt] The wavelength :math:`\lambda` is given by the following
equation, where :math:`c` is the speed of light at :math:`\approx` 3e8 m
and :math:`f` is the modulated frequency:

.. math::

   \begin{aligned}
    \lambda &= \frac{c}{f}  
                  &\approx \frac{\SI{3e8}{\meter\per\second}}{\SI{10e6}{(cycles).s^{-1}}}  [5pt]
                  &\approx \SI{30}{\meter \per cycle}\end{aligned}

**b.** *If you measure a 10 degree phase shift, this value corresponds
to what distances?* [15pt]

The round trip distances are given by the following equation, where
:math:`\theta` is the phase shift and :math:`\lambda` is the wavelength:

.. math::

   \begin{aligned}
       2D &= \frac{\theta}{2\pi} \lambda  [8pt]
            &= \frac{\left(\frac{10}{180}\pi\right)}{2\pi}30  [8pt]
            &= \frac{10}{360}30  [8pt]
            &\approx 0.833  [8pt]
            &\approx 0.833 +30n ~\mbox{for}~ n=0,1,2,3...\end{aligned}

Since :math:`2D` represents the round trip, the distances :math:`D` are
given by,

.. math::

   \begin{aligned}
   D &\approx 0.417  [8pt]
       &\approx 0.417 + 15n ~\mbox{for}~ n=0,1,2,3... \end{aligned}

**c.** *What if the phase shift measurement has noise: zero mean with
standard deviation 0.1?* [15pt] We know the following

.. math::

   \begin{tikzcd}
       &
       \lambda = \SI{30}{\meter}  [-10pt]
       &
       \theta = 10^{\circ} \arrow[draw=none]{dr}[name=aux, anchor = center]{} 
       \arrow[rounded corners=7pt, dashed, blue, to path={ (\tikztostart.south)
                         |- (aux.center) \tikztonodes
                         -| (\tikztotarget.north)
                         }]{dr}
       \arrow[draw=none]{dl}[name=aux, anchor = center]{} 
       \arrow[rounded corners=7pt, dashed, blue, to path={ (\tikztostart.south)
                         |- (aux.center) \tikztonodes
                         -| (\tikztotarget.north)
                         }]{dl}     
       \theta = 10^{\circ}
       &&
       \theta = 10^{\circ} + 0.1^{\circ}  
       2D = \left(\frac{10^{\circ}}{360^{\circ}}\right) \SI{30}{\meter}\approx \SI{0.833}{\meter}
       \arrow[draw=none]{dr}[name=aux, anchor = center]{} 
       \arrow[rounded corners=7pt, dashed, blue, to path={ (\tikztostart.south)
                         |- (aux.center) \tikztonodes
                         -| (\tikztotarget.north)
                         }]{dr}
       &&
       2D = \left(\frac{10.1^{\circ}}{360^{\circ}}\right) \SI{30}{\meter} \approx \SI{0.842}{\meter}
       \arrow[draw=none]{dl}[name=aux, anchor = center]{} 
       \arrow[rounded corners=7pt, dashed, blue, to path={ (\tikztostart.south)
                         |- (aux.center) \tikztonodes
                         -| (\tikztotarget.north)
                         }]{dl}  
       &\ 
       \end{tikzcd}

Difference in :math:`2D` with :math:`0.1^{\circ}` difference in
:math:`\theta`

.. math:: \SI{0.842}{\meter} - \SI{0.833}{\meter} \approx \SI{0.00833}{\meter}

\ [10pt]

:math:`\therefore` The standard deviation in the measured distance for
:math:`2D` is :math:`0.00833` meters [15pt]

**d.** *How does one get a good estimate of position if the ranges to be
measured are from 20 meters to 250 meters?* [15pt] Pick a second
frequency that is relatively prime to :math:`\SI{10}{\mega\hertz}`

.. math::

   \begin{aligned}
   f_{1} &= \SI{10e6}{(cycles).s^{-1}}  [10pt]
   f_{2} &= \SI{7e6}{(cycles).s^{-1}}\end{aligned}

Solve for the wavelength :math:`\lambda` for both frequencies

.. math::

   \begin{aligned}
    \lambda_{f_{1}} &= \frac{c}{f_{1}}  
                  &\approx \frac{\SI{3e8}{\meter\per\second}}{\SI{10e6}{(cycles).s^{-1}}}  [5pt]
                  &\approx \SI{30}{\meter \per cycle}\end{aligned}

.. math::

   \begin{aligned}
    \lambda_{f_{2}} &= \frac{c}{f_{2}}  
                  &\approx \frac{\SI{3e8}{\meter\per\second}}{\SI{7e6}{(cycles).s^{-1}}}  [5pt]
                  &\approx \SI{42.8}{\meter \per cycle}\end{aligned}

Solving for the distance with :math:`f_{1}` and a phase shift of
:math:`\theta_{1}` gives us:

.. math::

   \begin{aligned}
       2D &= \frac{\theta_{1}}{2\pi} \lambda_{f_{1}}  [8pt]
            &= \frac{\theta_{1}}{2\pi}(30)  [8pt]
   &\qquad \Big\Downarrow  [8pt]
       2D &\approx \frac{\theta_{1}}{2\pi}(30) +30n ~\mbox{for}~ n=0,1,2,3...\end{aligned}

Solving for the distance with :math:`f_{2}` and a phase shift of
:math:`\theta_{2}` gives us:

.. math::

   \begin{aligned}
      \quad\ 2D &= \frac{\theta_{2}}{2\pi} \lambda_{f_{2}}  [8pt]
            &= \frac{\theta_{2}}{2\pi}(42.8)  [8pt]
   &\qquad \Big\Downarrow  [8pt]
       2D &\approx \frac{\theta_{2}}{2\pi}(42.8) +42.8m ~\mbox{for}~ m=0,1,2,3...\end{aligned}

Put the equations together to set up for solving for :math:`m` and
:math:`n`:

.. math::

   \begin{aligned}
   (30)\cfrac{\theta_{1}}{2\pi}+ 30m &= (42.8)\cfrac{\theta_{2}}{2\pi} + 42.8n  \end{aligned}

We need to find integer values for :math:`m` and :math:`n` to solve this
equation. This can be done with a simple program that permutes the
possible combinations of the two variables.

Write a Python function to simulate a LIDAR. The simulated LIDAR will
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
Appendix \ `[section:imagemaps] <#section:imagemaps>`__ shows how one
may generate a map in a bit map editor like GIMP and then export in a
plain text format which is easily read into a Python (Numpy) array.
[Although you can fill the grid by a python function which sets the
values, using the bit map editor will be much easier in the long run.]

.. raw:: latex

   \Closesolutionfile{Answer}

.. [1]
   LEDs have a variety of operating specs and you have to read the
   datasheet to find out about the specific voltage-current properties.
   Normally one is given an operating range and one must work out a
   suitable way to power the diode. For example, assume we have and LED
   which operates in the 3-6 volt range and targeted current level is
   20mA. If we select :math:`V = 5`, then the resistor should be
   :math:`R = V/I = 5/.02 = 250`. Since 250 is not a standard value, we
   select the closest available resistor value which is :math:`R =270`
   ohms.

.. [2]
   One would assume that natural beacons are used by animals as well

.. |image| image:: math/graddescent
.. |image| image:: math/hough
.. |image| image:: math/hough1
.. |image| image:: math/hough2
.. |a) Switch and associated circuit. b) Generated signal. [switchdebounce1]| image:: circuit/ckt1
.. |a) Switch and associated circuit. b) Generated signal. [switchdebounce1]| image:: circuit/problem
.. |a) Basic debounce hardware. b) Signal produced. [switchdebounce2]| image:: circuit/ckt2
.. |a) Basic debounce hardware. b) Signal produced. [switchdebounce2]| image:: circuit/problem2
.. |Standard hardware approach to debounce. [switchdebounce3]| image:: circuit/ckt3
.. |Standard hardware approach to debounce. [switchdebounce3]| image:: circuit/problem3
.. |image| image:: sensor/lidardetails2
.. |image| image:: sensor/lidarhardware

