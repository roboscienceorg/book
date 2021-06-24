Location Sensors
-----------------

Beacons
~~~~~~~

While IMUs are amazing devices, they cannot give us accurate position
information. Estimating the position in the environment is essential for
navigation. One approach is to instrument the environment. An example of
this would be placing markers that the robot sensors can detect and
reliably interpret for location information. We will explore several of
these ideas with beacon systems as our guiding example.

:index:`Beacons` are probably the simplest approach to localization. A beacon is
any type of landmark with a known location. Natural beacons such as
stars, sun, moon, mountains, streams and other markers have been used
throughout human existence [#f2]_. Manmade beacons include towers, signs,
lighthouses and other marked locations.

Indoor beacon systems include using colored or IR lights, RFID tags,
ultrasonic transducers, QR codes, colored tags and other forms of
environmental instrumentation. Normally this means that the environment
is modified in some detectable manner. Similar approaches can be done
outdoors, but since the introduction of GPS, it has dominated the
localization approaches. :index:`GPS`, the :index:`Global Positioning System`, was
developed for the US military for their localization and navigation
requirements.

GPS uses signals from satellites to triangulate position. Conceptually
it is rather simple to use time of flight from four satellites to
exactly locate an object. The challenges are that the distances are
great, the speed of light is very high and the Earth is often in the
way. To address the line of sight requirement, 24 satellites with
several spares orbit the earth every 12 hours at an altitude of 20,190
km. They are arranged as four satellites in six planes offset by 55
degrees from the plane of the equator. Knowing the time of flight and
the speed of light, distance of the observer from the satellite can be
determined.

There are several challenges to be overcome. First is a precise
measurement of the time of flight. Time synchronization between
satellites and GPS receiver is essential. Secondly, a precise location
of the satellite is required. In addition one needs to deal with signal
quality and interference.

.. _`gpspng`:
.. figure:: SensorsFigures/gps.*
   :width: 50%
   :align: center

   Global Positioning System - GPS

Each satellite has an atomic clock for ultra-precise tracking of time.
They are monitored by ground stations. These ground stations also track
the location of the satellites. The ground station can perform the
location analysis and transmit the position estimate to the satellites.

A GPS receiver will grab a code from the satellite which has time stamp
data. Using this information, the distance between the satellite and the
receiver is computed. The clock on most receivers is not very accurate
so information from more than three satellites are required to adjust
for local clock errors. This allows for estimates to be accurate within
several meters.


.. figure:: SensorsFigures/dgps.*
   :width: 50%
   :align: center

   GPS with local correction.

Example
'''''''''

Assume that you have four beacon towers located in roughly a square over
a 10km x 10km patch of land. You place a coordinate system on the land
and measure the beacon locations. The locations in meters are B1 (0,0),
B2 (56, 9752), B3 (9126, 7797), B4 (9863, 218). If the beacons transmit
a packet with a time stamp, then a mobile system with an accurate clock
can determine its location in the instrumented area. Determine locations
if :math:`t_1 = 22793` ns, :math:`t_2 = 15930` ns, :math:`t_3 = 20817`
ns, :math:`t_4 =  29793` ns. The distances are found via :math:`d = ct`:
:math:`d_1 = 6838 m`, :math:`d_2 = 4779 m`, :math:`d_3 = 6245 m`,
:math:`d_4 = 8938 m`. So our object lies on a circle of distance
:math:`d_1` from beacon one and distance :math:`d_2` from beacon two,
etc.

One may intersect two circles to provide the location of the two
intersecting points and then proceed over all combinations:

.. math:: (x-a_i)^2 + (y-b_i)^2 = r_i^2 , \quad (x-a_j)^2 + (y-b_j)^2 = r_j^2 .

The algebra can be simplified by expanding each circle equation

.. math:: x^2 - 2a_ix + a_i^2 + y^2 - 2b_iy + b_i^2 = r_i^2 ,
   \quad x^2 - 2a_jx + a_j^2 + y^2 - 2b_jy + b_j^2 = r_j^2

and computing a difference

.. math:: 2(a_j-a_i)x + 2(b_j-b_i)y + a_i^2-a_j^2 + b_i^2-b_j^2 = r_i^2 - r_j^2 .

Using three circle equations, you can obtain two linear equations

.. math:: 2(a_j-a_i)x + 2(b_j-b_i)y  = r_i^2 - r_j^2 - a_i^2 + a_j^2 - b_i^2 + b_j^2

.. math:: 2(a_k-a_i)x + 2(b_k-b_i)y = r_i^2 - r_k^2  - a_i^2 + a_k^2 - b_i^2  + b_k^2  .

In a noise free world, the solution would be where the circles intersect
exactly such as seen in
:numref:`fig:exactintersection`. But this
does not happen due to noise and sensor inaccuracies. The circles do not
intersect as shown in
:numref:`fig:inexactintersection`.

.. _`fig:exactintersection`:
.. figure:: SensorsFigures/hough1.*
   :width: 50%
   :align: center

   Exact intersection of three circles.


.. _`fig:inexactintersection`:
.. figure:: SensorsFigures/hough2.*
   :width: 50%
   :align: center

   Non-intersection of three circles.

One way to approach this problem is to cast into a optimization problem.
If we are a certain distance (in two dimensions) away from a beacon,
then we lie on a circle where the radius of the circle is the distance
away from the beacon. The object must lie on all of the circles which
are have the given distance.

We would like to minimize the distance that our selected point
:math:`(x,y)` lies off of each circle. The distance the point misses the
circle from B1 is :math:`|\sqrt{x^2 + y^2} - 6838|`. From the individual
errors, we can form the total error function by summing up the
individual error terms.

.. math::

   \begin{array}{ll}
   E = & \quad  |\sqrt{x^2 + y^2} - 6838|
    + |\sqrt{(x-56)^2 + (y-9752)^2} - 4779|     \\[3mm]
   & + |\sqrt{(x-9126)^2  + (y-7797)^2} - 6245|
    + |\sqrt{(x-9863)^2 + (y-218)^2} - 8938|  .
   \end{array}

If :math:`E=0`, then we are at the :math:`(x,y)` point that matches all
four distances.

.. _`fig:radialerror`:
.. figure:: SensorsFigures/circerror.*
   :width: 50%
   :align: center

   Radial error function.

Since there is measurement error we will have in practice that
:math:`E > 0`, so we are looking for the minimum value for :math:`E`. A
traditional multivariate calculus approach is to take partial
derivatives and set them to zero. This produces a system of nonlinear
equations which must be solved numerically. It is the square root that
gives complicated algebra as well as division by zero errors.

One additional problem is the absolute value. The derivative of the
absolute :math:`(d/dx) |x| = x /|x|` is the sign function,
:math:`sign(x)` (not :math:`\sin ()`). This is not continuous and will
wreak havoc on some optimization codes. In addition, combinations of
absolute values can lead to non-single point minimums although unlikely
in our case. To address these issues, we change our error function by
replacing the absolute value with a square. Indeed this will change the
function but will allow for unique mins. Note that for a single
component element of the expression, :math:`|f(x,y)|` the minimum will
not move when we move to :math:`[f(x,y)]^2`. For sums,
:math:`|f(x,y) + g(x,y)|` this is no longer true, but not necessarily a
bad result.

There are several directions we can head to find the extremal. Many
variants of Newton’s Method are available. One can imagine custom search
algorithms. For simplicity we will leave those approaches to text’s on
numerical optimization and we will use gradient descent. Recall the
definition of the gradient is
:math:`\nabla E = \left< \partial E / \partial x, \partial E / \partial y \right>`.
The updated function to minimize is

.. math::

   \begin{array}{ll}
   E \quad = & \quad  \left(\sqrt{x^2 + y^2} - 6838\right)^2     \\[3mm]
    &+ \left(\sqrt{(x-56)^2 + (y-9752)^2} - 4779\right)^2    \\[3mm]
   & + \left(\sqrt{(x-9126)^2  + (y-7797)^2} - 6245\right)^2   \\[3mm]
   & + \left(\sqrt{(x-9863)^2 + (y-218)^2} - 8938\right)^2 .
   \end{array}

Since we are using a numerical method (gradient descent) and thus not an
exact method, it makes sense to use a numerical approach to computing
the partial derivatives. Recall that the approximation of the derivative
is

.. math:: \displaystyle \frac{\partial F}{\partial x_k} \approx \frac{F(x_1, x_2, \dots , x_k + \Delta x, \dots , x_n) - F(x_1, x_2, \dots  , x_n)}{\Delta x}

for small :math:`\Delta x`. For each item in the gradient vector, you
can estimate the derivative. This requires two function evaluations, a
difference and a multiply. [Precompute :math:`1/\Delta x` and then
multiply.] For the algorithm, if you have rough guess as to location,
you can use this for your initial guess for gradient descent. Otherwise
you can pick the center or a random point in the search region.

We can use the gradient descent method to find the solution. Set
  :math:`x_0 = 5000`, :math:`y_0=5000`, :math:`k=0`, :math:`t=1`:

While (:math:`t > t_0`)

-  :math:`u = \nabla E (x_k, y_k) /  \| \nabla E (x_k, y_k) \|`

-  :math:`(a,b) = (x_k,y_k) - t u`

-  while :math:`\left[ E(a,b) > E(x_k,y_k)\right]`

   -  :math:`t = t/2`

   -  :math:`(a,b) = (x_k,y_k) - t u`

-  :math:`k=k+1`

-  :math:`(x_k,y_k) = (a,b)`

::

    from math import *
    # The function definition
    def funct(x,y):
       E = (sqrt(x**2 + y**2) - 6838)**2 \
       + (sqrt((x-56)**2 + (y-9752)**2) - 4779)**2 \
       + (sqrt((x-9126)**2  + (y-7797)**2) - 6245)**2 \
       + (sqrt((x-9863)**2 + (y-218)**2) - 8938)**2
       return E

::

    # The numerical gradient approximation
    def grad(x,y):
        delta = 0.0001
        E = funct(x,y)
        E1 = funct(x+delta,y)
        E2 = funct(x,y+delta)
        dEx = (E1-E)/delta
        dEy = (E2-E)/delta
        return dEx, dEy

::

    # The size of the vector
    def norm(r,s):
        return sqrt(r*r+s*s)

    # The step in the direction (u,v)
    def step(x,y, u,v,t):
        a = x - t*u
        b = y - t*v
        return a, b


::

    # Globals
    x = 5000
    y = 5000
    t = 10.0
    tsmall = 0.00001

    # The descent algorithm
    while (t > tsmall):
        dx, dy = grad(x,y)
        size = norm(dx,dy)
        u = dx/size
        v = dy/size
        a,b = step(x,y,u,v,t)
        while (funct(a,b) > funct(x,y)):
            t = 0.5*t
            a,b = step(x,y,u,v,t)
        x,y = a,b

    print x, y



.. code-block:: julia

    # The function definition
    function funct(x,y)
       E = (sqrt(x^2 + y^2) - 6838)^2  +
       (sqrt((x-56)^2 + (y-9752)^2) - 4779)^2  +
       (sqrt((x-9126)^2  + (y-7797)^2) - 6245)^2 +
       (sqrt((x-9863)^2 + (y-218)^2) - 8938)^2
       return E
    end

.. code-block:: julia

    # The numerical gradient approximation
    function grad(x,y)
        delta = 0.0001
        E = funct(x,y)
        E1 = funct(x+delta,y)
        E2 = funct(x,y+delta)
        dEx = (E1-E)/delta
        dEy = (E2-E)/delta
        return dEx, dEy
    end

.. code-block:: julia

    # The size of the vector
    function norm(r,s)
        return sqrt(r*r+s*s)
    end

    # The step in the direction (u,v)
    function step(x,y, u,v,t)
        a = x - t*u
        b = y - t*v
        return a, b
    end  
    
.. code-block:: julia

    # Globals
    x = 5000
    y = 5000
    t = 10.0
    tsmall = 0.00001

    # The descent algorithm
    while (t > tsmall)
        dx, dy = grad(x,y)
        size = norm(dx,dy)
        u = dx/size
        v = dy/size
        a,b = step(x,y,u,v,t)
        while (funct(a,b) > funct(x,y))
            t = 0.5*t
            a,b = step(x,y,u,v,t)
        end
    
        x,y = a,b
    end

    println("x: ", x, " y: ", y)



.. figure:: SensorsFigures/graddescent.*
   :width: 50%
   :align: center

   Gradient Descent

The intersection point is :math:`x = 3120`, :math:`y = 6085`. Note that
this algorithm is not guaranteed to converge on the solution (the global
minimum). It can get trapped in local minima. To address this problem
you may re-run the algorithm with different random starting points.

There are plenty of other ways to treat this problem. An image
processing approach akin to the Hough Transform (with voting) would also
work. It is also possible to lay down a grid and then increment grid
cells for each circle that passes through. The cell with the largest
value is a candidate for the location. Starting with a course grid and
refining the grid is a way to produce a hierarchal method that can have
high accuracy but still be fast. See if you can come up with other
approaches to this example.

.. figure:: SensorsFigures/hough.*
   :width: 50%
   :align: center

   Hough Transform

.. rubric:: Footnote

.. [#f2] One would assume that natural beacons are used by animals as well
