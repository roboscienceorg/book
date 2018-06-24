Problems
--------

.. raw:: latex

   \setcounter{Exc}{0}

]

What is the difference between *effect* and *affect*?

What is the difference between accuracy and precision?

Define domain and range.

Is the following constraint holonomic:
:math:`\dot{x}_2\sin(x_1) + x_2 \cos(x_1)\dot{x}_1 = 1`.

:math:`x_2\sin(x_1) = t`

Sketch the workspace of a two-link manipulator centered at the origin
with :math:`a_1 = 15` and :math:`a_2 = 10`.

Assume that you have a two link planar manipulator. :math:`\theta_1` is
the angle between the x axis (measured clockwise as positive) and the
first link arm. :math:`\theta_2` is the angle between the second link
arm and the first link arm (again measured clockwise as positive). Given
the length of the first link :math:`a_1 = 12` and the second link
:math:`a_2 = 7` solve the following:

#. If :math:`\theta_1 = 45^\circ`, :math:`\theta_2 = 45^\circ`, find
   :math:`x` and :math:`y`.

#. If :math:`\theta_1 = 45^\circ`, :math:`\theta_2 = 45^\circ`,
   :math:`d\theta_1/dt = 5^\circ s^{-1}`,
   :math:`d\theta_2/dt = 10^\circ s^{-1}` find :math:`dx/dt` and
   :math:`dy/dt`.

#. If :math:`x = 12`, :math:`y = 14`, find :math:`\theta_1` and
   :math:`\theta_2`

Assume that you have a two link planar manipulator. :math:`\theta_1` is
the angle between the x axis (measured clockwise as positive) and the
first link arm. :math:`\theta_2` is the angle between the second link
arm and the first link arm (again measured clockwise as positive). Due
to servo limitations: :math:`-100^\circ < \theta_1 < 100^\circ`,
:math:`-150^\circ < \theta_2 < 150^\circ`. Also assume the first link is
20cm long and the second link is 15cm.

#. What is the configuration space?

#. What is the workspace?

Find the forward velocity kinematics equations for the two link
manipulator.

Assume that you have a two link manipulator that is operating in the
vertical plane :math:`x-z`. Attach the base to a rotational joint so the
arm rotates around the :math:`z` axis. [problem:twolink] See
Figure \ `[fig:two-link] <#fig:two-link>`__.

.. figure:: kinematics/twolinkalt2
   :alt: Two link manipulator in
   problem \ `[problem:twolink] <#problem:twolink>`__.[fig:two-link]

   Two link manipulator in
   problem \ `[problem:twolink] <#problem:twolink>`__.[fig:two-link]

#. Find the position of the end effector as a function of joint angles.

#. Find the inverse kinematic formula.

**A)** To find the position of the end effector as a function of joint
angles we need only to convert equation 2.1 in the text to polar
cylindrical coordinates. That is, y becomes z and x becomes r,
yielding:\ 

.. math::

   \label{two_link_eq}\begin{bmatrix}
       r\\
       z
   \end{bmatrix}
   =
   \begin{bmatrix}
       a_2\cos(\theta_1 + \theta_2) + a_1\cos(\theta_1) \\
       a_2\sin(\theta_1 + \theta_2) + a_1\sin(\theta_1)
   \end{bmatrix}

Math tells us that we may convert between cylindrical coordinates to
Cartesian using the formula:

.. math::

   \begin{bmatrix}
       x\\
       y
   \end{bmatrix}
   =
   \begin{bmatrix}
       r\cos(\theta) \\
       r\sin(\theta)
   \end{bmatrix}

Substitution into equation `[two_link_eq] <#two_link_eq>`__ gives us the
final equation for the 2 link manipulator on a pivot:

.. math::

   \label{two_link_pivot_eq}
   \boxed{
   \begin{bmatrix}
       x\\
       y\\
       z
   \end{bmatrix}
   =
   \begin{bmatrix}
       \cos(\theta_3)[a_2\cos(\theta_1 + \theta_2) + a_1\cos(\theta_1)] \\
       \sin(\theta_3)[a_2\cos(\theta_1 + \theta_2) + a_1\cos(\theta_1)] \\
       a_2\sin(\theta_1 + \theta_2) + a_1\sin(\theta_1)
   \end{bmatrix}
   }

**B)** To compute the inverse kinematics begin with the solutions from
the book while converting from Cartesian to polar cylindrical
coordinates (substituting r and z for x and y respectively):

.. math:: D = \frac{r^2 + z^2 - a_1^2 - a_2^2}{2a_{1}a_{2}}

.. math:: \theta_2 = \arctan\left(\frac{\pm\sqrt{1-D^2}}{D}\right)

.. math:: \theta_1 = \arctan\left(\frac{z}{r}\right) - \arctan\left(\frac{a_2\sin(\theta_2)}{a_1+a_2\cos(\theta_2)}\right)

Assume that you have a two link manipulator with :math:`a_1 = 15`\ cm
and :math:`a_2 = 15`\ cm and that the base of the manipulator is at the
origin of the coordinate system. Write a Python program to take the list
of workspace points and plug them into the inverse kinematics formulas
for the two link manipulator. Plot these points on a graph where
:math:`\theta_1` is the horizontal axis and :math:`\theta_2` is the
vertical axis. You will have to adjust some aspects to get a good
looking plot. (Scale factors etc.) Test your code on the workspace line
(a) :math:`x+y = 25`, :math:`x, y >0` and (b)
:math:`x = 10\cos (t) + 15`, :math:`y = 10\sin (t)` for
:math:`0 \leq t \leq \pi`. The point here is to see what the
configuration space curve looks like.

Using the code from the text as a starting point, we can code a NumPy
solution.

::

    from math import *
    import numpy as np
    import pylab as plt

    a1, a2 = 15, 15
    x = np.arange(0, 25, 0.2)   # a range of x values
    y = 25 - x    # an array of y values using this function
    denom = (2.0*a1*a2)
    numer = a1*a1+a2*a2

    d =  (x*x+y*y-numer)/denom
    t2 = np.arctan2(-np.sqrt(1.0-d*d),d)
    t1 = np.arctan2(y,x) - np.arctan2(a2*np.sin(t2),a1+a2*np.cos(t2))

    plt.plot(x,y,'r-')
    plt.savefig("hw1ch2p7a.pdf")
    plt.show()
    plt.plot(t1,t2,'b-')
    plt.savefig("hw1ch2p7b.pdf")
    plt.show()

| The plot for the line and the two link manipulator angles:
| |image| |image|
| Modify the function above (:math:`y-25-x`) and use the parametric form
  for a half circle.

::

    t = np.arange(0,pi,0.1)
    x = 10.0*np.cos (t) + 15.0
    y = 10.0*np.sin (t)

| and you obtain the following the circle shape and the angle values:
| |image| |image|

Assume that you have a two link manipulator with :math:`a_1 = 15`\ cm
and :math:`a_2 = 15`\ cm and that the base of the manipulator is at the
origin of the coordinate system. Write a two link manipulator location
program (Python). This program will take a list of angles and compute
the location of the end effector. Show how this program works with the
list of angles you generated in the previous problem. If the angle
inputs are generated by a square, the simulated robot arm’s end effector
should trace a square. Plot the end effector points. You need to plot
the input shape and the final shape to see if your code is correct. You
will need to use the previous problem for this problem. Demonstrate your
code to trace out the four segments which form the square with endpoints
(5,0), (5, 15), (20, 15), (20,0).

The text has the forward kinematics formulas and some example code.
Using that code as a starting point, we first need to find the angle
list for the square with endpoints (5,0), (5, 15), (20, 15), (20,0).
This can be done using the code from the last problem. Then we take the
angle list and place into the forward kinematics. The code for the
square is provided (the plot commands are removed for space). The list
of commands from z to zero are a bunch of arrays which will be appended
together to form the list of points for the square.

::

    from math import *
    import numpy as np
    import pylab as plt

    a1, a2 = 15, 15
    z = np.arange(0,15, 0.1)  # range points from 0 to 15
    z5 = z+5  # a range of points from 5 to 20
    z20 = 20-z  # a range of points from 20 to 5 (decreasing)
    z15 = 15-z  # a range of points from 15 to 0 (decreasing)
    c5 = 5*np.ones(z.size)  # an arrray of 5's (same size as z)
    c15 = c5+10.0    # an arrray of 15's (same size as z)
    c20 = c5+15.0    # an arrray of 20's (same size as z)
    zero = np.zeros(z.size)     # an arrray of 0's (same size as z)

    x = np.append(c5, z5)   ##  Glue the arrays above to produce
    x = np.append(x, c20)   ##  a list of points that take us around 
    x = np.append(x,z20)    ##  the square.
    y = np.append(z, c15)   ##  Same for the y values
    y = np.append(y,z15)
    y = np.append(y,zero)

    denom = (2.0*a1*a2)
    numer = a1*a1+a2*a2
    d =  (x*x+y*y-numer)/denom
    t2 = np.arctan2(-np.sqrt(1.0-d*d),d)
    t1 = np.arctan2(y,x) - np.arctan2(a2*np.sin(t2),a1+a2*np.cos(t2))

    xout = a2*np.cos(t1+t2) + a1*np.cos(t1)
    yout = a2*np.sin(t1+t2) + a1*np.sin(t1)

| The plots of the resulting line and half circle:
| |image| |image|
| The plot of the original square, the angles for the square and the
  square drawn by the two link kinematics:
| |image| |image|
| |image|

Typos can creep up in textbooks, papers and reference materials. How
would test the accuracy of the formulas given in equations
`[paralleltwolinkforward] <#paralleltwolinkforward>`__ and
`[paralleltwolinkIK] <#paralleltwolinkIK>`__? Discuss.

The first test would be to see if the forward and inverse kinematics are
actually inverses. This was demonstrated in the previous problems. We
started with (x,y) points (the square for example). The we used the
inverse kinematics to find the theta angles. Those angles were plugged
into the forward kinematics and the original figure (original (x,y)’s)
were recovered. [One should also start with thetas, find (x,y)’s and
then recover the thetas to be complete.] So, have shown that these two
are inverses, but not that they actually related to the problem at hand.
For example, :math:`f(x) = x^2` and :math:`g(x)=\sqrt{x}` are inverses
but not at all related to the manipulator. For this manipulator, both
forward and inverse kinematics were derived from the diagram and not
from one another. So, they are independent derivations which reduces our
concern. A careful diagram with accurate lengths and angles can also
help with our confidence. A careful analysis of the mathematics of the
derivation is the essential ingredient for belief in accuracy. The next
two problems walk through the derivation.

Find the forward velocity kinematics equations for the parallel two link
manipulator.

Derive the formula for
equation \ `[paralleltwolinkforward] <#paralleltwolinkforward>`__:

.. math:: (x,y) = \left( \frac{a+c}{2} + \frac{v (b-d)}{u} , \frac{b+d}{2} + \frac{v (c-a)}{u} \right)

Hint: define the segment from :math:`(a,b)` to :math:`(c,d)` as
:math:`B` (the base of the triangle), and :math:`\vec{A}` as a vector
which is a perpendicular to :math:`B`, see
Figure \ `[Fig:paralleltwolink3] <#Fig:paralleltwolink3>`__.

.. raw:: latex

   \centering

.. figure:: configuration/2dDelta3
   :alt: Extraction of the isosceles triangle. [Fig:paralleltwolink3]

   Extraction of the isosceles triangle. [Fig:paralleltwolink3]

Define the horizontal as the x-axis and the vertical (down) as the
y-axis and the origin be at the center of the base link :math:`L_0` as
shown:

|image|

The location of the point :math:`(a,b)` and :math:`(c,d)` can be found
using the angles

.. math:: (a,b) = \left( -L_0/2 - L_1 \cos(\theta_1) , L_1\sin(\theta_1) \right) , \quad   (c,d) = \left( L_0/2 + L_1 \cos(\theta_2) , L_1\sin(\theta_2) \right)

The distance between the points :math:`(a,b)` and :math:`(c,d)` is
:math:`u = \sqrt{(a-c)^2+(b-d)^2}` and the location of the point B is
given by

.. math:: \left(\frac{a+c}{2}, \frac{b+d}{2}\right).

The vector from :math:`(a,b)` to :math:`(c,d)` is
:math:`\langle c-a, d - b \rangle`. The length of this vector is
:math:`u`. To get from B to :math:`(x,y)` we must travel in the
direction which is perpendicular to :math:`\langle c-a, d - b \rangle`:
:math:`\langle b-d, c-a\rangle`.

|image|

To be a direction vector it should be normalized by dividing by its
length:

.. math:: B^\perp = \left< \frac{b-d}{u}, \frac{c-a}{u}\right> .

\ Lastly we need to determine how far to travel in this direction,
:math:`v`: :math:`A = vB^\perp`. This can be found by the Pythagorean
theorem: :math:`L_2^2 = (u/2)^2 + v^2`. Solving for :math:`v`:
:math:`v = \sqrt{L_2^2-u^2/4}`. :math:`v` is the distance we travel down
and so we gain the displacement vector:

.. math:: A = \left< \frac{v(b-d)}{u}, \frac{v(c-a)}{u}\right> .

Add the displacement vector to B and we gain :math:`(x,y)`:

.. math:: (x,y) = \left( \frac{a+c}{2} + \frac{v (b-d)}{u} , \frac{b+d}{2} + \frac{v (c-a)}{u} \right)

Derive the formulas for the parallel two link manipulator inverse
kinematics given in
equations \ `[paralleltwolinkIK] <#paralleltwolinkIK>`__. Hint:
Figure \ `[Fig:paralleltwolinkIK] <#Fig:paralleltwolinkIK>`__.

.. raw:: latex

   \centering

.. figure:: configuration/2dDelta4
   :alt: Parallel Two Link Inverse Kinematics variables
   [Fig:paralleltwolinkIK]

   Parallel Two Link Inverse Kinematics variables
   [Fig:paralleltwolinkIK]

Using the diagram for labels, we are given :math:`(x,y)`, :math:`L_0`,
:math:`L_1` and :math:`L_2`.

|image|

We also know the attachment points for the arms indicated by the green
dots. These are :math:`(-L_0/2,0)` on the left and :math:`(L_0/2,0)` on
the right. This allows one to compute the distances for the segments
:math:`G` and :math:`H`.

.. math:: \| H \| = \sqrt{(x+L_0/2)^2 + y^2}, \quad \| G \| = \sqrt{(x-L_0/2)^2 + y^2}

Knowing the three sides of a triangle allows one to compute interior
angles. So, we can determine :math:`\alpha`, :math:`\beta`,
:math:`\gamma` and :math:`\eta` using the law of cosines.

.. math:: \alpha = \cos^{-1} \frac{G^2 + L_0^2 - H^2 }{2GL_0}, \quad \quad \beta = \cos^{-1} \frac{H^2 + L_0^2 - G^2 }{2HL_0},

.. math:: \gamma = \cos^{-1} \frac{G^2 + L_1^2 - L_2^2 }{2GL_1},\quad \quad \eta =  \cos^{-1} \frac{H^2 + L_1^2 - L_2^2 }{2HL_1}.

One also recalls that the angles for the line sum to :math:`\pi` and we
obtain:

.. math:: \theta_1  = \pi - \beta - \eta , \quad \quad \theta_2 = \pi - \alpha - \gamma

Assume that you have a parallel two link manipulator with
:math:`L_0 = 10`\ cm, :math:`L_1 = 15`\ cm and :math:`L_2 = 20`\ cm.
Write a Python program to take the list of workspace points given and
plug them into the inverse kinematics formulas for the two link
manipulator. Plot these points on a graph where :math:`\theta_1` is the
horizontal axis and :math:`\theta_2` is the vertical axis. As above, you
will have to adjust some aspects to get a good looking plot. The point
here is to see what the configuration space curve looks like. The
workspace points are the list of points for the rectangle with corners
(-5, 18), (5, 18), (5, 27), (-5,27). Use 10 points in each side of the
rectangle.

For this problem we can generate the square by appending linspace
arrays. Those points are placed into the inverse kinematics formula and
the link arm angles are produced. The bottom of the file plots the data.

::

    from math import *
    import numpy as np
    import pylab as plt

    L0, L1, L2 = 10, 15, 20

    x = np.append(np.linspace(-5,5, 10), 5*np.ones(10))
    x = np.append(x, np.linspace(5,-5, 10))
    x = np.append( x,-5.0*np.ones(10))
    y = np.append(18*np.ones(10), np.linspace(18,27,10))
    y = np.append(y,27*np.ones(10))
    y = np.append(y,np.linspace(27,18,10))

    G = np.sqrt((x-L0/2.0)*(x-L0/2.0)+y*y)
    H = np.sqrt((x+L0/2.0)*(x+L0/2.0)+y*y)
    alpha = np.arccos((G*G + L0*L0 - H*H)/(2.0*G*L0))
    beta = np.arccos((H*H + L0*L0 - G*G)/(2.0*H*L0))
    gamma = np.arccos((G*G + L1*L1 - L2*L2)/(2.0*G*L1))
    eta = np.arccos((H*H + L1*L1 - L2*L2)/(2.0*H*L1))
    th1 = pi - beta - eta
    th2 = pi - alpha - gamma

    plt.gca().set_ylim([-1,max(y)+3])
    plt.gca().set_xlim([min(x)-11,max(x)+11])
    plt.gca().invert_yaxis()
    plt.scatter(x,y, color='#ADD8E6', marker='o')
    plt.axvline(linewidth=1.3, color = 'green')
    plt.axhline(linewidth=1.3, color = 'green')
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.show()

    plt.scatter(th1,th2, color= '#ADD8E6', marker='o')
    plt.axvline(linewidth=1.3, color = 'green')
    plt.axhline(linewidth=1.3, color = 'green')
    plt.xlabel("Theta 1")
    plt.ylabel("Theta 2")
    plt.xticks([0, pi/6, pi/4, pi/3, pi/2],['$0$', r'$\frac{\pi}{6}$', r'$\frac{\pi}{4}$', r'$\frac{\pi}{3}$', r'$\frac{\pi}{2}$'])
    plt.yticks([0, pi/6, pi/4, pi/3, pi/2],['$0$', r'$\frac{\pi}{6}$', r'$\frac{\pi}{4}$', r'$\frac{\pi}{3}$', r'$\frac{\pi}{2}$'])
    plt.grid()
    plt.show()

| The curves:
| |image|
| |image|

Assume that you have a parallel two link manipulator with
:math:`L_0 = 10`\ cm, :math:`L_1 = 15`\ cm and :math:`L_2 = 20`\ cm.
Write a Python program that will take a list of angles and compute the
location of the end effector. Show how this program works with the list
of angles you generated in the previous problem. [If the angle inputs
are generated by a rectangle, the simulated robot arm’s end effector
should trace a rectangle.] Plot the end effector points. You will need
to use the previous problem for this problem.

Adding to the solution above we can append the code and simulate the
manipulator drawing the rectangle.

::

    a = -L1*np.cos(th1) - L0/2.0
    b = L1*np.sin(th1)
    c = L1*np.cos(th2) + L0/2.0
    d = L1*np.sin(th2)

    dx = c-a
    dy = b-d
    u = np.sqrt(dx*dx+dy*dy)
    v = np.sqrt(L2*L2 - 0.25*u*u)
    xout = (a+c)/2.0 + v*dy/u
    yout = (b+d)/2.0 + v*dx/u

    plt.gca().set_ylim([-1,max(yout)+3])
    plt.gca().set_xlim([min(xout)-11,max(xout)+11])
    plt.gca().invert_yaxis()
    plt.scatter(x,y, color='#ADD8E6', marker='o')
    plt.axvline(linewidth=1.3, color = 'green')
    plt.axhline(linewidth=1.3, color = 'green')
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.show()

| The curve appears as:
| |image|

[Ex:HalfDisk] Using Numpy and the linspace command, build an array of
points for Figure \ `[Fig:shapeforhw] <#Fig:shapeforhw>`__. The top is
given by :math:`(x-10)^2 + (y-8)^2 = 25` and the bottom is the line
segment along :math:`y=8`. Traverse the figure starting at the right
corner, going counter clockwise (circle first) and ending on the line
segment. Check this with the Python plot command. Show the result.

.. raw:: latex

   \centering

.. figure:: configuration/halfcircle
   :alt: Half disk for Ex \ `[Ex:HalfDisk] <#Ex:HalfDisk>`__
   [Fig:shapeforhw]

   Half disk for Ex \ `[Ex:HalfDisk] <#Ex:HalfDisk>`__ [Fig:shapeforhw]

We will use the same “array appending” approach as in previous problems.

::

    from math import *
    import numpy as np
    import pylab as plt

    a1, a2 = 15, 15
    t = np.arange(0,pi,0.1)
    x1 = 5.0*np.cos (t) + 10.0
    y1 = 5.0*np.sin (t) + 8.0
    x2 = np.arange(5,15,0.1)
    y2 = 8.0*np.ones(x2.size)
    x = np.append(x1,x2)
    y = np.append(y1,y2)
    plt.xlim([0,20])
    plt.ylim([0,20])
    plt.plot(x,y,'g-')
    plt.savefig("hw1ch2p14.pdf")
    plt.show()

|image|

Is the differential drive motion model given by
Equations \ `[eqn:DDequationsTerms] <#eqn:DDequationsTerms>`__
holonomic? Why or why not?

When inflating an obstacle, how much do you inflate it by?

Find the rotation matrix that will rotate clockwise by :math:`30^\circ`.

Vector review

#. Given the vector :math:`<1, 2>`, rotate this by 37 degrees
   (positive),

#. If an axle is rotated off of the x-axis by 64 degrees, what is the
   vector that is in-line (parallel) the the axle?

#. What is the projection of :math:`<3,1>` onto the axle direction in
   the previous part?

Show that the inverse rotation matrix is the same matrix as replacing
:math:`\theta` by :math:`-\theta`.

The rotation matrix is

.. math::

   R(\theta) = 
   \begin{pmatrix}
   \cos\theta & -\sin\theta \\[2mm]
   \sin\theta & \cos\theta 
   \end{pmatrix}

Replacing :math:`\theta` by :math:`-\theta` and then using that
:math:`\cos` is even and :math:`\sin` is odd we have

.. math::

   R(-\theta) = 
   \begin{pmatrix}
   \cos(-\theta) & -\sin(-\theta) \\[2mm]
   \sin(-\theta) & \cos(-\theta)
   \end{pmatrix}
   = 
   \begin{pmatrix}
   \cos(\theta) & \sin(\theta) \\[2mm]
   -\sin(\theta) & \cos(\theta)
   \end{pmatrix}

Multiply :math:`R(\theta)` times :math:`R(-\theta)`:

.. math::

   R(\theta)R(-\theta)=
   \begin{pmatrix}
   \cos\theta & -\sin\theta \\[2mm]
   \sin\theta & \cos\theta 
   \end{pmatrix}
   \begin{pmatrix}
   \cos(\theta) & \sin(\theta) \\[2mm]
   -\sin(\theta) & \cos(\theta)
   \end{pmatrix}

.. math::

   = 
   \begin{pmatrix}
   \cos^2(\theta) + \sin^2(\theta) & \cos(\theta)\sin(\theta) - \cos(\theta)\sin(\theta)\\[2mm]
   \cos(\theta)\sin(\theta) - \cos(\theta)\sin(\theta) & \cos^1(\theta) + \sin^2(\theta)
   \end{pmatrix}
   = 
   \begin{pmatrix}
   1 & 0 \\[2mm]
   0 & 1
   \end{pmatrix}

We have the same result for :math:`R(-\theta)R(\theta)`, so this shows
that :math:`R(\theta)^{-1} = R(-\theta)`.

Assume that your differential drive robot has 10 cm diameter wheels and
a total axle length (wheel to wheel) of 20 cms. If both wheels are
turning at 0.8 revolutions per second, what is the speed of the robot.
[basicddhw]

The orientation does not matter here so we can assume the robot is
pointed in the direction of the :math:`x`-axis. We use the equations in
the text, \ `[ddkinematicsmodel] <#ddkinematicsmodel>`__,

.. math::

   \begin{array}{l}
    \dot{x} = \frac{r}{2} (\dot{\phi_1}+\dot{\phi_2})\cos(\theta) \\[5mm]
   \dot{y} = \frac{r}{2} (\dot{\phi_1}+\dot{\phi_2})\sin(\theta) \\[5mm]
   \dot{\theta} = \frac{r}{2L} (\dot{\phi_1}-\dot{\phi_2}).
   \end{array}

With :math:`\theta = 0` we see that :math:`\dot{y} = 0` and
:math:`\dot{\theta}=0`. One should note that we are given 0.8
revolutions per second and this converts to :math:`0.8(2\pi) = 1.6\pi`
radians per second. Using the remaining (first) equation

.. math:: \dot{x} = \frac{r}{2} (\dot{\phi_1}+\dot{\phi_2})\cos(\theta) = (5/2) (2* 1.6\pi) \approx 25.13274 ~\mbox{cm/s}

Using the same robot as problem \ `[basicddhw] <#basicddhw>`__, but
where the left wheel is turning at 1.5 radians per second and the right
wheel is turning at 1.8 radians per second. Determine the linear
velocity and path of the robot. You may assume the initial pose is
(0,0,0) at :math:`t=0`.

The velocity

.. math:: v = \sqrt{ \dot{x}^2 + \dot{y}^2} = \frac{r}{2} |\dot{\phi_1}+\dot{\phi_2}| = (5/2) |1.5 + 1.8| = 8.25 ~\mbox{cm/s}

The path can be determined from first solving the :math:`\theta`
equation:
:math:`\dot{\theta} = \frac{r}{2L} (\dot{\phi_1}-\dot{\phi_2}) = (5/20)(0.3) = 0.075`,
:math:`\theta = 0.075 t`. Then plug this into the first two equations:

.. math::

   \begin{array}{l}
    \dot{x} = \frac{r}{2} (\dot{\phi_1}+\dot{\phi_2})\cos(\theta)  = 8.25 \cos( 0.075 t)  \\[5mm]
   \dot{y} = \frac{r}{2} (\dot{\phi_1}+\dot{\phi_2})\sin(\theta)  = 8.25 \sin( 0.075 t) .
   \end{array}

Integrating gives us

.. math::

   \begin{array}{l}
    x = 110\sin( 0.0375 t) + x_0\\[5mm]
   y = 110 \cos( 0.0375 t) + y_0 .
   \end{array}

At :math:`t=0`, we have :math:`x = y = 0`. So :math:`x_0 = 0` and
:math:`y_0 = -110`.

.. math:: x^2 + (y+110)^2 = (110\sin( 0.075 t))^2 + (110 \cos( 0.075 t) )^2 = 110^2

The path is a circle of radius 110 centered at (0,-110).

For the differential drive robot, let :math:`r=10`, :math:`L=15`,
:math:`\dot{\phi_1} = 0.9` :math:`\dot{\phi_2}= 1.2`.

#. What is the angular velocity of the robot?

#. What is the velocity vector for the robot when
   :math:`\theta = 45^\circ`?

**1** Solve for the angular velocity :math:`\dot{\theta}` of the
robot:\ 

.. math::

   \begin{aligned}
   \dot{\theta} &= \frac{r}{2L}\left(\dot{\phi_1}-\dot{\phi_2}\right)\\[15pt]
                        &= \frac{\SI{10}{\centi\meter}}{2(\SI{15}{\centi\meter})}\left(\SI{0.9}{\radian \per\second}-\SI{1.2}{\radian\per\second}\right)\\[15pt]
                        &= (0.33)(\SI{-0.3}{\radian\per\second})\\[15pt]
                        &= \SI{-0.099}{\radian\per\second}\end{aligned}

**2** Solve for the velocity vector of the robot given
:math:`\theta = 45^\circ`:

.. math::

   \begin{aligned}
   \dot{x} &= \frac{r}{2} (\dot{\phi_1}+\dot{\phi_2})\cos(\theta)\\[15pt]
                &= \frac{\SI{10}{\centi\meter}}{2} (\SI{0.9}{\radian\per\second}+\SI{1.2}{\radian \per\second})\cos\left(\frac{pi}{4}\right)\\[15pt]
                &= \SI{7.42}{\centi\meter\per\second}\end{aligned}

.. math::

   \begin{aligned}
   \dot{y} &= \frac{r}{2} (\dot{\phi_1}+\dot{\phi_2})\cos(\theta)\\[15pt]
                &= \frac{\SI{10}{\centi\meter}}{\SI{2}{\radian}} (\SI{0.9}{\radian \per\second}+\SI{1.2}{\radian \per\second})\sin\left(\frac{pi}{4}\right)\\[15pt]
                &= \SI{7.42}{\centi\meter \per\second}\end{aligned}

.. math::

   \begin{aligned}
   \begin{bmatrix} \dot{x}\\ \dot{y}\\ \end{bmatrix} &=  \begin{bmatrix} 7.42\\ 7.42\\ \end{bmatrix}\end{aligned}

Let :math:`r=10`, :math:`L=15`. If you program the robot to drive
straight and the robot traces out a circle of diameter 3 meters while
traveling 1 m/s, what are the two wheel speeds?

Say you have a differential drive robot that has an axle length of 30cm
and wheel diameter of 10cm. Find the angular velocity for the left and
right wheel if the robot is going to

#. Spin in place at a rate of 6 rpm (revolutions per min),

#. Drive a circle of radius 1 meter (measured center of circle to middle
   of axle) at 3 rpm,

#. Drive a straight line at 1 meter / min.

#. *Spin in place at a rate of 6 rpm (revolutions per min).* We can
   approach this problem using the general equations or track wheel
   motion. Using the general equations, to spin in place
   :math:`\dot{x} = \dot{y}=0`. It also must be the case that
   :math:`\dot{\phi_1} = -\dot{\phi_2}`. 6 revolutions per minute will
   be :math:`12\pi` radians per minute. Using the :math:`\dot{\theta}`
   equation we have
   :math:`\dot{\theta} = \frac{r}{2L} (\dot{\phi_1}-\dot{\phi_2}) = (5/30)(2\dot{\phi_1}) =
   (1/3) \dot{\phi_1}`. This gives :math:`12\pi = (1/3) \dot{\phi_1}` or
   :math:`\dot{\phi_1} = 36\pi`, :math:`\dot{\phi_2} =-36\pi`.

#. *Drive a circle of radius 1 meter (measured center of circle to
   middle of axle) at 3 rpm*. The outer wheel will trace a circle of
   radius 1.15m and the inner wheel of radius 0.85m. This gives a
   distance of :math:`2.3\pi`\ m and :math:`1.7\pi`\ m. At 3rpm, we have
   :math:`6.9\pi`\ m/min and :math:`5.1\pi`\ m/min. Converting to wheel
   rotation :math:`6.9\pi /(0.1 \pi) = 69` rev/min,
   :math:`5.1\pi/(0.1\pi) = 51`\ revs/min. These can be converted to
   rads/min: outside wheel is 433.5 rad/min, inside wheel is 320.4
   rads/min.

#. *Drive a straight line at 1 meter / min.* Just use the angular to
   linear velocity conversion (:math:`v = r\omega`):
   :math:`\dot{\phi} = 1/r = 1/0.05 = 20`.

| Given a differential drive robot starting from (0,0,0) find the final
  position when wheel velocities are given by:
| t=0 to t=5: :math:`\omega_1` = 2, :math:`\omega_2` = 2
| t=5 to t=6: :math:`\omega_1` = 3, :math:`\omega_2` = 4
| t=6 to t=10: :math:`\omega_1` = 1, :math:`\omega_2` = 2
| where D=10, L=16.

List the variables in the configuration space of a circular ground robot
that can drive around and use a telescopic arm with a rotational base,
lifting servo and elbow joint servo.

Although a circular robot would appear to be rotationally symmetric, the
robotic arm breaks that symmetry. The vehicle has three degrees of
freedom :math:`x`, :math:`y`, :math:`\theta`. The arm has the base and
two servos. So the arm has three degrees of freedom. The total is six
degrees of freedom.

[DDisnotHolonomic] Show that the differential drive kinematic equations
are non-holonomic constraints.

.. raw:: latex

   \Closesolutionfile{Answer}

.. [1]
   Like the office chair races in the hallway.

.. [2]
   Well, not all points, but a dense sample of points will do just fine.

.. [3]
   Well, this is true on one Tuesday afternoon a long time ago with one
   little comparison of some loop/math code. Your results may be very
   different.

.. [4]
   Under normal conditions this is true, however, icy roads will allow
   for much greater freedom of vehicle orientation and travel direction.

.. [5]
   Cavers will tell you that you can crawl through a vertical gap
   spanned by the distance of your thumb and your fifth (pinky) finger.
   For the average American, this is a very small gap.

.. |a) Workspace for the two link manipulator with equal link lengths. b) Workspace obstacle for the two link manipulator. [two-link-disk]| image:: configuration/twolinkconfig
.. |a) Workspace for the two link manipulator with equal link lengths. b) Workspace obstacle for the two link manipulator. [two-link-disk]| image:: configuration/twolinkobs
.. |Configuration domain and configuration topology which is a torus. [intro-config-axis]| image:: configuration/twolinkconfigdomain
.. |Configuration domain and configuration topology which is a torus. [intro-config-axis]| image:: configuration/twolinktorus
.. |Parallel Two Link (a) configuration space (b) with coordinates [Fig:paralleltwolink2]| image:: configuration/2dtwolinkconfigdomain
.. |Parallel Two Link (a) configuration space (b) with coordinates [Fig:paralleltwolink2]| image:: configuration/2dDelta2
.. |Velocity of axle induced by wheel velocities.[axlevelocity]| image:: motion/ddaxle
.. |Velocity of axle induced by wheel velocities.[axlevelocity]| image:: motion/ddforward
.. |Configuration space as a function of robot size. [Fig:RobotSize]| image:: planning/circle1
.. |Configuration space as a function of robot size. [Fig:RobotSize]| image:: planning/circle2
.. |image| image:: solutions/Terms/hw1ch2p7a
.. |image| image:: solutions/Terms/hw1ch2p7b
.. |image| image:: solutions/Terms/hw1ch2p7c
.. |image| image:: solutions/Terms/hw1ch2p7d
.. |image| image:: solutions/Terms/hw1ch2p8c
.. |image| image:: solutions/Terms/hw1ch2p8d
.. |image| image:: solutions/Terms/hw1ch2p8a
.. |image| image:: solutions/Terms/hw1ch2p8b
.. |image| image:: solutions/Terms/hw1ch2p8e
.. |image| image:: configuration/2dDelta2
.. |image| image:: configuration/2dDelta3
.. |image| image:: configuration/2dDelta4
.. |image| image:: solutions/Terms/hw1ch2p12a
.. |image| image:: solutions/Terms/hw1ch2p12b
.. |image| image:: solutions/Terms/hw1ch2p13
.. |image| image:: solutions/Terms/hw1ch2p14

