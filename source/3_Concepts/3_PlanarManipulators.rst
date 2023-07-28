Simple Planar Manipulators
--------------------------

Serial Two Link Manipulator
~~~~~~~~~~~~~~~~~~~~~~~~~~~

The last few paragraphs have introduced lots of jargon. To understand
them, it helps to see them in action. The simple :index:`two link manipulator`,
:numref:`intro-two-link` is a good place to
start. Imagine a robotic arm that has two straight links with a rotary
joint at the base and a rotary joint connecting the two links. In
practice, these rotary joints would be run by motors or servos and
probably have some limits, but for now we will assume full
:math:`360^\circ` motion.

.. Owned by Roboscience

.. _`intro-two-link`:
.. figure:: TermsFigures/twolinkalt.*
   :width: 50%
   :align: center

   The two link manipulator.

The workspace that the arm operates inside is a disk,
:numref:`two-link-disk`. This is a two dimensional
workspace. The figure indicates the workspace in gray. It may also be
the case that there is something in the workspace, a workspace obstacle
indicated in red. This unit has two joints which define a two
dimensional configuration space,
:numref:`intro-config-axis`. The dimension of
the configuration space is the degrees of freedom, and so this has two
degrees of freedom. Since the joint is rotary and moving a full
:math:`360^\circ` degrees returns you to the same angle, the two
directions wrap back on themselves. This actually makes the
configuration space a two dimensional torus or “donut”.

.. Owned by Roboscience

.. _`two-link-disk`:
.. figure:: TermsFigures/twolinkconfigobs.*
   :width: 70%
   :align: center

   Two link manipulator: (a) Workspace with equal link lengths and (b) Workspace obstacle.


.. Owned by Roboscience

.. _`intro-config-axis`:
.. figure:: TermsFigures/twolinkconfigdomaintorus.*
   :width: 70%
   :align: center

   Configuration domain and configuration topology which is a torus.



We will illustrate what is meant by kinematics and inverse kinematics
using the two link manipulator. Forward kinematics will identify the
location of the end effector as a function of the joint angles,
:numref:`twolinklabeled`-(a). This is easily done
using a little trigonometry. First we find the location of
:math:`(\xi, \eta)` as a function of :math:`\theta_1` and the link
length :math:`a_1`, :numref:`twolinklabeled`-(b):

.. math:: \xi =  a_1 \cos \theta_1, \quad \eta = a_1 \sin \theta_1

.. Owned by Roboscience

.. _`twolinklabeled`:
.. figure:: TermsFigures/twolink2.*
   :width: 85%
   :align: center

   a) The two link manipulator with the links and joints labeled. b)
   Location of the middle joint.

The next link can be included with

.. math:: \Delta x =  a_2 \cos (\theta_1 + \theta_2), \quad \Delta y = a_2 \sin ( \theta_1 + \theta_2)

Note that :math:`x = \xi + \Delta x` and :math:`y = \eta + \Delta y`.
Combining the expressions, the forward kinematics are:

.. math::
   :label: twolinkforward

   \begin{matrix}
   x = a_2\cos (\theta_1+\theta_2) + a_1 \cos \theta_1 \\
   y = a_2 \sin (\theta_1 +\theta_2) + a_1\sin \theta_1
   \end{matrix}


As you move the servos in the system, you can change the angles
:math:`\theta_1` and :math:`\theta_2`. The
formula :eq:`twolinkforward` gives the location of
the end effector :math:`(x,y)` as a function of
:math:`(\theta_1, \theta_2)`. The values :math:`x`, :math:`y` live in
the workspace. The values :math:`\theta_1`, :math:`\theta_2` live in the
configuration space. This is a holonomic system. A common application is
to move the end effector along some path in the workspace. How does one
find the “path” in configuration space? Meaning how do we find the
values of :math:`\theta_1`, :math:`\theta_2` that give us the correct
:math:`x`, :math:`y` values? This requires inverting the kinematics
equations, hence the term inverse kinematics. The mathematics required
is some algebra and trigonometery for solving :math:`\theta_1`,
:math:`\theta_2` in terms of :math:`x`, :math:`y`.

To find the inverse kinematics formulas we must appeal to some
trigonometry (law of cosines):

.. math::
   :label: eqn:theta2step1

   x^2 + y^2 = a_1^2 + a_2^2 - 2a_1a_2 \cos (\pi - \theta_2).

Using :math:`\cos(\pi - \alpha) = -\cos(\alpha)`, we solve for
:math:`\cos` in Eqn :eq:`eqn:theta2step1`:

.. math:: \cos(\theta_2) = \frac{x^2 + y^2 - a_1^2 - a_2^2}{2a_1a_2 }\equiv D

Using a trig formula:

.. math:: \sin(\theta_2) = \pm \sqrt{1-D^2}

Dividing the sin and cos expressions to get tan and then inverting:

.. math:: \theta_2 = \tan^{-1}\frac{\pm\sqrt{1-D^2}}{D} = \mbox{atan2}(\pm\sqrt{1-D^2},D)

The +/-  gives the elbow up and elbow down solutions.  A source of errors arises with
the arctan or inverse tangent of the ratio.  The inverse function is multivalued and
calculators (as with most software) will return a single value known as the principle
value.   However, you may want one of the different values.  The problem normally
is that since :math:`-y/-x = y/x` the inverse tangent function will not know which
quadrant to select.  So it may hand you a value that is off by :math:`\pm \pi`.
We suggest that you use atan2 in your calculations
instead of atan which will isolate quadrant and also
avoid the divide by zero problem.  We will do the mathematics with :math:`\tan^{-1}`, but
keep our code using atan2.

.. Owned by Roboscience

.. _`twolinklabeled2`:
.. figure:: TermsFigures/twolink3.*
   :width: 40%
   :align: center

   The interior angles for the two link manipulator.

Continuing with the derivation,
from Figure :numref:`twolinklabeled2`, we have

.. math::
   :label: eqn:theta1step1

   \theta_1 = \phi - \gamma = \tan^{-1}\frac{y}{x} - \gamma .

If you look at the two dotted blue lines you can see that the line
opposite :math:`\gamma` has length :math:`a_2\sin \theta_2`. The segment
adjacent to :math:`\gamma` (blue solid and dotted lines) has length
:math:`a_1 + a_2\cos \theta_2`. Then

.. math:: \tan \gamma =  \frac{\mbox{Opposite}}{\mbox{Adjacent}} = \frac{a_2\sin \theta_2}{a_1 + a_2\cos\theta_2}

which gives us :math:`\gamma`:

.. math:: \gamma = \tan^{-1} \frac{a_2\sin \theta_2}{a_1 + a_2\cos\theta_2}.

Plug :math:`\gamma` into Eqn :eq:`eqn:theta1step1`
and we obtain

.. math:: \theta_1 = \tan^{-1}\frac{y}{x} - \tan^{-1} \frac{a_2\sin \theta_2}{a_1 + a_2\cos\theta_2}

Given the two link manipulator kinematic equations:

.. math::
   :label: TwoLinkKinematics

   \begin{matrix}
   x = a_2\cos (\theta_1+\theta_2) + a_1 \cos (\theta_1)\\
   y = a_2 \sin (\theta_1 +\theta_2) + a_1\sin (\theta_1)
   \end{matrix}

The inverse kinematics (IK) are

.. math:: D = \frac{x^2 + y^2 - a_1^2 - a_2^2}{2a_1a_2 }

.. math::
   :label: IKtwolink

   \theta_1 = \tan^{-1}\frac{y}{x} - \tan^{-1} \frac{a_2\sin \theta_2}{a_1 + a_2\cos\theta_2}, \quad\quad
   \theta_2 = \tan^{-1}\frac{\pm\sqrt{1-D^2}}{D}



Let :math:`a_1 = 15`, :math:`a_2 = 10`, :math:`x=10`, :math:`y=8`. Find
:math:`\theta_1` and :math:`\theta_2`:

#. :math:`D = (10^2 + 8^2 - 15^2-10^2)/(2*15*10) = -0.53667`

#. :math:`\theta_2 = \tan^{-1}(-\sqrt{1-(-0.53667)^2}/(-0.53667))\approx -2.137278`

#. :math:`\theta_1 = \tan^{-1}(8/10)-\tan^{-1}[(10\sin(-2.137278))/(15+ 10\cos(-2.137278))] \approx 1.394087`

| Check the answer:
| :math:`x = 10*\cos(1.394087-2.137278) + 15*\cos(1.394087) = 10.000`
| :math:`y = 10*\sin(1.394087-2.137278) + 15*\sin(1.394087) = 8.000`


Note that all angles in this text are in radians unless explicitly stated
as degrees.  This is to be consistent with standard math sources as well
as the default for most programming languages.
Be careful with arctan.  It can bite you.  Here is an example ...

Assume that :math:`(x,y) = (9,10)` and :math:`(a_1, a_2) = (15,15)`.
We compute

.. math::

   \begin{array}{l}
   D = \displaystyle  \frac{x^2 + y^2 - a_1^2 - a_2^2}{2a_1a_2 }
   = \displaystyle \frac{9^2 + 10^2 - 15^2 - 15^2}{2(15)(15) } = -0.5977777777777777 \\[4pt]
   \theta_2 = \tan^{-1}\displaystyle\frac{-\sqrt{1-D^2}}{D}  = 0.9300701118289644 \\[4pt]
   \theta_1 = \tan^{-1}\displaystyle\frac{y}{x} - \tan^{-1} \displaystyle\frac{a_2\sin \theta_2}{a_1 + a_2\cos\theta_2}
            = 0.3729461690939078
   \end{array}

Now check our answers ...

.. math::

   \begin{matrix}
   x = a_2\cos (\theta_1+\theta_2) + a_1 \cos \theta_1 = 17.93773762042545 \\
   y = a_2 \sin (\theta_1 +\theta_2) + a_1\sin \theta_1 = 19.9308195782505
   \end{matrix}

Not close.  What happened?  The first problem was that in :math:`\displaystyle\frac{-\sqrt{1-D^2}}{D}`
which is :math:`\displaystyle\frac{-0.801...}{-0.597...}`
becomes :math:`\displaystyle\frac{0.801...}{0.597...}`
and then atan returns a quadrant I angle of 0.930070... .  We needed
:math:`\theta_2 = 0.93007 + \pi = 4.0716`.  Then you get :math:`\theta_1 = 1.94374...`.

.. math::

   \begin{matrix}
   x = a_2\cos (\theta_1+\theta_2) + a_1 \cos \theta_1 = 9.0 \\
   y = a_2 \sin (\theta_1 +\theta_2) + a_1\sin \theta_1 = 9.99999 \approx 10
   \end{matrix}



A simple way to relate the end effector to the base for a serial chain
manipulator is to see each link as a transformation of the base coordinate
system.  This is the approach suggested by Denavit and Hartenberg which is
addressed in the next section.




Dual Two Link Parallel Manipulator
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The Delta configuration is not just found in *Pick and Place* machines
but has also become popular with the 3D printing community. This style
of printer is fast and accurate. Just to get started, we look at a two
dimensional analog shown in
:numref:`Fig:paralleltwolink`. The top (red)
is fixed and is of length :math:`L_0`. The two links on either side
shown in dark blue are connected by servos (in green). These links are
of length :math:`L_1`. The angles are measured from the dotted line (as
0 degrees) to straight down (90 degrees), see
:numref:`Fig:paralleltwolink2`. At the
other end of the dark blue links is a free rotational joint (pivot).
That connects the two light blue links which are joined together at the
bottom with a rotational joint.

.. Owned by Roboscience

.. _`Fig:paralleltwolink`:
.. figure:: TermsFigures/2dDelta.*
   :width: 30%
   :align: center

   Parallel Two Link Manipulator.

Unlike the previous two link manipulator, it is not completely obvious
what the workspace looks like (although you might guess something
elliptical). The configuration space is the space of all possible
angles. This is limited by the red base in theory and by the servos in
practice. Since 360\ :math:`^\circ` motion for the servos is not
possible, the configuration space is a simple square
:math:`[\theta_m , \theta_M]^2` where :math:`\theta_m`, :math:`\theta_M`
are the minimum and maximum servo angles respectively.

Define the coordinate system as :math:`x` is positive right and
:math:`y` is positive down. The origin is placed in the center of the
red base link. The question is to figure out the position of the end
effector at :math:`(x,y)` as a function of :math:`\theta_1` and
:math:`\theta_2` with fixed link lengths :math:`L_0`, :math:`L_1`,
:math:`L_2`,
Figure :numref:`Fig:paralleltwolink2`. As with the
serial chain manipulator, this is an exercise in trigonometry.

.. Owned by Roboscience

.. _`Fig:paralleltwolink2`:
.. figure:: TermsFigures/2dDeltaCombined.*
   :width: 80%
   :align: center

   Parallel Two Link (a) configuration space (b) with coordinates


The forward kinematics will provide :math:`(x,y)` as a function of
:math:`(\theta_1, \theta_2)`. The derivation is left as an exercise and
so the point :math:`(x,y)` is given by

.. math::
   :label: paralleltwolinkforward

   (x,y) = \left( \frac{a+c}{2} + \frac{v (b-d)}{u} , \frac{b+d}{2} + \frac{v (c-a)}{u} \right)

Where

.. math:: (a,b) = (-L_1 \cos(\theta_1) - L_0/2 , L_1 \sin(\theta_1) )

.. math:: (c,d) = (L_1 \cos(\theta_2) + L_0/2 , L_1 \sin(\theta_2) )

and :math:`u = \sqrt{(a-c)^2 + (b-d)^2}`,
:math:`v  = \sqrt{L_2^2 - u^2/4}`.

If you guessed that the workspace was an ellipse like the author did,
that would be wrong. If you guessed some type of warped rectangle, then
you have great intuition.
:numref:`Fig:paralleltwolinkWS` shows the
workspace for the configuration domain :math:`[0, \pi/2]^2`. The figure
graphs :math:`y` positive going upwards and for the manipulator
:math:`y` positive goes down (so a vertical flip is required to match
up). The workspace can be created by running a program that traces out
all the possible arm angles and plots the resulting end effector
position (not all points, but a dense sample of points will do just fine).
Sample code to plot this workspace is given in :numref:`lst:computeconfigdomain`.



.. Owned by Roboscience

.. _`Fig:paralleltwolinkWS`:
.. figure:: TermsFigures/2dDeltaWS.*
   :width: 40%
   :align: center

   Parallel Two Link Workspace

The inverse kinematics will give you :math:`(\theta_1, \theta_2)` as a
function of :math:`(x,y)`. This is another exercise in trigonometry. For
:math:`(x,y)` given, we obtain

.. math::
   :label: paralleltwolinkIK

   \theta_1  = \pi - \beta - \eta , \quad \quad \theta_2 = \pi - \alpha - \gamma

where

.. math:: \| G \| = \sqrt{(x-L_0/2)^2 + y^2},  \quad\quad \| H\| = \sqrt{(x+L_0/2)^2 + y^2}

.. math:: \alpha = \cos^{-1} \frac{G^2 + L_0^2 - H^2 }{2GL_0}, \quad \quad \beta = \cos^{-1} \frac{H^2 + L_0^2 - G^2 }{2HL_0}

.. math:: \gamma = \cos^{-1} \frac{G^2 + L_1^2 - L_2^2 }{2GL_1},\quad \quad \eta =  \cos^{-1} \frac{H^2 + L_1^2 - L_2^2 }{2HL_1}

:numref:`lst:IKParallelTwoLink` illustrates using the inverse kinematic formulas
for a specific pair of :math:`(x,y)` values.




For the general serial manipulator, given joint angles and actuator lengths can one compute the
end effector position?   Yes.   Is there a standard way to approach the problem?  Yes.  The
next section provides some structure to address and automate the process.
