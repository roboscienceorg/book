Machine Vision[Chap:Vision]
===========================

Triangulation and Structured Light
----------------------------------

The last section explored using time of flight to determine the distance
traveled. Distances can also be determined by using geometric properties
of the object (or image of the object). The approach is to project a
well defined light pattern (points, lines) onto the environment and
objects within. The reflected light is then captured by a
photo-sensitive line or matrix (camera) sensor device. Simple
triangulation then allows the computation of the distance in question.

Kinect and ASUS sensors use arrays of projected IR dots. The size of the
dot indicates distance of the dot. If the size of object is known, then
triangulation can be done without projecting light. Standard computer
vision techniques can recover relative image size.

.. raw:: latex

   \centering

.. figure:: sensor/lasertriangulation
   :alt: Laser Triangulation.[lasertriangulation]

   Laser Triangulation.[lasertriangulation]

Laser Triangulation is done by setting the measurement apparatus up so
that simple trigonometry can be used to measure distance,
Figure \ `[lasertriangulation] <#lasertriangulation>`__. In this case,
the laser setup uses similar triangles making the mathematics much
simpler. The distance :math:`D` is given by

.. math:: D = \displaystyle\frac{fL}{x}.

All sensing involves error. We will address types of errors and how to
mitigate errors in the chapter on filtering. To get a feel for how
errors affect a result, one can run a simple numerical experiment.
Compute the range of the input values based on the error. Plug the high
and low values in and you can compute the range on the output value. The
following example provides some details.

.. raw:: latex

   \normalfont

Assume that for the triangulation setup in
Figure \ `[lasertriangulation] <#lasertriangulation>`__, we have
:math:`f=8`\ mm, :math:`L = 3`\ cm and :math:`x = 2`\ mm. Using the
formula we see that :math:`D = 0.8*3/0.2 =12`\ cm. What if there is
error in the :math:`x` or :math:`L` measurement?

What is the error in the distance if we know that :math:`x` has a max of
20% error? A 20% variation means that our value ranges between
:math:`[1.6, 2.4]`. We can plug the two values in and see what the range
in D is. This works because :math:`D` is a monotonic function of both
:math:`x` and :math:`L`\  [1]_ Plugging these values in we have
:math:`10 \leq D \leq 15`. Which gives :math:`15/12 - 1 = 0.25` or 25%
error. If we have 10% error in :math:`L`, it gives
:math:`10.8 \leq D \leq 13.2` or a 10% error off of :math:`12`. To
combine these, we look for the largest and smallest values possible for
:math:`D`, given the range in input values. For :math:`x=1.6` and
:math:`L=3.3` we get :math:`D=16.5`. Likewise for :math:`x=2.4` and
:math:`L=2.7` we get :math:`D=9`. The max of the two is a 37.5% error
(from :math:`D=12`\ cm).

Is there a way to estimate combined error from the equation? In
Calculus, the total derivative for :math:`f(x,y,z)` :

.. math:: df = \frac{\partial f}{\partial x}  dx + \frac{\partial f}{\partial y} dy + \frac{\partial f}{\partial z} dz

\ can be used to gain an error formula:

.. math:: E = \Delta f \approx \frac{\partial f}{\partial x} \Delta x + \frac{\partial f}{\partial y} \Delta y + \frac{\partial f}{\partial z} \Delta z .

.. raw:: latex

   \normalfont

Using the values from the last example, :math:`f=8`\ mm,
:math:`L = 3`\ cm and :math:`x = 2`\ mm and the variations, :math:`x`
has a max of 20% error and 10% error in :math:`L`, find the error
estimate for :math:`D`. Using :math:`\Delta L = \pm 0.3`,
:math:`\Delta x = \pm 0.04`, and variation in :math:`f` means
:math:`\Delta f = 0` we have

.. math::

   E  = \frac{L}{x} \Delta f +  \frac{f}{x} \Delta L - \frac{fL}{x^2} \Delta x  
       = \frac{3}{0.2} (0) +  \frac{0.8}{0.2} (\pm 0.3) - \frac{(0.8)(3)}{(0.2)^2}(\pm 0.04)  
       = \pm 3.6

This estimates an error of 25%. This turns out to be not so accurate. A
20% error is a bit too large for the linear approximation to be close,
but works as a rough estimate.

A way to modify the previous laser distance example is a common
industrial vision setup. Look at the diagram and see what formulas can
we derive. Note that:

.. math:: \left(\frac{z}{x}\right) = \left(\frac{f}{u}\right) \qquad \mbox{and}\quad\tan(\alpha) = \left( \frac{z}{b-x} \right)

Flip the second formula:

.. math:: \cot(\alpha) = \left(\frac{b-x}{z}\right)

Then multiply by z:

.. math:: \left( z \right)\cot(\alpha) = \left( b-x \right)

Move the :math:`x` over:

.. math:: \left( z \right)\cot(\alpha) + x = b

From the first ratio: :math:`z = \cfrac{fx}{u}`.

.. raw:: latex

   \centering

.. figure:: sensor/lasertriangulation2
   :alt: Computer Vision[fig:lasertriangulation2]

   Computer Vision[fig:lasertriangulation2]

Plug this in for :math:`z`:

.. math:: \left(\cfrac{fx}{u}\right)\cot(\alpha) +x  = b.

Factor out the :math:`x` and divide the rest over:

.. math:: x = \frac{b}{\left(\frac{f}{u}\right)\cot(\alpha) + 1}

then using

.. math::

   z = \cfrac{fx}{u} = \left(\frac{f}{u}\right)\frac{bu}{\left(\frac{f}{u}\right)
   \cot(\alpha) + 1} .

Summarizing the formulas:

.. math::

   \label{eqn:industrialvision}
   x = \frac{b u}{f\cot \alpha + u},  \quad
   z = \frac{b f}{f\cot \alpha + u}

What are :math:`x` and :math:`z` if b = 20cm, f = 2cm, :math:`\alpha` =
60deg, and u = 7mm? So, using these formulas:

.. math:: x = 20*0.7/(2\cot(60)+0.7) = 7.55 cm,

\ 

.. math::

   z =
   20*2/(2\cot(60)+0.7) = 21.57 cm.

The Sharp distance sensor uses a very similar approach to estimate
distances. The displacement of the beam center on the beam detector is
used for the distance estimate, see
Figure \ `[fig:SharpIRsensor] <#fig:SharpIRsensor>`__. Distance D is
given by

.. math:: D=  \frac{fb}{2d} .

Because the focal length is small, the range of distances are limited by
the resolution of the detector (which provides :math:`D`). The Sharp
detector returns the distance estimate as an analog voltage. An analog
to digital converter can be used to provide the numerical value. In
practice, the relation between voltage and distance is not linear and
some calibration in software is required.

.. raw:: latex

   \centering

.. figure:: sensor/sharpIR
   :alt: Sensor package.

   Sensor package.

.. figure:: sensor/sharp
   :alt: The triangulation used to calculate distance

   The triangulation used to calculate distance

Another approach used in machine vision is **Structured Light**. A known
pattern of light is projected onto the environment. Common patterns are
dots, stripes and grids. A camera will view the instrumented scene and
determine the object heights using geometry.

.. raw:: latex

   \centering

.. figure:: sensor/structuredlight
   :alt: Structured light.[structuredlight]

   Structured light.[structuredlight]

Stereo Cameras
--------------

We can build cameras that are much more accurate than the eye, however,
understanding that the image belongs to a particular object is a much
harder task. Implicit in the process is our stereo vision. We have the
ability to reconstruct the 3D world through our eyes; an ability for
which significant effort has been expended to duplicate in computer
science. A branch of modern computer vision, stereo vision, uses
multiple cameras and algorithms to reconstruct and understand the
environment. Some of the tools developed to do this have been widely
distributed. For example, the automatic stitching of images that cameras
perform when building panoramas.

This is successfully employed using pairs of cameras which do a good job
at reconstructing the 3D world we live in. The process requires one to
determine the position shift in pixels of a fixed object. The difficult
part is automatically identifying common points in the image.
Significant effort has been invested in the determination of common
features between two images. Several well known algorithms such as SIFT
and SURF are available now to simplify this process. Once that is done,
it is easy to triangulate the depth of the point, Figure
(`[intro-stereo1] <#intro-stereo1>`__).

.. raw:: latex

   \centering

.. figure:: vision/stereo1
   :alt: Seeing in three dimensions with a pair of calibrated cameras:
   determining depth using basic Trigonometry. [intro-stereo1]

   Seeing in three dimensions with a pair of calibrated cameras:
   determining depth using basic Trigonometry. [intro-stereo1]

.. raw:: latex

   \centering

.. figure:: vision/stereo1a
   :alt: Seeing in three dimensions with a pair of calibrated cameras:
   determining depth using basic Trigonometry. [intro-stereo1]

   Seeing in three dimensions with a pair of calibrated cameras:
   determining depth using basic Trigonometry. [intro-stereo1]

Define a coordinate system where the horizontal axis is :math:`x` and
the vertical axis is :math:`z`. Let the focal point of the left camera
be at the origin of the :math:`x-z` coordinate system. Using both
cameras, we would like to find the coordinates :math:`(x,z)` for the
point :math:`w`. Assume that we are given the focal depth :math:`f`
(positive value) and pixel offsets in image sensor :math:`v_1`,
:math:`v_2` all as *unsigned* (positive) quantities. Then

.. math::

   \left(\frac{z}{x}\right) = \left(\frac{f}{v_1}\right),\quad\quad
   \left(\frac{z}{b-x}\right)  = \left(\frac{f}{v_2}\right)

Cross multiply and set equal to common fraction; then remove fractions:

.. math::

   \left(\frac{v_1}{x}\right) = \left(\frac{f}{z}\right) = \left(\frac{v_2}{b-x}\right)
   \quad \Rightarrow \quad v_2 x = v_1(b-x) = v_1 b - v_1 x \Rightarrow  (v_1+v_2) x = v_1b

Solving for :math:`x`, we obtain the equation below. Plugging this into
:math:`z = fx / v_1` we obtain the equation for :math:`z`.

.. math::

   \label{intro:stereodistance}
   x = \frac{v_1b}{v_1+v_2}, \quad 
   z = \frac{fb}{v_1+v_2}

.. raw:: latex

   \centering

|Seeing in three dimensions with a pair of calibrated cameras: building
a disparity (depth) map and reconstructing the 3D world.[fig:seeing3d]|
|Seeing in three dimensions with a pair of calibrated cameras: building
a disparity (depth) map and reconstructing the 3D world.[fig:seeing3d]|

Once depth for the collection of feature points are known, depth for
surrounding points is inferred. This allows the construction of a
disparity map which maps grey scale values to pixel. It is a depth map
which is shown in Figure \ `[intro-stereo2] <#intro-stereo2>`__. A depth
map is a useful tool in object identification. The depth map can be used
for segmentation, the process by which we separate an image into
distinct components or objects. Once we have the object segmented, then
we may lookup in a shape database to determine what the object is, known
as object recognition. The depth map is one of several ways to perform
object recognition and is a useful tool if we have already computed the
depth map.

.. raw:: latex

   \centering

.. figure:: slam/rgbdslam.jpg
   :alt: Seeing in three dimensions with RGBD sensors[intro-stereo2]

   Seeing in three dimensions with RGBD sensors[intro-stereo2]

Once a depth map is made, reconstruction of the environment can follow.
Essentially a 3D CAD type representation of the world surrounding the
robot. Thus the environment is mapped in 3D. This is useful for robots
which perform remote reconnaissance as well as for robots which need to
navigate through the environment according to some plan. The map
building process normally places the robot in the map, known as
localization. Thus we can compute optimal paths and safe trajectories.

Depth Sensing Cameras
~~~~~~~~~~~~~~~~~~~~~

We also have a choice of sensors which can directly measure the depth of
field. These are known as time of flight cameras or 3D cameras. The
Microsoft Kinect is a common example. The units range significantly in
cost depending on accuracy, range and speed of the device. These devices
directly provide depth without having to compute a disparity map or some
other intermediate data set. They are very helpful in doing 3D
reconstructions of the environment.

.. raw:: latex

   \centering

.. figure:: sensor/3dcamera
   :alt: 3D Camera

   3D Camera

.. raw:: latex

   \FloatBarrier

Problems
--------

.. raw:: latex

   \setcounter{Exc}{0}

]

Assume you have a laser triangulation system as shown in
figure \ `[fig:lasertriangulation2] <#fig:lasertriangulation2>`__ given
by equations \ `[eqn:industrialvision] <#eqn:industrialvision>`__ and
that :math:`f  = 8`\ mm, :math:`b = 30`\ cm. What are the ranges for
:math:`\alpha` and :math:`u` if we need to measure target distances in a
region (in cm) :math:`20 < z < 100` and :math:`10 < x < 30`?

.. raw:: latex

   \centering

.. figure:: sensor/lasertriangulation2.pdf
   :alt: Laser Triangulation System [fig:lasertriangulation]

   Laser Triangulation System [fig:lasertriangulation]

.. math::

   \label{eqn:triangulationformulas}
   x = \left(\frac{b u}{f\cot \alpha + u}\right) \qquad z = \left(\frac{b f}{f\cot \alpha + u}\right)

**a.** Use the equations to solve for :math:`u` and :math:`\alpha`:

.. math::

   \begin{aligned}
   u &= x \left(\frac{f\cot \alpha + u}{b}\right)      &&\alpha= \cot^{-1}\left(\left(\frac{b f}{z} - u\right) f^{-1}\right)  [8pt]
      &= x\left(\frac{f}{z}\right)                               &&\ \ = \cot^{-1}\left(\left(\frac{b f}{z} - \frac{xf}{z}\right) f^{-1}\right)  [8pt]
      &                                                                        &&\ \ = \tan^{-1}\left(\frac{z}{b-x}\right)  [10pt]\end{aligned}

**b.** Determine :math:`u` and :math:`\alpha` for each of the 4 limit
points of our rectangle. These outer points are not actually included,
so we will use points that are very close to the limits:

x = 10.01, z = 20.01

.. math::

   \begin{aligned}
   u  &= \SI{10.01}{\cm}\left(\frac{\SI{0.8}{\cm}}{\SI{20.01}{\cm}}\right) \approx \SI{0.40}{\cm}
   &&\alpha= \tan^{-1}\left(\frac{\SI{20.01}{\cm}}{\SI{30}{\cm}-\SI{10.01}{\cm}}\right) \approx 45.03^{\circ}\end{aligned}

 x = 29.99, z = 20.01

.. math::

   \begin{aligned}
   u  &= \SI{29.99}{\cm}\left(\frac{\SI{0.8}{\cm}}{\SI{20.01}{\cm}}\right) \approx \SI{1.20}{\cm}
   &&\alpha= \tan^{-1}\left(\frac{\SI{20.01}{\cm}}{\SI{30}{\cm}-\SI{29.99}{\cm}}\right) \approx 89.97^{\circ}\end{aligned}

 x = 10.01, z = 99.99

.. math::

   \begin{aligned}
   u  &= \SI{10.01}{\cm}\left(\frac{\SI{0.8}{\cm}}{\SI{99.99}{\cm}}\right) \approx \SI{0.08}{\cm}
   &&\alpha= \tan^{-1}\left(\frac{\SI{99.99}{\cm}}{\SI{30}{\cm}-\SI{10.01}{\cm}}\right) \approx 78.69^{\circ}\end{aligned}

 x = 29.99, z = 99.99

.. math::

   \begin{aligned}
   u  &= \SI{29.99}{\cm}\left(\frac{\SI{0.8}{\cm}}{\SI{99.99}{\cm}}\right) \approx \SI{0.24}{\cm}
   &&\alpha= \tan^{-1}\left(\frac{\SI{99.99}{\cm}}{\SI{30}{\cm}-\SI{29.99}{\cm}}\right) \approx 89.99^{\circ}\end{aligned}

 [15pt]

**c.** From the calculations above, our ranges for :math:`u` and
:math:`\alpha` are:

.. math:: \SI{0.08}{\cm} \leq u \leq \SI{1.20}{\cm}

.. math:: 45.03^{\circ} \leq \alpha \leq 89.99^{\circ}

Assume you have two cameras that are calibrated into a stereo pair with
a baseline of 10cm, and focal depth of 7mm. If the error is 10% on
:math:`v_1` and :math:`v_2`, :math:`v_1 =  2`\ mm and
:math:`v_2 = 3`\ mm, what is the error on the depth measurement
:math:`z`? Your answer should be a percentage relative to the error free
number. Hint: If :math:`v_1 = 2` then a 10% error ranges from 1.8 to
2.2. [Although not required, another way to approach this problem is the
total differential from calculus.]

To compute the distance of a point we begin with
equation \ `[intro:stereodistance] <#intro:stereodistance>`__

.. math:: z= \frac{fb}{v_2-v_1}

The error free distance value is

.. math:: z_{act}= \frac{7*100}{2+3} = 140mm

From Figure \ `[intro-stereo1] <#intro-stereo1>`__, the maximum error
occurs when :math:`v_1+v_2` is at a minimum and the greatest negative
error occurs when :math:`v_1+v_2` is at a maximum. These lead to
calculated distances of:

.. math:: z_{max}= \frac{7*100}{1.8+2.7} = 155.56mm

and

.. math::

   z_{min}= \frac{7*100}{2.2+3.3} = 127.27mm
   .

The percentage errors are

.. math:: E_+ = \frac{155.56-140}{140} = +11\%

above the actual distance and

.. math:: E_- = \frac{127.27-140}{140} = -9\%

below the actual distance.

Assume you have two cameras that are calibrated into a stereo pair with
an estimated baseline of 10cm, and focal depth of 10mm. If the error is
10% on the baseline, what is error on the depth measurement :math:`z`
with :math:`v_1 = 2`\ mm and :math:`v_2 = 3`\ mm? Your answer should be
a percentage relative to the error free number. See the hint above.

We know that the depth measurement :math:`z` and the :math:`x`-axis
offset are given by the following equations:

.. math:: x = \frac{v_1b}{v_1+v_2} \qquad z = \frac{fb}{v_1+v_2}

**1** The depth measurement :math:`z`, sans error, can be calculated as
follows:

.. math:: z = \frac{10*100}{3 + 2} = \SI{200}{\milli\meter}

**2** Let :math:`\alpha` be the error range of the baseline :math:`b`.
At a 10% error, this gives us:

.. math:: -10 \leq \alpha \leq 10

**3** From this, we know that the error on the depth measurement
:math:`z` is given by:

.. math::

   \begin{aligned}
   E_{z} &= \left| z_{error} - z \right|  [12pt]
             &= \left| \frac{f \left(b + \alpha\right)}{v_1+v_2} - \frac{fb}{v_1+v_2} \right|  [12pt]
             &= \left|\frac{10\left(100 + \alpha\right)}{3 + 2} - \frac{10*100}{3 + 2} \right|  [12pt]
             &= \left|2*\left(100 + \alpha\right) - 200\right|  [12pt]
             &= \left|2\alpha\right|  [12pt]\end{aligned}

**5** By graphing this resulting error :math:`E_{z}` against the error
range :math:`\alpha`, as shown in Figure \ `[fig:4.5] <#fig:4.5>`__, we
can determine which values of :math:`\alpha` cause the greatest and
smallest error. From this graph it is evident that :math:`E_{z}` is the
same for both :math:`\alpha = -10` and :math:`\alpha = 10`.

.. raw:: latex

   \centering

.. figure:: solutions/Sensors/fig4-5.jpg
   :alt: Error on Depth Measurement :math:`z` [fig:4.5]

   Error on Depth Measurement :math:`z` [fig:4.5]

Since error range :math:`\alpha` has the same maximum error in both the
positive and negative direction of :math:`z`, the percentage of error in
each direction will be the same:

.. math::

   \begin{aligned}
    \max E_{z} &= \left| 2\alpha \right|   [5pt]
                     &= \left|2*10 \right|   [5pt]
                     &= \SI{20}{\milli\meter}  [5pt]
   &\Big\Downarrow  [5pt]
   E_{z-} = \frac{\max E_{z}}{z}*100\% &=  \frac{20}{200}*100\% = 10.0\%   [5pt]
   E_{z+}= \frac{\max E_{z}}{z}*100\% &=  \frac{20}{200}*100\% = 10.0\%  \end{aligned}

With a single camera, explain how a straight line (produced by a laser)
can resolve depth information.

.. raw:: latex

   \Closesolutionfile{Answer}

.. [1]
   Monotonic means that :math:`f'>0` or :math:`f'<0` in the interval of
   interest.

.. |Seeing in three dimensions with a pair of calibrated cameras: building a disparity (depth) map and reconstructing the 3D world.[fig:seeing3d]| image:: vision/disparitya.png
.. |Seeing in three dimensions with a pair of calibrated cameras: building a disparity (depth) map and reconstructing the 3D world.[fig:seeing3d]| image:: vision/disparityb.png

