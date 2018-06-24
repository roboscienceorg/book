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

