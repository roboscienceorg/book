Sensors and Sensing[Chap:Sensors]
=================================

Robotics is a interdisciplinary subject which relies on mechanical,
electrical and software systems. Even though the focus of the text is on
the computational aspects of robotics, it is important to have an
overall understanding on the core systems and their functions. For this
chapter we briefly touch on some of the sensors encountered in current
robots.

Sensing
-------

Sensors are a key tool to perceiving the environment. Our eyes, ears,
nose, tongue, and skin are all sensors giving us details about our
surroundings. Whether the environment is known or unknown, a robot
requires sensors to perceive it as well. A roboticist needs to
understand how a sensor functions and what its limitations are in order
to use it to its full capacity. These limitations could include noise,
bandwidth, data errors, and many other issues that must be accounted for
in order to get accurate results. Understanding the physical principles
of the sensors available is the key to understanding, modeling, and
utilizing this information.

A sensor can be any device that converts energy into a usable signal.
Sensors fall into two classes: passive and active. **Passive Sensors**
use energy from the environment to power the measurement. A bump or
temperature sensor are examples of passive sensing. **Active sensors**
inject energy into the environment in a particular manner and measure
the reaction. Active sensing can often get better results, but at an
increased complexity, cost, and power requirements. It could also
require the modification of the surrounding environment, such as the
placement of beacons or tags. Laser and ultrasonic ranging are examples
of active sensing.

We can further classify sensors by which part of the robot’s
environment, internal or external, they are sensing. **Properioceptive
Sensors** measure the internal state of the system (robot), such as
motor speeds, wheel loads, turn angles, battery status, temperature and
other aspects that are internal to the machine. **Exterioceptive
Sensors** measure information from the robot’s external environment;
external to the robot, such as distances to objects, GPS, ambient light
and temperature, magnetic fields, accelerations, etc.

Another way to classify sensors is by the data they return. Sensors can
return information in **analog** or **digital** form. Typically an
analog sensor will vary voltage (maybe current) as the measured quantity
changes value. Normal application is to feed that signal into a device
known as an ADC or analog to digital converter. The vast array of
microcontrollers on the market offer built in ADC lines. For example
Arduino boards can take in analog signals. The voltage level is sampled
and converted to a numerical value (hence digital). This means you might
need some glue electronics to convert your sensor’s voltage range to the
full range of the ADC inputs. The ADC will be listed at having a certain
number of bits (say 12 bit). This gives the resolution. For a 12 bit
device, it means that the sampling will break the signal into
:math:`2^{12}` discrete values or 4096 different levels. More bits means
better resolution. But it takes more hardware and memory inside so there
is a tradeoff.

Digital sensors are often analog devices with built in ADC chips. To
save packaging space they will very often communicate via a bus and not
have the 12 pins required for a 12 bit sampling. So beyond the ADC, they
will have some other device inside to send the signal out on some type
of serial line protocol (uart, i2c, spi, ...). Digital devices can be
more accurate than analog devices but not always. It depends on the
number of bits used and other factors in fabrication.

The desired qualities of a sensor are high accuracy and resolution, wide
range of measurement, low delay times, stability of measurement with
respect to the environment (no temperature drift or magnetic field
interference for example), and above all very low cost. Sensors will
talk to computers using two standard methods: polling and interrupts.
For polling, the computer periodically reads the value on the correct
register. Simplistically this is implemented via a loop in the software
with a delay although better approaches involve setting a cpu timer and
using an interrupt. The other approach is to have the sensor generate
the interrupt and the cpu’s interrupt handler will read the value on the
register.

Sensor Metrics
~~~~~~~~~~~~~~

There is a significant range in the quality of the sensed data. Some
sensors may be very narrow in the range of sensing (e.g. range or angle
of perception) while other are very wide. Sensors have noise which can
vary depending on the sensor and the environment. Sensors will measure
some quantity over some range, there are maximum and minimum values for
inputs. The dynamic range is the ratio of the upper limit to the lower
limit. Ranges can be very large and so the decibel is normally used. The
formula for expressing the ratio depends on whether the sensed quantity
is related to power or a field. Use of the 20 instead of the 10 is based
on the standard use of the ratio of the squares and so we have a factor
of 2 which comes out front.

Power
    .. math:: L_p = 10\log_{10} \left( \frac{P}{P_0}\right) \mbox{dB}

Field
    .. math:: L_p = 20\log_{10} \left( \frac{F}{F_0}\right) \mbox{dB}

Note that the ratio makes the decibel a unitless quantity.

Compute the dynamic range in dB for a measurement from 20mW to 50kW.

.. math:: L_p = 10\log_{10} \left( \frac{50000}{.02}\right) \mbox{dB} = 63.979 \mbox{dB}

Compute the dynamic range in dB for a measurement from 0.1V to 12V.

.. math:: L_p = 20\log_{10} \left( \frac{12}{.1}\right) \mbox{dB} =  41.584\mbox{dB}

We have been using some terms that describe the sensor data and it is
worth reviewing these terms.

Resolution
    In the world of digital sensors, this is often described as the
    number of bits used. It is the smallest change in the sensed value
    that can be measured. It is what you see in your science courses on
    measurement precision.

Accuracy
    It is how close the reported or measured value is to the actual
    value.

Range
    Or measurement range. It is the range of input values the sensor can
    detect.

Repeatability
    This describes the changes in the measured parameter over multiple
    measurements with a fixed value.

Frequency
    Some sensors produce new values at some clock rate which is given by
    frequency.

Response time
    The time delay between the measurement and the output value.
    Sometimes this will be used as the time delay between when the cpu
    requests a measurement and when the measurement is the available to
    the cpu.

Linearity
    The signal output is a linear function of the input.

Sensitivity
    The ratio of measured value to sensor output value.

All sensing involves measurement errors. There are standard ways to
measure the error. Assume that :math:`x` is the true value and :math:`z`
is the measured value. The *absolute error* is given by
:math:`| x - z|`. The *relative error* is given by :math:`| 1 - z/x|`.
The reason we might choose relative error over absolute error is based
on scale. For example, which of the pairs would you say is a better
estimate: :math:`(x,z) = (0.1, 0.2)` and :math:`(x,z) = (100, 102)`? The
absolute error for the first is 0.1 and for the second is 2. Two is
larger than 0.1. But intuitively we see that going from 100 to 102 is
closer at the scale of 100. The relative error shows this with the first
being a relative error of :math:`|1 - 0.2/0.1| = 2`. The relative error
on the second one is :math:`|1-102/100| = 0.02`. This fits with our
intuition about the errors. Relative error removes the scale and can be
reported as a percentage which is called the percentage error,
:math:`100|1-z/x|`. The accuracy of a measurement is given by 100 -
percentage error.

Position and Velocity Sensors
-----------------------------

Navigation and localization is one of the more challenging problems in
mobile robotics and any estimate is welcome. If you have an estimate on
wheel velocity then you can by integration estimate rotation and through
wheel diameter estimate the linear travel (over very short distances).
Using the differential drive equations,
`[ddkinematicsmodel] <#ddkinematicsmodel>`__, the wheel velocity may be
integrated to determine position in the global or inertial reference
frame. So the obvious question is... how do we determine wheel position
or velocity?

Position sensors measure the absolute displacement of a joint or other
sensed item for both linear and rotary joints. Both analog and digital
technologies are used for measuring displacement. Variable resistors
have been common for years. Either as a rotary device such as found in
volume dials or slider (fader) such as found in mixers. The rotary
variable resistor is known as a potentiometer or "pot" for short.
Typically wired in a voltage divider (discussed later), the voltage
across the potentiometer can be measured as an analog signal. It can be
converted to a digital signal for use in a microcontroller. Variable
resistors have an element which slides over a coil of resistive wire or
over carbon base changing the electrical path which varies the
resistance.

A direct digital approach is to use encoders. Absolute encoders return a
Gray code as a function of position. These also come in linear and
rotary designs. Typically one has a collection of sensors which can
detect light and dark. The encoder allows light to pass through it in
some pattern that correlates to position. The sensed pattern is
converted to the output code. Another approach used is to count the
number of times a particular pattern occurs or a beam is interrupted.
This is explored in detail below with the optical wheel encoders which
can be used for velocity in addition to position estimates.

.. raw:: latex

   \centering

.. raw:: latex

   \centering

.. figure:: ./sensor/potentiometer.jpg
   :alt: Potentiometer.[fig:potentiometer]

   Potentiometer.[fig:potentiometer]

   

.. raw:: latex

   \centering

.. figure:: ./sensor/Faders.jpg
   :alt: Fader.[fig:fader]

   Fader.[fig:fader]

   

.. raw:: latex

   \centering

.. figure:: ./sensor/Encoder.jpg
   :alt: Encoder.[fig:encoder]

   Encoder.[fig:encoder]

Tachometers
~~~~~~~~~~~

An electric motor and a generator are very similar devices which just
operate in opposite fashions. Providing electrical power in a motor
causes the shaft to turn. Conversely turning the shaft of a generator
produces electricity. A tachometer can be built out of a generator (or
electric motor). The faster the shaft spins, the greater the voltage or
higher the frequency produced. This can be converted to a digital signal
and thus provides a measure of rpm.

Optical Wheel Encoders
~~~~~~~~~~~~~~~~~~~~~~

One option to tackle this problem involves using Light Emitting Diodes,
or LEDs [1]_. The dominant lighting source in electronics and robotics,
LEDs can run on very low power, are available in many frequencies and
can switch on/off quickly. Figure \ `[circuitled] <#circuitled>`__.

.. raw:: latex

   \centering

.. figure:: circuit/LED
   :alt: LED[circuitled]

   LED[circuitled]

LEDs can emit in non-visible ranges, ultraviolet and infrared. Many of
the non-visible frequencies are popular for simple object detection in
combination with a phototransistor,
Figure \ `[IRobstacleLED] <#IRobstacleLED>`__. In this example, the
infrared LED shines on some object and is reflected back to the
phototransistor. The IR light activates the transistor and causes it to
switch on and pull the output to low.

.. figure:: sensor/IRObs
   :alt: Infrared LEDs used for obstacle detection.[IRobstacleLED]

   Infrared LEDs used for obstacle detection.[IRobstacleLED]

This system can be used for simple occupancy detection or close obstacle
detection. We can also use the LED-transistor combination to determine
wheel rotation; to measure the speed or position of a wheel or dial. For
example the dials on electronic devices like a volume control. In
addition, knowing wheel rotation can assist in the process of localizing
the robot. The fundamental idea is to generate a radial or linear
pattern of black and white stripes (or slits). The IR light is either
reflected or not. This is sensed with the phototransistor. Counting the
stripes (or lists) can provide an estimate of wheel rotation. Over a
fixed interval of time this provides an estimate of wheel velocity. The
estimate is clearly improved if more stripes (or slits) per revolution
are used.

.. raw:: latex

   \centering

.. figure:: sensor/sensormount
   :alt: Mounting for the encoder sensor [mountingencoder]

   Mounting for the encoder sensor [mountingencoder]

There are two basic components needed to build your own. First you need
the light source and the detector. Second you need an encoder. To read
the encoder, you will need an optical sensor. Typically one uses an IR
LED (IR light emitting diode) and phototransistor pair,
Figure \ `[ledopticalsensor] <#ledopticalsensor>`__. These are packaged
in single units, for example the Fairchild QRD1313. This has the LED and
the phototransistor packaged into a unit that is 6.1mm x 4.39mm x 4.65mm
(height).

.. figure:: sensor/IR2
   :alt: IR LED (IR light emitting diode) and phototransistor pair
   [ledopticalsensor]

   IR LED (IR light emitting diode) and phototransistor pair
   [ledopticalsensor]

An encoder pattern may simply be a pattern printed on paper and attached
(glued) to the inside of a robot wheel. Simple encoder patterns are just
alternating black and white radial stripes. Two examples are given in
Figure \ `[encoderpattern] <#encoderpattern>`__.

.. raw:: latex

   \centering

.. figure:: sensor/WheelEncoder
   :alt: Wheel encoder pattern with 1-1 ratio

   Wheel encoder pattern with 1-1 ratio

.. raw:: latex

   \centering

.. figure:: sensor/encoder_var
   :alt: Encoder pattern with 1-4 ratio

   Encoder pattern with 1-4 ratio

Doppler Effect
~~~~~~~~~~~~~~

Direct measurement of velocity may be achieved by using the Doppler
Effect. Recall when a vehicle passes by, you notice a change in the
sound of the machine. The sound waves are compressed as the vehicle
approaches and are expanded as the vehicle retreats. This compression
results in a higher frequency of the sound and so as the vehicle passes,
you hear the drop in frequency. Transmitting a known frequency and
listening to the reflected sound, one can estimate the relative
velocity.

.. raw:: latex

   \centering

.. figure:: sensor/doppler
   :alt: Using the Doppler Effect to estimate velocity.

   Using the Doppler Effect to estimate velocity.

The formula that describes the change in frequency for a moving sound
source (a transmitter) is

.. math:: f_r = f_t (1 + v/c).

If the receiver is moving the formula for the frequency change is

.. math:: f_r = f_t / (1 + v/c).

If you know the frequency change you can then compute :math:`v`.

Orientation Sensors
-------------------

The word pose is used for both position and orientation. Measurement of
orientation is done through several basic sensing approaches.

Gyroscopes
~~~~~~~~~~

A gyroscope is a heading sensor that gives a measure of orientation with
respect to a fixed frame. A standard gyro provides an absolute measure
of the heading for a mobile robot normally measured in degrees off of
some fixed heading. The classical mechanical gyroscope uses a spinning
body and the resulting rotational inertia. Optical gyros can use phase
shifts of intersecting beams of light to measure changes in orientation.
Rate gyros give a measure of angular speeds which is the standard for
low-cost MEMS systems. These are the most common units found in mobile
robots, UAVs, phones and other portable devices. These gyros will return
data in degrees per second (deg/s). Like accelerometers, the MEMS units
(microelectromechanical systems) are packaged into 1, 2, 3 sensors to
provide rotational rates about the :math:`x`, :math:`y` and :math:`z`
axes. Also like the accelerometer, the gyro can have a digital or analog
interface.

Drift can be a significant issue. The absolute direction can be lost
over a period of time depending on the sensor quality. This is an issue
that must be addressed for systems which run for long periods of time
without a recalibration.

.. figure:: sensor/gyro
   :alt: Tuning fork MEMS gyroscope. [gyroscope]

   Tuning fork MEMS gyroscope. [gyroscope]

Compass or magnetometer
~~~~~~~~~~~~~~~~~~~~~~~

The compass is one of the oldest sensors in use dating back 4000 years.
Early forms would take a small piece of loadstone (natural magnetite)
and suspend it from a silk thread or place it on wood and float that in
water. This magnetic rock would orient along the Earth’s field lines and
could then be used for navigation. Due to the liquid iron core, the
Earth’s field is sufficiently strong to measure with portable devices.
Although pole reversals do occur, we have relatively stable pole
locations for long periods of time, so compass navigation has been a
popular orientation tool for thousands of years. This is an absolute
measure of orientation in contrast to the relative sensing we saw with
the radial encoder.

There are multiple ways to measure a magnetic field. The traditional
methods are known as mechanical in that the force of the field lines
induces a torque and moves part of the sensor. Other approaches use the
Hall-Effect or Magnetoresistive sensors. The earth’s magnetic field is
still relatively weak. Other magnetic sources such as inductors can
disturb and invalidate measurements. It is critical when building the
sensing system, the sensor is not placed near a motor, power supply or
any other device which can generate magnetic interference. Large amounts
of iron can alter the earth’s field or even shield it. This prevents a
magnetic sensing in certain environments.

Magnetic encoding
^^^^^^^^^^^^^^^^^

It is possible to use Hall-effect or other similar devices to do
encoding. Small Hall-Effect sensors with sub-degreee accuracy are
available. Placing a small ceramic magnet on the end of a shaft will
generate a rotating magnetic field which can be detected with the
Hall-Effect sensor. Figure \ `[halleffect] <#halleffect>`__ shows how
this is done.

.. figure:: sensor/magneticencoder
   :alt: Hall-Effect based shaft rotation sensor.[halleffect]

   Hall-Effect based shaft rotation sensor.[halleffect]

Inertial Sensors
----------------

Accelerometers
~~~~~~~~~~~~~~

An accelerometer measures acceleration in a particular direction. The
standard units on acceleration are meters per seconds squared
(:math:`m/s^2`). The sensor is normally a MEMS unit which are often
packaged together using two or three sensor units pointed in orthogonal
directions. This can provide acceleration information along each of the
coordinate axes. Common constructions use two plates with one moveable
and attached to a mass, and the other fixed. Acceleration will cause the
plate to move and change the capacitance. This change can be measured
and related to the acceleration. Output may be a voltage level in which
the sensor is known as an analog sensor or the output can be through a
digital interface, such as I\ :math:`^2`\ c making it a digital sensor.

.. figure:: sensor/accel
   :alt: Simple accelerometer structure. [accelerometer]

   Simple accelerometer structure. [accelerometer]

A simple application of an accelerometer is an inclineometer or tilt
sensor. These sensors can have a great deal of noise and extracting a
good signal can be very challenging. Note that it is temping to think
that this device can provide position information. After all, we learn
in calculus that if we integrate acceleration twice, we obtain position.
The problem is the noise. Even though integration is numerically a
smoothing process which can reduce noise by averaging it out, over time
small errors build and position accuracy is poor. In practice, using an
accelerometer does not provide adequate position or velocity estimates.

Inertial Measurement Unit (IMU)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

An Inertial Measurement Unit or IMU packages accelerometers, gyroscopes
and possibly a compass together into a single unit. A 6DOF (degrees of
freedom) IMU will have a three axis accelerometer and three axis
gyroscope. A 9DOF IMU will have the three axis accelerometer, three axis
gyroscope and a 3 axis magnetometer. These devices normally provide a
digital interface such as USB and return text strings of data at some
Hz. IMUs are used as the basis for AHRS: Attitude and Heading Reference
System.

Attitude and Heading Reference System (AHRS)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

AHRS consist of accelerometers, gyroscopes and magnetometers on all
three axes. So, AHRS includes an IMU. In addition to the IMU, the AHRS
has the algorithms to provide attitude and heading information as well
as the required hardware for the computation. These algorithms include
sensor fusion codes which take data from multiple sources and "fuse"
them into a hopefully more accurate picture of the measured quantity. A
popular estimator known as the Kalman Filter is used to do the fusion
and state estimation. A variant of AHRS is an Inertial Navigation System
(INS). The difference is that INS estimates attitude, position and
velocity.

Position Sensors
----------------

Beacons
~~~~~~~

While IMUs are amazing devices, they cannot give us accurate position
information. Estimating the position in the environment is essential for
navigation. One approach is to instrument the environment. An example of
this would be placing markers that the robot sensors can detect and
reliably interpret for location information. We will explore several of
these ideas with beacon systems as our guiding example.

Beacons are probably the simplest approach to localization. A beacon is
any type of landmark with a known location. Natural beacons such as
stars, sun, moon, mountains, streams and other markers have been used
throughout human existence [2]_. Manmade beacons include towers, signs,
lighthouses and other marked locations.

Indoor beacon systems include using colored or IR lights, RFID tags,
ultrasonic transducers, QR codes, colored tags and other forms of
environmental instrumentation. Normally this means that the environment
is modified in some detectable manner. Similar approaches can be done
outdoors, but since the introduction of GPS, it has dominated the
localization approaches. GPS, the Global Positioning System, was
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

.. raw:: latex

   \centering

.. figure:: sensor/gps
   :alt: Global Positioning System - GPS[gpspng]

   Global Positioning System - GPS[gpspng]

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

.. raw:: latex

   \centering

.. figure:: sensor/dgps
   :alt: GPS with local correction.

   GPS with local correction.

Example[partialexample]
'''''''''''''''''''''''

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

.. math:: x^2 - 2a_ix + a_i^2 + y^2 - 2b_iy + b_i^2 = r_i^2 , \quad x^2 - 2a_jx + a_j^2 + y^2 - 2b_jy + b_j^2 = r_j^2

and computing a difference

.. math:: 2(a_j-a_i)x + 2(b_j-b_i)y + a_i^2-a_j^2 + b_i^2-b_j^2 = r_i^2 - r_j^2 .

Using three circle equations, you can obtain two linear equations

.. math:: 2(a_j-a_i)x + 2(b_j-b_i)y  = r_i^2 - r_j^2 - a_i^2 + a_j^2 - b_i^2 + b_j^2

.. math:: 2(a_k-a_i)x + 2(b_k-b_i)y = r_i^2 - r_k^2  - a_i^2 + a_k^2 - b_i^2  + b_k^2  .

In a noise free world, the solution would be where the circles intersect
exactly such as seen in
Figure \ `[fig:exactintersection] <#fig:exactintersection>`__. But this
does not happen due to noise and sensor inaccuracies. The circles do not
intersect as shown in
Figure \ `[fig:inexactintersection] <#fig:inexactintersection>`__.

.. raw:: latex

   \centering

.. figure:: math/hough1
   :alt: Exact intersection of three circles.[fig:exactintersection]

   Exact intersection of three circles.[fig:exactintersection]

.. raw:: latex

   \centering

.. figure:: math/hough2
   :alt: Non-intersection of three circles.[fig:inexactintersection]

   Non-intersection of three circles.[fig:inexactintersection]

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

.. raw:: latex

   \centering

.. figure:: sensor/circerror
   :alt: Radial error function.[fig:radialerror]

   Radial error function.[fig:radialerror]

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

| We can use the gradient descent method to find the solution. Set
  :math:`x_0 = 5000`, :math:`y_0=5000`, :math:`k=0`, :math:`t=1`:
| While (:math:`t > t_0`)

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

.. raw:: latex

   \centering

|image|

The intersection point is :math:`x = 3120, \quad   y = 6085`. Note that
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

.. raw:: latex

   \centering

|image|

Compare the ideal case and the case with noise:

.. raw:: latex

   \centering

|image| |image|

Proximity Sensors
-----------------

Switches
~~~~~~~~

The most elementary sensor available is a switch and commonly used as a
bump or contact sensor on a robot.
Figure \ `[switchdebounce1] <#switchdebounce1>`__-(a) gives the
schematic for a pull down circuit. When the switch is open, the output
(labeled out) is pulled high. This is the connection of the resistor to
the 5V line. For many years, digital logic used 5 volts for the high or
on and zero volts for low or off. This was the TTL standard. It was
based on the bipolar junction transistor technology. With the increasing
popularity of CMOS due to lower power consumption, lower voltage devices
started to appear. More 3.3 volt system entered common use. Currently,
most sensors available at the hobby level are 3.3 volt boards. Many
microcontrollers used in hobby class robots have moved down to 1.8
volts.

|a) Switch and associated circuit. b) Generated signal.
[switchdebounce1]| |a) Switch and associated circuit. b) Generated
signal. [switchdebounce1]|

Lower voltages are used in high performance processors. [Lower voltage
means faster switching.] USB and a number of interface circuits still
run at 5 volts since this was such a standard for so many years. In
terms of the logic it does not matter what voltages are used as long as
high and low levels are easily distinguished. It does matter when you
are attempting to connect a sensor to controller. We will address this
issue later on in the chapter.

When you close a switch (or have a bump sensor contact), it does not
behave like you initially expect. The voltage on the output line is
given in Figure \ `[switchdebounce1] <#switchdebounce1>`__-(b). The
problem jumps right out. There is not a single close and then open. The
problem is that a switch is a mass spring system and will vibrate. At
the contact point, the switch is like the basketball player who lowers
their dribble hand making the ball bounce faster. The switch vibrates
until closed. Since a microcontroller can sample at microsecond
intervals, each one of these bounces appears like a button press. So, we
don’t just generate a single closing of the switch, but we may have
hundreds. You can imagine what this text would look like if the keyboard
did not address this iiisssssuuueeeeee. The process of removing the
false signals, the noise, is called debouncing. There are both hardware
and software solutions to the problem.

The first approach we will discuss is given in
Figure \ `[switchdebounce2] <#switchdebounce2>`__-(a). With the switch
open the output again is tied to the high (the 5 volts). The capacitor
between the output line and ground will be charged (after a short
interval following poweron). When the switch is depressed, the capactor
will discharge through R2. Voltage across a capacitor is the integral of
the current flowing. In English this means that the capacitor will
smooth the voltage level and cut down on the fast oscillations. It
filters out higher frequency noise. The voltage profile is given in
Figure \ `[switchdebounce2] <#switchdebounce2>`__-(b). The reverse
happens when the switch is released. A combination of a resistor and
capacitor filters out higher frequencies and is often called an RC
filter. Using an RC filter can remove the the alternating voltage levels
and appears to solve the problem. However another issue arises.

.. raw:: latex

   \centering 

|a) Basic debounce hardware. b) Signal produced. [switchdebounce2]| |a)
Basic debounce hardware. b) Signal produced. [switchdebounce2]|

The system will spend more time in transition; more time in the zone
between logic high and logic low. This middle region is not stable for
the electronics and can be interpreted by the input of the controller as
either logic level, or even jump back and forth. This again produces
multiple signals. To solve this aspect, we add another device called a
Schmidt trigger. It has a property called hysteresis. Assume for the
moment that the input to the Schmidt trigger is currently set at low
(close to zero volts). As the voltage increases, the trigger output will
stay at low (very close to zero). At some point, the voltage will cross
a threshold, V1, for which the trigger will "fire" and the output
switches to high. In the other direction, if the input is sitting at
high, the output will be high as well (say 5 volts). If the input starts
to drop, the ouput will hold at high until the input crosses a
threshold, V2. Then the output switches to low. So far we don’t have
anything that a transistor can’t do. However, the magic is in that
:math:`V1 > V2`. These values are not the same.

How does that help us? Once the switch is depressed in
Figure \ `[switchdebounce3] <#switchdebounce3>`__, the voltage across
the capacitor starts to drop. But the voltage must drop down to level V2
before the device switches the output to low. Any oscillation above V2
will not change the output. Once the voltage has gone below V2, the
device triggers and now the voltage must rise above V1 before another
change happens. If the values for V1 and V2 fall outside the
indeterminate region for the controller input, we have removed the
ambiguous region, and then have removed the mechanical and electrical
noise.

.. raw:: latex

   \centering 

|Standard hardware approach to debounce. [switchdebounce3]| |Standard
hardware approach to debounce. [switchdebounce3]|

Software solutions are also available and normally approach the problem
by introducing delays in the sampling to allow the switch to settle
down.

Assume that you have your robot completely surrounded by touch sensors -
say 24 sensors. Also assume that your robot has 8 general purpose
input-output (GPIO) lines. Seems like you can only use 8 of the 24. This
is where multiplexing and demultiplexing integrated circuit chips are
really useful, Figure \ `[multiplexer] <#multiplexer>`__. Essentially it
is the memory addressing question. The multiplexer unit can select a
line to read and make the connection from that line to output. The
figure shows 4 input lines, one output line and two select lines. So,
one connects the output line on the multiplexer to the GPIO line
configured as input. Also needed is connecting the two select line to
the GPIO configured as output. With 24 lines, one connects the bottom 5
select lines and the multiplexer output line to six of the GPIO lines.
This leaves two GPIO open for other use.

.. raw:: latex

   \centering

.. figure:: circuit/multiplex
   :alt: Multiplexers and demultiplexers allow one deal with dozens of
   devices and a few GPIO.[multiplexer]

   Multiplexers and demultiplexers allow one deal with dozens of devices
   and a few GPIO.[multiplexer]

The only issue is that you might miss a signal because you were looking
at a sensor on another line. If you know that the signal will last a
minimum amount of time, say 250 ms. Then you need to make sure that you
are running an polling loop that takes less than 250 ms to complete.
More on multiplexing and encoding can be found in basic texts on digital
systems.

Range Sensors
~~~~~~~~~~~~~

Sensors which estimate the distance are known as range sensors. Range
information is one of the main aspects of localization, navigation and
mapping. Note that distance sensors which perform short distance
measurements are sometimes called proximity detectors. The two main
ranging technologies use ultrasound or light. This is a form of active
sensing. The device will emit a short pulse and then listen for an echo.
The time of the echo provides an estimate of distance using the rate
equation. The traveled distance of a sound wave or light wave is given
by

.. math:: d = c\cdot t

is the distance traveled (round trip), :math:`c` is the speed of the
wave, :math:`t` is the time of flight.

From this information, we can also indirectly measure velocity by
looking at the relative displacement of the fixed object over a short
time interval.

Sound and light have vastly different propagation speeds. The speed of
sound is roughly 0.3 meters per millisecond where the speed of light is
0.3 meters per nanosecond. This places light at about one million times
faster. Off-the-shelf electronics are able to time and process the
signals for a ultrasonic basic ranging system. Light is another matter
and is much harder to type. Light based rangers, LIDAR or a laser range
finder is the preferred ranging hardware. Laser range finders are very
accurate, relatively fast and provide a greater number of range points.
The downside is that they cost significantly more and can be delicate
instruments.

The quality of range sensor data depends on several aspects of the
measurement system. Due to discretization, analog to digital conversion,
interrupt handling or polling, uncertainties about the exact time of
arrival of the reflected signal arise and reduce the accuracy of the
estimate. The beam will spread out and makes detection more difficult.
The beam may reflect off of the target in a complicated manner. These
issues can make it more difficult to detect a reflection. Light will
travel in a predictable way as the speed of light does not vary much.
The speed of sound is very different however, variations in the density
of the air or water can introduce errors in the distance estimation. A
more subtle problem can arise if the robot or the target is moving. The
Doppler affect can change the frequency of the reflected signal, and
again introduce errors.

Sonar
^^^^^

Sonar stands for sound navigation and ranging. The idea is to transmit a
packet of ultrasonic pressure waves and listen for the reflection. The
time of flight gives the distance. Distance :math:`d` of the reflecting
object can be calculated based on the propagation speed of sound,
:math:`c`, and the time of flight, :math:`t`:

.. math:: d = \frac{c\cdot t}{2}

The speed of sound, :math:`c` (about 340 m/s), in air is given by

.. math:: c = \sqrt{\gamma R T}

where :math:`\gamma` is the adiabatic index, :math:`R` is the gas
constant, and :math:`T` is the gas temperature in Kelvin.

.. raw:: latex

   \centering

.. figure:: sensor/sonar_echo
   :alt: Sonar Echos

   Sonar Echos

Sonar typically has a frequency: 40 - 180 KHz and so is above most human
hearing although some animals may detect the sonar. The pressure waves
are normally generated by a Piezo transducer. A transducer is any device
that can convert energy in one form to another. In this case, it is a
quartz crystal that vibrates when placed in an oscillating electrical
current (or generates an electric current when deformed or vibrated).

The sound wave from the transducer will propagate out just like a
disturbance in water. Objects will reflect the wave back towards the
transducer. Some systems use the same transducer for transmission and
reception. Others will have separate transducers. The sound will
propagate in a cone shape region with angles varying from 20 to 40
degrees in lower cost units. The vendor will normally provide an
intensity cone that shows signal strength in decibels as a function of
angle.

.. raw:: latex

   \centering

.. figure:: sensor/sonar_details
   :alt: Sonar Cone

   Sonar Cone

One of the obvious problems is with surfaces that absorb a considerable
amount of energy. This could be mistaken for no object at all since no
bounce is required. Surface properties like surface smoothness and angle
of incidence will have a significant impact on the return sign. A
surface that has the surface normal not pointed towards the receiver
will not deliver as much energy and again may produce incorrect results.

Laser Ranging, aka LIDAR
^^^^^^^^^^^^^^^^^^^^^^^^

Laser ranging follows essentially the same ideas that sound ranging
does. Light operates at a greater frequency with a much smaller
wavelength. This allows for much greater resolution. The downside is the
speed of light is so high that it is difficult to measure the return
time directly. LIDAR operates by sending a beam out to a target. That
beam is reflected back. These two beams are parallel which helps in
system design to filter out interference. Once the round trip time is
determined, the distance is easily computed. The laser is placed on a
panning system which then sweeps the field. This will provide a data set
which has angle and distance information from the LIDAR to the targets.

On most systems the round trip time is not timed (since sub nanosecond
timers are required). Time of flight measurement can be done by a phase
shift technique. An interference pattern between the reflected wave and
the emitted wave is setup. The resulting phase shift can be extracted.
This allows one to compute the propagation delay and thus the distance
traveled. A frequency modulated continuous wave is used and the beat
frequency formed by interference between reflected and transmitted waves
form the basis of the phase shift. A pulsed laser is often used instead
of a continuous beam laser. This can reduce power requirements.

From Figure \ `[basiclidarimage] <#basiclidarimage>`__, the beam is
split at point :math:`s`. One branch travels to the object and back, and
then up to the measurement unit for a distance of :math:`L+2D`. The
other branch just travels up to the measurement unit for a distance of
:math:`L`. The difference between these two distances is
:math:`(L+2D) - L = 2D`. This difference can be expressed in terms of
the phase shift:

.. math:: 2D =  \frac{\theta}{2\pi} \lambda

where :math:`\theta` is the phase shift and :math:`\lambda` is the
wavelength. If the total beam distance covered is :math:`D'`, :math:`c`
is the speed of light, :math:`f` is the modulating frequency, we see

.. math::

   D' = L + 2D =  L + \frac{\theta}{2\pi} \lambda ,\quad\quad
   \lambda = \frac{c}{f}.

.. raw:: latex

   \centering

.. figure:: sensor/lidar
   :alt: The basic operational diagram for a laser
   ranger.[basiclidarimage]

   The basic operational diagram for a laser ranger.[basiclidarimage]

.. raw:: latex

   \centering

|image|

For reference, if :math:`f = 5` Mhz then :math:`\lambda = 60` meters.
This allows us to compute :math:`D` as a function of :math:`\theta`

.. math:: D = \frac{\lambda}{4\pi}\theta .

One problem that is immediately clear is that the range estimate is not
unique. This is easy to see. A distance difference of a half wavelength
would generate the same phase shift as 1.5 wavelengths and 2.5
wavelengths and 3.5 wavelengths, etc. For example if
:math:`\lambda = 60` then a target at 5, 35, 65, ... meters will give
the same phase shift.

Example
'''''''

Assume you are using a laser diode to build a distance sensor.

-  What is the wavelength of the modulated frequency of 12MHz?

-  If you measure a 20 degree phase shift, this value corresponds to
   what distances?

-  What other modulation frequency would be a good choice to isolate the
   value? (show this)

-  How would you do the modulation and phase shift measurement?

The wavelength is given by
:math:`\lambda = c/f = 3(10^8)/(12(10^6)) = 25` meters. A 20 degree
shift is :math:`(20/360)` of the wavelength, so we get

.. math:: (20/360)*25 \approx 1.389m

The actual distance is 1/2 of this value since the beam travels to the
obstacle and back: :math:`0.6945m` but we will do our computations on
the full trip and then at the very end, cut our number on half. This
would correspond to 1.389, 26.389, 51.389, 76.389, 101.389, 126.389, or

.. math:: 1.389 + 25n ~\mbox{for}~ n=0,1,2,3 ...

If you select different frequencies that are multiplies of each other,
say 5MHz and 10MHz, you can see that it does not help much. You need
frequencies that are different enough. As long as our values are
relatively prime, frequency selection is pretty open. Factors of 12 are
2, 3, 4. So 5 Mhz would work (as would 17 Mhz and many others) for some
distance out. Using 5Mhz, we have a wavelength of 60 meters. For the
moment assume the distance was 26.389 (which would be a phase shift of
0.4398), then the 5Mhz would produce distances of

.. math:: 26.389+60m, \quad m=0,1,2 ...

\ as values. To find where the wavelengths give the same value, set

.. math:: 1.398 + 25n = 26.389 + 60m,

 and obtain

.. math:: m = 5(n-1)/12.

We thus need :math:`5(n-1)/12` to be an integer for these two to agree.
Inspection tells us that :math:`n-1 = 12` or :math:`n=13`. When
:math:`n=13` then :math:`m=5`. If you don’t see this, then you can run a
simple Python program to check. Step up the values:
:math:`n=0,1,2,3 ...` and see when you get an integer for :math:`m`:

::

    >>> for n in range(20):
    ...   m = 5.0*(n-1)/12.0
    ...   print "n = ", n, "  m = ", m
    ... 

The output becomes

2

::

    n =  0   m =  -0.416666666667
    n =  1   m =  0.0
    n =  2   m =  0.416666666667
    n =  3   m =  0.833333333333
    n =  4   m =  1.25
    n =  5   m =  1.66666666667
    n =  6   m =  2.08333333333
    n =  7   m =  2.5
    n =  8   m =  2.91666666667
    n =  9   m =  3.33333333333
    n =  10   m =  3.75
    n =  11   m =  4.16666666667
    n =  12   m =  4.58333333333
    n =  13   m =  5.0
    n =  14   m =  5.41666666667
    n =  15   m =  5.83333333333
    n =  16   m =  6.25
    n =  17   m =  6.66666666667
    n =  18   m =  7.08333333333
    n =  19   m =  7.5

So :math:`m=5`. This gives isolation out to about 165 meters using two
waves with a much shorter wavelength.

Just as with sonar, errors can arise based on the hardware construction
and the reflected object surface. Confidence in the range (phase/time
estimate) is inversely proportional to the square of the received signal
amplitude. Dark distant objects do not produce as good of range estimate
as closer brighter objects.

.. raw:: latex

   \centering

|image|

.. raw:: latex

   \centering

.. figure:: sensor/lidarmap
   :alt: Typical range image of a 2D laser range sensor with a rotating
   mirror. The length of the lines through the measurement points
   indicate the uncertainties.

   Typical range image of a 2D laser range sensor with a rotating
   mirror. The length of the lines through the measurement points
   indicate the uncertainties.

.. raw:: latex

   \FloatBarrier

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

