Position and Velocity Sensors
-----------------------------

Navigation and localization is one of the more challenging problems in
mobile robotics and any estimate is welcome. If you have an estimate on
wheel velocity then you can by integration estimate rotation and through
wheel diameter estimate the linear travel (over very short distances).
Using the differential drive equations,
:eq:`ddkinematicsmodel`, the wheel velocity may be
integrated to determine position in the global or inertial reference
frame. So the obvious question is... how do we determine wheel position
or velocity?

:index:`Position sensors` measure the absolute displacement of a joint or other
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

A direct digital approach is to use :index:`encoders`. Absolute encoders return a
Gray code as a function of position. These also come in linear and
rotary designs. Typically one has a collection of sensors which can
detect light and dark. The encoder allows light to pass through it in
some pattern that correlates to position. The sensed pattern is
converted to the output code. Another approach used is to count the
number of times a particular pattern occurs or a beam is interrupted.
This is explored in detail below with the optical wheel encoders which
can be used for velocity in addition to position estimates.

.. _`fig:positionsensor`:
.. figure:: SensorsFigures/positionsensor.png
   :width: 90%
   :align: center

   Position sensors:  (a) Potentiometer, (b) Fader, (c) Encoder.


Tachometers
~~~~~~~~~~~

An electric motor and a generator are very similar devices which just
operate in opposite fashions. Providing electrical power in a motor
causes the shaft to turn. Conversely turning the shaft of a generator
produces electricity. A :index:`tachometer` can be built out of a generator (or
electric motor). The faster the shaft spins, the greater the voltage or
higher the frequency produced. This can be converted to a digital signal
and thus provides a measure of rpm.

Optical Wheel Encoders
~~~~~~~~~~~~~~~~~~~~~~

One option to tackle this problem involves using Light Emitting Diodes,
or LEDs [1]_. The dominant lighting source in electronics and robotics,
LEDs can run on very low power, are available in many frequencies and
can switch on/off quickly. :numref:`circuitled`.

.. _`circuitled`:
.. figure:: SensorsFigures/LED.*
   :width: 25%
   :align: center

   LED

LEDs can emit in non-visible ranges, ultraviolet and infrared. Many of
the non-visible frequencies are popular for simple object detection in
combination with a phototransistor,
:numref:`IRobstacleLED`. In this example, the
infrared LED shines on some object and is reflected back to the
phototransistor. The IR light activates the transistor and causes it to
switch on and pull the output to low.

.. _`IRobstacleLED`:
.. figure:: SensorsFigures/IRObs.*
   :width: 35%
   :align: center

   Infrared LEDs used for obstacle detection.

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


.. _`mountingencoder`:
.. figure:: SensorsFigures/sensormount.*
   :width: 45%
   :align: center

   Mounting for the encoder sensor

There are two basic components needed to build your own. First you need
the light source and the detector. Second you need an encoder. To read
the encoder, you will need an optical sensor. Typically one uses an IR
LED (IR light emitting diode) and phototransistor pair,
:numref:`ledopticalsensor`. These are packaged
in single units, for example the Fairchild QRD1313. This has the LED and
the phototransistor packaged into a unit that is 6.1mm x 4.39mm x 4.65mm
(height).

.. _`ledopticalsensor`:
.. figure:: SensorsFigures/IR2.*
   :width: 60%
   :align: center

   IR LED (IR light emitting diode) and phototransistor pair.

An encoder pattern may simply be a pattern printed on paper and attached
(glued) to the inside of a robot wheel. Simple encoder patterns are just
alternating black and white radial stripes. Two examples are given in
:numref:`encoderpattern`.

.. _`encoderpattern`:
.. figure:: SensorsFigures/EncoderPatterns.*
   :width: 75%
   :align: center

   Wheel encoder pattern (a) with 1-1 ratio, (b) with 1-4 ratio.



Doppler Effect
~~~~~~~~~~~~~~

Direct measurement of velocity may be achieved by using the :index:`Doppler
Effect`. Recall when a vehicle passes by, you notice a change in the
sound of the machine. The sound waves are compressed as the vehicle
approaches and are expanded as the vehicle retreats. This compression
results in a higher frequency of the sound and so as the vehicle passes,
you hear the drop in frequency. Transmitting a known frequency and
listening to the reflected sound, one can estimate the relative
velocity.


.. figure:: SensorsFigures/doppler.*
   :width: 45%
   :align: center

   Using the Doppler Effect to estimate velocity.

The formula that describes the change in frequency for a moving sound
source (a transmitter) is

.. math:: f_r = f_t (1 + v/c).

If the receiver is moving the formula for the frequency change is

.. math:: f_r = f_t / (1 + v/c).

If you know the frequency change you can then compute :math:`v`.
