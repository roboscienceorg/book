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

.. figure:: SensorsFigures/gyro.*
   :width: 40%
   :align: center

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

.. figure:: SensorsFigures/magneticencoder.*
   :width: 40%
   :align: center

   Hall-Effect based shaft rotation sensor.[halleffect]
