Orientation Sensors
-------------------

The word pose is used for both position and orientation. Measurement of
orientation is done through several basic sensing approaches.

Gyroscopes
~~~~~~~~~~

A :index:`gyroscope` is a heading sensor that gives a measure of orientation with
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

.. _`gyroscope`:
.. figure:: SensorsFigures/gyro.*
   :width: 40%
   :align: center

   Tuning fork MEMS gyroscope.

Compass
~~~~~~~~

The :index:`compass` or :index:`magnetometer` is one of the oldest
sensors in use dating back 4000 years.
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

It is possible to use :index:`Hall-Effect` or other similar devices to do
encoding. Small Hall-Effect sensors with sub-degreee accuracy are
available. Placing a small ceramic magnet on the end of a shaft will
generate a rotating magnetic field which can be detected with the
Hall-Effect sensor. Figure :numref:`halleffect` shows how
this is done.


.. _`halleffect`:
.. figure:: SensorsFigures/magneticencoder.*
   :width: 40%
   :align: center

   Hall-Effect based shaft rotation sensor.



Inertial Sensors
----------------

Accelerometers
~~~~~~~~~~~~~~

An :index:`accelerometer` measures acceleration in a particular direction. The
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

.. _`accelerometer`:
.. figure:: SensorsFigures/accel.*
   :width: 40%
   :align: center

   Simple accelerometer structure.

A simple application of an accelerometer is an :index:`inclineometer` or tilt
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

An :index:`Inertial Measurement Unit` or :index:`IMU` packages accelerometers, gyroscopes
and possibly a compass together into a single unit. A 6DOF (degrees of
freedom) IMU will have a three axis accelerometer and three axis
gyroscope. A 9DOF IMU will have the three axis accelerometer, three axis
gyroscope and a 3 axis magnetometer. These devices normally provide a
digital interface such as USB and return text strings of data at some
Hz. IMUs are used as the basis for AHRS: Attitude and Heading Reference
System.

Attitude and Heading Reference System (AHRS)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

:index:`AHRS` consist of accelerometers, gyroscopes and magnetometers on all
three axes. So, AHRS includes an IMU. In addition to the IMU, the AHRS
has the algorithms to provide attitude and heading information as well
as the required hardware for the computation. These algorithms include
sensor fusion codes which take data from multiple sources and "fuse"
them into a hopefully more accurate picture of the measured quantity. A
popular estimator known as the Kalman Filter is used to do the fusion
and state estimation. A variant of AHRS is an Inertial Navigation System
(INS). The difference is that INS estimates attitude, position and
velocity.
