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
