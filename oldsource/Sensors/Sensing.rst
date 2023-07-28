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
Sensors fall into two classes: :index:`passive sensors` and
:index:`active sensors`. **Passive Sensors**
use energy from the environment to power the measurement. A bump or
temperature sensor are examples of passive sensing. **Active sensors**
inject energy into the environment in a particular manner and measure
the reaction. Active sensing can often get better results, but at an
increased complexity, cost, and power requirements. It could also
require the modification of the surrounding environment, such as the
placement of beacons or tags. Laser and ultrasonic ranging are examples
of active sensing.

We can further classify sensors by which part of the robot’s
environment, :index:`properioceptive` or :index:`exterioceptive`,
they are sensing. **Properioceptive
Sensors** measure the internal state of the system (robot), such as
motor speeds, wheel loads, turn angles, battery status, temperature and
other aspects that are internal to the machine. **Exterioceptive
Sensors** measure information from the robot’s external environment;
external to the robot, such as distances to objects, GPS, ambient light
and temperature, magnetic fields, accelerations, etc.

Another way to classify sensors is by the data they return. Sensors can
return information in **analog** or **digital** form. Typically an
:index:`analog sensor` will vary voltage (maybe current) as the measured quantity
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
is a tradeoff.  A :index:`digital sensor` is a sensor that returns the measurement
already in digitized or packet form and there is no need for an analog to digital
conversion.

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
