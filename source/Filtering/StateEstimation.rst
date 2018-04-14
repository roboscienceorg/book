State Estimation
----------------

State variables and Errors
~~~~~~~~~~~~~~~~~~~~~~~~~~

Recall that for any state variable we have

-  True value: :math:`y`

-  Measured value: :math:`\tilde{y}`

-  Estimated value: :math:`\hat{y}`

The true value is not known. It is what we seek. The measured value
comes from the sensor which is subject to error as listed above.
Estimated value comes from measurement and system model.

Basic errors that are used:

-  Measurement error: :math:`v = y - \tilde{y}`

-  Residual error: :math:`e = \tilde{y} - \hat{y}`

We don’t know :math:`v` obviously. The residual error, :math:`e`, is
based on a model of the system and is known explicitly. Basic error
types: we are concerned with two fundamental error types

-  Systematic error - deterministic

-  Random error - non-deterministic

Systematic errors are errors of design or implementation:

2

-  Incorrectly mounted sensor

-  Blocked sensor

-  Sensor biased by hardware

-  Sampling issues

-  Resolution issues

-  Incomplete measurements

-  Sensitivity

-  Nonlinearity

Random errors

-  Based on white or Gaussian noise

-  Actually could be any distribution, but Gaussian is standard.

Filters
~~~~~~~

The idea of filtering is to use the measurement PLUS the model to
provide a better estimate of the state. Finding the model may be the
hardest part but the part that makes the process effective. For example,
it is where the Kalman Filter enters. The Kalman filter uses a linear
time stepping model and environmental data to improve state estimation.

What does one mean by filter? In this case we are attempting to filter
out noise. Simple filters in signal processing often filter in the
frequency domain. For example filtering out high frequencies since this
is often noise. We can filter out noise by fitting the data to a model.
We assume that the data represents a constant and so we can compute the
mean of the data. This model can also be represented by the distribution
that the data appears to have come from, e.g. a normal distribution.

The distribution that the data comes from can change over time. If two
data items come from the same distribution then we have some reason to
believe a mean is a good filter value. If not, how do we balance data
items which have different reliability?

High and Low pass filters
~~~~~~~~~~~~~~~~~~~~~~~~~

Assume that you have digitized signal, meaning the analog sensed values
have been converted to numerical values sampled at regular times. Call
that signal :math:`\{ z[n]\}`. If that signal has high frequency noise
(static or white noise), how can you eliminate or filter out that noise?
If the signal has low frequency noise (like mechanical oscillations or
other forms of bias), can this be filtered out. The answer is yes. Two
common filters are low and high pass filters. The low pass refers to the
filter allowing low frequencies through but filtering out the higher
frequencies like the static. [And similarly for the high pass filter.]

Since integration tends to smooth out signals, we use an integration
formula that has an exponential decay built in. This removes the high
frequencies (the static) and leaves the core signal. The algorithm is
given below. Sample output may be found in
Figure \ `[fig:lowpass] <#fig:lowpass>`__.

Low Pass Filter (integration based)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

::

    for i from 1 to n
           y[i] := y[i-1] + a * (z[i] - y[i-1])

The Python code

::

    import numpy as np
    import pylab as plt

    N = 1000
    sigma = 1.0
    n = np.random.normal(0,sigma,N)
    t = np.linspace(0,12,N)
    x = np.sin(t)
    z = x + n
    y = np.zeros(N)

    y[0] = z[0]
    i = 1
    while(i<N):
        y[i] = y[i-1] + 0.075*(z[i] - y[i-1])
        i = i+1

.. raw:: latex

   \centering

.. figure:: filter/noisefilter1
   :alt: Signal in red, noisy version of the signal in
   blue.[fig:noisysignal1]

   Signal in red, noisy version of the signal in blue.[fig:noisysignal1]

.. figure:: filter/noisefilter2
   :alt: Noisy signal in blue, filtered signal in
   green.[fig:noisysignal2]

   Noisy signal in blue, filtered signal in green.[fig:noisysignal2]

Differentiation will set constants to zero and attenuate low
frequencies, filters based on differentiation formulas are employed. One
such formula is given below. The output of this filter is given in
Figure \ `[fig:highpass] <#fig:highpass>`__.

High Pass Filter (differentiation based)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

::

    for i from 1 to n
         y[i] := a * (z[i] - z[i-1])

.. raw:: latex

   \centering

.. figure:: filter/noisefilter3
   :alt: Signal in red, noisy version of the signal in
   blue.[fig:noisysignal3]

   Signal in red, noisy version of the signal in blue.[fig:noisysignal3]

.. figure:: filter/noisefilter4
   :alt: Noisy signal in blue, filtered signal in
   green.[fig:noisysignal4]

   Noisy signal in blue, filtered signal in green.[fig:noisysignal4]

A variation of the high pass filter is

::

    for i from 1 to n
         y[i] := a * (y[i-1]  + z[i] - z[i-1])

The band pass filter is a filter which allows a range of frequencies to
pass through. One may simply try applying both a low and high pass
filter. Although filters are easy to understand and to implement,
designing them for a specific application can be challenging.

Complementary Filter
~~~~~~~~~~~~~~~~~~~~

Assume that you have two different sensors (measurements from two
different sources) in which one sensor has high frequency noise and the
other sensor has low frequency noise. A complementary filter exploits
this situation by applying a low pass filter to the first sensor data
and a high pass filter to the second sensor. The two signals
“complement” each other in terms of information.

.. raw:: latex

   \centering

.. figure:: filter/complementary
   :alt: [fig:complementary]Complementary Filter

   [fig:complementary]Complementary Filter
