Sensor Noise and Measurement
----------------------------

The main concern of this chapter is the errors involved with the
measurements. There are plenty of ways error can enter. A brief list of
error types is given below.


-  Intrinsic ability of the sensor

-  Connection of the sensor to the world

-  Connection of the sensor to the electronics

-  Sampling and Aliasing

-  Interference

-  Precision

-  Accuracy

-  Signal to noise ratio

The current values of the variables in the system is called the *state*.
Often this is represented as :math:`x_k` (a vector of real values), the
state at time step :math:`k` (an integer). By :math:`x_k` we mean the
system’s configuration which includes pose, dynamics, and internal
measurements that are relevant to the problem. The temperature of the
CPU may be an important variable in the system, but probably has little
to do with localization. So :math:`x_k` does not contain everything. All
of the sensors have uncertainty, i.e. they are very noisy. For example,
smooth surfaces cause sonar and lasers to reflect less back to the
source and thus give wrong ranging results. This is known as specular
reflection. Normally this results in estimating the object as much
further away. So we don’t know the robot’s state. We can only estimate
it. To gain an accurate estimate, we must model the type of error
present in the system. Clearly we design the system to minimize the
errors, but one cannot design them all away. The greatest error normally
encountered is with the sensor itself. This will be our focus.

How can we model this error or uncertainty? Error is modeled using
probabilistic tools. Typically we ask "what is the error of this
measurement for a particular state"? We can define :math:`p(z_k|x_k)` as
the probability or likelihood of getting the measurement value
:math:`z_k` given we are reading state :math:`x_k`. Can we determine or
model :math:`p(z_k|x_k)`? Again, the question we ask here is ... what is
the current status of the robot? What are all the values of all the
relevant parameters for the robot’s relation to the environment?

If we happen to know something about the environment, say we have a map
and a set of expectations based on that map, does this change our
probability? This is pretty intuitive. If my previous location was near
San Francisco and as a ground robot can only travel at some speed, then
the probability of seeing the Eiffel tower should be low. In this case
can we write down :math:`p(z_k|x_k,m_k)` where :math:`m_k` is a map of
the environment?

In many sensing systems we may have redundant measurements of some
quantities. We may actually measure combinations of components and
possibly miss some. For example, I might be able to measure velocity but
not position. I might know speed but not the components of velocity.
This means that the measurement :math:`z` is some function of the state
:math:`x` with errors: :math:`z = h(x) + \delta`. Is it possible to
determine :math:`h` and :math:`\delta`? We will normally have an array
of values :math:`z_k = \{ z_k^1, z_k^2, \dots
, z_k^k\}`. We will assume the measurements are independent:

.. math:: p(z|x,m) = \prod_{k=1}^{N}p(z_k|x,m).

What is involved in a measurement? What can activate the sensor?
Measurement can be caused by:


-  a known obstacle

-  interference

-  other obstacles

-  random events

-  maximum range

-  sensor or software errors

The error or noise enters in the measurement of known item, the position
of known item, the position of other obstacles and as a missing item.




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
:numref:`fig:lowpass`.

Low Pass Filter (integration based)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

::

    for i from 1 to n
           y[i] := y[i-1] + a * (z[i] - y[i-1])




.. code-block:: julia

    using Random, Distributions

    N = 1000
    sigma = 1.0
    r = Normal(0, sigma)
    n = rand(r, N)

    t = range(0,12,length=N)
    x = sin.(t)
    z = x + n
    y = zeros(N)

    y[1] = z[1]
    for i = 1:N
        y[i] = y[i] + 0.075*(z[i] - y[i])
    end



.. _`fig:noisysignal1`:
.. figure:: FilteringFigures/noisefilter1.*
   :width: 50%
   :align: center

   Signal in red, noisy version of the signal in blue.

.. _`fig:lowpass`:
.. figure:: FilteringFigures/noisefilter2.*
   :width: 50%
   :align: center

   Noisy signal in blue, filtered signal in green.

Differentiation will set constants to zero and attenuate low
frequencies, filters based on differentiation formulas are employed. One
such formula is given below. The output of this filter is given in
:numref:`fig:highpass`.

High Pass Filter (differentiation based)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

::

    for i from 1 to n
         y[i] := a * (z[i] - z[i-1])

.. _`fig:noisysignal3`:
.. figure:: FilteringFigures/noisefilter3.*
   :width: 50%
   :align: center

   Signal in red, noisy version of the signal in blue.

.. _`fig:highpass`:
.. figure:: FilteringFigures/noisefilter4.*
   :width: 50%
   :align: center

   Noisy signal in blue, filtered signal in green.

A variation of the high pass filter is

::

    for i from 1 to n
         y[i] := a * (y[i-1]  + z[i] - z[i-1])

The band pass filter is a filter which allows a range of frequencies to
pass through. One may simply try applying both a low and high pass
filter. Although filters are easy to understand and to implement,
designing them for a specific application can be challenging.
