Sensor Noise and Measurement
----------------------------

The main concern of this chapter is the errors involved with the
measurements. There are plenty of ways error can enter. A brief list of
error types is given below.

2

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

2

-  a known obstacle

-  interference

-  other obstacles

-  random events

-  maximum range

-  sensor or software errors

The error or noise enters in the measurement of known item, the position
of known item, the position of other obstacles and as a missing item.
