Sensor Fusion
-------------

Simple Example of Sensor Fusion
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Consider a system with :math:`n` sensors each making a single
measurement:

.. math:: z_i, \quad i=1, \dots, n

of some unknown quantity :math:`x`. The measurements really are
described by

.. math:: z_i = x + v_i,  \quad i=1, \dots, n .

We seek an optimal estimate of :math:`x` based on a linear combination
of these measurements:

.. math:: \hat{x} = \sum_{i=1}^n k_i z_i .

How should we proceed? It is a matter of writing down an expression for
the error in the estimate and minimizing the error.

We will assume that the noise :math:`v_i` is zero mean white noise
(normally distributed) and independent. Thus we have that

.. math:: E(v_i) =0, \quad \mbox{and} \quad E(v_i v_j) = 0, \quad i\neq j,

where :math:`E(x)` is the expected value of :math:`x`. We want the
estimate to be unbiased which means that :math:`E(\hat{x}-x) = 0`. We
define optimality as minimizing the mean square error:

.. math:: E[(\hat{x}-x)^2].

:math:`E[\hat{x}-x]=0` implies :math:`\sum_{i=1}^n k_i = 1`

An unbiased estimate means that :math:`E(\hat{x}-x) = 0`,

  .. math:: E[\hat{x}-x] = E\left[\sum_{i=1}^n k_i z_i - x\right] = E\left[\sum_{i=1}^n k_i (x+v_i) - x\right]

  .. math::

     = E\left[\sum_{i=1}^n k_i x - x + \sum_{i=1}^n k_i v_i\right]
     = \sum_{i=1}^n k_i E[x] - E[x]  + \sum_{i=1}^n k_i E[v_i] = 0

since :math:`E(v_i)=0` and :math:`E(x)=x` we have that

  .. math:: \sum_{i=1}^n k_i = 1 .

[Lem:varianceformula]

.. math:: E[(\hat{x}-x)^2] =  \sum_{i=1}^n k_i^2\sigma_i^2

where :math:`\sigma_i` are the standard deviations for :math:`v_i`,
:math:`E((v-E(v))^2)=\sigma^2`.

.. math::

   E[(\hat{x}-x)^2] =  E\left[\left(\sum_{i=1}^n k_i z_i - x\right)^2\right]
   =  E\left[\left(\sum_{i=1}^n k_i (x+v_i) - x\right)^2\right]

.. math::

   = E\left[\left(\sum_{i=1}^n k_i x - x + \sum_{i=1}^n k_i v_i \right)^2\right]=
   E\left[\left(\sum_{i=1}^n k_i v_i \right)^2\right]

.. math::

   =E\left[\sum_{i=1}^n \sum_{j=1}^n k_ik_j v_iv_j \right]
   = \sum_{i=1}^n \sum_{j=1}^n k_ik_j E[v_iv_j] = \sum_{i=1}^n k_i^2\sigma_i^2 .


Optimal Estimate
^^^^^^^^^^^^^^^^

The main goal is to minimize the mean square error subject to the
constraint of having the unbiased estimate:

-  Minimize :math:`\sum_{i=1}^n k_i^2\sigma_i^2` (minimize mean square
   error),

-  Subject to :math:`\sum_{i=1}^n k_i = 1` (unbiased estimate).


We proceed using Lagrange Multipliers which will allow us to optimize a
constrained function. Expressing as the Lagrangian

.. math:: L = \sum_{i=1}^n k_i^2\sigma_i^2 - \lambda \left( \sum_{i=1}^n k_i - 1\right)

we must solve

.. math:: \nabla_k L =0 \quad \text{with} \quad \sum_{i=1}^n k_i = 1 .

Thus

.. math::

   \nabla L =
   \left[ 2k_1\sigma_1^2 - \lambda , 2k_2\sigma_2^2 - \lambda, \dots, 2k_n\sigma_n^2 - \lambda\right]=\vec{0}

.. math:: \sum_{i=1}^n k_i = 1

Solve for :math:`k_i` in each gradient equation and sum

.. math:: \sum_{i=1}^n k_i =  \sum_{i=1}^n \frac{\lambda}{2\sigma_i^2} = 1

So, we have that

.. math:: \lambda =  \left(\displaystyle\sum_{i=1}^n \displaystyle \frac{1}{2\sigma_i^2}\right)^{-1}

This provides :math:`k_i`

.. math:: k_i = \frac{1}{\sigma_i^2} \left(\displaystyle\sum_{i=1}^n \displaystyle \frac{1}{\sigma_i^2}\right)^{-1}

and we obtain the estimate

.. math::

   \hat{x} = \displaystyle \frac{\displaystyle \sum_{i=1}^n \frac{z_i}{\sigma_i^2}}
   {\displaystyle \sum_{i=1}^n \frac{1}{\sigma_i^2}}.

From Lemma \ `[Lem:varianceformula] <#Lem:varianceformula>`__ we can
also gain an estimate of the variance for the estimate, :math:`\hat{x}`
above:

.. math::
   :label: `Eq:weightaveragevariance`

   \sigma^2 =  \sum_{i=1}^n k_i^2\sigma_i^2 =  \sum_{i=1}^n\left( \frac{1}{\sigma_i^2} \left(\displaystyle\sum_{i=1}^n \displaystyle \frac{1}{\sigma_i^2}\right)^{-1}\right)^2 \sigma_i^2

.. math::
   =  \left(\displaystyle\sum_{i=1}^n \displaystyle \frac{1}{\sigma_i^2}\right)^{-2} \sum_{i=1}^n\left( \frac{1}{\sigma_i^2} \right) =  \left(\displaystyle\sum_{i=1}^n \displaystyle \frac{1}{\sigma_i^2}\right)^{-1}

Simple example using uniform variance
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If the variances are the same, :math:`\sigma_i = \sigma`, then

.. math:: \sum_{i=1}^n \frac{1}{\sigma_i^2} = \frac{1}{\sigma^2} \sum_{i=1}^n 1 = \frac{n}{\sigma^2}

and so

.. math::

   \hat{x} = \displaystyle \frac{\displaystyle \frac{1}{\sigma^2} \sum_{i=1}^n z_i}
   {\displaystyle \frac{n}{\sigma^2}} = \displaystyle \frac{1}{n} \sum_{i=1}^n z_i

which is the average.

.. _`dataexamplediffvar`:
Example with different variances
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Say you measure something three different ways and you want to merge
these measurements into a single estimate. How does one specifically go
about it. Assume that the three devices do return normally distributed
measurements. But what is the actual distribution? Keep in mind for
normal distributions, we only need to track the mean and standard
deviation and those are complete descriptors for the distribution.

Recall that the mean and the standard deviation are

.. math:: \mu = \frac{1}{n}\sum_{i=1}^n x_i, \quad\quad\sigma = \sqrt{\frac{1}{n-1} \sum_{i=1}^n (x_i - \mu)^2}

Assume that you sample three sensors with 20 measurements each for some
experiment. The data you gain is

::

    2.28333   1.87365    2.12419
    2.26493   1.77675    1.80968
    2.33626   1.85706    2.00608
    2.13676   1.83520    2.12145
    ... (middle removed to fit)
    2.14289   1.86792    1.86616
    2.21151   1.88855    2.20027
    2.17112   1.95257    1.77513
    2.19798   1.82083    2.25617
    Means:
    2.20548   1.85962    2.04204
    Standard Deviations:
    0.08698   0.04282    0.17674

The normal curves for the three sensors are

.. math:: P_i(x|\mu, \sigma) = \displaystyle\frac{1}{\sigma_i\sqrt{2\pi}}\, e^{\displaystyle-\frac{(x-\mu_i)^2}{2\sigma_i^2}}

and are given in :numref:`normalcurves`.

.. _`normalcurves`:
.. figure:: FilteringFigures/fusiondemo1.*
   :width: 50%
   :align: center

   The normal curves for the three sensors. Sensor A is shown in red,
   sensor B in green and sensor C in blue.

Assume the experimental setup was such that the true measurement was
2.0. The difference between the true measurement and the sensor average
constitutes the systematic error. It is a constant bias term which can
be removed. You need to compute the difference between the true value
and the dataset average. This provides the amount you need to shift your
measurement value:

::

    Shift data
    x shift (add) =  -0.205476607108
    y shift (add) =  0.140376647675
    z shift (add) =  -0.0420388951565

.. _`Fig:shiftednormalcurves`:
.. figure:: FilteringFigures/fusiondemo2.*
   :width: 50%
   :align: center

   The shifted curves for the three sensors. Sensor A is shown in red,
   sensor B in green and sensor C in blue.

Once you have the standard deviations, we can perform a single
measurement using the three sensor and then merge the three into a
single estimate of the state. Assume you get the following measurements
for sensors A, B and C respectively: 2.22685 1.90326 2.17253. Then the
corrected measurements for sensors A, B and C are :math:`z_1 = 2.02137`,
:math:`z_2 =  2.04363`, :math:`z_3 =  2.13049`.

Using the weighted sum derived above, we can fuse the measurements based
on standard deviations.

.. math::

   \hat{x} = \displaystyle \frac{\displaystyle \sum_{i=1}^n \frac{z_i}{\sigma_i^2}}{\displaystyle
   \sum_{i=1}^n \frac{1}{\sigma_i^2}} =
   \displaystyle \frac{\displaystyle  \frac{ 2.02137}{0.08698^2} + \frac{2.04363}{0.04282^2}    + \frac{2.13049}{0.17674^2}  }{\displaystyle
    \frac{ 1}{0.08698^2} + \frac{1}{0.04282^2}    + \frac{1}{0.17674^2}  } = 2.1063 .

The variance for this measurement is given by :math:`\sigma^2 =`

.. math::

   \left(\displaystyle\sum_{i=1}^n \displaystyle \frac{1}{\sigma_i^2}\right)^{-1}
    = \left( {\displaystyle
    \frac{ 1}{0.08698^2} + \frac{1}{0.04282^2}    + \frac{1}{0.17674^2}  } \right)
    \approx 0.03754^2

Note that the standard deviation is lower than all three of the
estimates, meaning the fused measurement is more accurate than any of
the measurements alone.

The code to implement the data fusion is given below. We assume we
already have three Numpy arrays (the sensor data arrays) filled with the
20 sensor test readings.

::

    a_shift = 2.0 - np.mean(sensor_a_data)
    b_shift = 2.0 - np.mean(sensor_b_data)
    c_shift = 2.0 - np.mean(sensor_c_data)

    a_std = np.std(sensor_a_data)
    b_std = np.std(sensor_b_data)
    c_std = np.std(sensor_c_data)

    x = sensor_a + a_shift
    y = sensor_b + b_shift
    z = sensor_c + c_shift

    print "Measurement: "
    print '{0:.5f}   {1:.5f}    {2:.5f}'.format(sensor_a, sensor_b, sensor_c)
    print "Corrected measurement: "
    print '{0:.5f}   {1:.5f}    {2:.5f}'.format(x, y, z)

    cdarray = np.array([x, y, z])
    sdarray = np.array([a_std, b_std, c_std])
    sdarray2 = sdarray*sdarray
    top = np.dot(sdarray2,cdarray)
    bottom = np.dot(sdarray2,np.ones((3)))
    print "Estimate = ", top/bottom

Assume you have two sensors, one good one and one that is no accurate at
all. Does it really make sense to always merge them? Seems like the
better sensor will always produce a more accurate measurement.


Given two sensors, does it always make sense to combine their
measurements? Assume that you have two variances:
:math:`\sigma_1^2 = 1`, :math:`\sigma_2^2 = 5`. The first sensor is
clearly better than the second. The variance formula for the combined
measurement is

.. math:: \frac{1}{\sigma^2} = \frac{1}{1} + \frac{1}{5} = 1.2 \quad \Rightarrow \quad \sigma^2 \approx 0.833.

The example showed a lower variance on the combined measurement. This is
true in general as the next result demonstrates. The fused measurement
is more accurate than any individual measurement.

For the weighted averaging process, we have that
:math:`\sigma^2 < \sigma_i^2` for all measurements :math:`i`.

.. math::

   \sigma^2 = \left(\displaystyle\sum_{i=1}^n \displaystyle \frac{1}{\sigma_i^2}\right)^{-1} \quad \Rightarrow
   \quad \displaystyle \frac{1}{\sigma^2} = \sum_{i=1}^n \displaystyle \frac{1}{\sigma_i^2}

.. math::

   \displaystyle \frac{1}{\sigma^2} =  \frac{1}{\sigma_k^2} +  \sum_{i=1, i\neq k}^n \displaystyle \frac{1}{\sigma_i^2} >
    \frac{1}{\sigma_k^2}

.. math:: \displaystyle \frac{1}{\sigma^2} >  \frac{1}{\sigma_k^2}    \quad \Rightarrow \quad \sigma^2 < \sigma_k^2

So there is value in including measurements from lower accuracy sensors.

Recursive Filtering
~~~~~~~~~~~~~~~~~~~

Say that you have computed an average over a dataset and another value
is added to the dataset. Using the previous formula, you need to repeat
the summation. However, it is clear that you are repeating much of the
work done before. We can rewrite the expression to simply update the
formula and build a running average formula. This is the first step to
recursive filtering. The average is given by

.. math:: \hat{x}_n = \displaystyle \frac{1}{n}\sum_{i=1}^n z_i

A new data point provides a new estimate:

.. math:: \hat{x}_{n+1} = \displaystyle \frac{1}{n+1}\sum_{i=1}^{n+1} z_i

Pull the last value out of the sum and rework the weight in front of the
sum:

.. math:: \hat{x}_{n+1} = \displaystyle \frac{n}{n+1}\left(\frac{1}{n}\sum_{i=1}^{n} z_i\right) + \frac{1}{n+1}z_{n+1}

.. math:: = \displaystyle \frac{1}{n+1}\left( n\hat{x}_n + z_{n+1}\right)

.. math:: = \displaystyle \frac{1}{n+1}\left( (n+1)\hat{x}_n + z_{n+1} - \hat{x}_n\right)

.. math:: = \displaystyle \frac{n+1-1}{n+1}\hat{x}_n + \frac{1}{n+1}z_{n+1}

.. math:: =  \displaystyle \hat{x}_n - \frac{1}{n+1}\hat{x}_n + \frac{1}{n+1}z_{n+1}

.. math:: = \displaystyle \hat{x}_n  + \frac{1}{n+1}\left( z_{n+1}-\hat{x}_n\right) .

Thus we have

.. math:: \hat{x}_{n+1} = \hat{x}_n + K_n\left( z_{n+1} - \hat{x}_n\right), \quad K_n = \displaystyle \frac{1}{n+1} .

Take the first column of the data set in
:numref:`dataexamplediffvar`. Assume that
you want to do this as a running average over the N points contained in
the file.

::

    x = 0
    n  = 1

    f = open('data2.txt','r')
    for line in f:
      item = line.split()
      z = eval(item[0])
      x = x + (z - x)/(n)
      n = n+1

    print x

Note that you did not need to know how many points were in the file to
get the average. It was built into the iteration formula.

This process can be weighted to produce a running weighted average. We
will rework the previous derivation for the case where the weighting is
not uniform. The running average will be denoted by :math:`\hat{x}_n`
and the running variance will be denoted by :math:`P_n`

.. math::

   \hat{x}_n = \displaystyle P_n \displaystyle \sum_{i=1}^n \frac{z_i}{\sigma_i^2}, \quad \quad P_n =
   \displaystyle \left( \sum_{i=1}^n \frac{1}{\sigma_i^2} \right)^{-1}

A new data point provides a new estimate:

.. math::

   \hat{x}_{n+1} = \displaystyle P_{n+1} \displaystyle \sum_{i=1}^{n+1} \frac{z_i}{\sigma_i^2},
   \quad \quad P_{n+1} =
   \displaystyle \left(\sum_{i=1}^{n+1} \frac{1}{\sigma_i^2}\right)^{-1}

As with the uniform weighting, pull the last value out of the sum and
rework the sum:

.. math::

   \hat{x}_{n+1} = \displaystyle \frac{P_{n+1}}{P_n}\left({P_n}\sum_{i=1}^{n}
   \frac{z_i}{\sigma_i^2}\right) + {P_{n+1}}\frac{z_{n+1}}{\sigma_{n+1}^2}

.. math:: = \displaystyle \frac{P_{n+1}}{P_n}\hat{x}_n +P_{n+1}\frac{z_{n+1}}{\sigma_{n+1}^2}

.. math::

   = \displaystyle \frac{P_{n+1}}{P_n}\hat{x}_n + \frac{P_{n+1}\hat{x}_n}{\sigma_{n+1}^2}  + P_{n+1}\frac{z_{n+1}}{\sigma_{n+1}^2}
   - \frac{P_{n+1}\hat{x}_n}{\sigma_{n+1}^2}

.. math::

   = \displaystyle P_{n+1} \left( \hat{x}_n\left(\frac{1}{P_n} + \frac{1}{\sigma_{n+1}^2} \right) + \frac{z_{n+1}}{\sigma_{n+1}^2}
   - \frac{\hat{x}_n}{\sigma_{n+1}^2}
   \right)

Since :math:`1/P_{n+1} = 1/P_n + 1/\sigma_{n+1}^2`

.. math::

   =
    \hat{x}_n + \frac{P_{n+1}z_{n+1}}{\sigma_{n+1}^2}  - \frac{P_{n+1}\hat{x}_n}{\sigma_{n+1}^2}

.. math:: = \hat{x}_n +  K_{n+1}\left(  z_{n+1}- \hat{x}_n \right),

with

.. math::

   K_{n+1} = \displaystyle \frac{P_{n+1}}{\sigma_{n+1}^2}  = \frac{1}{\sigma_{n+1}^2}\left(1/P_n + 1/\sigma_{n+1}^2\right)^{-1}
    = \displaystyle \frac{P_{n}}{\left(P_{n} + \sigma_{n+1}^2\right)} .

Using :math:`K` we can write a recursive formula for :math:`P_{n+1}`:

.. math:: P_{n+1} = \displaystyle  (1 -   K_{n+1}) P_{n}

This provides us with a recursive weighted filter:

.. math::
   :label: Eq:scalarrecursiveweighted

   \begin{array}{l}
   K_{n} = \displaystyle P_{n-1} \left(P_{n-1} + \sigma_n^2\right)^{-1} \\[8pt]
   \hat{x}_{n} =  \hat{x}_{n-1} +  K_{n}\left(  z_{n}- \hat{x}_{n-1} \right) \\[8pt]
   P_n = \displaystyle  (1 -   K_n) P_{n-1} ,
   \end{array}


 where :math:`P_0 = \sigma_0^2` and :math:`\hat{x}_0 = z_0`.

You have now seen two important aspects to the Kalman Filter. The
concept of sensor fusion, data from different distributions, and the
concept of recursive filtering.



Assume that you get successive measurements from three sensors which are
already corrected for deterministic errors. The data is
:math:`\{(z,\sigma)\} = \left\{ (1.5, 0.1), (1.3, 0.05), (1.4, 0.15)\right\}`.
Find the recursive fused estimate. For comparison, we first compute
using the non-recursive (regular) formula.

.. math::

   \displaystyle S = \frac{1.0}{0.1^2} + \frac{1.0}{0.05^2} + \frac{1.0}{0.15^2}, \quad
   \displaystyle y = \frac{1.5}{0.1^2} + \frac{1.3}{0.05^2} + \frac{1.4}{0.15^2}

.. math:: \hat{x}  = \frac{y}{S} \approx 1.34489795918

The recursive approach is given in the code listing below:

::

    z=np.array([1.5,1.3,1.4])
    sigma=np.array([0.1,0.05,0.15])
    p = sigma[0]**2
    xhat = z[0]

    for i in range(1,3):
      kal = p/(p + sigma[i]**2)
      xhat = xhat + kal*(z[i] - xhat)
      p = (1-kal)*p

    print xhat

The result of running the code: 1.34489795918

Multivariate Recursive Filtering[multivariatesensorfusion]
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Let :math:`W_i` is the variance for the sensor. The previous algorithm
extends to multiple variables as

-  Set :math:`x_0 = z_0`, :math:`P_0=W_0`

-  Let :math:`n=1` and repeat:

   -  :math:`K_n = P_{n-1}\left(P_{n-1} + W_n\right)^{-1}`

   -  :math:`\hat{x}_{n} =\hat{x}_{n-1} + K_n\left(z_n - \hat{x}_{n-1}\right)`

   -  :math:`P_{n} = (I - K_n ) P_{n-1}`

::

    while (i<n):
      y = z[i] - x
      S = P + W[i]
      kal = np.dot(P,linalg.inv(S))
      x = x + np.dot(kal,y)
      P = P - np.dot(kal,P)
      i = i+1

Sample Data Fusion
^^^^^^^^^^^^^^^^^^

Assume that you are given the two measurements
:math:`z_1 = (0.9, 2.1, 2.8)` and :math:`z_2 = (1.1, 2.0, 3.1)`. Also
assume the variance-covariance matrices for :math:`z_1` and :math:`z_2`
are

.. math::

   W_1 =
   \begin{pmatrix}
   0.2 & 0.02 & 0.002 \\
   0.02 & 0.3 & 0.01 \\
   0.002 & 0.01 & 0.4
   \end{pmatrix},
   \quad
   W_2 =
   \begin{pmatrix}
   0.1 & 0.01 & 0.001 \\
   0.01 & 0.16 & 0.008 \\
   0.001 & 0.008 & 0.2
   \end{pmatrix}

How can you merge these into a single estimate?

::

    import numpy as np
    from scipy import linalg
    z1 = np.array([0.9,2.1,2.8])
    z2 = np.array([1.1, 2.0,3.1])
    w1 = np.array([[0.2,0.02,0.002],[0.02, 0.3, 0.01],[0.002,0.01,0.4]])
    w2 = np.array([[0.1,0.01,0.001],[0.01, 0.16, 0.008],[0.001,0.008,0.2]])
    x = z1
    P = w1
    y = z2 - x
    S = P + w2
    kal = np.dot(P,linalg.inv(S))
    x = x + np.dot(kal,y)
    P = P - np.dot(kal,P)

.. math:: x = \begin{pmatrix} 1.03333333&  2.03420425,& 3.00056428\end{pmatrix}

.. math::

   P = \begin{pmatrix}
   0.06666667& 0.00666667&  0.00066667\\
   0.00666667& 0.10434213&  0.00463772 \\
   0.00066667&  0.00463772&  0.13332457
   \end{pmatrix}

Least Squares Observer
~~~~~~~~~~~~~~~~~~~~~~

Least Squares is used because there is noise in the data collection or
the observations. Here we will summarize the material above and use a
notation closer to what is used in the Kalman Filter. Let’s start with a
familiar example. Assume that you have a collection of similar sensors
(equal standard deviations for now) that you gather measurements from:
:math:`z_1`, :math:`z_2`, …, :math:`z_n`. You know that they are noisy
versions of a hidden state :math:`x`, with noise :math:`w` meaning that
:math:`z = Hx + w`, the observation of :math:`x` subject to noise
:math:`w`.

Given :math:`k` observations :math:`z` of state :math:`x\in{\Bbb R}^n`,
:math:`k>>n`, with noise :math:`w`:

.. math:: z = Hx+w.

As before, we aim to find :math:`\hat{x}` which minimizes the square
error:

.. math:: \| z - H\hat{x}\|.

So, we are seeking the least square solution to :math:`z = H\hat{x}`
which is

.. math:: \hat{x} = \left(H^TH\right)^{-1} H^T z.

The difference between the estimate and the actual value

.. math::

   \hat{x}-x = \left(H^TH\right)^{-1} H^T (Hx+w) -x
   = \left(H^TH\right)^{-1} H^T w

If :math:`w` has zero mean then :math:`\hat{x}-x` has zero mean and
:math:`\hat{x}` is an unbiased estimate (as we had before).

Example
^^^^^^^

In this example we observe just the state variable and without noise we
would just have :math:`z  = x`. Using this as our model we obtain a set
of equations:

.. math::

   \begin{array}{c}
   z_1 = x + w_1 \\
   z_2 = x  + w_2\\
   \vdots \\
   z_n = x + w_n.\\
   \end{array}

We have solved this problem earlier, but this time we will rewrite it in
a matrix form. Bear with me since it is a lot of machinery for a simple
problem, but it will help lead us to the more general case which
follows. It can be written as

.. math:: z = Hx + w

where

.. math::

   z = \begin{pmatrix} z_1 \\ z_2 \\ \vdots \\ z_n \end{pmatrix}, \quad  w = \begin{pmatrix} w_1 \\ w_2 \\ \vdots \\ w_n \end{pmatrix},\quad
   H = \begin{bmatrix} 1 \\ 1 \\ \vdots \\ 1 \end{bmatrix}.

Write out the estimate to see how it compares to the previous one:

.. math:: \hat{x} = \left(H^TH\right)^{-1} H^T z = \left(\begin{bmatrix} 1 & 1 & \dots & 1\end{bmatrix} \begin{bmatrix} 1 \\ 1 \\ \vdots \\ 1 \end{bmatrix}\right)^{-1} \left( \begin{bmatrix} 1 & 1 & \dots & 1\end{bmatrix} \begin{pmatrix} z_1 \\ z_2 \\ \vdots \\ z_n \end{pmatrix}\right)

.. math:: = \frac{1}{n} \sum_{i=1}^{n} z_i

which agrees with our earlier work (and below we will show that the
weighted one works out as well). The strength of this approach is in the
ease of generalization [#f1]_.

Weighted Least Squares Observer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Traditional least squares is formulated by minimizing using the normal
innerproduct:

.. math:: x^Ty = \sum_i x_iy_i.

If the inner product is weighted:

.. math:: x^Ty = \sum_i x_i y_i q_i = x^T Q y

 then the weighted least squares solution to

.. math:: z = Hx + w

 is

.. math:: \hat{x} = \left(H^T Q H\right)^{-1} H^TQz .

 The matrix :math:`Q` is any matrix for which the innerproduct above is
a valid. However, we will select :math:`Q` as a diagonal matrix
containing the reciprocals of the variances (the reason shown below in
the covariance computation). We can rework our simple example:

.. math::

   z = \begin{pmatrix} z_1 \\ z_2 \\ \vdots \\ z_n \end{pmatrix},  \quad w = \begin{pmatrix} w_1 \\ w_2 \\ \vdots \\ w_n \end{pmatrix}, \quad
   H = \begin{bmatrix} 1 \\ 1 \\ \vdots \\ 1 \end{bmatrix}

and

.. math::

   Q =
   \begin{bmatrix}
   \sigma_1^{-2} & 0 & \dots & 0 \\
   0 & \sigma_2^{-2} &  \dots & 0  \\
   0 & 0 &  \dots & 0  \\
   0 & 0 & 0 &\sigma_n^{-2}  \\
   \end{bmatrix}.

The estimate, :math:`\hat{x}` is then
:math:`\hat{x} = \left(H^TQH\right)^{-1} H^T Q z`,

.. math::

   \hat{x}= \left(\begin{bmatrix} 1 & 1 & \dots & 1\end{bmatrix}\begin{bmatrix}
   \sigma_1^{-2} & 0 & \dots & 0 \\
   0 & \sigma_2^{-2} &  \dots & 0  \\
   0 & 0 &  \dots & 0  \\
   0 & 0 & 0 &\sigma_n^{-2}  \\
   \end{bmatrix} \begin{bmatrix} 1 \\ 1 \\ \vdots \\ 1 \end{bmatrix}\right)^{-1}

.. math::

   \times \left( \begin{bmatrix} 1 & 1 & \dots & 1\end{bmatrix} \begin{bmatrix}
   \sigma_1^{-2} & 0 & \dots & 0 \\
   0 & \sigma_2^{-2} &  \dots & 0  \\
   0 & 0 &  \dots & 0  \\
   0 & 0 & 0 &\sigma_n^{-2}  \\
   \end{bmatrix}\begin{pmatrix} z_1 \\ z_2 \\ \vdots \\ z_n \end{pmatrix}\right) ,

.. math::

   \hat{x}=\displaystyle \frac{\displaystyle \sum_{i=1}^n \frac{z_i}{\sigma_i^2}}
   {\displaystyle \sum_{i=1}^n \frac{1}{\sigma_i^2}}

The covariance of this estimate is

.. math:: = \left(H^TQH\right)^{-1} H^T Q\, W\, Q H\left(H^TQH\right)^{-1}

Often one selects the weighting to be inversely proportional to
:math:`W` (the matrix of reciprocal variances) which is what we did
above:

.. math:: Q = W^{-1}.

A smaller standard deviation means better data, and thus we weigh this
more. Substituting in

.. math:: \hat{x} = \left(H^T W^{-1} H\right)^{-1} H^TW^{-1}z

with covariance

.. math:: P = \left(H^T W^{-1} H\right)^{-1}

Given an observation :math:`z` of state :math:`x` with noise :math:`w`:

.. math:: z = Hx+w

the :math:`\hat{x}` which minimizes the square error

.. math:: \| z - H\hat{x}\|

.. math:: \hat{x} = H^+z = W^{-1} H^T\left(H W^{-1} H^T\right)^{-1}z

with :math:`W` the covariance of :math:`w` and error covariance

.. math:: P = \left(H W^{-1} H^T\right)^{-1}

if we take the same weighting as before.

.. _kalman-example-1:

Example
^^^^^^^

Assume that we have two state variables :math:`x_1` and :math:`x_2` and
we are able to observe the first directly (with noise) and the sum of
the two (with noise). The model will be two constants we are observing
through a noisy observation process. This means:

.. math::

   z = Hx \quad \Rightarrow \quad
   \begin{bmatrix}
    z_1 \\ z_2
   \end{bmatrix}
   =
   \begin{bmatrix}
    1 & 0 \\
   1 & 1
   \end{bmatrix}
   \begin{bmatrix}
    x_1 \\ x_2
   \end{bmatrix}
   +
   \begin{bmatrix}
    w_1 \\ w_2
   \end{bmatrix}

Multiple observations give:

.. math::

   \begin{bmatrix}
    z_1 \\ z_2 \\ z_3 \\ z_4 \\ \vdots
   \end{bmatrix}
   =
   \begin{bmatrix}
    1 & 0 \\
   1 & 1  \\
   1 & 0 \\
   1 & 1  \\
   \vdots & \vdots
   \end{bmatrix}
   \begin{bmatrix}
    x_1 \\ x_2
   \end{bmatrix}
   +
   \begin{bmatrix}
    w_1 \\ w_2 \\ w_3 \\ w_4\\ \vdots
   \end{bmatrix}

The least square solution to :math:`z = H\hat{x}` is

.. math:: \hat{x} = \left(H^TH\right)^{-1} H^T z

Assume we have data:

::

    0.874328560532
    3.25683958794
    0.859486711669
    2.86834487616
    1.25271217589
    2.95373764186
    0.881013871661
    3.09066238259
    0.971121996741
    3.03754386081

Compute Normal Equation:

.. math::

   H^T H =
   \begin{bmatrix}
   10 & 5 \\ 5 & 5
   \end{bmatrix}
   \quad \quad
   H^Tz =
   \begin{bmatrix}
    20.04579167  \\15.20712835
   \end{bmatrix}

Solve :math:`H^T H x = H^Tz`: Then:
:math:`x_1 = 0.96773266, ~~ x_2=  2.07369301`

Note that the actual values were :math:`x_1 = 1, x_2=  2`

.. _example-2:

Example
^^^^^^^

Recall that there are two forms of the Least Squares Inverse (the
Pseudoinverse). The examples above used the left inverse. That applied
when we had more equations than unknowns (or variables), the problem was
overdetermined. There will be times for which the reverse is true; that
we will have more unknowns than equations. For the underdetermined
problem we use the right inverse. The following illustrates this idea.

Say that the system can observe two of three variables: :math:`(u,v)`
from :math:`(u,v,\theta)`,

.. math::

   z_k = Hx_k \quad \Rightarrow \quad \begin{bmatrix} \xi_k \\ \eta_k \end{bmatrix}
   =
   \begin{bmatrix}
    1 & 0 & 0 \\
   0 & 1 & 0
   \end{bmatrix}
   \begin{bmatrix}
    u_k \\ v_k \\ \theta_k
   \end{bmatrix}

For this problem we solve using the right inverse:

.. math:: x_k = H^+ z_k .

The reason can be seen by looking at the object to be inverted in the
two pseudo-inverse formulas:

.. math::

   H^TH = \begin{bmatrix}
    1 & 0 & 0 \\
   0 & 1 & 0 \\
   0 & 0 & 0
   \end{bmatrix} ,
   \quad
   HH^T = \begin{bmatrix}
    1 & 0  \\
   0 & 1
   \end{bmatrix}.

The left matrix is not invertable. A right pseudo-inverse

.. math::

   \begin{bmatrix}
    u_k \\ v_k \\ \theta_k
   \end{bmatrix}
   =
   \begin{bmatrix}
    1 & 0  \\
   0 & 1 \\
   0 & 0
   \end{bmatrix}
   \left(
   \begin{bmatrix}
    1 & 0  \\
   0 & 1
   \end{bmatrix}
   \right)^{-1}
   \begin{bmatrix} \xi_k \\ \eta_k \end{bmatrix}
   =
   \begin{bmatrix}
    1 & 0  \\
   0 & 1 \\
   0 & 0
   \end{bmatrix}
   \begin{bmatrix} \xi_k \\ \eta_k \end{bmatrix}
   =
   \begin{bmatrix} \xi_k \\ \eta_k \\ 0 \end{bmatrix}

Effectively we have produced a projection. This projection restricted
our variables to the relevant observational data. It can then be used in
sensor fusion applications.

Example 3
^^^^^^^^^

Assume that we have a noisy data set :math:`(x_i, y_i)` which we know
lies on a line:



::

    [[  0.          -5.65520482]
     [  0.10204082   4.53774258]
     [  0.20408163   3.71191423]
     [  0.30612245   1.44760549]
     [  0.40816327   0.88024529]
     [  0.51020408   4.25592703]
     [  0.6122449    0.81475181]
     [  0.71428571   0.9275501 ]
     [  0.81632653   2.70301802]
     [  0.91836735   5.74002313]
     [  1.02040816   1.27503184]
     [  1.12244898   3.82976944]
     [  1.2244898    2.34108935]
     [  1.32653061   6.44934519]
     [  1.42857143   6.10025845]
     [  1.53061224   2.0450073 ]
     [  1.63265306   8.08201653]
     [  1.73469388   3.79104473]
     [  1.83673469   5.40629739]
     [  1.93877551   4.15556209]
     [  2.04081633   4.49578503]
     [  2.14285714   7.48854739]
     [  2.24489796   5.07750616]
     [  2.34693878   4.29701526]
     [  2.44897959   7.20452521]
     [  2.55102041   6.72492257]
     [  2.65306122   7.56408995]
     [  2.75510204   7.2419468 ]
     [  2.85714286   3.45946936]
     [  2.95918367   3.54635642]
     [  3.06122449   5.54792305]
     [  3.16326531   8.60804178]
     [  3.26530612   5.41562294]
     [  3.36734694  10.3737351 ]
     [  3.46938776   7.89065344]
     [  3.57142857   6.86298534]
     [  3.67346939   7.81332673]
     [  3.7755102    8.55556688]
     [  3.87755102   9.56774192]
     [  3.97959184   8.10000457]
     [  4.08163265   8.98656353]
     [  4.18367347   6.34429316]
     [  4.28571429   4.62596754]
     [  4.3877551    5.46160224]
     [  4.48979592  11.6944026 ]
     [  4.59183673   9.44392528]
     [  4.69387755   8.49333718]
     [  4.79591837  12.5121096 ]
     [  4.89795918   7.59781085]
     [  5.           9.60759719]]


If we know the formula for the line we can project onto the line. For
this example, we will assume we don’t have the formula and are
attempting to deduce the line. Meaning the model is that the data has a
linear relation, we just lack the parameters. [So we are doing a
parametric curve fit.] We use the same approach as with previous
datasets. The model is :math:`y = a_1x + a_0`. Application of the data
set and we have an overconstrained system of equations. Using the left
pseudoinverse as before we can determine :math:`a_1, a_0`. We may get
something like :math:`a_1=2.2231`, :math:`a_0 =  1.0124`, see
:numref:`fig:LSnoiseReduction` for data and
plot. How would this be a filter? You can project points onto the line
via the line projection formula found in calculus: :math:`a = a_1`,
:math:`b = -1.0`, :math:`c = a_0`,

.. math::

   \begin{matrix}
   \displaystyle d = a^2+b^2\\[5pt]
   \displaystyle px = \frac{b(bx - ay)-ac}{d} \\[5pt]
   \displaystyle py = \frac{a(-bx+ay)-bc}{d}
   \end{matrix}

The application of this as a filter is shown in
:numref:`fig:LSnoiseReductionO`.

.. _`fig:LSnoiseReduction`:
.. figure:: FilteringFigures/LSnoiseReduction.*
   :width: 50%
   :align: center

   Dataset and least square fit. The data is in
   red, the curve fit is the solid blue line and the projection of the
   data is the blue dots.

.. _`fig:LSnoiseReductionO`:
.. figure:: FilteringFigures/LSnoiseReductionO.*
   :width: 50%
   :align: center

   Projecting data onto the line as a filter.
   Green dots are new data, the curve fit is the solid blue line and
   blue dots are their projections.

.. rubric:: Footnotes

.. [#f1] Generalization is not our goal, we have a specific problem to address.
