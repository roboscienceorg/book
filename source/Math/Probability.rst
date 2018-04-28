Probability
-----------

Let :math:`X` denote a random variable. Let :math:`P` denote the
probability that :math:`X` takes on a specific value :math:`x`:
:math:`P(X=x)`. If :math:`X` takes on discrete values we say that
:math:`X` is a discrete variable. If :math:`X` takes on continuous
values we say that :math:`X` is a continuous variable.

Normally :math:`P(X=x)` makes sense for discrete spaces and we use
:math:`P(x_1 <
X < x_2)` for continuous spaces. For continuous spaces we define the
probability density function (pdf) :math:`p(x)` in the following manner:

.. math:: P(x_1 \leq X \leq x_2) = \int_{x_1}^{x_2} p(x)\, dx

Uncertainty and Distributions
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Recall that random variables are drawn from some probability
distribution function. Often these are the normal distributions seen in
many areas of the sciences, but can be any shape as long as the area
under the curve is one. Specifically, the normal distribution is given
by

.. math:: p(x) = \frac{1}{\sqrt{2\pi}\, \sigma}e^{-(x-\mu)^2/2\sigma^2}

where :math:`\mu` is the mean and :math:`\sigma^2` is the variance
(:math:`\sigma` is the standard deviation). For multivariate
distributions (vector valued random variables) we can exend to

.. math:: p(x) = \frac{1}{(2\pi)^{n/2}\sqrt{\det(\Sigma)}}e^{-\frac{1}{2}(x-\mu)^T\Sigma^{-1}(x-\mu)}

where :math:`\mu` - mean vector, :math:`\Sigma` - covariance matrix
(symmetric positive definite).

.. figure:: MathFigures/pdf.svg
   :width: 70%
   :align: center

   [fig:pdfplot] Probability Distribution Function

Let :math:`X,Y` be two random variables, the joint distribution is

.. math:: P(x,y) = P(X=x~\mbox{and}~Y=y).

We say the the variables are independent if

.. math:: P(x,y) = P(x)P(y)

Conditional probability: what is the probability of :math:`x` if we know
:math:`y` has occurred? Denoted :math:`P(x|y)`,

.. math:: P(x|y) = \frac{P(x,y)}{P(y)}

If they are independent

.. math:: P(x|y) = \frac{P(x,y)}{P(y)}=\frac{P(x)P(y)}{P(y)} = P(x)

Total probability (relax the uppercase formalism)

.. math:: p(x) = \sum_{y} p(x|y)p(y)\quad \left[= \int_Y p(x|y)p(y)\, dy \right]

**Bayes Rule** (way to invert conditional probabilities)

.. math:: p(x|y) = \frac{p(y|x)p(x)}{p(y)}

**Expectation** or the mean or average for a distribution is given by

.. math:: E(x) = \sum x p(x) \quad \left[ =\int_X x p(x)\, dx \right]

Moments for a distribution are given by

.. math:: \tilde{\mu_r} = E(x^r) = \int_X x^rp(x)\, dx

.. math:: \mu = \tilde{\mu_1} = \quad \mbox{Mean - expected value}

Moments about the mean

.. math:: \mu_r = \int_X (x-\mu)^rp(x) \,dx

Second moment about the mean is called the *Variance*: :math:`\mu_2 =
\sigma^2`, where :math:`\sigma` is called the *Standard Deviation*. Note
that variance :math:`=E[(x-\mu)^2]` and covariance
:math:`E(X\cdot Y)-\mu\nu`

where :math:`\mu`, :math:`\nu` are the means for :math:`X` and
:math:`Y`.

The **Covariance** Matrix is given by :math:`\Sigma =`

.. math::

   \left( \begin{array}{cccc}E[(x_1-\mu_1)(x_1-\mu_1)^T]& \dots & E[(x_1-\mu_1)(x_n-\mu_n)^T]
    \\     \dots & \ddots & \dots
     \\ E[(x_n-\mu_n)(x_1-\mu_1)^T]  & \dots &
     E[(x_n-\mu_n)(x_n-\mu_n)^T]\end{array}\right)

.. math:: = E[(x-\mu)(x-\mu)^T]

There are many terms to describe the variance of a set of random
variables. Variance, covariance and cross-variance, variance-covariance
are a few example terms. We will use variance for scalar terms and
covariance for vector terms.

Sample covariance
^^^^^^^^^^^^^^^^^

If you know the population mean, the covariance is given by

.. math:: Q = \frac{1}{N} \sum_{k=1}^{N}(x_k - E(x))(x_k - E(x))^T

and if you don’t know the mean the covariance is given by

.. math:: Q = \frac{1}{N-1} \sum_{k=1}^{N}(x_k - \overline{x})(x_k - \overline{x})^T

Note: :math:`(x_1-\overline{x})`, :math:`(x_2-\overline{x})`,
:math:`(x_2-\overline{x})` has :math:`n-1` residuals (since they sum to
zero).
