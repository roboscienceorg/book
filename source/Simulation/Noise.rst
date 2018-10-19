Simulation with Noise
----------------------

You will see later on we go to great efforts to remove noise from a
dataset. So, it might seem odd to have a section on generating noise.
However, it is very useful to be able to generate noise for more
realistic simulations and to test the filters that are intended to
remove the noise. The Numpy library supports the generation of random
numbers as well as some convenient functions to draw numbers from
certain types of distributions. Most of our work will use normal
distributions. The numpy function random.normal will generate random
(well, approximately) values drawn from a normal distribution. For
example, the following code will generate a scatter plot, see
:numref:`fig:samplescatterplot`-(a).

::

    mu1 = 1.0
    sigma1 = 0.5
    mu2 = 2.0
    sigma2 = 1.0
    x = np.random.normal(mu1,sigma1,100)
    y = np.random.normal(mu2,sigma2,100)
    plt.plot(x,y,'b.')
    plt.show()

This code will generate a sampled line with noise, see
:numref:`fig:samplescatterplot`-(b).

::

    mu = 0.0
    sigma = 1.0
    error = np.random.normal(mu,sigma,100)
    x = np.linspace(0,5,100)
    y = 2*x+1.0 + error
    plt.plot(x,y,'b.')
    plt.show()


.. _`fig:samplescatterplot`:
.. figure:: SciPyFigures/randomvals.*
   :width: 95%
   :align: center

   Scatter type plots.  a) A scatter type plot.
   b)  line with lots of noise.

Above we are sampling from a single normal distribution (univariate),
however, later on we will need to sample from multivariate distribution.
We provide the algorithm below or this can be done with
np.random.multivariate_normal.

::

    >>> mean = [0,0]
    >>> covar = [[.5,.05],[.05,1.0]]
    >>> N = 10
    >>> np.random.multivariate_normal(mean,covar,N)
    array([[ 0.88598172, -0.4423436 ],
           [ 0.13454988, -0.72543919],
           [-0.37652703,  0.74301719],
           [ 0.25273237, -0.63923146],
           [-1.43009133, -0.53752537],
           [ 0.27189567, -0.56165933],
           [-0.23506435,  0.82847583],
           [ 0.47206316,  0.46425447],
           [-0.33998358,  0.4583102 ],
           [-1.07647896,  0.90586496]])
    >>>

Creating your own distribution
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you want to do this by hand:

#. Generate the random numbers for each variable.
#. Place them into an array.
#. Compute their variance-covariance matrix.
#. Perform a Cholesky factorization on the variance-covariance matrix.
#. Invert the Cholesky factor and multiple it by the random matrix data.
   This normalizes the dataset.
#. Compute a Cholesky factorization of the desired variance-covariance
   matrix.
#. Multiply the last Cholesky factor times the normalized data.

::

    import numpy as np

    N = 100
    sigma = 1.0

    # Create two vectors of random numbers
    #
    ex = np.random.normal(0,sigma,N)
    ey = np.random.normal(0,sigma,N)

    # Stack them into an array
    #
    D = np.vstack((ex,ey))

    # Normalize the distribution
    M = np.cov(D)
    MC = np.linalg.cholesky(M)
    MCI = np.linalg.inv(MC)
    MD = np.dot(MCI,D)

    # Enter the desired covariance matrix:
    #
    W = np.array([[0.1, 0.01],[0.01,0.2]])

    # Perform the Cholesky decomposition
    #
    L = np.linalg.cholesky(W)

    # Multiply the Cholesky factor L with the data
    # (which transforms the data to having the correct
    # covariance)
    #
    # LD is the random data with the correct covariance
    LD = np.dot(L,MD)

    # Print the result to check if it is close to w
    #print(np.cov(LD))

The previous gives you a method to generate random values from a distribution.
Next we want to use them for various simulation events, normally to understand
the system in the presence of noise.

Noise in the DD Robot
^^^^^^^^^^^^^^^^^^^^^^^

The differential drive robot has two control inputs, the right and left wheel
speeds.  To simulate motion with noise, we can inject small random values
into each iteration of the simulation.   Assume that we have random values (or vector)
:math:`\epsilon_i`, :math:`i=1,2,3` drawn from some normal distribution :math:`N(\mu,\sigma)`.
Note that the distribution :math:`N` in this example is a Gaussian distribution, but it need not be
in general.

Recall the basic discrete motion
equations for the differential drive:

.. math::

   \begin{array}{l}
    x_{k+1} = x_k + \frac{r\Delta t}{2} (\omega_{1, k}+\omega_{2, k})\cos(\theta_k) \\[2mm]
   y_{k+1} = y_k + \frac{r\Delta t}{2} (\omega_{1, k}+\omega_{2, k})\sin(\theta_k) \\[2mm]
   \theta_{k+1} = \theta_k + \frac{r\Delta t}{2L} (\omega_{1, k}-\omega_{2, k})
   \end{array}

Noise can be injected directly into the state variables :math:`(x,y,\theta)`:

.. math::
   :label: noisedd

   \begin{array}{l}
    x_{k+1} = x_k + \frac{r\Delta t}{2} (\omega_{1, k}+\omega_{2, k})\cos(\theta_k) + \epsilon_1\\[2mm]
   y_{k+1} = y_k + \frac{r\Delta t}{2} (\omega_{1, k}+\omega_{2, k})\sin(\theta_k) + \epsilon_2\\[2mm]
   \theta_{k+1} = \theta_k + \frac{r\Delta t}{2L} (\omega_{1, k}-\omega_{2, k}) + \epsilon_3
   \end{array}

You will note that we are adding a small amount of noise at each iteration step.
This is not the same as adding the noise at the end since for the iterative process
with noise injected at each step, the noise modifies the path at each step and has
a cumulative effect.   Adding noise at the end, will just create an end distribution
which mirrors the distribution that the noise was drawn from.  However, noise injected
into the DD forward kinematics time step is subject to a non-linear process and
the final distribution is not Gaussian.

Simulation with random variables can be very helpful in understanding
the exact impact of noise in a particular
state's update.  It also models the aggregate noise from various sources into a single
additive term.   If one wants to study the effects of just noise in the wheel speed, then
we inject the noise into the :math:`\omega` terms:

.. math::

   \begin{array}{l}
    x_{k+1} = x_k + \frac{r\Delta t}{2} (\omega_{1, k}+\omega_{2, k} + \epsilon_1)\cos(\theta_k)\\[2mm]
   y_{k+1} = y_k + \frac{r\Delta t}{2} (\omega_{1, k}+\omega_{2, k}  + \epsilon_1)\sin(\theta_k)\\[2mm]
   \theta_{k+1} = \theta_k + \frac{r\Delta t}{2L} (\omega_{1, k}-\omega_{2, k} + \epsilon_2)
   \end{array}

Using :eq:`noisedd`, we can illustrate adding noise.   This example uses a robot with r=20,
L = 12, :math:`\Delta t = 0.01` and has a simple turn:

.. math::

   \begin{array}{l}
   \phi_1 = 1.0, \phi_2 = 1.0,   0 \leq t < 1.5 \\
   \phi_1 = 2.0, \phi_2 = 1.0    1.5 \leq t < 3.0 \\
   \phi_1 = 1.0, \phi_2 = 1.0    3.0 \leq t
   \end{array}


::

   def wheels(t):
     if (t < 1.5):
       w1 = 1.0
       w2 = 1.0
       return w1, w2
     if (t < 3):
       w1 = 2.0
       w2 = 1.0
       return w1, w2
     w1 = 1.0
     w2 = 1.0
     return w1, w2

The setup for the simulation is

::

   r = 20.0
   l = 12.0
   N = 10
   dt  = 0.01
   Tend = 5
   NumP = int(Tend/dt)

   mu1, sigma1 = 0.0, 0.05
   mu2, sigma2 = 0.0, 0.01
   tp = np.arange(0,Tend,dt)

   xpath  = np.zeros((N,NumP))
   ypath = np.zeros((N,NumP))
   thpath = np.zeros((N,NumP))

We selected the same noise range for the :math:`x,y` variables but a smaller range
for the :math:`\theta` variable.  Small changes in :math:`\theta` variable can have a
greater impact on the final location than small changes in :math:`x,y`.
The arrays xpath, ypath and thpath are declared as two dimensional arrays.  This
is so we can store multiple paths.  Meaning we are storing :math:`N` paths which
are comprised of :math:`Nump` points.

To create the paths we run a double loop, where the outside loop is over the
paths and the inside loop creates the points on a specific path.

::

   for k in range(N):
       thv = 0.0
       xv = 0.0
       yv = 0.0
       i = 0
       xerr = np.random.normal(mu1,sigma1, NumP)
       yerr = np.random.normal(mu1,sigma1, NumP)
       therr = np.random.normal(mu2,sigma2, NumP)
       for t in tp:
         w1,w2 = wheels(t)
         dx = (r*dt/2.0)*(w1+w2)*cos(thv) + xerr[i]
         dy = (r*dt/2.0)*(w1+w2)*sin(thv) + yerr[i]
         dth = (r*dt/(2.0*l))*(w1-w2) + therr[i]
         xv = xv + dx
         yv = yv + dy
         thv = thv + dth
         xpath[k][i] = xv
         ypath[k][i] = yv
         thpath[k][i] = thv
         i = i+1

This can be visualized by

::

   for k in range(N):
       plt.plot(xpath[k],ypath[k], 'b-')
   plt.xlim(-3, 130)
   plt.ylim(-20, 130)
   plt.show()


which is shown in :numref:`multiplepathsnoise`.

.. _`multiplepathsnoise`:
.. figure:: SciPyFigures/multiplepaths.*
   :width: 80%
   :align: center

   Multiple paths from the same starting point using a noisy model.

We can keep adding additional paths to see the distribution of the final locations.
It gets too hard to see what it going on, so we only plot the last point
on a particular path.  The noise free path is also included and result is
shown in :numref:`multipleendpts`.

.. _`multipleendpts`:
.. figure:: SciPyFigures/multipleendpts.*
   :width: 80%
   :align: center

   Showing the noise free path and the endpoints for the noisy paths.

For linear Gaussian processs, if we ran this millions of times and then produced
a histogram of the results, we would see a 2D normal distribution emerge.   Cross sections
of the 2D normal would be ellipses.   The larger the ellipse the greater confidence value
we have.  Since this is *not* a linear process, we don't expect a normal distribution,
but we do expect some distribution.  So we will treat this in a similar
fashion.   To compute the error ellipse for the 95% confidence, we store the
final points in parallel arrays for x and y, and run the following code
block:

::

   s = 2.447651936039926
   mat = np.array([x,y])
   cmat = np.cov(mat)
   evals, evec = linalg.eigh(cmat)
   r1 = 2*s*sqrt(evals[0])
   r2 = 2*s*sqrt(evals[1])
   angle = 180*atan2(evec[0,1],evec[0,0])/np.pi
   ell = Ellipse((np.mean(x),np.mean(y)),r1,r2,angle)
   a = plt.subplot(111, aspect='equal')
   ell.set_alpha(0.2)
   a.add_artist(ell)

   plt.plot(xpath,ypath, 'b-', x,y, 'r.')
   plt.xlim(-3, 130)
   plt.ylim(-20, 130)
   plt.show()


What this does is to take the two data sets, x and y and compute the covariance
matrix, stored in cmat.   The eigenvectors and eigenvalues for cmat are computed.
The eigenvectors tell you the tilt angle of the array,  The eigenvalues (which are
variances) are used to find the major and minor axes lengths (r1, r2).

.. math::   r = 2\sqrt{5.991\lambda}


.. _`pathsellipse`:
.. figure:: SciPyFigures/pathellipse.*
   :width: 80%
   :align: center

   The error ellipse.
