Probability
-----------

Generation of Noise
~~~~~~~~~~~~~~~~~~~

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
Figure \ `[fig:samplescatterplot] <#fig:samplescatterplot>`__.

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
Figure \ `[fig:samplenoisylineplot] <#fig:samplenoisylineplot>`__.

::

    mu = 0.0
    sigma = 1.0
    error = np.random.normal(mu,sigma,100)
    x = np.linspace(0,5,100)
    y = 2*x+1.0 + error
    plt.plot(x,y,'b.')
    plt.show()

.. raw:: latex

   \centering

.. figure:: math/randomvalues
   :alt: A scatter type plot.[fig:samplescatterplot]

   A scatter type plot.[fig:samplescatterplot]

.. figure:: math/randomvalues2
   :alt: A line with lots of noise.[fig:samplenoisylineplot]

   A line with lots of noise.[fig:samplenoisylineplot]

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
