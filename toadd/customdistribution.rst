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
