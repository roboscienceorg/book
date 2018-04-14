Linear Algebra
--------------

We use both NumPy and SciPy for Linear Algebra problems. NumPy is used
to provide the array data structure and the numerical methods are
provided in SciPy.

::

    In [1]: import numpy as np

    In [2]: import scipy as sp

    In [3]: from scipy import linalg as spl

    In [4]: A = np.array([[3,1,0],[1,5,1],[0,2,6]])

    In [5]: A
    Out[5]: 
    array([[3, 1, 0],
           [1, 5, 1],
           [0, 2, 6]])

    In [6]: b = np.array([[3,2,1]]).T

    In [7]: b
    Out[7]: 
    array([[3],
           [2],
           [1]])

    In [8]: x1 = spl.inv(A).dot(b)  # x = inverse(A)*b

    In [9]: x1
    Out[9]: 
    array([[ 0.93589744],
           [ 0.19230769],
           [ 0.1025641 ]])

    In [10]: x2 = spl.solve(A,b)  # solve Ax = b

    In [11]: x2
    Out[11]: 
    array([[ 0.93589744],
           [ 0.19230769],
           [ 0.1025641 ]])

    In [12]: A.dot(x1)
    Out[12]: 
    array([[ 3.],
           [ 2.],
           [ 1.]])

One question that arises is regarding performance. There is a
significant difference between plain Python and NumPy. This author’s
experiments have shown that NumPy performs very well and has fallen
within 10-20% of plain C in some cases. Given how powerful the
Python-NumPy combination is, this is a small price.

::

    In [2]: import scipy.linalg as lin

    In [3]: a = np.array([[3, 1, 0], [1, 5, 1],  [0, 2, 6]])

    In [4]: lin.eig(a)
    Out[4]: 
    (array([ 2.48586307+0.j,  4.42800673+0.j,  7.08613020+0.j]),
     array([[ 0.86067643,  0.39715065,  0.11600488],
           [-0.44250554,  0.5671338 ,  0.47401104],
           [ 0.25184308, -0.72154737,  0.87284386]]))

Eigenvalues pop up all through engineering computations and we will use
the built in SciPy routines to compute them. The most common application
later will be finding the error ellipses for variance-covariance
matrices in the Kalman Filter.

Least Squares Examples
~~~~~~~~~~~~~~~~~~~~~~

| Assume that you have the raw data ready in arrays :math:`x` and
  :math:`y`. Then
  Figures \ `[Fig:weightedLSdata] <#Fig:weightedLSdata>`__ and
  `[Fig:weightedLSplot] <#Fig:weightedLSplot>`__ can be produced by:

::

    one = np.ones((N))
    A = np.array([ x, one]).T
    AT = A.T
    AA = np.dot(AT,A)
    ATy = np.dot(AT,y)
    t = np.arange(0,10, 0.2)
    B = np.array([t,np.ones(len(t))]).T

    c = linalg.solve(AA,ATy)
    line1 = np.dot(B,c)

    weights =[]
    sum = 0
    for i in range(1,N+1):
        v = 1.0/(i*i*i)
        sum = sum + v
        weights.append(v)

    for i in range(N):
        weights[i] = weights[i]/sum

    ww = np.diagflat(weights)
    A1 = np.dot(ww,A)
    AA = np.dot(AT,A1)
    y1 = np.dot(ww,y)
    ATy = np.dot(AT,y1)
    coeff2 = linalg.solve(AA,ATy)
    line2 = np.dot(B,coeff2)

    # Plot result: red is data, blue is uniformly weighted,
    #  green is weighted to points near the origin.
    plt.plot(t,line1, 'b-', t,line2, 'g-', x,y, 'r.')
    plt.show()
