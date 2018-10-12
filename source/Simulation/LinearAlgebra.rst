
Vectors and Matrices
--------------------

Creation of an array is easy:

::

    In [1]: import numpy as np

    In [2]: x = np.array([2,3,6,4,5,0])

    In [3]: x
    Out[3]: array([2, 3, 6, 4, 5, 0])

    In [4]: len(x)
    Out[4]: 6

The array command takes a list and converts it to an array object.
Arrays are stored like C does it, in row major order. To create an array
of numbers from 0 to 10:

::

    In [2]: x = np.arange(10)

    In [3]: x
    Out[3]: array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9])

    In [4]: 2*x+1
    Out[4]: array([ 1,  3,  5,  7,  9, 11, 13, 15, 17, 19])

    In [5]: x*x
    Out[5]: array([ 0,  1,  4,  9, 16, 25, 36, 49, 64, 81])

    In [6]: np.sqrt(x)
    Out[6]:
    array([ 0.        ,  1.        ,  1.41421356,  1.73205081,  2.        ,
            2.23606798,  2.44948974,  2.64575131,  2.82842712,  3.        ])

    In [7]: np.sin(0.2*x)
    Out[7]:
    array([ 0.        ,  0.19866933,  0.38941834,  0.56464247,  0.71735609,
            0.84147098,  0.93203909,  0.98544973,  0.9995736 ,  0.97384763])

    In [8]: np.sin(0.2*x)[3]
    Out[8]: 0.56464247339503548

    In [9]: x < 7
    Out[9]: array([ True,  True,  True,  True,  True,
                      True,  True, False, False, False], dtype=bool)

    In [10]: x.sum()
    Out[10]: 45

    In [11]: x.max()
    Out[11]: 9

    In [12]: x.min()
    Out[12]: 0

    In [13]: x.mean()
    Out[13]: 4.5

    In [14]: x.std()
    Out[14]: 2.8722813232690143

    In [15]: np.where(x < 7)
    Out[15]: (array([0, 1, 2, 3, 4, 5, 6]),)

Note that indexing works like normal Python lists. A few vector
operations are also available as methods.

::

    In [2]: x = np.arange(10)

    In [3]: y = np.ones(10)

    In [4]: x
    Out[4]: array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9])

    In [5]: y
    Out[5]: array([ 1.,  1.,  1.,  1.,  1.,  1.,  1.,  1.,  1.,  1.])

    In [6]: np.dot(x,y)
    Out[6]: 45.0

    In [7]: np.outer(x,y)
    Out[7]:
    array([[ 0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.],
           [ 1.,  1.,  1.,  1.,  1.,  1.,  1.,  1.,  1.,  1.],
           [ 2.,  2.,  2.,  2.,  2.,  2.,  2.,  2.,  2.,  2.],
           [ 3.,  3.,  3.,  3.,  3.,  3.,  3.,  3.,  3.,  3.],
           [ 4.,  4.,  4.,  4.,  4.,  4.,  4.,  4.,  4.,  4.],
           [ 5.,  5.,  5.,  5.,  5.,  5.,  5.,  5.,  5.,  5.],
           [ 6.,  6.,  6.,  6.,  6.,  6.,  6.,  6.,  6.,  6.],
           [ 7.,  7.,  7.,  7.,  7.,  7.,  7.,  7.,  7.,  7.],
           [ 8.,  8.,  8.,  8.,  8.,  8.,  8.,  8.,  8.,  8.],
           [ 9.,  9.,  9.,  9.,  9.,  9.,  9.,  9.,  9.,  9.]])

Some NumPy examples using 2D arrays (or matrices):

::

    In [2]: A = np.array([[1,2,3],[4,5,6]])

    In [3]: print A
    [[1 2 3]
     [4 5 6]]

    In [4]: B = np.array([[9,8],[7,6],[5,4]])

    In [5]: print B
    [[9 8]
     [7 6]
     [5 4]]

    In [6]: A*B
    --------------------------
    ValueError                             Traceback (most recent call last)
    <ipython-input-6-e2f71f566704> in <module>()
    ----> 1 A*B

    ValueError: operands could not be broadcast together with shapes
    (2,3) (3,2)

    In [7]: np.dot(A,B)
    Out[7]:
    array([[ 38,  32],
           [101,  86]])

    In [8]: A.T
    Out[8]:
    array([[1, 4],
           [2, 5],
           [3, 6]])

    In [9]: A.T + B
    Out[9]:
    array([[10, 12],
           [ 9, 11],
           [ 8, 10]])

Note: Most of the python overloaded math operators are defined
elementwise. As such :math:`*` does not make sense for :math:`A*B` since
the arrays are not the same dimension. The point is that you need to be
careful and in this case you need to call the correct function to do
matrix multiplication and not array multiplication.

One can easily create a two dimensional array by reshaping:

::

    In [10]: z = np.arange(16)

    In [11]: z
    Out[11]: array([ 0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12,
                                    13, 14, 15])

    In [12]: z.shape = (4,4)

    In [13]: z
    Out[13]:
    array([[ 0,  1,  2,  3],
           [ 4,  5,  6,  7],
           [ 8,  9, 10, 11],
           [12, 13, 14, 15]])

    In [14]: z[1,3]
    Out[14]: 7

    In [15]: z[1,-4]
    Out[15]: 4

Using previous examples of :math:`A` and :math:`B`:

::

    In [16]: import numpy.linalg as npl

    In [17]: npl.det(np.dot(A,B))
    Out[17]: 35.99999999999968




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
  :numref:`Fig:weightedLSdata` and
  :numref:`Fig:weightedLSplot` can be produced by:

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
