Numerical Methods and SciPy [Chap:SciPy]
========================================

Note: This is under significant development.

The following material assumes that you are familiar with Python. Python
reads like pseudocode and so it is possible to follow along without a
background in Python if you have seen some other programming language. A
quick introduction is given in the Appendices for those who want to ramp
up before reading on.

SciPy and Mathematics
---------------------

SciPy, , is a collection of open-source packages for Scientific
Computing. One of the packages, redundantly named, SciPy library is a
collection of numerical methods including special functions,
integration, optimization, linear algebra, interpolation, and other
standard mathematics routines. NumPy is an open-source Python package
supporting data structures and low level algorithms for scientific
computing which is used by SciPy. [1]_ The main data structure of
interest to us from numpy is an array type and efficient methods to
access array elements.

Many of the implementations of iPython load NumPy and SciPy (as well as
the plotting package matlibplot) automatically. The idea is that most
users of iPython are going to use these. To use the NumPy or SciPy
libraries you need to import them. Since the scientific libraries are
large, we don’t want to drop them into the main namespace. The Python
community now uses the standard names for the namespaces:

::

    >>> import numpy as np
    >>> import scipy as sp
    >>> import matplotlib as mpl
    >>> import matplotlib.pyplot as plt

Scipy sub-packages need to be imported separately, for example:

::

    >>> from scipy import linalg, optimize

As stated above, some versions of iPython (some IDEs) will import
certain libraries for you. Say you are tired of typing the five import
lines above each time you run iPython. There is a full configuration
system available. To find the location of the config files, type at the
command prompt

.. code:: bash

    $ ipython locate
    /home/yourloginname/.ipython

    $ ipython locate profile foo
    /home/yourloginname/.ipython/profile_foo

| You may create a profile for each of the different iPython activities.
  We will stick with the default which is profile_default. The startup
  files, files that get run when you start iPython, are located in the
  startup subdirectory. In my case this is:
| /Users/jmcgough/.ipython/profile_default/startup

Inside the startup directory, I created a file: 05-early.py containing

::

    import numpy as np
    import scipy as sp
    import matplotlib as mpl
    import matplotlib.pyplot as plt

which then runs those import commands each time iPython is invoked. In
this next section, we will review some needed mathematics and introduce
SciPy as we proceed.

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

Graphing parametric functions
-----------------------------

The Python plot command (well, this is actually the MatPlotLib library
for Python) takes an array of x values and an array of y values. This
means that it is very easy to generate explicit plots, :math:`y=f(x)` or
parametric plots, :math:`x=f(t)`, :math:`y=g(t)`. So, for example one
can easily plot a regular function via

::

    import numpy as np
    import pylab as plt

    x = np.linspace(0,5,25) # 25 equally spaced points on [0,5]
    y = 0.15*x*x*x  #  Generate the y values from y = 0.15x^3  

    plt.plot(x,y,'bo')  #  Plot x-y values using blue dots
    plt.show()

    plt.plot(x,y,'b-')  #  Plot x-y values using a blue line
    plt.show()

.. raw:: latex

   \centering

.. figure:: control/plot1
   :alt: The plot of :math:`y=0.1x^3` using points.

   The plot of :math:`y=0.1x^3` using points.

.. figure:: control/plot2
   :alt: The plot of :math:`y=0.1x^3` using a line.

   The plot of :math:`y=0.1x^3` using a line.

The two plots should look like
Figure \ `[Fig:exampleplots] <#Fig:exampleplots>`__. You will notice
that the line plot hides the fact that the underlying data is actually
discrete. The point plot provides the actual points. The same thing can
be done using a parametric version making the small change in the code:

::

    t = np.linspace(0,5,25)
    x = t
    y = 0.15*t*t*t

You will also notice that the space between the points is not the same
even though x (or t) was generated using uniform spacing. The x spacing
is uniform, but the y value is s nonlinear function of x and the spacing
between is not constant.

Code Sample (heart):
^^^^^^^^^^^^^^^^^^^^

::

    import numpy as np
    import pylab as plt
    import math
    t = np.linspace(-math.pi,math.pi,200)
    x = 16*(np.sin(t))**3
    y = 13*np.cos(t) - 5*np.cos(2*t) - 2*np.cos(3*t) - np.cos(4*t)
    plt.plot(x,y,'r-')
    plt.show()

[cubicsplineexample] Assume you want the spline that connects the points
(1,-1) with (3,4). Also assume that the derivative at (1,-1) is given by
:math:`<1,-3>` and at (3,4) is given by :math:`<0,2>`. We can take
:math:`t_0=0` and :math:`t_1 = 1`. This gives :math:`z = t`,
:math:`\dot{z} = 1`, :math:`a = 1 - 2 = -1`, :math:`b = 2`,
:math:`c = -8`, :math:`d = 3`. This gives us the two splines for the
parametric description of the curve:

.. math:: x(t) = (1-t) + 3t + t(1-t)[-1(1-t) + 2t]  = -3 t^3+4 t^2+t+1

.. math:: y(t) = -(1-t) + 4t + t(1-t)[-4(1-t)+3t] =  -11 t^3+19 t^2-3 t-1

.. math:: \dot{x} = -9t^2+8t+1, \quad \ddot{x} =   -18t+8

.. math:: \dot{y} =   -33t^2 +38t -3, \quad \ddot{y} =  -66t+38

See Figure \ `[cubicsplinefigure] <#cubicsplinefigure>`__ for a plot.

::

    t0, t1 = 0, 1
    x0, y0 = 1, -1
    x1, y1 = 3, 4
    xd0 , yd0 = 1, -3
    xd1 = 0
    yd1 = 2
    dt = (t1-t0)
    dx = (x1-x0)
    dy = (y1-y0)
    a = xd0*dt- dx
    b = -xd1*dt+dx
    c = yd0*dt-dy
    d = -yd1*dt+dy
    t = np.linspace(t0,t1,100)
    dotz = 1.0/dt
    z = (dotz)*(t-t0)
    x = (1-z)*x0 + z*x1+z*(1-z)*(a*(1-z)+b*z)
    y = (1-z)*y0 + z*y1+z*(1-z)*(c*(1-z)+d*z)
    ptx = np.array([x0,x1])
    pty = np.array([y0,y1])

    plt.figure()
    plt.xlim(0,4)
    plt.ylim(-2,5)
    plt.plot(ptx,pty, 'ro',x,y,'g-')
    plt.legend(['Data', 'Interpolant'],loc='best')
    plt.title('Cubic Spline')
    plt.show()

.. raw:: latex

   \centering

.. figure:: control/cubicspline.pdf
   :alt: Graph of the spline for
   example \ `[cubicsplineexample] <#cubicsplineexample>`__.[cubicsplinefigure]

   Graph of the spline for
   example \ `[cubicsplineexample] <#cubicsplineexample>`__.[cubicsplinefigure]

Error Ellipses
^^^^^^^^^^^^^^

In the section on Kalman filters, we will want to track the progress of
the filter by tracking the error of the estimate. It is normally
represented by an error ellipse where the ellipse size is the variances
or standard deviations of the Kalman estimate. Thus the larger the
standard deviations then the larger the ellipse. As you will see later
Kalman process produces a covariance, :math:`P`. The eigenvalues and
eigenvectors of :math:`P` can be used for the basic variance
information. The eigenvectors represent the major and minor axis
directions and the eigenvalues represent the lengths of those axes.
Note: in some applications it makes sense to graph the standard
deviations instead of the variances and so one should take the square
root of the eigenvalues. The algorithm follows.

-  Compute the eigenvalues and eigenvectors of :math:`P`:
   :math:`(\lambda_1, v_1)`, :math:`(\lambda_2, v_2)`. Call the larger
   one :math:`a` and the smaller one :math:`b`.

-  Compute the square roots of the eigenvalues IF desired (if the
   variances are really small or really huge).

-  Compute the smaller angle between the eigenvector and the
   :math:`x`-axis. Call this :math:`\theta` and assume it is for
   :math:`v_1`.

-  Call an ellipse routine to plot.

Let a, b be the major and minor axis lengths, x0, y0 be the center and
angle be the tilt angle. The function to plot an rotated ellipse is
given by:

::

    def Ellipse(a,b,angle,x0,y0): 
        points=100 
        cos_a,sin_a=math.cos(angle*math.pi/180),math.sin(angle*math.pi/180)
        theta=np.linspace(0,2*np.pi,points)
        X=a*np.cos(theta)*cos_a-sin_a*b*np.sin(theta)+x0
        Y=a*np.cos(theta)*sin_a+cos_a*b*np.sin(theta)+y0
        return X,Y

The following is an example of how to plot an error ellipse for the
covariance matrix

.. math:: P = \begin{pmatrix} 0.9 & 0.1 \\ 0.1 & 0.5 \end{pmatrix}

\ about the point :math:`(4,5)`. We use the eigenvalues and eigenvectors
to plot the major and minor axes. The following is a quick example on
how to extract eigenvalues and plot an ellipse.

::

    import math
    import numpy as np
    import pylab as plt
    from numpy import linalg 
    P = np.array([[0.9, 0.1],[0.1, 0.5]])
    w, v = linalg.eig(P)
    angle = 180*math.atan2(v[1][0],v[0][0])/math.pi
    u,v = Ellipse(w[0],w[1],angle, 4,5)
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(u,v,'b-')
    ax.set_aspect('equal')
    fig.savefig("Ellipse.pdf")
    plt.show()

.. raw:: latex

   \centering

.. figure:: localization/Ellipse
   :alt: Tilted ellipse

   Tilted ellipse

Data Plots
^^^^^^^^^^

| For Figure in
  subection\ `[ex:curvefitexample] <#ex:curvefitexample>`__:

::

    import numpy as np
    import pylab as plt
    x = []
    y = []
    f = open('data.txt','r')
    for line in f:
      item = line.split()
      xt = eval(item[0])
      yt = eval(item[1])
      x.append(xt)
      y.append(yt)

    plt.plot(x,y, 'ro')
    plt.show()

| For Figure in subsection\ `[plot:quadgraph] <#plot:quadgraph>`__:

::

    import numpy as np
    import pylab as plt
    from scipy import linalg

    xl = []
    yl = []
    f = open('data.txt','r')
    for line in f:
      item = line.split()
      xt = eval(item[0])
      yt = eval(item[1])
      xl.append(xt)
      yl.append(yt)

    N = len(xl)
    x = np.array(xl)
    y = np.array(yl)
    xx = x*x
    A = np.array([xx, x, np.ones((N))]).T
    AT = np.array([xx, x, np.ones((N))])
    AA = np.dot(AT,A)
    ATy = np.dot(AT,y)

    c = linalg.solve(AA,ATy)
    t = np.arange(0,3, 0.1)
    tt = t*t
    B = np.array([tt,t,np.ones(len(t))]).T
    s = np.dot(B,c)

    plt.plot(t,s, 'b-', x,y, 'ro')
    plt.xlim(0,3)
    plt.ylim(0,2)
    plt.show()

Note that NumPy/SciPy provides some built in functions to fit
polynomials to lines. The NumPy function linalg.lstsq will compute the
pseudoinverse via the normal equations directly and the NumPy function
polyfit will do this assuming you are fitting a polynomial. In terms of
speed, doing it ourselves tends to be fastest, with the next fastest is
the lstsq function and the polyfit function the slowest.

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

MatPlotLib
----------

MatPlotLib is the SciPy library used for generating plots. We will be
using the *pyplot* functions from it. The standard import convention is
import matplotlib.pyplot as plt. The basic tool is the function plot.
Let x and y be lists of numbers representing the points
:math:`(x_i , y_i)`. Simple plots can be made using plot(x,y).

::

    In [1]: x = [0,1,2,3,4]

    In [2]: y = [0,1,4,9,16]

    In [3]: plt.plot(x,y)
    Out[3]: [<matplotlib.lines.Line2D at 0x105079490>]

    In [4]: plt.show()

    In [5]: plt.plot([1,2,3,4], [1,4,9,16], 'ro')
    Out[5]: [<matplotlib.lines.Line2D at 0x110a586d0>]

    In [6]: plt.axis([0, 6, 0, 20])
    Out[6]: [0, 6, 0, 20]

    In [7]: plt.show()

This code produces the following two plots:

|image| |image|

This is efficiently done using NumPy arrays instead of lists and using
NumPy functions to generate the arrays.

::

    In [1]: x = np.arange(0,10,0.1)

    In [2]: y = np.sin(x)

    In [3]: plt.plot(x,y,'b-')
    Out[3]: [<matplotlib.lines.Line2D at 0x104880490>]

    In [4]: plt.show()

    In [5]: z = np.cos(x)

    In [6]: plt.plot(y,z)
    Out[6]: [<matplotlib.lines.Line2D at 0x105ea7810>]

    In [7]: plt.show()

|image| |image|

Surface plots may be done by importing the library mpl_toolkits.mplot3d.
For surface plotting to work, a meshgrid needs to be created. This can
be easily built from the x and y array data. The 3D plotting support is
in a toolit shipped wiht matplotlib. It is accessed via the axis setting
in the figure function:

::

    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

An example of a quadratic
surface \ `[plot:basicsurfaceplot] <#plot:basicsurfaceplot>`__ Many
other plot examples can be found at the MatPlotLib website.

::

    from mpl_toolkits.mplot3d import Axes3D
    import matplotlib.pyplot as plt
    import numpy as np
    x = np.arange(0, 10, 0.2)
    y = np.arange(0, 10, 0.2)
    N,M = x.size, y.size

    x,y = np.meshgrid(x,y)
    z = (x-5)*(x-5) + (y-6)*(y-6)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot_surface(x, y, z, rstride=1, cstride=1, color='b')
    plt.show()

Another example will illustrate both the plotting capability as well as
linear regression \ `[plot:fitcurveexample] <#plot:fitcurveexample>`__.

::

    import numpy as np
    import matplotlib.pyplot as plt
    from scipy import linalg

    xl = [  0.        ,   1.11111111,   2.22222222,   3.33333333,
          4.44444444,   5.55555556,   6.66666667,   7.77777778,
          8.88888889,  10.        ]
    yl = [  1.86113482,   3.81083902,   4.1465256 ,   7.37843476,
          10.76437019,  11.99975421,  14.59486508,  16.0576472 ,
          20.77206089,  20.4204027 ]

    N = len(xl)
    x = np.array(xl)
    y = np.array(yl)
    A = np.array([x, np.ones((N))]).T
    AT = np.array([x, np.ones((N))])
    AA = np.dot(AT,A)
    ATy = np.dot(AT,y)

    c = linalg.solve(AA,ATy)
    t = np.arange(0,10, 0.25)
    B = np.array([t,np.ones(len(t))]).T
    s = np.dot(B,c)

    plt.plot(t,s, 'b-', x,y, 'ro')
    plt.xlim(0,10)
    plt.ylim(0,20)
    plt.show()

.. raw:: latex

   \centering

.. figure:: math/plot_5
   :alt: Surface plot example.[plot:basicsurfaceplot]

   Surface plot example.[plot:basicsurfaceplot]

.. figure:: math/plot_6
   :alt: Line fit and plot example.[plot:fitcurveexample]

   Line fit and plot example.[plot:fitcurveexample]

Animation
~~~~~~~~~

Animation is done using the draw command. Create a plot with the plot
command and then update the lists using the set_ydata command. The draw
commend will draw the updated data into the existing plot window.

::

    from pylab import *
    import time

    ion()

    tstart = time.time()               # for profiling
    x = arange(0,2*pi,0.01)            # x-array
    line, = plot(x,sin(x))
    for i in arange(1,200):
        line.set_ydata(sin(x+i/10.0))  # update the data
        draw()                         # redraw the canvas

    print 'FPS:' , 200/(time.time()-tstart)

Interactive mode needs to be toggled using ion() and an empty plot
created. Next a loop runs through the positions of the points. The setp
command updates the plot data values. Appended to the plot values (the
plot comand) is the previous points to give the effect of a traced path.
After the animation, interactive mode is toggled, ioff() and the show()
command is executed to hold the image.

::

    import numpy as np
    import matplotlib.pyplot as plt
    import time
    from math import *

    plt.ion()

    line = plt.plot([],[],'ro')
    plt.xlim(0, 10)
    plt.ylim(0, 10)
    plt.xlabel('x')
    plt.ylabel('y')
    plt.draw()
    dt = 0.1

    for t in np.arange(0,8,dt):
        x = t
        y =  x*(8-x)/2.0
        plt.setp(line,xdata = x, ydata = y)
        plt.draw()
        plt.plot([x],[y],'b.')

    plt.ioff()
    plt.show()

.. raw:: latex

   \centering

|image|

Another animation example is to give virtual velocity commands to move a
point. Say you wanted to animate an object which was moving by

.. math::

   \displaystyle \left(\frac{dx}{dt}, \frac{dy}{dt}\right) =
   \left\{
   \begin{array}{ll}
   (0.5, 0.0),  & 0 \leq t < 2, \\[3mm]
   (0.25, 1.0),  & 2 \leq t < 5, \\[3mm]
   (1.0, 0.0),  & 5 \leq t < 8, \\[3mm]
   (0.3, -1.0), & 8 \leq t < 10, 
   \end{array}
   \right.

and starting at :math:`t=0`, :math:`(x,y)  = (0.1, 3)`. Using the
approximation of the derivative

.. math::

   \displaystyle \frac{dx}{dt} \approx \frac{x(t+\Delta t) - x(t)}{\Delta t}
   \quad\quad \Rightarrow \quad\quad
    \left[ x_\text{current} + \left(\frac{dx}{dt}\right) \Delta t \right] \rightarrow   x_\text{new}

::

    import numpy as np
    import matplotlib.pyplot as plt
    import time
    from math import *

    plt.ion()

    line, = plt.plot([],[],'bo')
    plt.xlim(0, 10)
    plt.ylim(0, 10)
    plt.xlabel('x')
    plt.ylabel('y')
    plt.draw()
    x = 0.1
    y = 3
    dt = 0.1


    for t in np.arange(0,10,dt):
        if t < 2:
            x = x + 0.5*dt
        if (t>=2) and (t<5):
            x = x + 0.25*dt
            y = y + dt
        if (t>=5) and (t<8):
            x = x + dt
        if (t>=8):
            x = x+0.3*dt
            y = y - dt
        line.set_xdata([x])
        line.set_ydata([y])
        plt.draw()
        time.sleep(0.1)

    plt.ioff()
    plt.show()

.. [1]
   Thanks to the NumPy and SciPy online tutorials for great examples.

.. |image| image:: math/plot_1
.. |image| image:: math/plot_2
.. |image| image:: math/plot_3
.. |image| image:: math/plot_4
.. |image| image:: math/plot_8

