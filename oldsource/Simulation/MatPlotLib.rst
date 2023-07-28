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

.. figure:: SciPyFigures/plot_1.*
   :width: 60%
   :align: center

.. figure:: SciPyFigures/plot_2.*
   :width: 60%
   :align: center


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

.. figure:: SciPyFigures/plot_3.*
   :width: 60%
   :align: center

.. figure:: SciPyFigures/plot_4.*
   :width: 60%
   :align: center


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
surface in :numref:`plot:basicsurfaceplot`. Many
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
linear regression :numref:`plot:fitcurveexample`.

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

.. _`plot:basicsurfaceplot`:
.. figure:: SciPyFigures/plot_5.*
   :width: 70%
   :align: center

   Surface plot example.


.. _`plot:fitcurveexample`:
.. figure:: SciPyFigures/plot_6.*
   :width: 70%
   :align: center

   Line fit and plot example.


Animation
^^^^^^^^^^

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



.. figure:: SciPyFigures/plot_8.*
   :width: 60%
   :align: center


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

Thanks to the NumPy and SciPy online tutorials for great examples.


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



.. Owned by Roboscience

.. _`fig:exampleplots`:
.. figure:: SciPyFigures/plot.*
   :width: 90%
   :align: center

   The plot of :math:`y=0.1x^3` using a) points b) a line.


The two plots should look like
:numref:`Fig:exampleplots`. You will notice
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

.. _`cubicsplineexample`:


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

about the point :math:`(4,5)`. We use the eigenvalues and eigenvectors
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


.. figure:: SciPyFigures/Ellipse.*
   :width: 70%
   :align: center

   Tilted ellipse

Data Plots
^^^^^^^^^^

To plot the data used in the curve fitting examples:

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

For Figure in subsection :numref:`plot:quadgraph`:

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
