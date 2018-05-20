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
surface :ref:`plot:basicsurfaceplot` Many
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


.. figure:: SciPyFigures/plot_5.*
   :width: 70%
   :align: center

   Surface plot example.[plot:basicsurfaceplot]

.. figure:: SciPyFigures/plot_6.*
   :width: 70%
   :align: center

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
