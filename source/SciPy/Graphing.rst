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
