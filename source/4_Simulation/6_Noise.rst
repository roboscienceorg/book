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
:numref:`fig:samplescatterplot`.  The figure on the left is a plot of 100 values.
The figure on the right is a perturbation of a simple line.  

.. code-block:: julia
   :caption: Random number generation
   :name: lst:randomnumbers
   :dedent: 1

    using Random, Distributions, Plots
    Random.seed!(123) # Setting the seed
    mu1 = 1.0
    sigma1 = 0.5
    d1 = Normal(mu1, sigma1)
    mu2 = 2.0
    sigma2 = 1.0
    d2 = Normal(mu2, sigma2)
    x = rand(d1,100)
    y = rand(d2,100)
    p1=scatter(x,y,ratio=1)

    error = rand(Normal(mu1, sigma1),100)
    x = LinRange(0,5,100)
    y = 2 .* x.+1.0 .+ error
    p2 = scatter(x,y,ratio=1)
    plot(p1,p2, layout=(1,2))
    savefig("random.svg")
    readline()

    

.. _`fig:samplescatterplot`:
.. figure:: SimulationFigures/random.*
   :width: 90%
   :align: center

   Scatter type plots.  a) A scatter type plot.
   b)  line with lots of noise.

Above we are sampling from a single normal distribution (univariate),
however, later on we will need to sample from multivariate distribution.
We provide the algorithm below or this can be done with
np.random.multivariate_normal.

.. code-block:: julia
   :caption: Random number generation with given parameters
   :name: lst:randomnumberswithparameters
   :dedent: 1

    using Distributions, LinearAlgebra

    mean = [0.0 ; 0.0]
    covar = [.5 .05; .05 1.0]
    d = MvNormal(mean, covar)
    x = rand(d,10)
    println(x)



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


.. code-block:: julia
   :caption: Wheel velocity functions
   :name: lst:wheelvelocityfn
   :dedent: 1

    using Distributions, Plots, StatsPlots
    
    function wheels(t)
       if (t < 1.5)
           w1 = 1.0
           w2 = 1.0
           return w1, w2
       end
       if (t < 3)
           w1 = 2.0
           w2 = 1.0
           return w1, w2
       end
       w1 = 1.0
       w2 = 1.0
       return w1, w2
    end


The setup for the simulation is

.. code-block:: julia
   :caption: Setup arrays
   :name: lst:setuparrays
   :dedent: 1

    r = 20.0
    l = 12.0
    N = 10
    dt = 0.01
    Tend = 5
    NumP = Int(Tend/dt)

    mu1, sigma1 = 0.0, 0.05
    mu2, sigma2 = 0.0, 0.01
    d1 = Normal(mu1, sigma1)
    d2 = Normal(mu2, sigma2)
    tp = LinRange(0,Tend,NumP)

    xpath  = zeros(N,NumP)
    ypath = zeros(N,NumP)
    thpath = zeros(N,NumP)



We selected the same noise range for the :math:`x,y` variables but a smaller range
for the :math:`\theta` variable.  Small changes in :math:`\theta` variable can have a
greater impact on the final location than small changes in :math:`x,y`.
The arrays xpath, ypath and thpath are declared as two dimensional arrays.  This
is so we can store multiple paths.  Meaning we are storing :math:`N` paths which
are comprised of :math:`Nump` points.

To create the paths we run a double loop, where the outside loop is over the
paths and the inside loop creates the points on a specific path.

.. code-block:: julia
   :caption: Generate Points
   :name: lst:generatepoints
   :dedent: 1

    for k = 1:N
        th = 0.0
        x = 0.0
        y = 0.0
        errx = rand(d1,NumP)
        erry = rand(d1,NumP)
        errth = rand(d2,NumP)
        for i = 2:NumP
            w1,w2 = wheels(tp[i])
            dx = (r*dt/2.0)*(w1+w2)*cos(th) + errx[i]
            dy = (r*dt/2.0)*(w1+w2)*sin(th) + erry[i]
            dth = (r*dt/(2.0*l))*(w1-w2) + errth[i]
            x = x + dx
            y = y + dy
            th = th + dth
            xpath[k,i] = x
            ypath[k,i] = y
            thpath[k,i] = th
        end
    end

This can be visualized by

.. code-block:: julia
   :caption: Plot Points
   :name: lst:plotpoints
   :dedent: 1

    plot(0,0,legend=false)
    for j=1:N
        plot!(xpath[j,:], ypath[j,:], legend=false)
    end

    p = plot!(title="Paths with noise")
    display(p)
    readline()

which is shown in :numref:`multiplepathsnoise`.

.. _`multiplepathsnoise`:
.. figure:: SimulationFigures/noisypaths.*
   :width: 70%
   :align: center

   Multiple paths from the same starting point using a noisy model.

We can keep adding additional paths to see the distribution of the final locations.
It gets too hard to see what it going on, so we only plot the last point
on a particular path.  The noise free path is also included and result is
shown in :numref:`multipleendpts`.

.. _`multipleendpts`:
.. figure:: SimulationFigures/noisyendpoints.*
   :width: 70%
   :align: center

   Showing the noise free path and the endpoints for the noisy paths.

For linear Gaussian processs, if we ran this millions of times and then produced
a histogram of the results, we would see a 2D normal distribution emerge.   Cross sections
of the 2D normal would be ellipses.   The larger the ellipse the greater confidence value
we have.  Since this is *not* a linear process, we don't expect a normal distribution,
but we do expect some distribution.  So we will treat this in a similar
fashion.   To compute the error ellipse for the 95% confidence, we store the
final points in parallel arrays for xpts and ypts, and run the following code
block:

.. code-block:: julia
   :caption: Covariance Ellipse
   :name: lst:covarianceellipse
   :dedent: 1

    A = [xpts ypts]
    cmat = cov(A)
    mx = mean(xpts)
    my = mean(ypts)
    covellipse([mx,my], cmat, n_std=2)
    plot!(xp,yp, legend=false)
    p = scatter!(xpts, ypts)
    display(p)
    savefig("ellipsepath.svg")
    readline()

What this does is to take the two data sets, x and y and compute the covariance
matrix, stored in cmat.   



.. _`pathsellipse`:
.. figure:: SimulationFigures/ellipsepath.*
   :width: 70%
   :align: center

   The error ellipse.
