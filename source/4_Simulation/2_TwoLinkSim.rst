
Two Link Simulation
-----------------------------------------

Position Computations
~~~~~~~~~~~~~~~~~~~~~~~


In this section we walk through some very simple computations
seen in the previous chapters.  It also shows how we can use Julia
as a computational tool like seen with Matlab or Python.
This chapter is not intended to teach Julia.  Julia is a modern
programming language which is well documented at https://docs.julialang.org .
The language is pretty easy to follow and if you have a background in any
standard procedural programming language, Julia is easy to read.


The Julia code to do the forward kinematics for the
two link manipulator is


.. code-block:: julia
   :caption: Forward Kinematics for the Two Link Manipulator
   :name: listFKTwoLink
   :dedent: 1

    a1,a2 = 15.0,10.0
    t1, t2 = 0.1, 0.2
    x = a2*cos(t1+t2) + a1*cos(t1)
    y = a2*sin(t1+t2) + a1*sin(t1)
    println("x = ", x, ", y = ", y)



The Julia code to do the Inverse Kinematics for the Two Link Manipulator follows.
The computation is check by plugging the computed values into the forward kinematics
formula.


.. code-block:: julia
   :caption: Inverse Kinematics for the Two Link Manipulator
   :name: listIKTwoLink
   :dedent: 1

    a1,a2 = 15.0,10.0
    x,y = 10.0,8.0
    d =  (x*x+y*y-a1*a1-a2*a2)/(2*a1*a2)
    println("x = ", x, " y = ", y)
    println("d = ", d)

    t2 = atan(-sqrt(1.0-d*d),d)
    t1 = atan(y,x) - atan(a2*sin(t2),a1+a2*cos(t2))
    println("Angles:  t1 =", t1,",  t2 = ", t2)

    x1 = a2*cos(t1+t2) + a1*cos(t1)
    y1 = a2*sin(t1+t2) + a1*sin(t1)
    println("Check: x = ", x1, ", y = ", y1)



The code to compute the position for the Parallel Two LInk manipulator
based on the parameters is


.. code-block:: julia
   :caption: Forward Kinematics for the Parallel Two Link Manipulator
   :name: lst:FKParallelTwoLink
   :dedent: 1

    L0,L1,L2 = 8, 5, 10
    th1 = 0.7
    th2 = 0.8

    a = -L1*cos(th1)-L0/2.0
    b = L1*sin(th1)
    c =  L1*cos(th2)+L0/2.0
    d = L1*sin(th2)
    u = sqrt((a-c)^2 +(b-d)^2)
    v = sqrt(L2^2 - (u^2)/4.0)

    x = (a+c)/2.0 + v*(b-d)/u
    y = (b+d)/2.0 + v*(c-a)/u

    println("x = ", x,",  y = ",y)


The inverse kinematics for the parallel two link manipulator
is given by

.. code-block:: julia
   :caption: Inverse Kinematics example for the Parallel Two Link manipulator.
   :name: lst:IKParallelTwoLink
   :dedent: 1


    L0,L1,L2 = 8, 5, 10
    x, y = -0.5, 9


    G = sqrt((x-L0/2.0)*(x-L0/2.0)+y*y)
    H = sqrt((x+L0/2.0)*(x+L0/2.0)+y*y)

    alpha = acos((G*G + L0*L0 - H*H)/(2.0*G*L0))
    beta = acos((H*H + L0*L0 - G*G)/(2.0*H*L0))
    gamma = acos((G*G + L1*L1 - L2*L2)/(2.0*G*L1))
    eta = acos((H*H + L1*L1 - L2*L2)/(2.0*H*L1))

    th1 = pi - beta - eta
    th2 = pi - alpha - gamma

    println("Angles theta1 = ", th1, ",  theta2 = ", th2)


Velocity computations
~~~~~~~~~~~~~~~~~~~~~~~

In addition to forward and inverse position kinematics, one may need to compute the forward and inverse velocity kinematics.  Starting with :math:`x(t), y(t)`

.. math::

   \begin{matrix}
   x = a_2\cos (\theta_1+\theta_2) + a_1 \cos (\theta_1)\\
   y = a_2 \sin (\theta_1 +\theta_2) + a_1\sin (\theta_1)
   \end{matrix}

we compute the derivatives

.. math::
   :label: twolinkforward2

   \displaystyle \frac{dx}{dt} = -a_2\sin (\theta_1+\theta_2) \left( \frac{d\theta_1}{dt} + \frac{d\theta_2}{dt} \right)
   -  a_1 \sin (\theta_1) \left( \frac{d\theta_1}{dt}  \right)
   =  \left( -a_2\sin (\theta_1+\theta_2) -  a_1 \sin (\theta_1) \right) \left( \frac{d\theta_1}{dt}  \right)
    -a_2\sin (\theta_1+\theta_2) \left(\frac{d\theta_2}{dt} \right)

   \displaystyle  \frac{dy}{dt} = a_2 \cos (\theta_1 +\theta_2) \left( \frac{d\theta_1}{dt} + \frac{d\theta_2}{dt} \right)
   + a_1\cos (\theta_1)  \left( \frac{d\theta_1}{dt}  \right)
   =  \left( a_2\cos (\theta_1+\theta_2) +  a_1 \cos (\theta_1) \right) \left( \frac{d\theta_1}{dt}  \right)
    + a_2\cos (\theta_1+\theta_2) \left(\frac{d\theta_2}{dt} \right)


As an example, given the two link manipulator with the length of the first link :math:`a_1 = 10`
and the second link  :math:`a_2 = 10`.  If :math:`\theta_1 = 45^\circ`, :math:`\theta_2 = 45^\circ`,
:math:`d\theta_1/dt = 5^\circ s^{-1}`,  :math:`d\theta_2/dt = 10^\circ s^{-1}` find :math:`x`, :math:`y`,
:math:`dx/dt` and    :math:`dy/dt`.



.. code-block:: julia
   :caption: Forward Velocity Kinematics example
   :name: lst:FKvelocitytwolink
   :dedent: 1
   
    a1,a2 = 10.0,10.0
    θ1, θ2 = 45*(π/180), 45*(π/180)
    θ1dot, θ2dot = 5*(π/180),10*(π/180)
    x = a2*cos(θ1+θ2) + a1*cos(θ1)
    y = a2*sin(θ1+θ2) + a1*sin(θ1)
    xdot = -(a2*sin(θ1+θ2) + a1*sin(θ1))*θ1dot - (a2*sin(θ1+θ2))*θ2dot
    ydot = (a2*cos(θ1+θ2) + a1*cos(θ1))*θ1dot +  (a2*cos(θ1+θ2))*θ2dot
    println("x = ", x, ", y = ", y, ", xdot = ", xdot, ", ydot = ", ydot)



Because the equations are linear in the rate variables, it is much easier to to solve for the
joint velocities as a function of linear velocities.  In this case, the matrix for the 2x2 system is


.. math::

   A = \begin{bmatrix}
   \left( -a_2\sin (\theta_1+\theta_2) -  a_1 \sin (\theta_1) \right) & -a_2\sin (\theta_1+\theta_2) \\
   \left( a_2\cos (\theta_1+\theta_2) +  a_1 \cos (\theta_1) \right)  & a_2\cos (\theta_1+\theta_2)
   \end{bmatrix}

where we have

.. math::
   \begin{pmatrix} dx/dt\\  dy/dt \end{pmatrix} = A \begin{pmatrix} d\theta_1/dt \\ d\theta_2/dt \end{pmatrix}
    \quad \Rightarrow \quad
    \begin{pmatrix} d\theta_1/dt \\ d\theta_2/dt \end{pmatrix} = A^{-1} \begin{pmatrix} dx/dt\\  dy/dt \end{pmatrix}

The inverse of the matrix is

.. math::

   A^{-1} = \frac{1}{a_1a_2\sin(\theta_2)}
   \begin{bmatrix}
    a_2\cos (\theta_1+\theta_2) & a_2\sin (\theta_1+\theta_2) \\
   - \left( a_2\cos (\theta_1+\theta_2) +  a_1 \cos (\theta_1) \right)  &
   -\left(a_2\sin (\theta_1+\theta_2) +  a_1 \sin (\theta_1) \right)
   \end{bmatrix}

This gives use the velocity formulas for the inverse kinematics:

.. math::
   :label: twolinkinverse2

   \frac{d\theta_1}{dt} = \frac{1}{a_1a_2\sin(\theta_2)} \left[  a_2\cos (\theta_1+\theta_2) \frac{dx}{dt}
   + a_2\sin (\theta_1+\theta_2) \frac{dy}{dt} \right]

   \frac{d\theta_2}{dt} = - \frac{1}{a_1a_2\sin(\theta_2)} \left[  \left( a_2\cos (\theta_1+\theta_2) +  a_1 \cos (\theta_1) \right)  \frac{dx}{dt} +
   \left(a_2\sin (\theta_1+\theta_2) +  a_1 \sin (\theta_1) \right) \frac{dy}{dt} \right]

We can gain :math:`\theta_1, \theta_2` from the inverse kinematics formulas.

Example Given the two link manipulator as above with  the length of the first link :math:`a_1 = 10` and the second link   :math:`a_2 = 10`.  If :math:`x = 12`, :math:`y = 14`, :math:`dx/dt = -0.25`, :math:`dy/dt = 0.5`, find :math:`d\theta_1/dt` and :math:`d\theta_2/dt`



.. code-block:: julia
   :caption: Inverse Velocity Kinematics example
   :name: lst:IKvelocitytwolink
   :dedent: 1

    a1,a2 = 10.0,10.0
    x,y = 12, 14
    xdot, ydot = -0.25, 0.5
    d =  (x*x+y*y-a1*a1-a2*a2)/(2*a1*a2)
    θ2 = atan(sqrt(1.0-d*d),d)
    θ1 = atan(y,x) - atan(a2*sin(θ2),a1+a2*cos(θ2))
    r = 1.0/(a1*a2*sin(θ2))
    θ1dot = r*(a2*cos(θ1+θ2)*xdot + a2*sin(θ1+θ2)*ydot)
    θ2dot = -r*(x*xdot + y*ydot)
    println("θ1 = ", 180*θ1/π, ", θ2 = ", 180*θ2/π, ", θ1dot = ", 180*θ1dot/π, ", θ2dot = ", 180*θ2dot/π)


Arm computations
~~~~~~~~~~~~~~~~~~~~~~~

In application we will want the robot end effector trace out a curve in the workspace.
A simple version of this would be to provide a sequence of points for the arm to trace out.
There are two curves that we have in mind.  One is the curve in the physical workspace and
the other is the corresponding curve in the parameter or Configuration space.

To drive a physical arm we send commands to the joints.  This means we are controlling the
arm by driving it along a curve in configuration space.  A simple way to approach this is to
create a sequence of points in configuration space and then move the arm in short updates
on the joint angles.   We will see in the next section a much smoother way to approach
this task however for now, this approach will get the manipulator moving along
the desired path.

.. figure:: SimulationFigures/twolinkpositioncntrl.*
   :width: 50%
   :align: center

   Try a simple position control.  Send a discrete set of control points.



The workflow will be to parametrize the path in the workspace, then find
the curve in the parameter space using the inverse kinematics.  To keep things
simple, we will assume the curve lies inside the reachable workspace.   The goal
is then to have the arm trace out :math:`x(t)` and :math:`y(t)`.  If we want to trace
out

.. math::

   x(t) = t+1, \quad y(t) = 2t - 1

Where :math:`t` ranges from 0 to 5 with 100 points.  A simple Julia program can generate
the :math:`(x,y)` pairs:

.. code-block:: julia
   :name: lst:mostbasicloop
   :caption: A very simple for loop
   :dedent: 1

    for i = 1:100
       t = i/20
       x = t+1
       y = 2*t-1
       println("(", x, ", ",y,")")
    end


For most of our examples we will need to store the points in an array.  Julia has
many ways to perform this and we will show a couple of approaches.


.. code-block:: julia
   :name: lst:basicloop
   :caption: Storing in an array.
   :dedent: 1

    N = 100
    x = zeros(N)
    y = zeros(N)
    delta = 5/N

    for i = 1:N
       t = delta*i
       x[i]= t+1
       y[i] = 2*t-1
    end


Julia supports implicit parallel computations in the same way found in Python's Numpy.
Using the LinRange constructor we can produce t directly.  Then x and y can be found
by the elementwise dot operator.  Note the period in front of the binary operator.

.. code-block:: julia
   :name: lst:basicimplicitloop
   :caption:  Array construction
   :dedent: 1

    t = LinRange(0, 5, 100)
    x =  t .+ 1.0
    y = 2 .* t .- 1.0


We can gain a plot of this easily by adding a couple of statements

.. code-block:: julia
   :name: lst:basicplot
   :caption:  Plot Example
   :dedent: 1

    using Plots
    t = LinRange(0, 5, 100)
    x =  t .+ 1.0
    y = 2 .* t .- 1.0
    display(plot(x,y))
    #readline()

The readline() function is there to prevent the process from exiting before the plot appears when
you are using the REPL.  This is not an issue when using Jupyter and you can skip it.  You also
don't need the display function wrapping the plot function when working in a Jupyter notebook.
Note - if you want to keep these plots, use the savefig function: *savefig("foo.svg")*.  It will save the recent plot using the extension as the file format.  Figure :numref:`Fig:sampleline` is what gets generated.

.. _`Fig:sampleline`:
.. figure:: SimulationFigures/line.*
   :width: 40%
   :align: center

   Sample plot generated by the plot function.


You may have noticed that it took much longer than expected to get the plot window (if you are used to Matlab or Python).  Even though the programs look very similar to Python, Julia is compiled (at run time).  The delay you experience is due to the compilation of the Plots package.  After the compile is completed, the program runs close to the speed of C.   For short programs you notice the delay due to the compile process.   So for the shorter scripts, it reduces the overall speed since each time you run the script from the command line you recompile.  If you plan to run little experiments or plots, it is better to stay in the REPL (the Julia interpreter) or a Jupyter Notebook since you only see the package compile delay once per session.


For larger progams or longer running processes, the increase in speed can be dramatic.  This book will present the code as blocks that can be copied to a file which allows one to run the file from the command line.    However, you may choose to cut and paste the code into the REPL or Jupyter for faster plots so you are not recompiling on each script.    There is some variation is the plotting code to give some
additional examples.

We can enter the array of workspace points into the inverse kinematics formulas.  Then to check we take those points into
the forward kinematics.  Plots for the configuration (parameter) space curve and then the final workspace curve are produced.

.. code-block:: julia
   :name: lst:basicfkandik
   :caption:  The inverse kinematics for a line in the workspace.
   :dedent: 1

    using Plots

    t = LinRange(1, 5, 100)
    x =  t .+ 1.0
    y = 2 .* t .- 1.0
    a1,a2 = 15.0,15.0
    d =  ((x.*x) .+ (y.*y) .- (a1.*a1) .- (a2.*a2))/(2.0 .* (a1.*a2))
    t2 = atan.(-sqrt.(1.0 .- (d.*d)),d)
    t1 = atan.(y,x) - atan.(a2.*sin.(t2), a1 .+ a2.*cos.(t2))

    display(plot(t1,t2))
    #savefig("IK.svg")
    #readline()
    x1 = a2.*cos.(t1.+t2) .+ a1.*cos.(t1)
    y1 = a2.*sin.(t1.+t2) .+ a1.*sin.(t1)
    display(plot(x1,y1))
    #savefig("FK.svg")
    #readline()


.. _`Fig:basicfkandik`:
.. figure:: SimulationFigures/basic_fk_ik.*
   :width: 40%
   :align: center

   First plot generated by :numref:`lst:basicfkandik` .



.. _`Fig:basicfkandikcheck`:
.. figure:: SimulationFigures/basic_fk_ik_final.*
   :width: 40%
   :align: center

   The check on the IK values from :numref:`lst:basicfkandik` .



For another two link example, determine the joint angles to trace out a
circle centered at (10,8) of radius 5 with :math:`a_1 = a_2 = 15`.   The circle can be parametrized by
:math:`x(t) = 5\cos (t) + 10`, :math:`y(t) = 5 \sin(t) + 8`, :math:`0 \leq t \leq 2\pi`.
Generate an array of points on the circle and plug them into the inverse kinematics.
Julia has a macro available ``@.`` that can replace the binary operators with their dotted operators.
It is very handy in translating to a vector formulation.


.. code-block:: julia
   :name: lst:circletrace
   :caption: Julia code to trace a circle
   :dedent: 1

    using Plots

    t = LinRange(0, 6.28318530718, 100)
    x = @. 5 * cos(t) + 10.0
    y = @. 5 * sin(t) + 8.0
    a1,a2 = 15.0,15.0
    d =  @. ((x*x) + (y*y) - (a1*a1) - (a2*a2))/(2.0 * (a1*a2))
    t2 = @. atan(-sqrt(1.0 - (d*d)),d)
    t1 = @. atan(y,x) - atan(a2*sin(t2), a1 + a2*cos(t2))

    display(plot(t1,t2))
    savefig("IK.svg")
    #readline()
    x1 = @. a2*cos(t1+t2) + a1*cos(t1)
    y1 = @. a2*sin(t1+t2) + a1*sin(t1)
    display(plot(x1,y1))
    savefig("FK.svg")
    #readline()




.. _`Fig:circlefkandik`:
.. figure:: SimulationFigures/IKFK.*
   :width: 60%
   :align: center

   Plot generated by :numref:`lst:circletrace` .





The implicit formulation above is very similar to how one might approach developing code
for Numpy.  The traditional approach works well and does not require that much
additional code.  You can see the loops explicitly which is not always a bad thing.


.. code-block:: julia
   :caption: The more traditional approach
   :name: listIKTwoLinkFunction
   :dedent: 1

    function ik(x, y, a1, a2)
       d =  (x*x+y*y-a1*a1-a2*a2)/(2*a1*a2)
       th2 = atan(-sqrt(1.0-d*d),d)
       th1 = atan(y,x) - atan(a2*sin(th2),a1+a2*cos(th2))
       return (th1,th2)
    end


    function fk(t1, t2, a1, a2)
        x = a2*cos(t1+t2) + a1*cos(t1)
        y = a2*sin(t1+t2) + a1*sin(t1)
        return x,y
    end

    a1, a2 = 15.0, 15.0

    t1 = zeros(10)
    t2 = zeros(10)

    for i = 1:10
        global a1, a2, t1, t2
        x = 9 + 0.1*i
        y = 7 + 0.1*i
       (t1[i], t2[i]) = ik(x, y, a1, a2)
       x1, y1 = fk(t1[i], t2[i], a1, a2)
       println((x-x1)^2 + (y-y1)*2)
    end
    p=plot(t1,t2)
    display(p)
    #readline()




Parallel Two Link Computations
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The following sbows the code used to produce the parallel manipulator
workspace shown in :numref:`Fig:paralleltwolinkWS` .
uses a double loop over :math:`\theta_1` and :math:`\theta_2`, which places
these values in the forward kinematics and then gathers the resulting
:math:`(x,y)` values. Like the serial manipulator, this is a holonomic
robot as well.


.. code-block:: julia
   :name: lst:computeconfigdomain
   :caption: Configuration Domain Code
   :dedent: 1

    using Plots

    # Set the link lengths
    L0 = 8
    L1 = 5
    L2 = 10

    # Initialize the arrays
    xlist = zeros(0)
    ylist = zeros(0)

    # Loop over the two angles,
    #  stepping about 1.8 degrees each step

    for i = 1:100
       for j = 1:100
          th1 = 0 + 1.57*i/100.0
          th2 = 0 + 1.57*j/100.0

          a = -L1*cos(th1) - L0/2.0
          b = L1*sin(th1)
          c = L1*cos(th2) + L0/2.0
          d = L1*sin(th2)

          dx = c-a
          dy = b-d
          u = sqrt(dx*dx+dy*dy)
          v = sqrt(L2*L2 - 0.25*u*u)

          x = (a+c)/2.0 + v*dy/u
          y = (b+d)/2.0 + v*dx/u
          push!(xlist, x)
          push!(ylist, y)
      end
    end

    display(scatter(xlist,ylist,legend= false,markersize=5))
    #savefig("2dDeltaWS.svg")
    #readline()


A more compact version of this code can be produced using the implicit loops.


.. code-block:: julia
   :name: lst:computeconfigdomain2
   :caption: Configuration Domain Code with implicit looping.
   :dedent: 1

    using Plots

    # Set the link lengths
    L0 = 8
    L02 = L0/2.0
    L1 = 5
    L2 = 10

    t = LinRange(0, 1.57, 100)
    th1 = t' .* ones(100)
    th2 = ones(100)' .* t
    a = -L1 .* cos.(th1) .- L02
    b = L1 .* sin.(th1)
    c = L1 .* cos.(th2) .+ L02
    d = L1 .* sin.(th2)
    dx = c.-a
    dy = b.-d
    u = sqrt.((dx .* dx) .+ (dy .* dy))
    v = sqrt.((L2 .* L2) .- (0.25.*u.*u))
    x = (a.+c)./2.0 .+ (v.*dy./u)
    y = (b.+d)./2.0 .+ (v.*dx./u)
    display(scatter(x,y,legend= false,markersize=5,markercolor="blue"))
    #readline()
