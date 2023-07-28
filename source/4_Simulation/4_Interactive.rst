
Simulation Code Examples
--------------------------------------------

This section is here to provide some examples of simulations and interactive code.   

The first cell imports the libraries.  First time use you will need to add the package.  The commented code gives an example for adding the Interact package.

.. code-block:: julia
   :dedent: 1

    #using Pkg
    #Pkg.add("Interact")

    using Interact
    using Plots
    using LinearAlgebra


If you have a series of joint values, here is how you might compute the end-effector location and list the values.

.. code-block:: julia
   :dedent: 1

    a1, a2 = 10, 10
    θ1 = range(π/4, π/3, length = 10)
    θ2 = range(π/6, π/4, length = 10)
    for i in 1:10
        x = a2*cos(θ1[i]+θ2[i]) + a1*cos(θ1[i])
        y = a2*sin(θ1[i]+θ2[i]) + a1*sin(θ1[i])
        println("x = ", x, ", y = ", y)
    end
    
    
Normally we want to provide :math:`(x,y)` location data and the compute the joint values.   So the inverse kinematics is first.  In this next example we will use the implicit loop notation (the dot).  

.. code-block:: julia
   :dedent: 1
   
    N = 50
    t = range(1, 5, length = N)
    x =  t .+ 1.0
    y = 2 .* t .- 1.0
    plot(x,y, title="Desired Path", aspect_ratio = :equal, legend=false)
    
    
Taking these input :math:`(x,y)` values, we can plug into the IK and obtain the values for :math:`\theta_1`, :math:`\theta_2`.

.. code-block:: julia
   :dedent: 1
   
    a1,a2 = 10.0,10.0
    d =  ((x.*x) .+ (y.*y) .- (a1.*a1) .- (a2.*a2))/(2.0 .* (a1.*a2))
    θ2 = atan.(-sqrt.(1.0 .- (d.*d)),d)
    θ1 = atan.(y,x) - atan.(a2.* sin.(θ2), a1 .+ a2.*cos.(θ2))
    plot(θ1,θ2, title="Plot of θ1, θ2", aspect_ratio = :equal, legend = false)
    
    
The obvious question, is this correct?  An easy way to figure this out is to plug those values into the forward kinematics and plot the results.

.. code-block:: julia
   :dedent: 1
   
    x1 = a2.*cos.(θ1 .+ θ2) .+ a1.*cos.(θ1)
    y1 = a2.*sin.(θ1 .+ θ2) .+ a1.*sin.(θ1)
    plot(x1,y1, title = "Checking Path", aspect_ratio = :equal, legend=false)
    # if you want to overlap the plots
    # plot!(x,y)
    
    
It can be helpful to visualize the dynamics of the manipulator.  The following example is Julia/Plots animation of the two link manipulator endpoints.   An animation needs a delay (the sleep function) and you need this clear output method to replot over the previous plot.

.. code-block:: julia
   :dedent: 1

    for i = 1:N
        IJulia.clear_output(true)
        p = scatter([x[i]],[y[i]], xlim = (0,10), ylim = (0,10), aspect_ratio = :equal, legend = false, color = :green)
        display(p)
        sleep(0.05)
    end
    
If you want to leave the path (the trace), you can try the following variant.

.. code-block:: julia
   :dedent: 1
   
    scatter([x[1]],[y[1]], xlim = (0,10), ylim = (0,10), aspect_ratio = :equal, legend = false, color = :green)
    for i = 2:N
        IJulia.clear_output(true)
        xl = x[1:i]
        yl = y[1:i]
        p = scatter(xl,yl, xlim = (0,10), ylim = (0,10), aspect_ratio = :equal, legend = false, color = :green)
        display(p)
        sleep(0.05)
    end

Just to play with the graphics, we change the trace.  plot! and scatter! are different functions than plot and scatter.  The "!" means this version will add to the previous plot.   Otherwise a new plot is created.  


.. code-block:: julia
   :dedent: 1

    scatter([x[1]],[y[1]], xlim = (0,10), ylim = (0,10), aspect_ratio = :equal, legend = false, color = :green)
    for i = 2:N
        IJulia.clear_output(true)
        xl = x[1:i]
        yl = y[1:i]
        p = scatter([x[i]],[y[i]], xlim = (0,10), ylim = (0,10), aspect_ratio = :equal, legend = false, color = :green)
        plot!(xl,yl, color=:red)
        display(p)
        sleep(0.05)
    end
    
An actual animation should in include the link arms.



.. code-block:: julia
   :dedent: 1
   
    xmid = a1 .* cos.(θ1)
    ymid = a1 .* sin.(θ1)
    scatter([x[1]],[y[1]], xlim = (-10,10), ylim = (0,10), aspect_ratio = :equal, legend = false, color = :blue)
    for i = 2:N
        IJulia.clear_output(true)
        p = scatter([x[i]],[y[i]], xlim = (-10,10), ylim = (0,10), aspect_ratio = :equal, legend = false, color = :blue)
        xl = [0, xmid[i], x[i]]
        yl = [0, ymid[i], y[i]]
        plot!(xl,yl, color=:blue,  linewidth=8)
        scatter!(xl, yl, color=:red, markershape=:circle)
        display(p)
        sleep(0.05)
    end
    
    
The Interact package connects up some Javascript widgets in the Notebook with Julia. It supports a variety of widgets and manages the callbacks for you. This is not a tutorial on the Interact package. There are some macros available that make the interact package easy to use. This example sets up two slider bars which are used to set the 𝜃1 , 𝜃2 values. 

The \@manipulate macro sets up the event loop and connects the slider values to values that can be used in the event loop.

.. code-block:: julia
   :dedent: 1

    function arm(θ1,θ2)
        x1 = cos(θ1)
        y1 = sin(θ1)
        x2 = x1 + cos(θ1+θ2)
        y2 = y1 + sin(θ1+θ2)
        return x1,x2,y1,y2
    end

    
    
.. code-block:: julia
   :dedent: 1

    s1 = slider(-π:0.05:π ,value = 0.0, label="Theta1")
    s2 = slider(-π:0.05:π, value = 0.0, label="Theta2")

    mp = @manipulate for θ1 in s1, θ2 in s2
        x1,x2,y1,y2 = arm(θ1,θ2)
        xl = [0,x1,x2]
        yl = [0,y1,y2]
        plot(xl,yl, legend=false,xlim=(-2,2),ylim=(-2,2), aspect_ratio = :equal, linewidth=8)
        scatter!(xl, yl, color=:red, markershape=:circle)
    end
    
    
To demonstrate how this can be used in 3D, here is the manipulator from the last homework (#23).  

.. code-block:: julia
   :dedent: 1

    function arm3(d, a1, a2, θ1)
        x1 = 0
        y1 = 0
        z1 = d
        x2 = a1*cos(θ1)
        y2 = a1*sin(θ1)
        z2 = z1
        x3 = x2
        y3 = y2
        z3 = z1 - a2
        j = [x1,y1,z1,x2,y2,z2,x3,y3,z3]
        return j
    end
    
    
This gives an example of plots in 3D.


.. code-block:: julia
   :dedent: 1
   
    s1 = slider(0.0:0.01:π/2 ,value = 0.0, label="Theta1")
    s2 = slider(1.0:0.01:5, value = 2.0, label="a1")
    s3 = slider(1.0:0.01:5, value = 3.0, label="a2")
    d = 5

    mp = @manipulate for θ1 in s1, a1 in s2, a2 in s3
        j = arm3(d,a1,a2,θ1)
        p1 = [0, j[1], j[4], j[7]]
        p2 = [0,  j[2], j[5], j[8]]
        p3 = [0, j[3], j[6], j[9]]
        plot(p1,p2,p3, xlim=(0,6),ylim=(0,6),zlim=(0,6),linewidth=10,legend=false)
    end
    
    
A simple "Etch-a-Sketch" type demo:


.. code-block:: julia
   :dedent: 1
   
    s1 = slider(-1:0.1:1, value = 0.0, label="x")
    s2 = slider(-1:0.1:1, value = 0.0, label="y")
    plot(legend=false,xlim=(-1.5,1.5),ylim=(-1.5,1.5))

    mp = @manipulate for x in s1, y in s2
        l1 = [x]
        l2 = [y]
        plot!(l1,l2, markershape=:circle, markercolor=:blue)
    end
    
An interactive plotting tool:

.. code-block:: julia
   :dedent: 1
   
    x = range(0, 10, length=100)
    y = sin.(x) .+ 1.5

    s1 = slider(1:100, value = 1, label="time")

    scatter(legend=false,xlim=(0,10),ylim=(0,3))

    mp = @manipulate for t in s1
        i = trunc(Int,t)
        l1 = x[1:i]
        l2 = y[1:i]
        scatter!(l1,l2, markershape=:circle, markercolor=:blue)
    end
    
    
or

.. code-block:: julia
   :dedent: 1
   
    x = y = 0:0.1:30

    freqs = OrderedDict(zip(["pi/4", "π/2", "3π/4", "π"], [π/4, π/2, 3π/4, π]))

    mp = @manipulate for freq1 in freqs, freq2 in slider(0.01:0.1:4π; label="freq2")
        y = @. sin(freq1*x) * sin(freq2*x)
        plot(x, y)
    end

    
An example showing how to clear a plot.
 
 
 
.. code-block:: julia
   :dedent: 1
   
    x = range(0, 10, length=100)
    y = sin.(x) .+ 1.5

    s1 = slider(1:100, value = 1, label="Time")
    s2 = OrderedDict(zip(["Plot", "Clear"], [1, 0]))

    scatter(legend=false,xlim=(0,10),ylim=(0,3))

    mp = @manipulate for t in s1, Select in s2
        i = trunc(Int,t)
        if Select == 0
            scatter(legend=false,xlim=(0,10),ylim=(0,3))
        else
            l1 = x[1:i]
            l2 = y[1:i]
            scatter!(l1,l2, markershape=:circle, markercolor=:blue)
        end
    end
    
A differential drive example ...


.. code-block:: julia
   :dedent: 1

    function DDstep(θ, r, L, ϕ1dot, ϕ2dot, dt)
        δx = (r*dt/2)*(ϕ1dot+ϕ2dot)*cos(θ)
        δy = (r*dt/2)*(ϕ1dot+ϕ2dot)*sin(θ)
        δθ = (r*dt/(2*L))*(ϕ1dot-ϕ2dot)
        return δx, δy, δθ
    end
    
    
Variable setup for the simulation.


.. code-block:: julia
   :dedent: 1
   
    r = 1
    L = 2
    N = 100
    t = range(0, 5, length = N)
    ω1 = 1.25 .+ cos.(t)
    ω2 = 1.0 .+ sin.(t)
    dt = 0.1
    x, y = 0, 0
    θ = 0

    lx = zeros(N)
    ly = zeros(N)
    lθ = zeros(N)

    for i = 1:(N-1)
        δx, δy, δθ = DDstep(lθ[i], r, L, ω1[i], ω2[i], dt)
        lx[i+1] = lx[i] + δx
        ly[i+1] = ly[i] + δy
        lθ[i+1] = lθ[i] + δθ
    end
    scatter(lx,ly, xlim = (0,12), ylim = (-1,2.5), legend = false, color = :blue)
    
    
The animation of the simulation loop


.. code-block:: julia
   :dedent: 1

    for i = 1:N
        global x, y, θ
        δx, δy, δθ = DDstep(θ, r, L, ω1[i], ω2[i], dt)
        x = x + δx
        y = y + δy
        θ = θ + δθ
        p = scatter([x],[y], xlim = (0,12), ylim = (-1,3), legend = false, color = :blue)
        display(p)
        sleep(0.2)
        IJulia.clear_output(true)
    end
