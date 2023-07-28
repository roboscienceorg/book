Differential Kinematics
-------------------------------------------------


The last chapter focused on the forward and inverse position kinematics.   There were several problems we had not completely resolved.  We want to address forward and inverse velocity kinematics.  This means find formulas for the end-effector velocities and joint velocities.  We also want to develop some methods to numerically compute the inverse kinematics.  All of these issues can be addressed once we know how to compute the derivative of the forward kinematics.


.. _jacobians:

The Jacobian
~~~~~~~~~~~~~~~~~~~~


To get started, we will assume that we have the forward kinematic equations for position in analytic form.  Later we can remove this restriction.  For the moment, set aside the DH formalism from the last section and assume that you have extracted the analytic formulas (or derived them by some other means). Thus we have the position of the end-effector as a function of parameters (set of joint values):

.. math::

   p = f(q)

where :math:`p` is the position vector and :math:`q` is the joint configuration.  The velocity of the end-effector is given by :math:`dp/dt`, the time derivative of position.   Since :math:`f(q)` is a vector valued function, the derivative of :math:`f` is the time derivative of each component of :math:`f`:

.. math::

   \frac{dp}{dt} =  \begin{pmatrix} \displaystyle \frac{dp_x}{dt} \\[3mm] \displaystyle \frac{dp_y}{dt}  \\[3mm] \displaystyle \frac{dp_z}{dt}  \end{pmatrix}
   = \begin{pmatrix} \displaystyle \frac{df_1(q)}{dt} \\[3mm] \displaystyle \frac{df_2(q)}{dt}  \\[3mm] \displaystyle \frac{df_3(q)}{dt}  \end{pmatrix}
   = \begin{pmatrix} \displaystyle \nabla f_1 \cdot\frac{dq}{dt} \\[3mm] \displaystyle \nabla f_2\cdot\frac{dq}{dt}  \\[3mm] \displaystyle \nabla f_3 \cdot\frac{dq}{dt}  \end{pmatrix}
   = \begin{pmatrix} \displaystyle \nabla f_1 \\[3mm] \displaystyle \nabla f_2  \\[3mm] \displaystyle \nabla f_3  \end{pmatrix} \frac{dq}{dt}
     \equiv J_f \frac{dq}{dt}

The term :math:`J_f` is called the Jacobian and written out, it appears as

.. math::

   J_f = \begin{pmatrix}  \frac{\partial f_1} {\partial q_1} & \frac{\partial f_1} {\partial q_2} & \dots & \frac{\partial f_1} {\partial q_m} \\[3mm]
    \frac{\partial f_2} {\partial q_1} & \frac{\partial f_2} {\partial q_2} & \dots & \frac{\partial f_2} {\partial q_m} \\[3mm]
    \frac{\partial f_3} {\partial q_1} & \frac{\partial f_3} {\partial q_2} & \dots & \frac{\partial f_3} {\partial q_m} \end{pmatrix}

Note that the version above is specific to :math:`f \in {\Bbb R}^3` but in general works for any number of dependent and independent variables.   There will be examples later in the text where we have more than three dependent variables but for our purposes here, two or three will be sufficient.

Two and Three link example
^^^^^^^^^^^^^^^^^^^^^^^^^^^

The two and three link manipulators are good examples.  The two link equations are

.. math::

   \begin{matrix}
   p_x = f_1 = a_2\cos (\theta_1+\theta_2) + a_1 \cos \theta_1 \\
   p_y = f_2 = a_2 \sin (\theta_1 +\theta_2) + a_1\sin \theta_1
   \end{matrix}

and the Jacobian of the position equations is

.. math::

   J_f = \begin{pmatrix}  \frac{\partial f_1} {\partial q_1} & \frac{\partial f_1} {\partial q_2}  \\[2mm]
    \frac{\partial f_2} {\partial q_1} & \frac{\partial f_2} {\partial q_2}  \end{pmatrix}
    = \begin{pmatrix}  -a_2\sin(\theta_1+\theta_2) - a_1\sin(\theta_1) & -a_2\sin(\theta_1+\theta_2)  \\[2mm]
     a_2\cos (\theta_1+\theta_2) + a_1 \cos (\theta_1) & a_2\cos (\theta_1+\theta_2)  \end{pmatrix}


where :math:`q_1 = \theta_1` and :math:`q_2 = \theta_2`.


The three link is given by

.. math::

   \begin{matrix}
   p_x = f_1 = a_3\cos(\theta_1 + \theta_2 + \theta_3)+ a_2 \cos(\theta_1 + \theta_2) + a_1 \cos(\theta_1)\\
   p_y = f_2 = a_3\sin(\theta_1 + \theta_2 + \theta_3)+ a_2 \sin(\theta_1 + \theta_2) + a_1 \sin(\theta_1)
   \end{matrix}


and the Jacobian of position is

.. math::

   J_f = \begin{pmatrix}  \frac{\partial f_1} {\partial q_1} & \frac{\partial f_1} {\partial q_2} & \frac{\partial f_1} {\partial q_3} \\[2mm]
    \frac{\partial f_2} {\partial q_1} & \frac{\partial f_2} {\partial q_2} & \frac{\partial f_2} {\partial q_3}  \end{pmatrix}


.. math::

    =\begin{pmatrix}  -a_3\sin(\theta_1 + \theta_2 + \theta_3)- a_2 \sin(\theta_1 + \theta_2) - a_1 \sin(\theta_1) & -a_3\sin(\theta_1 + \theta_2 + \theta_3)- a_2 \sin(\theta_1 + \theta_2)  & -a_3\sin(\theta_1 + \theta_2 + \theta_3) \\[2mm]
     a_3\cos(\theta_1 + \theta_2 + \theta_3)+ a_2 \cos(\theta_1 + \theta_2) + a_1 \cos(\theta_1) & a_3\cos(\theta_1 + \theta_2 + \theta_3)+ a_2 \cos(\theta_1 + \theta_2)  & a_3\cos(\theta_1 + \theta_2 + \theta_3)  \end{pmatrix}

with the obvious choice on :math:`q_k`.

Compute the Jacobian for the two link manipulator where :math:`\theta_1 = 10^\circ, \theta_2 = 20^\circ`.

.. code-block:: julia
   :caption: Jacobian computation
   :name: listjacobian
   :dedent: 1

    θ1 = 10*π/180
    θ2 = 20*π/180
    a1 = a2 = 10
    j11 = -a2*sin(θ1+θ2) - a1*sin(θ1)
    j12 = -a2*sin(θ1+θ2)
    j21 = a2*cos(θ1+θ2) + a1*cos(θ1)
    j22 = a2*cos(θ1+θ2)
    J = [ j11 j12 ; j21 j22]
    print(J)



.. _jacobianvelocity:

Velocity
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Linear velocity is given by :math:`v = dp/dt`.   The joint velocities are represented by :math:`\dot{q}`.  Returning to the general formula

.. math::

   \displaystyle v = \frac{dp}{dt} = J_f \frac{dq}{dt} = J_f \dot{q}

This is the forward velocity kinematics.    Given the joint velocity vector, :math:`\dot{q}`, we can compute the velocity vector, :math:`v`, by matrix vector multiplication :math:`J_f \dot{q}`.  Angular
velocities should be converted to radians per unit time.

Example:  assume that you have the two link manipulator and given angles :math:`\theta_1 = 10^\circ, \theta_2 = 20^\circ` and  angular velocities :math:`d\theta_1 / dt = 20^\circ s^{-1}, d\theta_2 / dt = 45^\circ s^{-1}`   First, convert to radians:  :math:`d\theta_1 / dt = 20\pi/180 s^{-1}, d\theta_2 / dt = 45\pi/180 s^{-1}`.  Then multiply the Jacobian and the angular velocity vector:

.. math::

   J\dot{q} = \begin{pmatrix}  -a_2\sin(\theta_1+\theta_2) - a_1\sin(\theta_1) & -a_2\sin(\theta_1+\theta_2)  \\[2mm]
     a_2\cos (\theta_1+\theta_2) + a_1 \cos (\theta_1) & a_2\cos (\theta_1+\theta_2)  \end{pmatrix}
     \begin{pmatrix} d\theta_1/dt \\ d\theta_2 /dt\end{pmatrix}
     =
      \begin{pmatrix} -6.73648 & -5.0 \\  18.5083  &  8.66025 \end{pmatrix}
       \begin{pmatrix} 20\pi/180 \\ 45\pi/180 \end{pmatrix}
       =  \begin{pmatrix}  -6.866335492 \\  14.87753073867 \end{pmatrix}


.. code-block:: julia
   :caption: Velocity computation
   :name: listvelocity
   :dedent: 1

    θ1 = 10*π/180
    θ2 = 20*π/180
    a1 = a2 = 10
    j11 = -a2*sin(θ1+θ2) - a1*sin(θ1)
    j12 = -a2*sin(θ1+θ2)
    j21 = a2*cos(θ1+θ2) + a1*cos(θ1)
    j22 = a2*cos(θ1+θ2)
    J = [ j11 j12 ; j21 j22]
    qd = [25*π/180 ; 45*π/180]
    J*qd


Two and three link example continued
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Again we can use the two and three link manipulators as examples.   We first examine the invertability of the two link manipulator Jacobian.  A quick test is to find the determinant.  Recall when the determinant is non-zero, the matrix is invertable.  So we have,


.. math::

   det(J_f)     = \begin{vmatrix}  -a_2\sin(\theta_1+\theta_2) - a_1\sin(\theta_1) & -a_2\sin(\theta_1+\theta_2)  \\[2mm]
     a_2\cos (\theta_1+\theta_2) + a_1 \cos (\theta_1) & a_2\cos (\theta_1+\theta_2)  \end{vmatrix}

.. math::

     = -a_2^2 \sin(\theta_1+\theta_2)\cos (\theta_1+\theta_2)  - a_1a_2 \sin(\theta_1)\cos (\theta_1+\theta_2) + a_2^2\sin(\theta_1+\theta_2)\cos (\theta_1+\theta_2)
     + a_1a_2 \sin(\theta_1+\theta_2) \cos (\theta_1)

.. math::

     = a_1a_2 \sin(\theta_2)

which implies that this Jacobian is invertable when :math:`\theta_2 \neq n\pi`.   When :math:`\theta_2 = n\pi` then the Jacobian is rank 1.
For the three-link manipulator, the Jacobian is a 2x3 matrix and as such is not invertable.   Since many robot arms have more degrees of freedom than the dimension of the workspace, can we extract joint velocity from velocity? For our needs, yes, we will be able to compute joint velocities using some standard tools from linear algebra.  Before this, we look at the issue of end effector orientation and angular velocity.



Orientation and angular velocity
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Using the DH derivation, we can determine the end-effector orientation in addition to position.   Recall that we can extract the tool tip direction from the vector :math:`\vec{a}` directly from the DH transformation :math:`A`.   For the moment, assume that you have an analytic expression for the orientation vector (Euler angle vector) for the tool tip:  :math:`\phi`.   We can compute the time rate of change of :math:`\phi`.

.. math::

   \dot{\phi} =  J_\phi \dot{q}

However, this is not angular velocity in the traditional sense.   Angular velocity, :math:`\omega` is the rotation rates in the global or base frame where :math:`\dot{\phi}` gives information in a non-orthogonal sense related to the end-effector frame.   The former being a more intuitive notion of rotational velocity.  It is possible to relate :math:`\dot{\phi}` to :math:`\omega`, we direct the reader to :cite:`siciliano2009robotics`



Numerical Inverse Kinematics
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

In this section we will look at numerical approaches to the inverse kinematics problem.   First we will assume that the Jacobian is  invertable and second we have a path that we want the end effector to travel.  In addition we are going to restrict our focus on the position kinematics.

Recall the velocity function

.. math::

   \displaystyle v = \frac{dp}{dt} = J_f \frac{dq}{dt} = J_f \dot{q}

we can invert and gain

.. math::

   \displaystyle \dot{q}  = J_f^{-1} v


A path in the workspace is represented by :math:`<x(t),y(t),z(t)>`.   You can compute the derivative of this path :math:`v = <\dot{x}(t),\dot{y}(t),\dot{z}(t)>` .    This can be plugged into the velocity function.   Analytically, one can recover the IK via integration

.. math::

   q(t) = \int_0^t J_f^{-1} v(s)ds

The path  :math:`<x(t),y(t),z(t)>` can be known from precise engineering design or it might be induced from cubic interpolation based on a sequence of way points from a path plannig algorithm.   In practice, the antiderivative may not be available.   And discrete methods are needed.   Discrete methods can focus on discretization of the integration, or backing up a step and solving the velocity equation in a direct manner.

Assume that you are at a location in space :math:`<x_0,y_0,z_0>` and want to get to  :math:`<x_1,y_1,z_1>` .  The displacement vector, *v* can be used to find the estimate of :math:`\dot{q}`.

.. math::

   \Delta q = J_f^{-1}v, \quad \mbox{where} \quad v = <x_1-x_0, y_1-y_0, z_1-z_0>

We can update *q* via :math:`q_k = q_{k-1} + \Delta q`.
This is a type of Euler integration (similar to a trapezoid method for integration).  It has discretization error and so after moving the manipulator over the :math:`\Delta q` step, the end effector will not normmally be at :math:`<x_1,y_1,z_1>`.  Using smaller steps on *v* can help with accuracy.  Using the current point and the old target point, an updated *v* may be used to correct the step.

A benefit here is that this approach is the basis for a purely numerical method for inverting the kinematics.  The inversion algorithm is

.. code::

    Input desired target:  p*
    Set q values to intial guess (random)
    Set stepsize: step
    
    p = FK(q)
    d = distance(p, p*)
    
    while d > error
        find v = (p* - p)/||(p* - p)||
        set u = step*v
        find w = inv(J) * u
        set q = q + w
        p = FK(q)
        d = distance(p,p*)
    print("q = ", q)

Note that much more sophisticated algorithms exist.  Variations of Newton's Method can be employed here which we will leave to textbooks on Numerical Methods.  

Example
^^^^^^^^^^^

In this example we show how one can find the inverse kinematics numerically for the two link example.  Since we have the analytic formula, we can check our answer.   Find :math:`\theta_1`, :math:`\theta_2` when x=10, y=12 for the two link manipulator when :math:`a_1=a_2=10`


.. code-block:: julia
   :caption: Example of a numerical inverse
   :name: lstnumericaltwolinkik
   :dedent: 1

    function FK(θ1, θ2, a1, a2)
        x = a2*cos(θ1+θ2) + a1*cos(θ1)
        y = a2*sin(θ1+θ2) + a1*sin(θ1)
        return x,y
    end


    function FKJ(θ1, θ2, a1, a2)
        j11 = -a2*sin(θ1+θ2) - a1*sin(θ1)
        j12 = -a2*sin(θ1+θ2)
        j21 = a2*cos(θ1+θ2) + a1*cos(θ1)
        j22 = a2*cos(θ1+θ2)
        J = [ j11 j12 ; j21 j22]
        return J
    end

    function distance(x1, y1, x2, y2)
        d = sqrt((x1-x2)^2 + (y1-y2)^2)
        return d
    end

    function size(v1,v2)
        size = sqrt(v1*v1+v2*v2)
        return size
    end

    x = 10
    y = 12
    θ1 = .1
    θ2 = .2
    a1 = a2 = 10
    step = 0.1
    xc, yc = FK(θ1, θ2, a1, a2)
    d = distance(x,y,xc,yc)

    while d > .01
        vx = x - xc
        vy = y - yc
        s = size(vx,vy)
        ux = step*vx/s
        uy = step*vy/s
        J = FKJ(θ1, θ2, a1, a2)
        u = [ux, uy]
        w = J\u
        θ1 = θ1 + w[1]
        θ2 = θ2 + w[2]
        xc, yc = FK(θ1, θ2, a1, a2)
        d = distance(x,y,xc,yc)
    end
    println("θ1 = ",θ1, "θ2 = ", θ2, "  x = ", xc, "  y = ", yc)

We can use the builtin nonlinear solver in Julia as well:




.. code-block:: julia
   :caption: Example of a numerical inverse using NLsolve
   :name: lstnumericaltwolinkiknlsolve
   :dedent: 1
   
    #import Pkg
    #Pkg.add("NLsolve")

    using NLsolve

    function f!(F, x)
        F[1] = 10*cos(x[1]+x[2]) + 10*cos(x[1]) - 10
        F[2] = 10*sin(x[1]+x[2]) + 10*sin(x[1]) - 12
    end


    function j!(J, x)
        J[1, 1] = -10*sin(x[1]+x[2]) - 10*sin(x[1])
        J[1, 2] = -10*sin(x[1]+x[2])
        J[2, 1] = 10*cos(x[1]+x[2]) + 10*cos(x[1])
        J[2, 2] = 10*cos(x[1]+x[2])
    end

    nlsolve(f!, j!, [ 0.1; .2])

The 

Both of these programs will exit with errors if you set the intitial value to [0,0].  The Jacobian is singular and so the embedded linear solve will fail.   This is addressed below.  First we discuss the approach to numerically computing the Jacobian. 


.. _numerical-jacobian-1:



Jacobian numerical approaches
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Using the DH Convention approach, we have the manipulator can be represented by

.. math::

   A = \prod_{i=0}^m A_i

Assume for the moment that each :math:`A_i` has only on parameter that varies.   One can generalize from this case and it is sufficient here.  This gives us a nice way to compute the Jacobian.  We start by computing a single partial derivative.

.. math::

   \displaystyle \frac{\partial A}{\partial q_k} = \displaystyle \frac{\partial}{\partial q_k} \prod_{i=0}^m A_i (q_i)=
   \displaystyle A_1(q_1) A_2(q_2) \dots A_{k-1}(q_{k-1}) \frac{\partial A_k}{\partial q_k} A_{k+1}(q_{k+1})\dots A_m (q_m)

Define :math:`\frac{\partial A_k}{\partial q_k} = A_k'` and we have

.. math::

   B_k = A_1A_2 \dots A_k' \dots A_m

If we have the row index as *i* and the column index as *j*, we now add the slice index *k*.   All of the :math:`B_k` can be stacked into a tensor :math:`B`.


.. figure:: SimulationFigures/tensor.*
   :align: center
   :width: 75%

The Jacobian of the position function can be found in this tensor as a partial slice where *j=4* and for *i=1:3* and all *k*:


.. figure:: SimulationFigures/tensor2.*
   :align: center
   :width: 25%




Invertability of the Jacobian
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

We saw that for the two link manipulator, the Jacobian of the forward position kinematics was not invertable for a discrete set of parameter values.  For the three link manipulator we saw that the Jacobian FPK was not square and so not invertable.  For the general case, we have an non-square Jacobian, :math:`n \times m`:

.. math::

   v = J_f \dot{q}

We have from earlier that we can find a least squares solution (via right inverse or the SVD):

.. math::

   \dot{q} = J_f^+ v

This does work like the case above when the Jacobian is invertable.  The pseudoinverse can move the robot arm in the desired direction.  It is helpful to understand the geometry.
Normally this is an underdetermined system.  Generically we are mapping :math:`{\Bbb R}^m \to {\Bbb R}^3` with :math:`m > 3`.  The extra dimensions need to be compressed down to fit into three dimensions.   This happens by mapping those dimensions to zero.  Let :math:`u` be one of these directions.

.. math::

   J_f u = 0

u is an element of the Nullspace of :math:`J_f`.   The domain, :math:`{\Bbb R}^m` can be partitioned into the row space and the nullspace.

.. math::

   x = w + u

where :math:`w` is in the row space and :math:`u` is in the nullspace.  Another terminology is that *w* is the particular solution and *u* is the homgeneous solution.    It is not hard to verify that :math:`u \perp w`.  Moving joints in the null direction does not move the end-effector but does reconfigure the robot arm.  Moving joints in the rowspace direction does move the end-effector.

There is one and only one solution to :math:`v = J_f \dot{q}` when :math:`\dot{q}` is restricted to the row space.  A general solution to the problem is

.. math::

   x_g = w + cu

where *c* is any real number.  We can address this via the SVD.   Given the matrix :math:`J_f` we can factor in the following manner:

.. math::

   J_f  = U \Sigma^+ V^*

where :math:`U` is a :math:`n \times n` unitary matrix, :math:`\Sigma` is a :math:`n \times m` diagonal matrix, and :math:`V` is a :math:`m \times m` unitary matrix.  Recall that unitary means :math:`UU^* = I` where :math:`U^* = \bar{U}^T`, the complex conjugate transpose.

The pseudoinverse will provide the row and nullspace *w* :

.. math::

   w = J_f^+ v = V \Sigma^+ U^T v

If we used the SVD to compute :math:`J_f^+` then we have the nullspace as well.  A basis for the nullspace are the rows of :math:`V^*` that correspond to **zero** diagonal elements in :math:`\Sigma`.



Example
^^^^^^^^^^^

We will illustrate these ideas with the three link manipulator.   

