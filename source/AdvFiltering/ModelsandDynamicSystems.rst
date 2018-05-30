Models and Dynamical Systems
----------------------------

In this section we learn how to model the dynamics of the system.
For our purposes, it is sufficient to use a simplified model of the
dynamics. We will assume that the system dynamics are linear and the
variations in the dynamics (the noise) is normal or Gaussian. We will
immediately see that this assumptions leaves out the most basic of our
robot models, the differential drive. Our resolution will be to develop
the theory with the linear Gaussian systems and then develop ways to
adapt it to real systems.

Creating Filters from data and models
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Say that you have a lidar at the side a road or railway which you intend
to track velocity of the passing vehicle. [#f1]_ The lidar will estimate
the distance at the various sweep angles. This can be converted from the
polar coordinates of the lidar to the rectangular coordinates natural
for tracking the vehicle. Because of measurement error in
:math:`(r, \theta)`, you have a distribution of possible values in
:math:`(x,y)` of the actual vehicle state (location). If you did not
have any additional information, you would have to report the
measurement and the standard deviations (assuming you believe the error
is normally distributed), and be done.

However, in this case you do have some additional information. Focusing
on the train, you know the vehicle is restricted to the rail. This
information can be used to reduce or filter out some of the error. The
idea then would be to project the estimate of location onto the track
since you know (well actually assume) the restriction of the vehicle.
The rail here is our proxy for kinematic constraints. The kinematic
constraints describes the restrictions on motion just like the rail
restricts the train motion. It makes sense then to use the kinematics to
help filter out the noise in our measurements.

Measurement :math:`\to` Model :math:`\to` Estimate/Predict
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The work we did earlier in fitting curves is one form of this idea. We
have a model in mind and we then “project” the data onto the model.

.. figure:: KalmanFigures/quadpts.*
   :width: 60%
   :align: center

 :math:`\Rightarrow`

.. figure:: KalmanFigures/quadgraph.*
   :width: 60%
   :align: center



Using quadratic model, :math:`y = c_2x^2 + c_1x + c_0`, and least
squares we find :math:`y = 0.49x^2 - 1.21x + 1.42`. So we can use this
to extrapolate at :math:`x=5` we have :math:`y = 6.5`.

.. figure:: KalmanFigures/quadpts.*
   :width: 60%
   :align: center

 :math:`\Rightarrow`

.. figure:: KalmanFigures/lscompare.*
   :width: 60%
   :align: center


Note that this is very different from using a sinusoidal model and
gaining the fit :math:`y= -0.95\sin(1.2x+0.1)+1.525`, where for
:math:`x=5` we have :math:`y=1.698`

A Physics Example
^^^^^^^^^^^^^^^^^

In this example, we go one step further. Using observational data of
motion and knowing the motion is restricted in some manner, can we then
use the data + model to predict a missing piece of information. Formally
this is known as interpolation or extrapolation depending on the
dataset. We use the data plus the model to answer the question. Physical
systems have equations describing the behavior of the objects in the
system. For example one may know about the forces acting on an object of
mass :math:`m`, :math:`F_x` and :math:`F_y` which will give the
equations of motion

.. math:: \ddot{x} = \frac{F_x}{m} \quad \quad \ddot{y} = \frac{F_y}{m} .

For this example, assume that these forces are constant and so we may
easily integrate them

.. math:: x(t) =  \frac{F_x}{2m} t^2 + v_{x,0} t + x_0 \quad \mbox{and} \quad y(t) =  \frac{F_y}{2m} t^2 + v_{y,0} t + y_0 .

These equations restrict the possible values of :math:`x` and :math:`y`
while still being general enough to allow for a variety of starting
conditions. Assume for the moment that we do not have knowledge of the
initial data for a particular application (but understand the forces
involved) and would like to use some observed data to determine those
values. The observations of the moving object are probably very noisy.
Thus you would obtain :math:`(t_i, x_i,y_i)` data (:math:`i=1 \dots k`).
Each data item should satisfy the equations

.. math:: x_i =  \frac{F_x}{2m} t_i^2 + v_{x,0} t_i + x_0 \quad \mbox{and} \quad y_i =  \frac{F_y}{2m} t_i^2 + v_{y,0} t_i + y_i, \quad i=1 \dots k .

There are two unknowns for each equation. Two data points would allow an
exact answer (two equations and two unknowns). But what if you had a
dozen observations? With noisy data, getting many observations should
provide a better estimate of the initial values than just picking two
observed values. So, how do we do this?

We assume that we have obtained some data on the motion of an object and
wish to compute its equations of motion. Plugging the data in gives the
equations:

.. math:: x_i =  \frac{F_x}{2m} t_i^2 + v_{x,0} t_i + x_0 \quad \mbox{and} \quad y_i =  \frac{F_y}{2m} t_i^2 + v_{y,0} t_i + y_i, \quad i=1 \dots k .

Rewrite the expression as

.. math:: \xi_i = x_i -  \frac{F_x}{2m} t_i^2 = v_{x,0} t_i + x_0 \quad \mbox{and} \quad \eta_i = y_i -  \frac{F_y}{2m} t_i^2 = v_{y,0} t_i + y_i, \quad i=1 \dots k .

Then

.. math::

   \begin{pmatrix} \xi_1 \\ \xi_2 \\ \dots \\ \xi_k \end{pmatrix} = \begin{pmatrix} t_1 & 1 \\ t_2 & 1 \\ \vdots & \vdots \\ t_k & 1 \end{pmatrix} \begin{pmatrix}  v_{x,0} \\ x_0 \end{pmatrix}
   \quad \mbox{and} \quad
   \begin{pmatrix} \eta_1 \\ \eta_2 \\ \dots \\ \eta_k \end{pmatrix} = \begin{pmatrix} t_1 & 1 \\ t_2 & 1 \\ \vdots & \vdots \\ t_k & 1 \end{pmatrix} \begin{pmatrix}  v_{y,0} \\ y_0 \end{pmatrix}

The pseudo-inverse is :math:`M = (X^T X)^{-1} X^T`

.. math::

   =  \left[\begin{pmatrix} t_1 & t_2 & \dots & t_k  \\ 1 & 1 & \dots & 1\end{pmatrix} \begin{pmatrix} t_1 & 1 \\ t_2 & 1 \\ \vdots & \vdots \\ t_k & 1 \end{pmatrix} \right]^{-1}
   \begin{pmatrix} t_1 & t_2 & \dots & t_k  \\ 1 & 1 & \dots & 1\end{pmatrix}

which gives the least squares estimate

.. math::

   \begin{pmatrix}  v_{x,0} \\ x_0 \end{pmatrix} =  M  \begin{pmatrix} \xi_1 \\ \xi_2 \\ \dots \\ \xi_k \end{pmatrix}
   \quad \mbox{and}
   \quad
   \begin{pmatrix}  v_{y,0} \\ y_0 \end{pmatrix} =
   M \begin{pmatrix} \eta_1 \\ \eta_2 \\ \dots \\ \eta_k \end{pmatrix}

Physics example with numbers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Assume that we had the following data: (1, 10, 22), (2, 19, 60), (3, 32,
51) and that :math:`F_x=0` and :math:`F_y = -2`, :math:`m=0.25`. So
first we gain: :math:`t = [1, 2, 3]`, :math:`\xi = [10, 19, 32]`,
:math:`\eta = [26, 46, 12]`. We first compute

.. math::

   \left[\begin{pmatrix} 1 & 2 &  3  \\ 1 & 1 & 1\end{pmatrix} \begin{pmatrix} 1 & 1 \\ 2 & 1  \\ 3 & 1 \end{pmatrix} \right]^{-1}
   = \left[\begin{pmatrix} 14 & 6 \\ 6 & 3 \end{pmatrix}\right]^{-1} =  \frac{1}{6} \begin{pmatrix} 3 & -6 \\ -6 & 14 \end{pmatrix}



.. math:: = \begin{pmatrix} 1/2 & -1 \\ -1 & 7/3 \end{pmatrix}

.. math::

   \begin{pmatrix}  v_{x,0} \\ x_0 \end{pmatrix} = \begin{pmatrix} 1/2 & -1 \\ -1 & 7/3 \end{pmatrix}
   \begin{pmatrix} 1 & 2 &  3  \\ 1 & 1 & 1\end{pmatrix}  \begin{pmatrix} 10 \\ 19 \\ 32 \end{pmatrix}  = \begin{pmatrix} 11 \\ -1.666667\end{pmatrix}

and

.. math::

   \begin{pmatrix}  v_{y,0} \\ y_0 \end{pmatrix} = \begin{pmatrix} 1/2 & -1 \\ -1 & 7/3 \end{pmatrix}
   \begin{pmatrix} 1 & 2 &  3  \\ 1 & 1 & 1\end{pmatrix} \begin{pmatrix} 28 \\ 46 \\ 12\end{pmatrix} = \begin{pmatrix} 30.5 \\ 2.0\end{pmatrix}

So we have that the start location is :math:`(-1.666667, 2.0)` with
initial velocity of :math:`(11 , 30.5)`.

Linear Dynamical System
~~~~~~~~~~~~~~~~~~~~~~~

An operator, :math:`L`, is said to be linear if for scalars :math:`a,b`
and vectors :math:`x,y` we have

.. math:: L(ax+by) = aLx + bLy

A dynamical system

.. math:: x_k = Lx_{k-1} \quad \text{(discrete)}

or

.. math:: \dot{x} = Lx \quad \text{(continuous)}

is said to be linear if :math:`L` is a linear operator. Linearity means
we may construct solutions using simple addition.

Example of linear operators
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Some examples of linear operators are matrices,
:math:`A(\alpha x + \beta y) = \alpha Ax + \beta Ay`, and derivatives,
:math:`(d/dx) [\alpha u+\beta v] = \alpha du/dx + \beta dv/dx`.

Example: Nonlinear Kinematic Models
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The dynamics of a differential drive robot is **NOT** linear:

.. math::

   \begin{array}{l}
    \dot{x} = \frac{r}{2} (\dot{\phi_1}+\dot{\phi_2})\cos(\theta) \\[5mm]
   \dot{y} = \frac{r}{2} (\dot{\phi_1}+\dot{\phi_2})\sin(\theta) \\[5mm]
   \dot{\theta} = \frac{r}{2L} (\dot{\phi_1}-\dot{\phi_2}).
   \end{array}

This follows from noting that :math:`\cos(\theta)` and
:math:`\sin(\theta)` are nonlinear functions of the state variable
:math:`\theta`.

Let :math:`x_k` be the current state and :math:`z_k` be the observation.
We study the linear system with noise:

.. math::

   \begin{array}{l}
   x_k = Fx_{k-1} + Gu_k + v_k\\
   z_k = Hx_k + w_k
   \end{array}

where :math:`v_k`, :math:`w_k` are assumed to be zero mean Gaussian
noise with covariance matrices :math:`V_k` and :math:`W_k` respectively.

Dynamics with Noise
~~~~~~~~~~~~~~~~~~~

There are many possibilities for a linear dynamical system. We are
interested in tracking not just the estimate of the state, but the
state’s distribution as well since the addition of noise produces random
variations in values. The simplest distribution to track is a Normal or
Gaussian distribution. Using the Bayes Filter terminology, we have three
elements:

#. The state transition probability :math:`p(x_k|u_k,x_{k-1})` must
   arise from

   .. math:: x_k = Fx_{k-1}+Gu_k + v_k

   \ where :math:`x_k`, :math:`x_{k-1}` are state vectors, :math:`u_k`
   controls, :math:`v_k` is the noise, :math:`F` and :math:`G` are
   matrices. :math:`v_k` is a mean zero normally distributed random
   variable with covariance matrix :math:`V_k`. This is linear system
   dynamics. Thus the mean of the posterior state is

   .. math::

      E(x_k) =
      Fx_{k-1}+Gu_k,

    :math:`p(x_k|u_k, x_{k-1})`

   .. math::

      = \frac{1}{\sqrt{\det
          (2\pi V_k)}}e^{-\frac{1}{2}(x_k-Fx_{k-1}-Gu_k)^T V^{-1}(x_k-Fx_{k-1}-Gu_k)}.

#. The measurement probability :math:`p(z_k|x_k)` must also be linear

   .. math:: z_k = Hx_k + w_k

   where :math:`H` is a :math:`m \times n` matrix and :math:`w_k` is
   Gaussian mean zero random variable (noise) with covariance matrix
   :math:`W_k`. The mean of the observation

   .. math:: E(z_k) = Hx_k,

   .. math::

      p(z_k|x_k) = \frac{1}{\sqrt{\det
          (2\pi W_k)}}e^{-\frac{1}{2}(z_k-Hx_k)^T W^{-1}(z_k-Hx_k)}

#. Initial belief, :math:`\mbox{bel}(x_0)` must be normally distributed,
   say with mean :math:`\hat{x}_0` and covariance :math:`P_0`

   .. math::

      \mbox{bel}(x_0) = \frac{1}{\sqrt{\det
          (2\pi P_0)}}e^{-\frac{1}{2}(x_0-\hat{x}_0)^T P_0^{-1}(x_0-\hat{x}_0)}

   If assumptions 1,2,3 hold then :math:`\mbox{bel}(x_k)` is also a
   Gaussian distribution.

Terminology
^^^^^^^^^^^

We will introduce some fairly common notation used in state estimation.
As stated before, we cannot observe the actual value of the quantity
:math:`x`, and so we will indicate with a “hat” the estimate of the
value, :math:`\hat{x}`.

-  Let :math:`\hat{x}_{k-1|k-1}` be the current state estimate at time
   step :math:`k-1`.

-  Let :math:`\hat{x}_{k|k-1}` be the prediction of the next state using
   a model of the dynamics.

-  Let :math:`P_{k|k-1}` be the covariance of :math:`\hat{x}_{k|k-1}`
   (:math:`E[(x_k-\hat{x}_{k|k-1})(x_k-\hat{x}_{k|k-1})^T]`)

-  Let :math:`z_{k}` be the observation or measurement of :math:`x_{k}`.

-  Let :math:`\hat{x}_{k|k}` be the update based on the observation.
   :math:`\hat{x}_{k|k}` is our best estimate of :math:`x_{k}`

-  Let :math:`P_{k|k}` be the covariance of :math:`\hat{x}_{k|k}`
   (:math:`E[(x_k-\hat{x}_{k|k})(x_k-\hat{x}_{k|k})^T]`)


.. rubric:: Footnotes

.. [#f1] Ignoring for the moment that this is a ridiculously expensive way to track velocity.
