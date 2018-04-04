Kalman FIlter[Chap:Kalman]
==========================

Models and Dynamical Systems
----------------------------

Observation is not the whole story. The robot will be out observing the
world, but the observations will not be random. For example, if we are
observing motion, we know that not all types of motion will be possible.
The basic laws of physics clearly play a role. In addition the design
and details of the robot also play a role. These constrain the
possibilities and in doing so, help improve our estimates of the robot
state. In this section we learn how to model the dynamics of the system.
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
to track velocity of the passing vehicle. [1]_ The lidar will estimate
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

|image|

 :math:`\Rightarrow`

|image|

| 
| Using quadratic model, :math:`y = c_2x^2 + c_1x + c_0`, and least
  squares we find :math:`y = 0.49x^2 - 1.21x + 1.42`. So we can use this
  to extrapolate at :math:`x=5` we have :math:`y = 6.5`.

|image|

 :math:`\Rightarrow`

|image|

| 
| Note that this is very different from using a sinusoidal model and
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

\ 

.. math:: = \begin{pmatrix} 1/2 & -1 \\ -1 & 7/3 \end{pmatrix}

.. math::

   \begin{pmatrix}  v_{x,0} \\ x_0 \end{pmatrix} = \begin{pmatrix} 1/2 & -1 \\ -1 & 7/3 \end{pmatrix}
   \begin{pmatrix} 1 & 2 &  3  \\ 1 & 1 & 1\end{pmatrix}  \begin{pmatrix} 10 \\ 19 \\ 32 \end{pmatrix}  = \begin{pmatrix} 11 \\ -1.666667\end{pmatrix}

\ and

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

Kalman Filters
--------------

Scalar Kalman Filter
~~~~~~~~~~~~~~~~~~~~

For the moment assume that :math:`x, F, G, u` are scalars. Also assume
we have a starting value for the state :math:`x_0` and some estimate of
the error in that starting value, :math:`\sigma_0^2`. The error in the
process is measured and has variance :math:`\sigma_v^2`, meaning
:math:`v_k` is drawn from a zero mean Gaussian distribution with
variance :math:`\sigma_v^2` which gives us the process:

.. math:: x_k = Fx_{k-1} + Gu_k  + v_k .

The estimate of state based on the process is simply

.. math:: \tilde{x}_k = F\hat{x}_{k-1} + Gu_k .

Prior to the process, the variance estimate for :math:`x_{k-1}` is
:math:`\sigma_{k-1}^2`. What happens? It is transformed via

.. math:: \tilde{\sigma}_{k}^2 = (F \sigma_{k-1})^2 + \sigma_v^2 = F^2\sigma_{k-1}^2 + \sigma_v^2 .

The next thing required is to merge the process prediction with the
observation data, :math:`z_k` (scalar), this observation has quality
:math:`\sigma_w^2`. These are fused using
formula \ `[Eq:scalarrecursiveweighted] <#Eq:scalarrecursiveweighted>`__:

.. math:: S_k = \frac{1}{\tilde{\sigma}_k^2} + \frac{1}{{\sigma}_w^2}

.. math::

   K_{k} = \displaystyle \left[ S_{k}\sigma_{w}^2\right]^{-1} =  \left[ {\sigma}_{w}^2 \left(\frac{1}{\tilde{\sigma}_k^2} + \frac{1}{\sigma_w^2}\right) \right]^{-1}
   =  \left[ {\sigma}_{w}^2 \left(\frac{\tilde{\sigma}_k^2 + \sigma_w^2}{\tilde{\sigma}_k^2  \sigma_w^2}\right) \right]^{-1}

\ 

.. math:: =  \frac{\tilde{\sigma}_k^2}{\tilde{\sigma}_k^2 + \sigma_w^2}

.. math:: \hat{x}_{k} =  \tilde{x}_{k-1} +  K_{k}\left(  z_{k}- \tilde{x}_{k-1} \right)

.. math:: \displaystyle \sigma_k^{2} = (1 - K_k)\tilde{\sigma}_k^{2}

We can summarize the process

.. math::

   \begin{array}{l}
   x_k = Fx_{k-1} + Gu_k + v_k\\
   z_k = x_k + w_k
   \end{array}

in the standard notation of the Kalman Filter. Let the process noise
:math:`v_k` have variance :math:`V = \sigma_v^2` and the observation
noise :math:`w_k` have variance :math:`W = \sigma_w^2`. We track the
estimate (or mean) :math:`\hat{x}_{k|k}` and the variance
:math:`p_{k|k}`. We will also make the following substitutions:
:math:`P_{k-1|k-1} = \sigma_{k-1}^2`,
:math:`P_{k|k-1} = \tilde{\sigma}_k^2` and
:math:`P_{k|k} = \sigma_{k}^2`.

The Scalar Kalman Filter Algorithm
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

-  Predicted state:
   :math:`\hat{x}_{k|k-1} = F\hat{x}_{k-1|k-1} + G u_{k}`

-  Predicted estimate error: :math:`P_{k|k-1} = F^2 P_{k-1|k-1}  + V`

-  Optimal Kalman gain: :math:`K_k = P_{k|k-1}/( P_{k|k-1}  + W)`

-  Updated state estimate
   :math:`\hat{x}_{k|k} =\hat{x}_{k|k-1} + K_k (z_k - \hat{x}_{k|k-1})`

-  Updated estimate variance: :math:`P_{k|k} = (1 - K_k) P_{k|k-1}`

Example
^^^^^^^

Assume that you are given a simple scalar process on
:math:`0 \leq k < N`:

.. math:: x_k = x_{k-1} + u_k

where the control input is

.. math:: u_k = 0.5*(1 - 1.75k/N).

Also assume that you have process noise with standard deviation of
:math:`0.2` and observation noise with standard deviation of
:math:`0.75`.

When we don’t have actual experimental data, we need to simulate the
data. To illustrate the filter, we will create a noisy dataset; we
pretend to run the dynamical system and get the observations.

::

    N = 100
    mu1, sigma1 = 0.0, 0.2
    mu2, sigma2 = 0.0, 0.75
    r = np.random.normal(mu1,sigma1, N)
    q = np.random.normal(mu2,sigma2, N)
    x = np.zeros(N)
    z = np.zeros(N)
    u = np.arange(N)
    k = 1
    while (k<N):
      x[k] = x[k-1] + 0.5*(N-1.75*u[k])/N + r[k-1]
      z[k] = x[k] + q[k-1]
      k = k+1

.. raw:: latex

   \centering

|A. Plot of :math:`x_0`. B. Noisy observation of :math:`x_0`.| |A. Plot
of :math:`x_0`. B. Noisy observation of :math:`x_0`.|

Using the fake observations, we can test the filter.

::

    xf = np.zeros(N)
    pf = np.zeros(N)
    k = 1
    while (k<N):
      xp = xf[k-1] + 0.5*(N-1.75*u[k])/N
      pp = pf[k-1] + sigma1*sigma1
      kal = pp/(pp + sigma2*sigma2)
      xf[k] = xp + kal*(z[k-1] - xp)
      pf[k] = (1-kal)*pp
      k = k+1

.. raw:: latex

   \centering

|A. Kalman estimate of :math:`x_0`. B. Comparison of state estimate to
real state.| |A. Kalman estimate of :math:`x_0`. B. Comparison of state
estimate to real state.|

The Multivariate Kalman Filter
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The Kalman Filter has two stages. A predictive step based on the system
dynamics and an update based on observations or measurements.

-  *Prediction*: :math:`\hat{x}_{k|k-1}`, :math:`P_{k|k-1}`

-  *Update*: :math:`\hat{x}_{k|k}`, :math:`P_{k|k}`

-  :math:`P_{k|k} =  \textrm{cov}(x_k -  \hat{x}_{k|k})`

-  :math:`P_{k|k-1} = \textrm{cov}(x_k - \hat{x}_{k|k-1})`

-  :math:`S_{k} = \textrm{cov}(z_k - H\hat{x}_{k|k-1})`

The prediction step uses the system dynamics, the linear dynamical
model, to predict where the system should be. This prediction is for
both the state estimate :math:`\hat{x}` and the covariance of
:math:`\hat{x}`. This stage is also known as the *a priori* since it
occurs before the observation.

The update step takes the observation at that step and compares it to
the prediction. The difference between the two is known as the
innovation. It is what is new compared to the system dynamics. Using a
weighted least squares approach, the two are merged. This is done by
determining how reliable the new information is based on the innovation
covariance. The weight term is known as the Kalman gain. The weighted
innovation is added to the prediction of the state estimate to obtain
the Kalman estimate. As before, this stage is also known as the *a
posteriori* because it occurs after the observation. Repeated steps or
iterations of the Kalman filter allow the filter to track sequential
stages of a process. These sequential steps make this a recursive linear
gaussian state estimator.

Formally we have a dynamical process

.. math::

   \label{kalmanderivation1}
    x_{k+1} = F_k x_k + Gu_k + v_k

 where :math:`F_k` is the state transition matrix, :math:`Gu_k` is the
input control and and observation

.. math::

   \label{kalmanderivation2}
     z_k = Hx_k + w_k

 where :math:`H` is the observation matrix. The random variables
:math:`v_k`, :math:`w_k` are drawn from Gaussian distributions with
covariance models given by

.. math:: V = E[v_kv_k^T], \quad\quad W = E[w_kw_k^T].

The error covariance of the estimate is

.. math::

   \label{kalmanderivation3} 
    P_k = E[e_ke_k^T] = E[(x_k - \hat{x}_k)(x_k - \hat{x}_k)^T] .

 The state estimate will be denoted :math:`\hat{x}_k` and the process
update to the state is denoted :math:`\tilde{x}_k`

As before we assume that we can write our estimate as a combination of
the process update and the observation

.. math:: \label{kalmanderivation4} \hat{x}_k = \tilde{x}_k + K_k (z_k - H\tilde{x}_k)

The optimal choice of the Kalman gain parameter is to select :math:`K_k`
to minimize the mean square error
:math:`E[ \| x_k - \hat{x}_{k|k} \|^2 ]`. You will notice that

.. math::

   E[ \| x_k - \hat{x}_{k|k} \| ] = E \left[ \sum_i (x^i_{k}- \hat{x}^i_{k|k})^2\right]
    = Tr(P_{k|k})

where :math:`Tr(P_{k|k})` is the trace of :math:`P_{k|k}`. So, we need
an expression for :math:`P_{k|k}` in terms of the Kalman gain.

We can plug in the observation,
`[kalmanderivation1] <#kalmanderivation1>`__ into
`[kalmanderivation4] <#kalmanderivation4>`__

.. math:: \hat{x}_k = \tilde{x}_k + K_k (Hx_k + w_k - H\tilde{x}_k)

This form of the estimate can be substituted into the error covariance

.. math:: P_{k|k} = E[e_ke_k^T] = E[[(I - K_kH)(x_k-\tilde{x}_k)-K_kw_k][(I - K_kH)(x_k-\tilde{x}_k)-K_kw_k]^T] .

Since observation or measurement noise is not correlated to process
noise we can rewite

.. math:: P_{k|k} = (I - K_kH) E[(x_k-\tilde{x}_k)(x_k-\tilde{x}_k)^T](I - K_kH)^T -  K_kE[w_kw_k^T] K_k^T.

Since :math:`P_{k|k-1} = E[(x_k-\tilde{x}_k)(x_k-\tilde{x}_k)^T]` we
obtain

.. math:: P_{k|k} = (I - K_kH) P_{k|k-1} (I - K_kH)^T -  K_k W K_k^T .

Expanding the expression and using :math:`S_k = H P_{k|k-1} H^T + W_k`
we have

.. math:: P_{k|k}  = P_{k|k-1}  - K_kH P_{k|k-1} - P_{k|k-1} H^T K_K^T + K_k S_k K_k^T

As stated above, we want to minimize :math:`Tr(P_{k|k})` with respect to
:math:`K_k`:

.. math:: \frac{\partial Tr(P_{k|k})}{\partial K_k} = -2(H P_{k|k-1})^T + 2K_k S_k = 0,

solving for the Kalman gain gives

.. math:: K_k = P_{k|k-1}H^T S^{-1}_k .

| We can collect the results into the following algorithm:
| **Kalman Filter**
| **Predict:** Prediction or a priori stage

-  Predicted state:
   :math:`\hat{x}_{k|k-1} = F_{k}\hat{x}_{k-1|k-1} + G_{k} u_{k}`

-  Predicted estimate covariance:
   :math:`P_{k|k-1} = F_{k} P_{k-1|k-1} F_{k}^{T} + V_{k}`

**Update:** Update or a posteriori stage

-  Innovation or measurement residual:
   :math:`y_k = z_k - H_k\hat{x}_{k|k-1}`

-  Innovation (or residual) covariance: :math:`S_k = H_k
   P_{k|k-1} H_k^\text{T} + W_k`

-  Optimal Kalman gain: :math:`K_k = P_{k|k-1}H_k^\text{T}S_k^{-1}`

-  Updated state estimate
   :math:`\hat{x}_{k|k} =\hat{x}_{k|k-1} + K_k y_k`

-  Updated estimate covariance: :math:`P_{k|k} =
     (I - K_k H_k) P_{k|k-1}`

The control input is the current control input and depends on how you
index it as to being :math:`u_k` or :math:`u_{k-1}`. You can think of
this control being injected between :math:`k` and :math:`k-1`. So it is
not critical how you index the term and will be clear from the process
equations.

If the model is accurate, and the values for :math:`\hat{x}_{0|0}`

and :math:`P_{0|0}` accurately reflect the distribution of the initial
state values, then the following invariants are preserved: (all
estimates have mean error zero)

-  :math:`\textrm{E}[x_k - \hat{x}_{k|k}] =\textrm{E}[x_k - \hat{x}_{k|k-1}] = 0`

-  :math:`\textrm{E}[z_k] = 0`

where :math:`E[\xi]` is the expected value of :math:`\xi`.

:math:`x_0`, :math:`P_0`

Estimates of :math:`x_k`, :math:`P_k`

:math:`k=0`

.. raw:: latex

   \WHILE {Not Terminated}

:math:`k=k+1`

:math:`x_k = F_{k}x_{k-1} + G_{k} u_{k}`
:math:`P_{k} = F_{k} P_{k-1} F_{k}^{T} + V_{k}`

:math:`y_k = z_k - H_kx_{k}`

:math:`S_k = H_k P_{k} H_k^\text{T} + W_k`
:math:`K_k = P_{k}H_k^\text{T}S_k^{-1}` :math:`x_k =   x_{k} + K_k y_k`
:math:`P_{k} = (I - K_k H_k) P_{k}`

.. raw:: latex

   \ENDWHILE

Simple Example of a Single Step
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Assume that you have the following Gaussian process and observation:

.. math::

   \begin{array}{l}
   x_k = Fx_{k-1} + Gu_k + v_k\\
   z_k = Hx_k + w_k
   \end{array}

Let

.. math:: x = \begin{bmatrix}a \\ b\end{bmatrix}, \quad F = \begin{bmatrix} 0.9 &-.01 \\0.02 &0.75\end{bmatrix}, \quad G = \begin{bmatrix} 0.1\\ 0.05\end{bmatrix}, \quad H = \begin{bmatrix} 1& 0 \end{bmatrix},

\ 

.. math:: V = \begin{bmatrix} 0.005265&0\\0& 0.005265\end{bmatrix}, \quad W = 0.7225,\quad z_1 = 0.01

.. math:: \quad u_k = \sin (7*k/100), \quad x_0 = \begin{bmatrix} 0\\0\end{bmatrix}, \quad P_0 = \begin{bmatrix}0 & 0\\ 0&0\end{bmatrix}.

 Apply the Kalman Filter process and compute :math:`\hat{x}_{1|1}` and
:math:`P_{1|1}`.

Process update:

.. math::

   \hat{x}_{1|0} = \begin{bmatrix} 0.9 &-.01 \\0.02 &0.75\end{bmatrix}\hat{x}_{0|0}
   + \begin{bmatrix} 0.1\\ 0.05\end{bmatrix} u_k
   =  \begin{bmatrix} 0.9 &-.01 \\0.02 &0.75\end{bmatrix}\begin{bmatrix} 0\\0\end{bmatrix}
   + \begin{bmatrix} 0.1\\ 0.05\end{bmatrix}\sin (7/100)

\ 

.. math:: \approx \begin{bmatrix} 0.0069942847\\  0.0034971424\end{bmatrix}

Process covariance update:

.. math:: P_{1|0} = F P_{0|0} F^{T} + V =

.. math:: P_{1|0} = \begin{bmatrix} 0.9 &-.01 \\0.02 &0.75\end{bmatrix}\begin{bmatrix}0 & 0\\ 0&0\end{bmatrix} \begin{bmatrix} 0.9 &0.02 \\ -.01&0.75\end{bmatrix} +\begin{bmatrix} 0.005265&0\\0& 0.005265\end{bmatrix}

\ 

.. math:: = \begin{bmatrix} 0.005265&0\\0& 0.005265\end{bmatrix}.

Innovation and innovation covariance:

.. math:: y_1 = 0.01 - \begin{bmatrix} 1& 0 \end{bmatrix}\hat{x}_{1|0} = 0.01 - \begin{bmatrix} 1& 0 \end{bmatrix}\begin{bmatrix} 0.0069942847\\  0.0034971424\end{bmatrix}

\ 

.. math:: = 0.0030057153

.. math:: S_1 = HP_{1|0} H^\text{T} + W = \begin{bmatrix} 1 & 0\end{bmatrix} \begin{bmatrix} 0.005265&0\\0& 0.005265\end{bmatrix}\begin{bmatrix} 1\\0\end{bmatrix} + 0.7225

\ 

.. math:: =0.728125

Kalman Gain

.. math::

   K_1 = P_{1|0}H_1^\text{T}S_1^{-1} = \begin{bmatrix} 0.005265&0\\0& 0.005265\end{bmatrix}
   \begin{bmatrix} 1\\0\end{bmatrix}/0.728125

\ 

.. math:: = \begin{bmatrix} 0.00772532 \\ 0.0 \end{bmatrix}

Updated state variables

.. math::

   \hat{x}_{1|1} =
     \hat{x}_{1|0} + K_1 y_1 = \begin{bmatrix} 0.0069942847\\  0.0034971424\end{bmatrix} + \begin{bmatrix} 0.00772532 \\ 0.0 \end{bmatrix} (0.00300572)

.. math:: = \begin{bmatrix} 0.007017504813\\  0.0034971424\end{bmatrix}

State variable covariance:

.. math::

   P_{1|1} =
     (I - K_1 H_1) P_{1|0} =  \begin{bmatrix} 0.99227468 & 0.0 \\ 0.0 & 1.0 \end{bmatrix} P_{1|0}

\ 

.. math::

   = \begin{bmatrix} 0.005224326  &  0.0 \\ 
     0.0  &  0.005265 \end{bmatrix}

It is useful to visualize the effects of a single Kalman step. The
images are provided in
Figures \ `[fig:kalmancloudsa] <#fig:kalmancloudsa>`__, \ `[fig:kalmancloudsb] <#fig:kalmancloudsb>`__
and the numbers used are not the same as the example above [2]_. The
system we use is Let

.. math:: x_0 = \begin{bmatrix} 1\\1\end{bmatrix}, \quad P_0 = \begin{bmatrix}0.01& 0\\ 0&0.001\end{bmatrix}, \quad F = \begin{bmatrix} 0.85 &-.1 \\0.02 &0.75\end{bmatrix},

\ 

.. math::

   G = \begin{bmatrix} 0.025\\ 0.05\end{bmatrix}, \quad H = I,
    V = \begin{bmatrix} 0.0075^2&0\\0& 0.0075^2\end{bmatrix},

.. math:: W = \begin{bmatrix} 0.035^2&0\\0& 0.035^2\end{bmatrix}, \quad  a = \begin{bmatrix} 0.01\\ 0.02\end{bmatrix} ,\quad z = \hat{x}  +a+ w_k.

Starting with a single point, we move this forward using the process
update. From the same starting point we run each forward with the
process update, :math:`\hat{x}_{k|k-1}` many times to generate a
distribution. The resulting points are different since the process
update has noise.
Figure \ `[fig:kalmancloudsa] <#fig:kalmancloudsa>`__\ (a) shows the
point cloud (in blue). This process does not have a great deal of noise
so the cloud is tightly clustered.
Figure \ `[fig:kalmancloudsa] <#fig:kalmancloudsa>`__\ (b) shows the
observation :math:`z_k`.
Figure \ `[fig:kalmancloudsb] <#fig:kalmancloudsb>`__\ (a) shows the
observation update, the fusion of the observation with the state update.

::

    for i in range(M):
        xp = np.dot(F,xf0) + G + np.random.normal(mu1,sigma1, 2)
        pp = np.dot(F,np.dot(P,FT)) + V
        z = np.dot(F,xf0) + G + a + np.random.normal(mu2,sigma2, 2)
        res = z - xp
        S = pp + W
        kal = np.dot(pp,linalg.inv(S))
        xf = xp + np.dot(kal,res)

You will notice that it is not circular. The covariance matrix really
trusted the :math:`y` process estimate and so weighted the process more
than the observation. In the :math:`x` estimate, much more of the
observation was used. So the resulting point cloud has lower variation
in :math:`y` than :math:`x`.
Figure \ `[fig:kalmancloudsb] <#fig:kalmancloudsb>`__\ (b) graphs the
error ellipses for the previous point clouds. It is easier to see the
changes from this than looking at the raw data.

.. raw:: latex

   \centering

.. raw:: latex

   \centering

.. figure:: math/cloud1.pdf
   :alt: Point distribution after process update.

   Point distribution after process update.

.. raw:: latex

   \centering

.. figure:: math/cloud2.pdf
   :alt: Observed point distribution.

   Observed point distribution.

.. raw:: latex

   \centering

.. figure:: math/cloud3.pdf
   :alt: Final distribution after update step.

   Final distribution after update step.

.. raw:: latex

   \centering

.. figure:: math/cloud4.pdf
   :alt: The standard deviation based ellipses.

   The standard deviation based ellipses.

Kalman Code and Generation of Testing Data
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The development of filtering software needs to have datasets to test the
software. The early stages of software development are about removing
simple errors such as syntax errors. In the absence of a real robot
producing actual data, how do we develop and test our code? This can be
done using pure simulation. We can simulate the motion of a robot. In
practice we just compute the location and orientation of the robot based
on the motion equations or kinematics derived in the Motion chapter. For
example, for the differential drive robot, we can send control signals
(the wheel speeds) and compute the location of the robot. Each step of
the simulation produces a small motion and a small amount of error. That
error will accumulate which is consistent with what we see in actual
systems. Assume that the robot moves along according to the kinematic
model :math:`F` and :math:`G` plus the noise, we have

.. math:: x_{k+1} = Fx_k + Gu_k + v_k

This produces the robot path as a vector of values :math:`\{ x \}`.

At each step along the computed path, we can make an observation
(:math:`z_k`) which is noise added to the exact values :math:`x_k + v_k`
where :math:`v_k` is Gaussian noise. Since :math:`z_k` is not added back
into the computation for :math:`x_{k+1}`, the observation noise,
:math:`w_k`, does not accumulate. The process is the following:

.. math:: x_{k+1} = Fx_k + Gu_k + v_k

\ 

.. math:: z_{k+1} = Hx_k + w_k

These can be computed together.

::

    k = 1
    while (k<N):
      q = np.random.normal(mu1,sigma1,2)
      r = np.random.normal(mu1,sigma1, 1)
      x[k] = np.dot(F,x[k-1]) + G[k-1] + q
      z[k] = np.dot(H,x[k]) + r
      k = k+1

You may have noticed that we have added noise to the end of the
expression. Why add? Why not multiply? Assume that we have two signals

.. math:: a(t) = \cos(t) , \quad  b(t) = 20\cos(t)

and to them we add mean zero Gaussian noise with standard deviation
:math:`\sigma = 0.25`, :math:`v`:

.. math:: a_1(t) = \cos(t) +v, \quad  b_1(t) = 20\cos(t) + v

or we multiply that noise

.. math:: a_2(t) = v\cos(t), \quad  b_2(t) = 20v\cos(t)

We then subtract off the signal and compute the standard deviations. For
:math:`a_1` and :math:`b_1`, it is mathematically clear that you would
get :math:`\sigma = 0.25` back - if the sample size large enough.

::

    >>> c = np.cos(t)
    >>> a1 = c + np.random.normal(0, 0.25,100)
    >>> b1 = 20*c + np.random.normal(0, 0.25,100)
    >>> a2 = np.random.normal(0, 0.25,100)*c
    >>> b2 = 20*np.random.normal(0, 0.25,100)*c
    >>> a1sub = a1 - c
    >>> b1sub = b1 - 20*c
    >>> a2sub = a2 - c
    >>> b2sub = b2 - 20*c
    >>> np.std(a1sub)
    0.26168514491592509
    >>> np.std(b1sub)
    0.20957486503563907
    >>> np.std(a2sub)
    0.73517338736953186
    >>> np.std(b2sub)
    14.687819454616823

The multiplication by the signal will amplify the noise by the signal
strength and this changes the effective standard deviation. We will for
this text focus on adding noise via addition. One issue we will address
later in this chapter is the difference between process noise and
control noise. By process noise we mean the addition of noise in the
process step, the addition of :math:`v`:

.. math:: x_{k+1} = Fx_k + Gu_k + v_k .

Noise in the control would appear as :math:`u_k + r_k` where :math:`r_k`
is some zero mean noise term. This would get changed by the term
:math:`G`

.. math:: x_{k+1} = Fx_k + G(u_k + r_k)  + v_k  = Fx_k + Gu_k + Gr_k  + v_k  = Fx_k + Gu_k + v'_k .

For now, we just assume we can lump the two together with a modified
process noise term.

The linear dynamical system in the prevous example can be simulated
which will produce data that can be used in Kalman filtering software
testing. The code block below will generate a list of values which can
be used as the observations for a run of a Kalman filtering algorithm.
The numbers in the various arrays come from the example above, but
certainly can be changed for different applications. As above, let

.. math:: x = \begin{bmatrix}a \\ b\end{bmatrix}, \quad F = \begin{bmatrix} 0.9 &-.01 \\0.02 &0.75\end{bmatrix}, \quad G = \begin{bmatrix} 0.1\\ 0.05\end{bmatrix}, \quad H = \begin{bmatrix} 1& 0 \end{bmatrix},

\ 

.. math:: V = \begin{bmatrix} 0.075^2&0\\0& 0.075^2\end{bmatrix}, \quad W = 0.85^2,\quad z_1 = 0.01

.. math:: \quad u_k = \sin (7*k/100), \quad x_0 = \begin{bmatrix} 0\\0\end{bmatrix}, \quad P_0 = \begin{bmatrix}0 & 0\\ 0&0\end{bmatrix}.

::

    #  Create fake dataset for experiment
    N = 100
    t = np.linspace(0, 7, 100)
    u = 0.1*np.sin(t)
    mu1, sigma1 = 0.0, 0.075
    mu2, sigma2 = 0.0, 0.85
    var1 = sigma1*sigma1
    x = np.zeros((N,2))
    F = np.array([[0.9,-0.01],[0.02,0.75]])
    FT = F.T
    G = np.array([u, 0.5*u]).T


    H = np.array([1,0])
    HT = H.T
    V = np.array([[var1,0],[0,var1]])
    W = sigma2*sigma2
    P = np.zeros((N,2,2))
    z = np.zeros(N)
    xf = np.zeros((N,2))

    k = 1
    while (k<N):
      q = np.random.normal(mu1,sigma1,2)
      r = np.random.normal(mu1,sigma1, 1)
      x[k] = np.dot(F,x[k-1]) + G[k-1] + q
      z[k] = np.dot(H,x[k]) + r
      k = k+1
    # done with fake data

The code block above provides the array z which is then piped into the
Kalman Filter

::

    k = 1
    while (k<N):
      xp = np.dot(F,xf[k-1]) + G[k-1]
      pp = np.dot(F,np.dot(P[k-1],FT)) + V
      y = z[k] - np.dot(H,xp)
      S = np.dot(H,np.dot(pp,HT)) + W
      kal = np.dot(pp,HT)/S
      xf[k] = xp + y*kal
      P[k] = pp - np.outer(kal,np.dot(H,pp))
      k = k+1

    t = np.arange(0,N,1)
    plt.plot(t, x[:,0], 'bo', t,z,'ro', t, xf[:,0],'g-')
    plt.show()

The blue dots are a graph of :math:`(x_0)_k`, the red dots are the
observations :math:`z_k`, and the green dots are the Kalman estimate of
the state.

|image|

The blue dots are a graph of :math:`(x_1)_k`, and the green dots are the
Kalman estimate of the state.

|image|

The Classic Vehicle on Track Example
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Consider a mobile robot along a track. Let the state
:math:`x = [x_r , s_r]`

where :math:`x_r` and :math:`s_r` are the vehicle position and speed.
Let :math:`m`

denote the mass of the vehicle and :math:`u` be the force acting on the
vehicle. Note that

.. math:: \frac{ds_r}{dt} = \frac{u}{m}

Discretize

.. math:: \frac{s_r(t+T)-s_r(t)}{T} \approx \frac{ds_r}{dt}

:math:`T` is the sample rate. Thus

.. math:: s_r(k+1) = s_r(k) + \frac{T}{m} \, u(k).

From calculus we know that

.. math:: \frac{dx_r}{dt} = s_r.

Discretizing this equation

.. math:: \frac{dx_r}{dt} \approx \frac{x_r(k+1) - x_r(k)}{T} =  s_r(k)

and rewriting gives

.. math:: x_r(k+1) = x_r(k) + T s_r(k).

This gives the pair of equations

.. math::

   \begin{array}{l}
   x_r(k+1) = x_r(k) + T s_r(k) \\
   s_r(k+1) = s_r(k) + \frac{T}{m} \, u(k)
   \end{array}

Load the variables into an array

.. math::

   x_{k+1} = \begin{bmatrix}1 & T \\ 0 & 1\end{bmatrix} x_k
     + \begin{bmatrix} 0 \\ T/m \end{bmatrix}u_k + v_k

Assume that you have some sensors

.. math:: z_{k+1} = \begin{bmatrix}0 & 1\end{bmatrix} x_k + w_k

where :math:`v` and :math:`w` are zero mean Gaussian noise. Thus

.. math::

   F_k = \begin{bmatrix} 1 & T \\ 0 & 1\end{bmatrix}, \quad
     G_k = \begin{bmatrix} 0 \\ T/m \end{bmatrix}, \quad
     H_k = \begin{bmatrix} 0 & 1\end{bmatrix}

For this example take :math:`m=1` and :math:`T=0.5`. Assume the
covariance of :math:`v_k`

.. math:: V_k = \begin{bmatrix}0.2 & 0.05 \\ 0.05 & 0.1\end{bmatrix}

Assume the covariance for :math:`w_k` is :math:`W_k = [0.5]`, and at
:math:`k=0`, :math:`u(0) = 0` and
:math:`\hat{x}_{0|0} = \begin{bmatrix}2 & 4\end{bmatrix}^T`,

.. math::

   P_{0|0}
           = \begin{bmatrix}1 & 0 \\ 0 & 2\end{bmatrix}

Next we compute one iteration of the Kalman Filter.

-  State estimate prediction:

   .. math::

      \hat{x}_{1|0} = F_{1}\hat{x}_{0|0} + G_{1} u_{1} =
      \begin{bmatrix}1 & 0.5 \\ 0 & 1\end{bmatrix}
                  \begin{bmatrix}2 \\4 \end{bmatrix} + \begin{bmatrix}0
                    \\ 0.5\end{bmatrix} 0 =
      \begin{bmatrix}4 \\ 4\end{bmatrix}

-  Covariance prediction

   .. math:: P_{1|0} = F_{1} P_{0|0} F_{1}^{T} + V_{1}

   .. math::

      = \begin{bmatrix}1 & 0.5 \\ 0 &
        1\end{bmatrix}
      \begin{bmatrix}1 & 0 \\ 0 & 2\end{bmatrix}
      \begin{bmatrix}1 & 0 \\ 0.5 & 1\end{bmatrix} +
      \begin{bmatrix}0.2 & 0.05 \\ 0.05 & 0.1\end{bmatrix}
      = \begin{bmatrix}1.7 & 1.05 \\ 1.05 & 2.1\end{bmatrix}

Assume that you measure and obtain

.. math:: z_1 = 3.8

-  Innovation:

   .. math::

      y_k = z_1 - H\hat{x}_{1|0} = 3.8 - \begin{bmatrix} 0 & 1\end{bmatrix} 
      \begin{bmatrix}4 \\ 4\end{bmatrix} = -.2

-  The matrix :math:`S`

   .. math::

      S_1 = H P_{1|0} H^\text{T} + W_1
      = \begin{bmatrix} 0 & 1\end{bmatrix} \begin{bmatrix}1.7 & 1.05 \\ 1.05 & 2.1\end{bmatrix} 
      \begin{bmatrix}0 \\ 1\end{bmatrix} +0.5 = 2.6

-  The matrix :math:`K` (Kalman Gain)

   .. math::

      K_1 = P_{1|0}H^\text{T}S_1^{-1} = \begin{bmatrix}1.7 & 1.05 \\ 1.05 & 2.1\end{bmatrix}
      \begin{bmatrix}0 \\ 1\end{bmatrix}
      \left( 2.6 \right)^{-1} =
      \begin{bmatrix}0.404 \\ 0.808\end{bmatrix}

-  The estimate update:

   .. math:: \hat{x}_{1|1} = \hat{x}_{1|0} + K_1 y_1 =\begin{bmatrix}4 \\ 4\end{bmatrix} +\begin{bmatrix}0.404 \\ 0.808\end{bmatrix}(-.2) = \begin{bmatrix}3.9192 \\ 3.8384 \end{bmatrix}

-  The covariance estimate update:

   .. math:: P_{1|1} = (I - K_1 H) P_{1|0}

   .. math::

      = \left( \begin{bmatrix}1 & 0 \\ 0& 1\end{bmatrix}
      -  \begin{bmatrix}0.404 \\ 0.808\end{bmatrix} \begin{bmatrix} 0 & 1\end{bmatrix} \right)
      \begin{bmatrix}1.7 & 1.05 \\ 1.05 & 2.1\end{bmatrix}
      =\begin{bmatrix}.4242 & .8484 \\ .8484 & 1.6968\end{bmatrix}

Some issues to address
~~~~~~~~~~~~~~~~~~~~~~

Because the Kalman filter is trying to estimate the state, and determine
the process as well as the observation quality, the initial iterations
may be very inaccurate. Assuming you have a convergent process, it can
still take some time for the filter to converge and provide a good state
estimate. What the filter is doing is figuring out the errors for the
state estimate (the covariance :math:`P`). Many robotics applications
will have the robot sit still for a few seconds to allow the filter to
converge.

A common question is what should the initial values be? For the state
estimate, one clearly uses starting information that one has. The
problem is that maybe not all the data is known. For unknown variables,
setting to zero is about all you can do. The corresponding entry in the
covariance matrix should be infinity (or a very large value). Another
approach for the covariance is to set it to zero and let the first dozen
iterations figure out the covariance or one can populate it with values.
One could even store the covariance after the filter settles and use
that to initialize the filter.

For matrix :math:`W`, we use the sensor datasheets which can provide
standard deviations for sensor readings. The squares of those can be
placed on the diagonal of :math:`W`. The matrix :math:`V` is harder to
determine and may require some experimentation. A simplistic approach
would be to run the robot for a single step and measure the end state.
Repeat this process for a large enough times as possible. That endstate
measurement data can be used to determine the variances of the process
as well as can be used to adjust the process in case of parameter
issues.

A variation on this approach for :math:`V` is to run the robot in for
multiple time steps and do the statistics on the end state as before.
Another method is to compare the Kalman estimation with the actual state
(done by hand measurement and not onboard sensing). The tune the
parameters. You can then optimize to gain good choices for :math:`V`,
:math:`W`. It should be noted that :math:`V` is the estimate for a given
:math:`\Delta t`. It needs to be scaled for time steps other than the
one it was developed for. So, if :math:`V` was developed for a time step
of :math:`\Delta t` and the Kalman estimation loops are using a time
step of :math:`T` , then :math:`V' = (T/ \Delta t) V` would scale the
covariance.

One concern follows from unreliable sensor connections. What happens
when a sensor is down or is not sending data? The Kalman gain is the
term that selects the relative amount of the model verses the sensor to
use in the estimate. Lacking a sensor, the Kalman gain will after some
iterations shut off that sensor. It will do this even if the sensor is
operational. It the sensor is giving readings that don’t make sense
given the physical model, the Kalman gain will reset to where only the
physical model is used.

.. math:: K_k = P_kH_k^TS_k^{-1} \to 0

\ This can be a problem for sensors that have drift or some type of
uncorrected deterministic error (DC bias).

The Kalman filter does not correct for drift that occurs in gyros and
other instruments. The common fix is to periodically reset (zero) the
sensor when in a known configuration - for example when the vehicle is
stopped and you know it is not turning. The issue of course is that
after a period of time the Kalman estimate becomes just the process
update step. The Kalman Gain parameter can be monitored. When it falls
below some threshold, then the sensor needs to be reset.

Work Estimates
^^^^^^^^^^^^^^

If you have :math:`n` equations, the work (multiplications) in the
filter is:

#. :math:`\hat{x}_{k|k-1} = F_{k}\hat{x}_{k-1|k-1} + G_{k} u_{k}` :  
   :math:`O(n^2)`

#. :math:`P_{k|k-1} = F_{k} P_{k-1|k-1} F_{k}^{T} + V_{k}` :
    :math:`O(n^3)`

#. :math:`K_k = P_{k|k-1}H_k^\text{T}\left[ H_k P_{k|k-1} H_k^\text{T} + W_k  \right]^{-1}`
   :  :math:`O(m!)` + :math:`O(n^2m)`

#. :math:`\hat{x}_{k|k} =   \hat{x}_{k|k-1} + K_k \left(z_k - H_k\hat{x}_{k|k-1} \right)`
   :  :math:`O(n^2)`

#. :math:`P_{k|k} =   (I - K_k H_k) P_{k|k-1}` :  :math:`O(n^3)`

The largest work is in step 3. By using an :math:`LU` factorization, we
can move this down to :math:`\text{max}(O(m^3),O(n^2m))` work. Step 2
can exploit symmetry to reduce work as only 1/2 the matrix needs to be
computed. For small matrices, explicit formulas for the inverse can be
used.

Different Sensor Types
~~~~~~~~~~~~~~~~~~~~~~

Now that we have the basic Kalman Filter process, we can look at some
variations on how it is applied. One question that arises is “What
should we do if we have multiple sensors?” Currently, the update stage
runs a single measurement fusion. The solution is to run the update loop
for each sensor. This is equivalent to running the full Kalman loop but
skipping the prediction step between the different sensors. The
algorithm follows.

**Predict:**

-  :math:`\hat{x}_{k|k-1} = F_{k}\hat{x}_{k-1|k-1} + G_{k} u_{k}`

-  :math:`P_{k|k-1} = F_{k} P_{k-1|k-1} F_{k}^{T} + V_{k}`

**Update:**

-  foreach sensor :math:`i`:

   -  :math:`y_k = z_k^i - (H^i)_k\hat{x}_{k|k-1}`

   -  :math:`S_k = (H^i)_k P_{k|k-1} (H^i)_k^\text{T} + W_k^i`

   -  :math:`K_k = P_{k|k-1}(H^i)_k^\text{T}S_k^{-1}`

   -  :math:`\hat{x}_{k|k-1} = \hat{x}_{k|k-1} + K_k y_k`

   -  :math:`P_{k|k-1} = (I - K_k (H^i)_k) P_{k|k-1}`

-  :math:`\hat{x}_{k|k} = \hat{x}_{k|k-1}`

-  :math:`P_{k|k} = P_{k|k-1}`

From this algorithm we notice that we have the ability to fuse multiple
different sensors; meaning you have multiple sensors measuring a single
state :math:`x_k`. Using the update steps we can fuse sensor
measurements without the need to perform the prediction step. Sensor
fusion can be done using a simplification of the Kalman Filter. Since we
only have observations, :math:`F=I`, :math:`G=0`, :math:`V=0` and so the
apriori stage of the filter drops out: So, we can just skip the apriori
step. This means we can define :math:`\hat{x}_{k}  = \hat{x}_{k|k}` and
:math:`P_{k} = P_{k|k}` and we have a basic formula to merge the sensed
data. Since we don’t have the time loop (in :math:`k`), we can redefine
:math:`k` to loop over the sensors. This reduces to exactly the sensor
fusion algorithm given in
Section \ `[multivariatesensorfusion] <#multivariatesensorfusion>`__.

In the last section we discussed the issue regarding unreliable sensor
readings in the situation where the data is occasionally not available.
This brings up a concern about having the data ready when the update
step is done. The assumption so far was that the Kalman loop is run at
the same frequency that the data is arriving.

However, there are several situations for which this is a problem. One
such situation is when several different classes of sensors are being
used. For example, your magnetometer may run at 80 Hz and your Lidar
might operate at 10 Hz. One solution is to run at 10Hz and just skip the
extra measurements from the magnetometer. Another possible problem
arises when the time between the sensor readings are very long giving a
:math:`\Delta t` that is very large. A large :math:`\Delta t` can make
the predictive step inaccurate.

**Predict:**

-  :math:`\hat{x}_{k|k-1} = F_{k}\hat{x}_{k-1|k-1} + G_{k} u_{k}`

-  :math:`P_{k|k-1} = F_{k} P_{k-1|k-1} F_{k}^{T} + V_{k}`

**Update:**

-  Loop over available sensor data during :math:`\Delta t` :

   -  :math:`y_k = z_k^i - (H^i)_k\hat{x}_{k|k-1}`

   -  :math:`S_k = (H^i)_k P_{k|k-1} (H^i)_k^\text{T} + W_k^i`

   -  :math:`K_k = P_{k|k-1}(H^i)_k^\text{T}S_k^{-1}`

   -  :math:`\hat{x}_{k|k-1} = \hat{x}_{k|k-1} + K_k y_k`

   -  :math:`P_{k|k-1} = (I - K_k (H^i)_k) P_{k|k-1}`

-  :math:`\hat{x}_{k|k} = \hat{x}_{k|k-1}`

-  :math:`P_{k|k} = P_{k|k-1}`

Extended Kalman Filter
----------------------

Recall that when we started the section on linear dynamical systems the
first example was the differential drive dynamics. These equations are
not linear due to the cos and sin terms. What does one do in the case
where the process or the observation is a nonlinear function? For a
nonlinear process, let :math:`x_k \in R^n`, :math:`u_k \in R^p`,
:math:`v_k  \in R^n`, :math:`w_k  \in R^m`,

.. math:: x_k = f(u_k,x_{k-1}) + v_k,

.. math:: z_k = h(x_{k-1})+w_k

 where :math:`v_k` has variance :math:`V_k` and :math:`w_k` has variance
:math:`W_k`, and let

.. math::

   \displaystyle F_k =
     \begin{bmatrix} \frac{\partial f_1}{\partial x_1} & \frac{\partial f_1}{\partial x_2}  & \dots &
   \frac{\partial f_1}{\partial x_n}  \\[8pt]
   \frac{\partial f_2}{\partial x_1} & \frac{\partial f_2}{\partial x_2}  & \dots &
   \frac{\partial f_2}{\partial x_n}  \\[8pt] \vdots & \vdots & \vdots \\[8pt]
   \frac{\partial f_n}{\partial x_1} & \frac{\partial f_n}{\partial x_2}  & \dots &
   \frac{\partial f_n}{\partial x_n}  \end{bmatrix},  
   \displaystyle H_k = \begin{bmatrix} \frac{\partial h_1}{\partial x_1} & \frac{\partial h_1}{\partial x_2}  & \dots &
   \frac{\partial h_1}{\partial x_n}  \\[8pt]
   \frac{\partial h_2}{\partial x_1} & \frac{\partial h_2}{\partial x_2}  & \dots &
   \frac{\partial h_2}{\partial x_n}  \\[8pt] \vdots & \vdots & \vdots \\[8pt]
   \frac{\partial h_m}{\partial x_1} & \frac{\partial h_m}{\partial x_2}  & \dots &
   \frac{\partial h_m}{\partial x_n}  \end{bmatrix}

This is a Taylor expansion approach to dealing with nonlinear mappings.
For more information about Jacobians,
https://en.wikipedia.org/wiki/Jacobian_matrix_and_determinant.

#. Predicted state:

   .. math:: \hat{x}_{k|k-1} = f(\hat{x}_{k-1|k-1}, u_{k})

#. Predicted estimate covariance:

   .. math:: P_{k|k-1} = F_{k} P_{k-1|k-1} F_{k}^{T} + V_{k}

#. Optimal Kalman gain:

   .. math::

      K_k = P_{k|k-1}H_k^\text{T}\left(H_k
      P_{k|k-1} H_k^\text{T} + W_k\right)^{-1}

#. Updated state estimate:

   .. math:: \hat{x}_{k|k} =\hat{x}_{k|k-1} + K_k \left(z_k - h(\hat{x}_{k|k-1})\right)

#. Updated estimate covariance:

   .. math::

      P_{k|k} =
        (I - K_k H_k) P_{k|k-1}

Short Example
~~~~~~~~~~~~~

What is the Extended Kalman Filter formulation of the motion model:

.. math:: \begin{array}{l}\dot{x} = y \\\dot{y} = -\cos(x) + 0.4\sin(t)\end{array},

\ and observation

.. math:: h(x,y) = \begin{bmatrix}x \\ y\end{bmatrix}

 with step size :math:`\Delta t = 0.1,` and noise

.. math:: V = \begin{bmatrix} 0.1&0.01\\0.01& 0.1\end{bmatrix}, \quad , W = \begin{bmatrix} 0.05&0\\0& 0.05\end{bmatrix}.

 Assume that the initial values, :math:`t_0 = 0`,

.. math:: x_0 = \begin{pmatrix} 1 \\ 1 \end{pmatrix}

 and we have a reasonably accurate start

.. math:: P_0 = \begin{pmatrix} 0.05 & 0 \\ 0 & 0.05   \end{pmatrix}

Using a basic Euler formulation we replace the derivative:

.. math::

   \begin{array}{l}\displaystyle\frac{x_{k+1} - x_k}{0.1} = y_k \\[3mm]
   \displaystyle\frac{y_{k+1} - y_k}{0.1} = -\cos(x_k) + 0.4\sin(t_k)\end{array}

This gives the discrete form:

.. math:: \begin{array}{l}x_{k+1} = x_k + 0.1 y_k \\y_{k+1} = y_k -0.1\cos(x_k) + 0.04\sin(t_k)\end{array}

and so

.. math::

   F = \begin{bmatrix} 1 & 0.1 \\ 0.1\sin(x_k) & 1 \end{bmatrix},  \quad 
   H = \begin{bmatrix} 1 & 0 \\ 0 & 1 \end{bmatrix}

| **EKF:**

#. Predicted state:

   .. math:: \hat{x}_{1|0} = f(\hat{x}_{0|0}, u_{1})  =\begin{pmatrix} 1+0.1(1) \\ 1 - 0.1\cos(1) + 0.04\sin(0) \end{pmatrix} =   \begin{pmatrix} 1.1 \\ 0.45969769\end{pmatrix}

#. Predicted estimate covariance:

   .. math:: P_{1|0} = F_{1} P_{0|0} F_{1}^{T} + V_{1}

   .. math:: =  \begin{bmatrix} 1 & 0.1 \\ 0.1\sin(1) & 1 \end{bmatrix}   \begin{bmatrix} 0.05 & 0 \\ 0 & 0.05   \end{bmatrix} \begin{bmatrix} 1 & 0.1\sin(1) \\ 0.1 & 1 \end{bmatrix} +

   \ 

   .. math::

      \begin{bmatrix} 0.1&0.01\\0.01& 0.1\end{bmatrix}   
      =  \begin{bmatrix} 0.605   &    0.10207355 \\ 
       0.10207355  &  0.60354037 \end{bmatrix}

#. Optimal Kalman gain:

   .. math::

      K_1 = P_{1|0}H_1^\text{T}\left(H_1
      P_{1|0} H_1^\text{T} + W_1\right)^{-1}

   .. math::

      =  \begin{bmatrix} 0.605   &    0.10207355 \\ 
       0.10207355  &  0.60354037 \end{bmatrix}

   .. math::

      \times \left(  \begin{bmatrix} 0.605   &    0.10207355 \\ 
       0.10207355  &  0.60354037 \end{bmatrix}  + \begin{bmatrix} 0.05&0\\0& 0.05\end{bmatrix}\right)^{-1}

   .. math::

      = \begin{bmatrix} 0.92175979 & 0.01221999 \\
              0.01221999 &  0.92158505 \end{bmatrix}

#. Updated state estimate:

   .. math:: \hat{x}_{1|1} =\hat{x}_{1|0} + K_1 \left(z_1 - h(\hat{x}_{1|0})\right)

   .. math::

      =\begin{pmatrix} 1.1 \\ 0.45969769\end{pmatrix} + \begin{bmatrix} 0.92175979 & 0.01221999 \\
              0.01221999 &  0.92158505 \end{bmatrix}

   .. math::

      \times \left(\begin{pmatrix}1.15 \\ 0.5\end{pmatrix} -  \begin{pmatrix} 1.1 \\ 0.45969769 \end{pmatrix} \right) 
        =  \begin{pmatrix} 1.14658048 \\ 0.4974507 \end{pmatrix}

#. Updated estimate covariance:

   .. math::

      P_{1|1} =
        (I - K_1 H_1) P_{1|0}

   .. math::

      = \left( \begin{pmatrix}1&0\\0&1\end{pmatrix} -  \begin{bmatrix} 0.92175979 & 0.01221999 \\
              0.01221999 &  0.92158505 \end{bmatrix}\right)

   .. math::

      \times \begin{bmatrix} 0.605   &    0.10207355 \\ 
       0.10207355  &  0.60354037 \end{bmatrix} = 
      \begin{bmatrix} 0.04608799&  0.000611 \\
              0.000611  &  0.04607925 \end{bmatrix}

Differential Drive Example
~~~~~~~~~~~~~~~~~~~~~~~~~~

In Chapter \ `[Chap:Motion] <#Chap:Motion>`__, we derived the equations
for the motion of the differential drive robot. In that chapter we also
simulated the motion of the robot based on wheel velocity data. Small
amounts of noise in the wheel velocity data could cause significant
errors in position estimation. Using the Extended Kalman Filter, we can
improve the location estimate as well as gain estimates for the
uncertainty of the location. Figure \ `[Fig:DDagain] <#Fig:DDagain>`__
recalls the variables and equations derived in
Chapter \ `[Chap:Motion] <#Chap:Motion>`__.

.. raw:: latex

   \centering

.. figure:: motion/dddim
   :alt: The variables used in the DD model.

   The variables used in the DD model.

.. raw:: latex

   \hspace{5mm}

.. math::

   \begin{array}{l}
    \dot{x} = \frac{r}{2} (\dot{\phi_1}+\dot{\phi_2})\cos(\theta) \\[5mm]
   \dot{y} = \frac{r}{2} (\dot{\phi_1}+\dot{\phi_2})\sin(\theta) \\[5mm]
   \dot{\theta} = \frac{r}{2L} (\dot{\phi_1}-\dot{\phi_2})
   \end{array}

As with the linear continuous models, both the Kalman and Extended
Kalman filters act on discrete dynamics. So as before, we need to
discretize the equations.

.. math::

   \begin{array}{l}
   \displaystyle \frac{x(t+\Delta t) - x(t)}{\Delta t}\approx \dot{x} = \frac{r}{2} (\dot{\phi_1}+\dot{\phi_2})\cos(\theta) \\[5mm]
   \displaystyle \frac{y(t+\Delta t) - y(t)}{\Delta t}\approx \dot{y} = \frac{r}{2} (\dot{\phi_1}+\dot{\phi_2})\sin(\theta) \\[5mm]
   \displaystyle \frac{\theta (t+\Delta t) - \theta (t)}{\Delta t}\approx \dot{\theta} = \frac{r}{2L} (\dot{\phi_1}-\dot{\phi_2})
   \end{array}

The discretized variables are

.. math:: t_k \equiv k\Delta t, \quad t_{k+1} = (k+1)\Delta t

.. math:: x_k \equiv x(t_k), \hspace*{1cm} y_k \equiv y(t_k)

.. math::

   \omega_{1, k}\equiv \dot{\phi}_{1}(t_k), \hspace*{1cm}
   \omega_{2, k}\equiv \dot{\phi}_{2}(t_k)

The discrete approximations to the differential drive equations are:

.. math::

   \begin{array}{l}
    x_{k+1} = x_k + \frac{r\Delta t}{2} (\omega_{1, k}+\omega_{2, k})\cos(\theta_k) \\[5mm]
   y_{k+1} = y_k + \frac{r\Delta t}{2} (\omega_{1, k}+\omega_{2, k})\sin(\theta_k) \\[5mm]
   \theta_{k+1} = \theta_k + \frac{r\Delta t}{2L} (\omega_{1, k}-\omega_{2, k})
   \end{array}

The next step is to linearize the process dynamics. This means that we
must compute the matrix :math:`F` from the nonlinear model :math:`f`.

.. math::

   x_k = \begin{bmatrix} x_k \\ y_k \\ \theta_k \end{bmatrix}, \quad
   u_k = \begin{bmatrix} \omega_{1, k} \\ \omega_{2, k}\end{bmatrix},

.. math::

   f(x_k,u_k) = \begin{bmatrix}
                  x_k + \frac{r\Delta t}{2} (\omega_{1, k}+\omega_{2, k})\cos(\theta_k) \\[5mm]
   y_k + \frac{r\Delta t}{2} (\omega_{1, k}+\omega_{2, k})\sin(\theta_k) \\[5mm]
   \theta_k + \frac{r\Delta t}{2L} (\omega_{1, k}-\omega_{2, k})
                \end{bmatrix}

.. math::

   \displaystyle F_k =
     \begin{bmatrix} \frac{\partial f_1}{\partial x_1} & \frac{\partial f_1}{\partial x_2}  &
   \frac{\partial f_1}{\partial x_3}  \\[8pt]
   \frac{\partial f_2}{\partial x_1} & \frac{\partial f_2}{\partial x_2}  & 
   \frac{\partial f_2}{\partial x_3}  \\[8pt] 
   \frac{\partial f_3}{\partial x_1} & \frac{\partial f_3}{\partial x_2}  & 
   \frac{\partial f_3}{\partial x_3}  \end{bmatrix}
   \displaystyle  = \begin{bmatrix} 1 & 0  & 
   -\frac{r\Delta t}{2} (\omega_{1, k}+\omega_{2, k})\sin(\theta_k)  \\[8pt]
   0 & 1  & 
   \frac{r\Delta t}{2} (\omega_{1, k}+\omega_{2, k})\cos(\theta_k)  \\[8pt] 
   0 & 0  & 1  \end{bmatrix}

Assume that you start the robot with pose :math:`[0,0,0]` and you know
this is exact so

.. math:: P_{0|0} = \begin{bmatrix} 0 & 0 & 0\\ 0 & 0 & 0 \\ 0 & 0 & 0 \end{bmatrix}.

\ Let the process noise and measurement noise covariances be

.. math::

   V = \begin{bmatrix} 0.2 & 0.01 & 0.1 \\ 0.01 & 0.2 & 0.01  \\ 0.1 & 0.01 & 0.3 \end{bmatrix},~~~
   W = \begin{bmatrix} 0.25 & 0 & 0.1 \\ 0 & 0.25 & 0.1  \\ 0.1 & 0.1 & 0.4 \end{bmatrix}

and the control inputs be :math:`\omega_{1,0} = 1`,
:math:`\omega_{2,0} = 2`. Take :math:`\Delta t = 0.1`, :math:`r=4`,
:math:`L = 6`.

Take

.. math::

   h_k(x_k) = \begin{bmatrix} x_k \\ y_k \\ \theta_k \end{bmatrix}, \quad
   H_k = \begin{bmatrix} 1 & 0  & 0  \\
   0 & 1  & 0  \\
   0 & 0  & 1  \end{bmatrix}

and so we plug in :math:`H` into our process and express:

#. :math:`\hat{x}_{k|k-1} = f(\hat{x}_{k-1|k-1}, u_{k})`

#. :math:`P_{k|k-1} = F_{k} P_{k-1|k-1} F_{k}^{T} + V_{k}`

#. :math:`K_k = P_{k|k-1}\left(
   P_{k|k-1} + W_k\right)^{-1}`

#. :math:`\hat{x}_{k|k} =\hat{x}_{k|k-1} + K_k \left(z_k - \hat{x}_{k|k-1}\right)`

#. :math:`P_{k|k} =   (I - K_k ) P_{k|k-1}`

.. math::

   \hat{x}_{1|0} = f(\hat{x}_{0|0}, u_{0}) =
   \begin{pmatrix}
    \frac{4(0.1)}{2} (1+2)\cos(0) \\[5mm]
    \frac{4(0.1)}{2} (1+2)\sin(0) \\[5mm]
    \frac{4(0.1)}{12} (1-2)
   \end{pmatrix}
   =
   \begin{pmatrix}
    0.6 \\[5mm]
    0 \\[5mm]
   -0.333
   \end{pmatrix}

.. math::

   F = \begin{bmatrix} 1 & 0  & 
   -\frac{r\Delta t}{2} (\omega_{1, k}+\omega_{2, k})\sin(\theta_k)  \\[8pt]
   0 & 1  & 
   \frac{r\Delta t}{2} (\omega_{1, k}+\omega_{2, k})\cos(\theta_k)  \\[8pt] 
   0 & 0  & 1  \end{bmatrix} =
   \begin{bmatrix} 1 & 0  & 
   0  \\
   0 & 1  & 
   0.6  \\
   0 & 0  & 1  \end{bmatrix}

\ so ...

.. math::

   P_{1|0} = \begin{bmatrix} 1 & 0  & 
   0  \\
   0 & 1  & 
   0.6  \\
   0 & 0  & 1  \end{bmatrix}
   \begin{bmatrix} 0 & 0 & 0\\ 0 & 0 & 0 \\ 0 & 0 & 0 \end{bmatrix}
   \begin{bmatrix} 1 & 0  & 
   0  \\
   0 & 1  & 
   0 \\
   0 & 0.6  & 1  \end{bmatrix}
   + 
   \begin{bmatrix} 0.2 & 0.01 & 0.1 \\ 0.01 & 0.2 & 0.01  \\ 0.1 & 0.01 & 0.3 \end{bmatrix}

.. math:: =  \begin{bmatrix} 0.2 & 0.01 & 0.1 \\ 0.01 & 0.2 & 0.01  \\ 0.1 & 0.01 & 0.3 \end{bmatrix}

.. math::

   K = \begin{bmatrix} 0.2 & 0.01 & 0.1 \\ 0.01 & 0.2 & 0.01  \\ 0.1 & 0.01 & 0.3 \end{bmatrix}
   \left[ \begin{bmatrix} 0.2 & 0.01 & 0.1 \\ 0.01 & 0.2 & 0.01  \\ 0.1 & 0.01 & 0.3 \end{bmatrix} +
   \begin{bmatrix} 0.25 & 0 & 0.1 \\ 0 & 0.25 & 0.1  \\ 0.1 & 0.1 & 0.4 \end{bmatrix}
   \right]^{-1}

\ 

.. math::

   = \begin{bmatrix} 0.2 & 0.01 & 0.1 \\ 0.01 & 0.2 & 0.01  \\ 0.1 & 0.01 & 0.3 \end{bmatrix}
   \begin{bmatrix}  2.552 & 0.126 & -0.749 \\
    0.126 & 2.317 & -0.400 \\
   -0.749& -0.400 & 1.705
   \end{bmatrix}

.. math::

   =
   \begin{bmatrix} 
   0.437 & 0.008 & 0.017\\
    0.043 & 0.461 & -0.070\\
    0.032 & -0.084 & 0.433
   \end{bmatrix}

Assume we have the observation: :math:`z_k = [0.5, 0.025, -0.3]^T` then
the innovation

.. math:: z_k - \hat{x}_{k|k-1} = \begin{pmatrix}-.1\\ 0.025\\ 0.033\end{pmatrix}

\ So,

.. math:: \hat{x}_{1|1} = \hat{x}_{1|0} + K_k \left(z_k - \hat{x}_{k|k-1}\right)

.. math::

   =
   \begin{pmatrix}
    0.6 \\
    0 \\
   -0.333
   \end{pmatrix}
   +
   \begin{bmatrix} 
   0.437 & 0.008 & 0.017\\
    0.043 & 0.461 & -0.070\\
    0.032 & -0.084 & 0.433
   \end{bmatrix} 
   \begin{pmatrix}
    -0.1\\
    0.025 \\
   0.033
   \end{pmatrix}

.. math::

   \hat{x}_{1|1}
   = 
   \begin{pmatrix}
   0.557\\
    0.005\\
    -0.324
   \end{pmatrix}

.. math::

   P_{1|1} = (I - K ) P_{1|0} =
   \begin{bmatrix} 
   0.563 & -0.008 & -0.017\\
    -0.043 & 0.539 & 0.070\\
    -0.032 & 0.084 & 0.567
   \end{bmatrix}
   \begin{bmatrix} 
   0.2 & 0.01 & 0.1 \\ 
   0.01 & 0.2 & 0.01  \\ 
   0.1 & 0.01 & 0.3 
   \end{bmatrix}

.. math::

   P_{1|1}
   =
   \begin{bmatrix} 
   0.111& 0.004& 0.051\\
   0.004& 0.108& 0.022\\
   0.051& 0.022& 0.168
   \end{bmatrix}

EKF Python Example[DDEKFexample]
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

We will take a similar setup as before, with a few values modified, and
generate the Python code required. For this simulation, we place the
noise only in the process equations and the observation. It is also
reasonable to consider placing the noise in the control inputs as well.
Assume that you start the robot with pose :math:`[0,0,0]` and you know
this is exact so

.. math:: P_{0|0} = \begin{bmatrix} 0 & 0 & 0\\ 0 & 0 & 0 \\ 0 & 0 & 0 \end{bmatrix}.

\ Let the process noise and measurement noise covariances be

.. math::

   V = \begin{bmatrix} 0.025^2 & 0 & 0 \\ 0 & 0.025^2& 0  \\ 0 & 0 & 0.025^2\end{bmatrix},~~~
   W = \begin{bmatrix} 0.85^2 & 0 & 0 \\ 0 & 0.85^2 & 0  \\ 0 & 0 & 0.85^2 \end{bmatrix}

and the control inputs be :math:`\omega_1 = 1.5\sin(t/10)`,
:math:`\omega_2 = \cos(t/10)`. Take :math:`\Delta t = 0.1`, :math:`r=4`,
:math:`L = 6`, and

.. math::

   h_k(x_k) = \begin{bmatrix} x_k \\ y_k \\ \theta_k \end{bmatrix}, \quad
   H_k = \begin{bmatrix} 1 & 0  & 0  \\
   0 & 1  & 0  \\
   0 & 0  & 1  \end{bmatrix}

To create the observation data we have a simulation:

::

    N = 100
    mu1, sigma1 = 0.0, 0.025
    mu2, sigma2 = 0.0, 0.85
    var1 = sigma1*sigma1
    var2 = sigma2*sigma2
    dt = 0.1
    r = 4
    dd = r*dt/2.0
    L = 6
    x = np.zeros((N,3))
    z = np.zeros((N,3))
    t = np.linspace(0, 10, 100)
    w1 = 1.5*np.sin(t)
    w2 = 1.0*np.cos(t)


    k = 1
    while (k<N):
      q = np.random.normal(mu1,sigma1,3)
      r = np.random.normal(mu2,sigma2, 3)
      x[k,0] = x[k-1,0] + dd*(w1[k]+w2[k])*cos(x[k-1,2]) + q[0]
      x[k,1] = x[k-1,1] + dd*(w1[k]+w2[k])*sin(x[k-1,2]) + q[1]
      x[k,2] = x[k-1,2] + dd*(w1[k]-w2[k])/L + q[2]
      z[k,0] = x[k,0] + r[0]
      z[k,1] = x[k,1] + r[1]
      z[k,2] = x[k,2] + r[2]
      k = k+1

The code to implement the Extended Kalman Filter is very similar to the
regular Kalman filter. The only difference is the inclusion of the
Jacobians for the process and observations. The observation is a linear
relation, so we just use the Jacobian from the last example. The first
plot the code generates is the time plots of simulation pose (blue
line), observation of the pose (red dots) and the pose estimate via
Kalman (green dots). The second plot is a workspace domain plot of
:math:`x` values against :math:`y` values, with :math:`\theta` ignored.

::

    H = np.array([[1,0,0],[0,1,0],[0,0,1]])
    HT = H.T
    V = np.array([[var1,0,0],[0,var1,0],[0,0,var1]])
    W = np.array([[var2,0,0],[0,var2,0],[0,0,var2]])
    P = np.zeros((N,3,3))
    xf = np.zeros((N,3))
    xp = np.zeros(3)
    sp = np.zeros(3)

    k = 1
    while (k<N):
      xp[0] = xf[k-1,0] + dd*(w1[k]+w2[k])*cos(xf[k-1,2])
      xp[1] = xf[k-1,1] + dd*(w1[k]+w2[k])*sin(xf[k-1,2])
      xp[2] = xf[k-1,2] + dd*(w1[k]-w2[k])/L
      F1 = [1.0,0.0, -dd*(w1[k]+w2[k])*sin(xf[k-1,2])]
      F2 =[0,1,dd*(w1[k]+w2[k])*cos(xf[k-1,2])]
      F = np.array([F1,F2,[0,0,1]])
      FT = F.T
      pp = np.dot(F,np.dot(P[k-1],FT)) + V
      y = z[k] - np.dot(H,xp)
      S = np.dot(H,np.dot(pp,HT)) + W
      SI = linalg.inv(S)
      kal = np.dot(pp,np.dot(HT,SI))
      xf[k] = xp + np.dot(kal,y)
      P[k] = pp - np.dot(kal,np.dot(H,pp))
      k = k+1

    t = np.arange(0,N,1)
    plt.plot(t, x, 'b-', t,z,'r.', t, xf,'go')
    plt.show()

    plt.plot(x[:,0], x[:,1], 'b-',z[:,0], z[:,1] ,'r.', xf[:,0], xf[:,1],'go')
    plt.show()

.. raw:: latex

   \centering

.. figure:: math/extendedkalmanfilter1.pdf
   :alt: The Extended Kalman Filter applied to the motion of a
   differential drive robot. Domain axis is time and vertical axis are
   the state variables. The simulation pose is given by the blue line,
   the observation of the pose given by the red dots and the pose
   estimate is given by the green dots.

   The Extended Kalman Filter applied to the motion of a differential
   drive robot. Domain axis is time and vertical axis are the state
   variables. The simulation pose is given by the blue line, the
   observation of the pose given by the red dots and the pose estimate
   is given by the green dots.

.. raw:: latex

   \centering

.. figure:: math/extendedkalmanfilter2.pdf
   :alt: The Extended Kalman Filter applied to the motion of a
   differential drive robot. This figure plots the :math:`y` state
   variable against the :math:`x` state variable with :math:`\theta`
   ignored. The simulation pose is given by the blue line, the
   observation of the pose given by the red dots and the pose estimate
   is given by the green dots.

   The Extended Kalman Filter applied to the motion of a differential
   drive robot. This figure plots the :math:`y` state variable against
   the :math:`x` state variable with :math:`\theta` ignored. The
   simulation pose is given by the blue line, the observation of the
   pose given by the red dots and the pose estimate is given by the
   green dots.

Mecanum EKF Example
~~~~~~~~~~~~~~~~~~~

| Developing the Extended Kalman Filter for the Mecanum drive is
  basically the same process. The only thing to derive is the matrix
  :math:`F`. Recalling
  equation \ `[mecanumforwardkinematics] <#mecanumforwardkinematics>`__:
| 

  .. math::

     \begin{bmatrix} x_{k+1}\\[3mm] y_{k+1}\\[3mm] \theta_{k+1} \end{bmatrix}
     =   \begin{bmatrix} x_{k}\\[3mm] y_{k}\\[3mm] \theta_{k} \end{bmatrix} +
     \frac{ r\Delta t }{4} \begin{bmatrix} A\cos(\theta_{k})  - B \sin(\theta_{k})   \\[3mm]
     A\sin(\theta_{k})  + B \cos(\theta_{k})                     \\[3mm]
                                 \frac{2}{(L_1+L_2) } C
              \end{bmatrix}

| where
  :math:`A = \left( \omega_{FL,k} + \omega_{FR,k} + \omega_{BL,k} + \omega_{BR,k} \right)`,
| :math:`B = \left(-\omega_{FL,k} + \omega_{FR,k} + \omega_{BL,k} - \omega_{BR,k}  \right)`,
| and
  :math:`C =  \left( -\omega_{FL,k} + \omega_{FR,k} - \omega_{BL,k} +\omega_{BR,k} \right)`.
  If we define
  :math:`\xi_k = \left( x_{k} , y_{k} , \theta_{k} \right)^T`,
  :math:`u_k =\left(  \omega_{FL,k} , \omega_{FR,k} , \omega_{BL,k} ,\omega_{BR,k} \right)^T`
  and reduce the :math:`k` index by one, then the process can be written
  compactly as

  .. math:: \xi_{k} = f(\xi_{k-1}, u_k) .

Computing the Jacobian of :math:`f`:

.. math::

   F = \begin{bmatrix} 1 & 0 & \frac{ r\Delta t }{4}  \left[ - A\sin(\theta_{k-1})  - B\cos(\theta_{k-1})  \right] \\[3mm]
   0 & 1 & \frac{ r\Delta t }{4}  \left[ A \cos(\theta_{k-1})  - B \sin(\theta_{k-1})  \right] \\[3mm]
   0 & 0 & 1 
   \end{bmatrix} .

The rest of the process is identical to the differential drive examples.

Process Noise
~~~~~~~~~~~~~

We return to our original nonlinear process,

.. math:: x_k = f(u_k,x_{k-1}) + v_k,

.. math:: z_k = h(x_{k-1})+w_k

 where, :math:`x_k \in R^n`, :math:`u_k \in R^p`, :math:`v_k  \in R^n`,
:math:`w_k  \in R^m`, :math:`v_k` has variance :math:`V_k` and
:math:`w_k` has variance :math:`W_k`, and let

.. math::

   \displaystyle F_k =
     \begin{bmatrix} \frac{\partial f_1}{\partial x_1} & \frac{\partial f_1}{\partial x_2}  & \dots &
   \frac{\partial f_1}{\partial x_n}  \\[8pt]
   \frac{\partial f_2}{\partial x_1} & \frac{\partial f_2}{\partial x_2}  & \dots &
   \frac{\partial f_2}{\partial x_n}  \\[8pt] \vdots & \vdots & \vdots \\[8pt]
   \frac{\partial f_n}{\partial x_1} & \frac{\partial f_n}{\partial x_2}  & \dots &
   \frac{\partial f_n}{\partial x_n}  \end{bmatrix},  
   \displaystyle H_k = \begin{bmatrix} \frac{\partial h_1}{\partial x_1} & \frac{\partial h_1}{\partial x_2}  & \dots &
   \frac{\partial h_1}{\partial x_n}  \\[8pt]
   \frac{\partial h_2}{\partial x_1} & \frac{\partial h_2}{\partial x_2}  & \dots &
   \frac{\partial h_2}{\partial x_n}  \\[8pt] \vdots & \vdots & \vdots \\[8pt]
   \frac{\partial h_m}{\partial x_1} & \frac{\partial h_m}{\partial x_2}  & \dots &
   \frac{\partial h_m}{\partial x_n}  \end{bmatrix}

How to model the noise? The noise in the controls is the input and it
drives the process noise. We assume here that we are going to gain all
of our noise from the control noise and develop the model. We first
assume that the control noise is drawn from a zero mean normal
distribution with a covariance matrix :math:`R_k`: :math:`N(0,R_k)`. We
also assume that the process noise depends on control noise:
:math:`f(u_k + N(0,R_k) ,x_k)` . The details are outside the scope of
this text, but we have that a change of coordinates can relate the
resulting process noise :math:`V_k` to control noise :math:`R_k`. The
transformation that relates the noise term :math:`V_k` to the covariance
:math:`R_k` is

.. math:: V_k = G_k R_k G_k^T

where :math:`G_k` is the Jacobian of :math:`g_k` with respect to the
control variables.

.. math::

   \displaystyle G_k =
     \begin{bmatrix} \frac{\partial g_1}{\partial u_1} & \frac{\partial g_1}{\partial u_2}  & \dots &
   \frac{\partial g_1}{\partial u_p}  \\[8pt]
   \frac{\partial g_2}{\partial u_1} & \frac{\partial g_2}{\partial u_2}  & \dots &
   \frac{\partial g_2}{\partial u_p}  \\[8pt] \vdots & \vdots & \vdots \\[8pt]
   \frac{\partial g_n}{\partial u_1} & \frac{\partial g_n}{\partial u_2}  & \dots &
   \frac{\partial g_n}{\partial u_p}  \end{bmatrix}

**Example with the DD model:** The linearization of :math:`g` with
respect to the control:

.. math::

   G =  \frac{\partial g}{\partial u_{k}}=
     \left( \begin{array}{cc}\displaystyle\frac{r\Delta
         t}{2}\cos\theta_k& \displaystyle\frac{r\Delta
       t}{2}\cos\theta_k\\[8pt]\displaystyle\frac{r\Delta
       t}{2}\sin\theta_k& \displaystyle\frac{r\Delta
       t}{2}\sin\theta_k \\[8pt]
     \displaystyle\frac{r\Delta
       t}{2L}& -\displaystyle\frac{r\Delta
       t}{2L} \end{array}\right)

We can map the control noise into process space via

.. math::

   V_k = 
   \begin{pmatrix}
   \displaystyle\frac{r\Delta t}{2}\cos\theta_k& \displaystyle\frac{r\Delta t}{2}\cos\theta_k\\[8pt]
   \displaystyle\frac{r\Delta t}{2}\sin\theta_k& \displaystyle\frac{r\Delta  t}{2}\sin\theta_k \\[8pt]
     \displaystyle\frac{r\Delta t}{2L}& -\displaystyle\frac{r\Delta t}{2L} 
     \end{pmatrix}
   \begin{pmatrix}
   \sigma_1^2 & 0 \\[8pt]
   0 & \sigma_2^2   
   \end{pmatrix}
   \begin{pmatrix}
   \displaystyle\frac{r\Delta t}{2}\cos\theta_k & \displaystyle\frac{r\Delta t}{2}\sin\theta_k & \displaystyle\frac{r\Delta t}{2L} \\[8pt]
   \displaystyle \frac{r\Delta t}{2}\cos\theta_k &\displaystyle \frac{r\Delta  t}{2}\sin\theta_k & -\displaystyle\frac{r\Delta t}{2L} 
   \end{pmatrix}

.. raw:: latex

   \FloatBarrier

Problems
--------

.. raw:: latex

   \setcounter{Exc}{0}

]

Basic Kalman Filter. Let

.. math:: x = \begin{bmatrix}x_1 \\ x_2\end{bmatrix}, \quad F = \begin{bmatrix} 0 &0.1 \\-0.02 &0.2\end{bmatrix}, \quad G_k u_k= \begin{bmatrix} 0\\ 2*\sin(k/25)\end{bmatrix},

\ 

.. math::

   H = \begin{bmatrix} 1& 0 \end{bmatrix}, 
   \quad V = \begin{bmatrix} 0.05^2&0\\0.& 0.05^2\end{bmatrix}, \quad W = 0.25^2,

.. math:: x(0) = \begin{bmatrix} 0.025\\0.1\end{bmatrix}, \quad P(0) = \begin{bmatrix}0 & 0\\ 0&0\end{bmatrix}.

 Apply the Kalman Filter process to compute 100 iterations and plot
them. Hint: run the simulation to create your observation data :math:`z`
and then run your Kalman Filter.

Assume that one has three different measurements for the location of
some object. The three measurements with the covariances are

.. math::

   (10.5, 18.2), \quad \left(\begin{array}{cc} 0.1 & 0.01 \\ 0.01 & 0.15
     \end{array}\right); \quad 
   (10.75, 18.0), \quad \left(\begin{array}{cc} 0.05 & 0.005 \\ 0.005 & 0.05
       \end{array}\right);

\ 

.. math::

   (9.9, 19.1), \quad \left(\begin{array}{cc} 0.2 & 0.05 \\ 0.05 & 0.25
   \end{array}\right).

Fuse this data into one measurement and provide an estimate of the
covariance.

Run a simulation on

.. math:: \begin{array}{l}\dot{x} = y \\\dot{y} = -\cos(x) + 0.5\sin(t)\end{array}

\ adding noise to the :math:`x` and :math:`y` components (with variance
= 0.2 on each). Let :math:`\Delta t = 0.1`. Assume that you can observe
the first variable, :math:`x`, with variance :math:`0.25`. Record the
observations. Write a program to run the EKF on the observed data and
compare the state estimate to the original values.

Differential Drive - EKF. The motion equations for a differential drive
robot are given below. Assume that the wheels are 5cm in radius and the
wheelbase is 12cm. Recall that the kinematics for this is (r = radius, L
= wheelbase):

.. math::

   \begin{array}{l}
    \dot{x} = \frac{r}{2} (\dot{\phi_1}+\dot{\phi_2})\cos(\theta) \\[5mm]
   \dot{y} = \frac{r}{2} (\dot{\phi_1}+\dot{\phi_2})\sin(\theta) \\[5mm]
   \dot{\theta} = \frac{r}{2L} (\dot{\phi_1}-\dot{\phi_2})
   \end{array}

\ The covariance of the state transition is :math:`V`. Assume that you
have a local GPS system that gives :math:`(x,y)` data subject to
Gaussian noise with covariance :math:`W`. The units on the noise are
given in cm. If you want to use meters then you will need to divide your
noise by 100.

#. Starting at :math:`t=0`, :math:`x=0`, :math:`y=0`, :math:`\theta=0`,
   predict location when wheel velocities are:

   ::

               t=0 -> 5:  omega1 = omega2 = 3 (rads/time), 
               t=5 -> 6:  omega1 = - omega2 = 1,
               t=6 -> 10: omega1 = omega2 = 3,
               t=10 -> 11:  - omega1 = omega2 = 1,
               t=11 -> 16: omega1 =  omega2 = 3,
               

   assuming that you have Gaussian noise on the omegas per time step.
   Select :math:`\Delta t = 0.2` (time increment). Test with a standard
   deviation of 0.1 on both wheels.

#. Write out the formulas for the Extended Kalman Filter.

#. Apply an Extended Kalman filter to the motion simulation above to
   track the location of the vehicle. Observations can be simulated by
   using previous simulation data as actual data, i.e. use this as the
   observed data (:math:`z_k`). Parameters:

   .. math:: x_{0|0} = (0,0,0), \quad V = \begin{bmatrix}.05 &  .02 & 0.01\\.02& .05& 0.01\\ 0.01& 0.01& .1\end{bmatrix},

   \ 

   .. math:: W= \begin{bmatrix} .08& .02 \\.02&  .07\end{bmatrix}, \quad P_{0|0} = \begin{bmatrix}2 &0& 0\\0 &1& 0\\0& 0& 0.5\end{bmatrix}.

#. Output the x-y locations on a 0.5 sec grid and compare in a plot.

#. The covariance matrix P gives the uncertainly ellipse for the
   location of the robot. Plot 5 ellipses along the path. This ellipse
   has major and minor axes given by the eigenvectors of P and the axes
   lengths are given by the associated eigenvalues. Matplotlib can plot
   an ellipse, `click
   here. <https://matplotlib.org/api/_as_gen/matplotlib.patches.Ellipse.html#matplotlib.patches.Ellipse>`__

Assume that you have a differential drive robot located in a lab with
special landmarks placed in various locations around the lab. Also
assume that your robot has a forward looking stereo vision system which
can determine the distance and angle off the forward direction (relative
to the robot) of the landmark. You don’t know the locations of the
landmarks and the stereo system can only see the landmarks if the angle
off of the front is less than 75\ :math:`^{\circ}`.

#. Write the equations for the apriori EKF step (:math:`f_k`) for some
   process noise covariance :math:`V_k`.

#. Assuming that the error of the angular measurement is 2 degrees in
   standard deviation - when you can observe the landmark, and the
   distance measurement error is 5 percent; what is the observation
   formula (:math:`h_k`) and the error :math:`W_k`?

#. What are the linearizations of :math:`f` and :math:`h`?

#. What are the aposteriori formulas? Don’t forget about the conversions
   from the robot (sensor) coordinates to the global or map coordinates.

#. Write out the EKF process to track the location of the robot and the
   discovered landmarks. You should assume that you start at (0,0,0).

| Graph the set :math:`z=Hx` where :math:`H = [1, -2]^T` and
  :math:`z=1`. Let :math:`\hat{x} = (3,2)` and
  :math:`\Delta x = H^T (HH^T)^{-1}(z - H\hat{x})`.
| Plot :math:`\hat{x}`, :math:`\Delta x` and :math:`\hat{x}+\Delta x`.
  Hint: If :math:`A = \begin{bmatrix}a & b \\ c & d\end{bmatrix}` then
  :math:`A^{-1} = \frac{1}{ad-bc}\begin{bmatrix}d & -b \\ -c & a\end{bmatrix}`

.. [1]
   Ignoring for the moment that this is a ridiculously expensive way to
   track velocity.

.. [2]
   The numbers were selected to help visualize the process.

.. |image| image:: math/quadpts
.. |image| image:: math/quadgraph
.. |image| image:: math/quadpts
.. |image| image:: math/lscompare
.. |A. Plot of :math:`x_0`. B. Noisy observation of :math:`x_0`.| image:: math/scalarkalmandata1
.. |A. Plot of :math:`x_0`. B. Noisy observation of :math:`x_0`.| image:: math/scalarkalmandata2
.. |A. Kalman estimate of :math:`x_0`. B. Comparison of state estimate to real state.| image:: math/scalarkalmandata3
.. |A. Kalman estimate of :math:`x_0`. B. Comparison of state estimate to real state.| image:: math/scalarkalmandata4
.. |image| image:: math/kalmanexample2.pdf
.. |image| image:: math/kalmanexample2b.pdf

