Models, Dynamical Systems and Filters
---------------------------------------

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

.. figure:: AdvFilteringFigures/quadpts.*
   :width: 60%
   :align: center

 :math:`\Rightarrow`

.. figure:: AdvFilteringFigures/quadgraph.*
   :width: 60%
   :align: center



Using quadratic model, :math:`y = c_2x^2 + c_1x + c_0`, and least
squares we find :math:`y = 0.49x^2 - 1.21x + 1.42`. So we can use this
to extrapolate at :math:`x=5` we have :math:`y = 6.5`.

.. figure:: AdvFilteringFigures/quadpts.*
   :width: 60%
   :align: center

 :math:`\Rightarrow`

.. figure:: AdvFilteringFigures/lscompare.*
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

Assume that we had the following data: :math:`(t,x,y) =  (1, 10, 22), (2, 19, 60), (3, 32,
51)` and that :math:`F_x=0` and :math:`F_y = -2`, :math:`m=0.25`. So
first we gain: :math:`t = [1, 2, 3]`, :math:`\xi = [10, 19, 32]`,
:math:`\eta = [26, 76, 87]`. We first compute

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
   \begin{pmatrix} 1 & 2 &  3  \\ 1 & 1 & 1\end{pmatrix} \begin{pmatrix} 26 \\ 76 \\ 87\end{pmatrix} = \begin{pmatrix} 30.5 \\ 2.0\end{pmatrix}

So we have that the start location is :math:`(-1.666667, 2.0)` with
initial velocity of :math:`(11 , 30.5)`.

:index:`Linear Dynamical System`
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

An operator, :math:`L`, is said to be :index:`linear` if for scalars :math:`a,b`
and vectors :math:`x,y` we have

.. math:: L(ax+by) = aLx + bLy

A :index:`dynamical system`

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

Dynamics with Noise
~~~~~~~~~~~~~~~~~~~

Let :math:`x_k` be the current state and :math:`z_k` be the observation.
We study the linear system with noise:

.. math::

   \begin{array}{l}
   x_k = Fx_{k-1} + Gu_k + v_k\\
   z_k = Hx_k + w_k
   \end{array}

where :math:`v_k`, :math:`w_k` are assumed to be zero mean Gaussian
noise with covariance matrices :math:`V_k` and :math:`W_k` respectively.

We are
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


At the risk of being redundant, we need to address a common misunderstanding.
The state vector :math:`x` is not something that normally can be observed.
We would not need to do any type of filtering if we could observe it.  The
observation of :math:`x` is :math:`z`.  It differs from :math:`x` in two
primary manners.  First there is noise in the observation.  Meaning that
:math:`x` and :math:`z` differ by the added noise.  Second, we don't
observe all of the components of :math:`x`.  Some are missing.  This means
that the lengths of the vectors for :math:`x` and :math:`z` are often
different.

For the algorithms in the next couple of sections, we will be estimating
:math:`x` by using :math:`z`.   So :math:`z` is an input variable.  To develop
code, we need to test on actual data sets, so we will need to create
some fake :math:`z` to run our tests.  The creation of the :math:`z` data
is not part of any of the filters.  This is no different than when you
create unit tests.  They are essential to the development process, but not
part of the primary codebase.


:index:`Scalar Kalman Filter`
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

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
:math:`\sigma_w^2`. These are fused using :eq:`scalarrecursiveweighted` into

.. math:: S_k = \frac{1}{\tilde{\sigma}_k^2} + \frac{1}{{\sigma}_w^2}

.. math::

   K_{k} = \displaystyle \left[ S_{k}\sigma_{w}^2\right]^{-1} =  \left[ {\sigma}_{w}^2 \left(\frac{1}{\tilde{\sigma}_k^2} + \frac{1}{\sigma_w^2}\right) \right]^{-1}
   =  \left[ {\sigma}_{w}^2 \left(\frac{\tilde{\sigma}_k^2 + \sigma_w^2}{\tilde{\sigma}_k^2  \sigma_w^2}\right) \right]^{-1}


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

In the case where the state vector to be estimated is a scalar, the derivation
is much easier and sets the stage for the multivariate version shown in the
next section.   


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
pretend to run the dynamical system and get the observations.  Later on
in the multivariate content, much greater detail is given to the creation
of noisy data.  For now, focus on the filter aspect and not on the creation
of :math:`z`.

::

    N = 100
    mu1, sigma1 = 0.0, 0.2
    mu2, sigma2 = 0.0, 0.75
    process_noise = np.random.normal(mu1,sigma1, N)
    observation_noise = np.random.normal(mu2,sigma2, N)
    x_sim = np.zeros(N)
    z_sim = np.zeros(N)
    u = np.arange(N)
    k = 1
    while (k<N):
      x_sim[k] = x_sim[k-1] + 0.5*(N-1.75*u[k])/N + process_noise[k-1]
      z_sim[k] = x_sim[k] + observation_noise[k-1]
      k = k+1

.. figure:: AdvFilteringFigures/scalarkalmandata1.*
   :width: 50%
   :align: center

   Plot of :math:`x_0`.

.. figure:: AdvFilteringFigures/scalarkalmandata2.*
   :width: 50%
   :align: center

   Noisy observation of :math:`x_0`.


Using the fake observations, we can test the filter.

::

    x_filtered = np.zeros(N)
    covariance_filtered = np.zeros(N)
    k = 1
    while (k<N):
      x_process_update = x_filtered[k-1] + 0.5*(N-1.75*u[k])/N
      variance_update = pf[k-1] + sigma1*sigma1
      kal_gain = variance_update/(variance_update + sigma2*sigma2)
      x_filtered[k] = x_process_update + kal_gain*(z_sim[k-1] - x_process_update)
      covariance_filtered[k] = (1-kal_gain)*variance_update
      k = k+1


.. figure:: AdvFilteringFigures/scalarkalmandata3.*
   :width: 50%
   :align: center

   Kalman estimate of :math:`x_0`.

.. figure:: AdvFilteringFigures/scalarkalmandata4.*
   :width: 50%
   :align: center

   Comparison of state estimate to
   real state.



.. rubric:: Footnotes

.. [#f1] Ignoring for the moment that this is a ridiculously expensive way to track velocity.
