The Kalman Filter
------------------

In this section we develop the multivariate version of the Kalman Filter.
The steps are basically the same as the scalar version described in the
last section, but the derivation is more involved.

The Multivariate Version
~~~~~~~~~~~~~~~~~~~~~~~~~~

The :index:`Kalman Filter` has two stages. A predictive step based on the system
dynamics and an update based on observations or measurements.

The full Kalman Filter has the following objects to track:

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
   :label: kalmanderivation1

    x_{k+1} = F_k x_k + Gu_k + v_k

where :math:`F_k` is the state transition matrix, :math:`Gu_k` is the
input control and and observation

.. math::
   :label:  kalmanderivation2

     z_k = Hx_k + w_k

where :math:`H` is the observation matrix. The random variables
:math:`v_k`, :math:`w_k` are drawn from Gaussian distributions with
covariance models given by

.. math:: V = E[v_kv_k^T], \quad\quad W = E[w_kw_k^T].

The error covariance of the estimate is

.. math::
   :label: kalmanderivation3

    P_k = E[e_ke_k^T] = E[(x_k - \hat{x}_k)(x_k - \hat{x}_k)^T] .

The state estimate will be denoted :math:`\hat{x}_k` and the process
update to the state is denoted :math:`\tilde{x}_k`

Before we go into the details on the filter design, a couple of comments
about the matrices given in the dynamical process.

   The matrix :math:`F` is given by the model of the physical process.  It
   is a square matrix with dimension :math:`n \times n` where :math:`n` is the
   number of state variables (the length of :math:`x`).    When you are
   given a continuous dynamical system, make sure you first discretize the
   problem.  Only then can you extract the correct matrix :math:`F`.

   The matrix :math:`G` is more of a placeholder for now.  We assume that
   we have some type of control input :math:`Gu_k` but for our discussion
   you don't need to write this in any special form as long as you add the
   control values into the process update.  Meaning you don't need to figure
   out matrix :math:`G` to do the process update step.

   The matrix :math:`H` is the observation matrix.  This acts to relate the
   observed variables to the state variables.  For example, say that you have
   a state vector of :math:`(x_1, x_2, x_3)` and can observe all three as
   :math:`z = (z_{x_1}, z_{x_2}, z_{x_3})`.  Then

   .. math::  H = \begin{bmatrix} 1 & 0 & 0 \\ 0 & 1 & 0\\ 0 & 0 &1 \end{bmatrix}.

   However, if we observe :math:`x_1` and :math:`x_3` as  :math:`z = (z_{x_1}, z_{x_3})` then

   .. math::  H = \begin{bmatrix} 1 & 0 & 0 \\ 0 & 0 &1 \end{bmatrix}

   or if we only observe :math:`x_2` as  :math:`z = (z_{x_2})`  then

   .. math::  H = \begin{bmatrix} 0 & 1 & 0  \end{bmatrix}

   Note what the matrix :math:`H` does in the following product :math:`H A H^T` for
   the observation :math:`z = (z_{x_1}, z_{x_3})`:

   .. math::

      H A H^T = \begin{bmatrix} 1 & 0 & 0 \\ 0 & 0 &1 \end{bmatrix}
      \begin{bmatrix} a & b & c \\ d & e & f\\ g & h &i \end{bmatrix}
      \begin{bmatrix} 1 & 0 \\ 0 & 0  \\ 0 & 1 \end{bmatrix}
      =
      \begin{bmatrix} 1 & 0 & 0 \\ 0 & 0 &1 \end{bmatrix}
      \begin{bmatrix} a & c \\ d  & f\\ g &i \end{bmatrix}
      =
      \begin{bmatrix} a & c \\ g &i \end{bmatrix}



Moving on to the derivation, we assume that we can write our estimate as a combination of
the process update and the observation

.. math::  \hat{x}_k = \tilde{x}_k + K_k (z_k - H\tilde{x}_k)
   :label: kalmanderivation4

The optimal choice of the Kalman gain parameter is to select :math:`K_k`
to minimize the mean square error
:math:`E[ \| x_k - \hat{x}_{k|k} \|^2 ]`. You will notice that

.. math::

   E[ \| x_k - \hat{x}_{k|k} \| ] = E \left[ \sum_i (x^i_{k}- \hat{x}^i_{k|k})^2\right]
    = Tr(P_{k|k})

where :math:`Tr(P_{k|k})` is the trace of :math:`P_{k|k}`. So, we need
an expression for :math:`P_{k|k}` in terms of the Kalman gain.

We can plug in the observation,
:eq:`kalmanderivation1` into :eq:`kalmanderivation4`

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

We can collect the results into the following algorithm:

**Kalman Filter**

**Predict:** Prediction or a priori stage

-  Predicted state:
   :math:`\hat{x}_{k|k-1} = F_{k}\hat{x}_{k-1|k-1} + G_{k} u_{k}`

-  Predicted estimate covariance:
   :math:`P_{k|k-1} = F_{k} P_{k-1|k-1} F_{k}^{T} + V_{k}`

**Update:** Update or a posteriori stage

-  :index:`Innovation residual` or :index:`measurement residual`:
   :math:`y_k = z_k - H_k\hat{x}_{k|k-1}`

-  Innovation (or residual) covariance: :math:`S_k = H_k P_{k|k-1} H_k^\text{T} + W_k`

-  :index:`Optimal Kalman gain`: :math:`K_k = P_{k|k-1}H_k^\text{T}S_k^{-1}`

-  Updated state estimate
   :math:`\hat{x}_{k|k} =\hat{x}_{k|k-1} + K_k y_k`

-  Updated estimate covariance: :math:`P_{k|k} = (I - K_k H_k) P_{k|k-1}`

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


Assume that you have the following Gaussian process and observation:

.. math::

   \begin{array}{l}
   x_k = Fx_{k-1} + Gu_k + v_k\\
   z_k = Hx_k + w_k
   \end{array}


then


.. _kalmanfilteralg:
.. topic::  Kalman Algorithm

   | **Input** :math:`x_0`, :math:`P_0`
   | **Output** Estimates of :math:`x_k`, :math:`P_k`
   | :math:`k=0`
   | **while** (not terminated) **do**
   |    :math:`k=k+1`
   |    :math:`x_k = F_{k}x_{k-1} + G_{k} u_{k}`
   |    :math:`P_{k} = F_{k} P_{k-1} F_{k}^{T} + V_{k}`
   |    :math:`y_k = z_k - H_kx_{k}`
   |    :math:`S_k = H_k P_{k} H_k^\text{T} + W_k`
   |    :math:`K_k = P_{k}H_k^\text{T}S_k^{-1}`
   |    :math:`x_k =   x_{k} + K_k y_k`
   |    :math:`P_{k} = (I - K_k H_k) P_{k}`
   | **end while**


.. figure:: AdvFilteringFigures/pointmapcloud.*
   :width: 50%
   :align: center

   Single Step of Kalman process.

The Kalman code generally looks like

.. code:: julia

   for k = 2:N
     x_process_update = F * x_estimate[k-1] + G[k]
     P_variance_update = F * (P_variance[k-1] * FT) + V
     innovation = z_observation[k] - H * x_process_update
     Innovation_covariance = H * (P_variance_update * HT) + W
     Kal_gain = P_variance_update * (HT * inv(Innovation_covariance) )
     x_estimate[k] = x_process_update + Kal_gain * y
     P_variance[k] = P_variance_update - Kal_gain * (H * P_variance_update )
   end


Simple Example of a Single Step
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Let

.. math:: x = \begin{bmatrix}a \\ b\end{bmatrix}, \quad F = \begin{bmatrix} 0.9 &-.01 \\0.02 &0.75\end{bmatrix},
   \quad G = \begin{bmatrix} 0.1\\ 0.05\end{bmatrix}, \quad H = \begin{bmatrix} 1& 0 \end{bmatrix},


.. math:: V = \begin{bmatrix} 0.005265&0\\0& 0.005265\end{bmatrix}, \quad W = 0.7225,\quad z_1 = 0.01

.. math:: \quad u_k = \sin (7*k/100), \quad x_0 = \begin{bmatrix} 0\\0\end{bmatrix},
   \quad P_0 = \begin{bmatrix}0 & 0\\ 0&0\end{bmatrix}.

Apply the Kalman Filter process and compute :math:`\hat{x}_{1|1}` and
:math:`P_{1|1}`.

Process update:

.. math::

   \hat{x}_{1|0} = \begin{bmatrix} 0.9 &-.01 \\0.02 &0.75\end{bmatrix}\hat{x}_{0|0}
   + \begin{bmatrix} 0.1\\ 0.05\end{bmatrix} u_k
   =  \begin{bmatrix} 0.9 &-.01 \\0.02 &0.75\end{bmatrix}\begin{bmatrix} 0\\0\end{bmatrix}
   + \begin{bmatrix} 0.1\\ 0.05\end{bmatrix}\sin (7/100)


.. math:: \approx \begin{bmatrix} 0.0069942847\\  0.0034971424\end{bmatrix}

Process covariance update:

.. math:: P_{1|0} = F P_{0|0} F^{T} + V =

.. math:: P_{1|0} = \begin{bmatrix} 0.9 &-.01 \\0.02 &0.75\end{bmatrix}\begin{bmatrix}0 & 0\\ 0&0\end{bmatrix} \begin{bmatrix} 0.9 &0.02 \\ -.01&0.75\end{bmatrix} +\begin{bmatrix} 0.005265&0\\0& 0.005265\end{bmatrix}

.. math:: = \begin{bmatrix} 0.005265&0\\0& 0.005265\end{bmatrix}.

Innovation and innovation covariance:

.. math:: y_1 = 0.01 - \begin{bmatrix} 1& 0 \end{bmatrix}\hat{x}_{1|0} = 0.01 - \begin{bmatrix} 1& 0 \end{bmatrix}\begin{bmatrix} 0.0069942847\\  0.0034971424\end{bmatrix}


.. math:: = 0.0030057153

.. math:: S_1 = HP_{1|0} H^\text{T} + W = \begin{bmatrix} 1 & 0\end{bmatrix} \begin{bmatrix} 0.005265&0\\0& 0.005265\end{bmatrix}\begin{bmatrix} 1\\0\end{bmatrix} + 0.7225


.. math:: =0.728125

Kalman Gain

.. math::

   K_1 = P_{1|0}H_1^\text{T}S_1^{-1} = \begin{bmatrix} 0.005265&0\\0& 0.005265\end{bmatrix}
   \begin{bmatrix} 1\\0\end{bmatrix}/0.728125


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


.. math::

   = \begin{bmatrix} 0.005224326  &  0.0 \\
   0.0  &  0.005265 \end{bmatrix}

It is useful to visualize the effects of a single Kalman step. The
images are provided in
:numref:`fig:kalmanclouds1` -  :numref:`fig:kalmanclouds3`
and the numbers used are not the same as the example above [#f2]_. The
system we use is Let

.. math:: x_0 = \begin{bmatrix} 1\\1\end{bmatrix}, \quad P_0 = \begin{bmatrix}0.01& 0\\ 0&0.001\end{bmatrix}, \quad F = \begin{bmatrix} 0.85 &-.1 \\0.02 &0.75\end{bmatrix},



.. math::

   G = \begin{bmatrix} 0.025\\ 0.05\end{bmatrix}, \quad H = I,
    V = \begin{bmatrix} 0.0075^2&0\\0& 0.0075^2\end{bmatrix},

.. math:: W = \begin{bmatrix} 0.035^2&0\\0& 0.035^2\end{bmatrix}, \quad  a = \begin{bmatrix} 0.01\\ 0.02\end{bmatrix} ,\quad z = \hat{x}  +a+ w_k.


.. figure:: AdvFilteringFigures/kalmanupdatedia.*
   :width: 65%
   :align: center

   Parts of the single Kalman step - estimate.


.. figure:: AdvFilteringFigures/kalmanupdatedia2.*
   :width: 65%
   :align: center

   Parts of the single Kalman step - covariances.


Starting with a single point, we move this forward using the process
update. From the same starting point we run each forward with the
process update, :math:`\hat{x}_{k|k-1}` many times to generate a
distribution. The resulting points are different since the process
update has noise.
:numref:`fig:kalmanclouds1` shows the
point cloud (in blue). This process does not have a great deal of noise
so the cloud is tightly clustered.
:numref:`fig:kalmanclouds2` shows the
observation :math:`z_k`.
:numref:`fig:kalmanclouds3` shows the
observation update, the fusion of the observation with the state update.

.. code:: julia

   using Plots, Random, Distributions
   M = 250
   F = [0.85 -0.1 ; 0.02 0.75]
   FT = transpose(F)
   a = [0.01 ; 0.02]
   G = [0.025,0.05]
   x_initial = [1 ; 1]
   P_initial = [0.01 0.0 ; 0.0 0.001]
   mu1, mu2 = 0.0, 0.0
   sigma1, sigma2 = 0.0075, 0.035
   var1 = sigma1*sigma1
   var2 = sigma2*sigma2
   r1 = Normal(mu1,sigma1)
   r2 = Normal(mu2,sigma2)
   V = [var1 0.0 ; 0.0 var1]
   W = [var2 0.0 ; 0.0 var2]
   x_apriori = Vector{Float64}()
   y_apriori = Vector{Float64}()
   obsx = Vector{Float64}()
   obsy= Vector{Float64}()
   x_post = Vector{Float64}()
   y_post = Vector{Float64}()


   for i = 1:M
     x_process_update = F*x_initial  + G + rand(r1, 2)
     append!(x_apriori,x_process_update[1])
     append!(y_apriori,x_process_update[2])
     P_variance_update = F*P_initial*FT + V
     z_test_data = F*x_initial  + G  + a + rand(r2,2)
     append!(obsx,z_test_data[1])
     append!(obsy,z_test_data[2])

     innovation = z_test_data - x_process_update
     Innovation_variance = P_variance_update + W

     kal_gain = P_variance_update*inv(Innovation_variance)
     x_filter = x_process_update + kal_gain*innovation

     append!(x_post,x_filter[1])
     append!(y_post,x_filter[2])

   end

   scatter(x_apriori,y_apriori, xlim=(0.7,0.95), ylim=(0.7, 0.95), aspect_ratio = 1.0)
   scatter!(obsx,obsy, xlim=(0.7,0.95), ylim=(0.7, 0.95), aspect_ratio = 1.0)
   scatter!(x_post,y_post, xlim=(0.7,0.95), ylim=(0.7, 0.95), aspect_ratio = 1.0)


You will notice that it is not circular. The covariance matrix really
trusted the :math:`y` process estimate and so weighted the process more
than the observation. In the :math:`x` estimate, much more of the
observation was used. So the resulting point cloud has lower variation
in :math:`y` than :math:`x`.
:numref:`fig:kalmanclouds4` graphs the
error ellipses for the previous point clouds. It is easier to see the
changes from this than looking at the raw data.

.. _`fig:kalmanclouds1`:
.. figure:: AdvFilteringFigures/cloud1.*
   :width: 50%
   :align: center

   Point distribution after process update.

.. _`fig:kalmanclouds2`:
.. figure:: AdvFilteringFigures/cloud2.*
   :width: 50%
   :align: center

   Observed point distribution.

.. _`fig:kalmanclouds3`:
.. figure:: AdvFilteringFigures/cloud3.*
   :width: 50%
   :align: center

   Final distribution after update step.

.. _`fig:kalmanclouds4`:
.. figure:: AdvFilteringFigures/cloud4.*
   :width: 50%
   :align: center


   The standard deviation based ellipses.

Generation of Testing Data
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. figure:: AdvFilteringFigures/kalmanblock.*
   :width: 50%
   :align: center

   Kalman Code as a black box.


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

.. math:: z_{k+1} = Hx_k + w_k

.. figure:: AdvFilteringFigures/KalmanSimulationBlock.*
   :width: 95%
   :align: center

   Simulation and testing.


The point is that the observations :math:`z` can be computed after we compute the :math:`x`
values or they can be computed together in the loop.  It does not matter in this
case.

Kalman Code Examples
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


For this next example we modify the last example in a couple of ways.
We will observe both variables.  This will have the effect of making
the innovation covariance :math:`S` a matrix and we will need to compute
a matrix inverse.  Next we will use a non-diagonal noise covariance for
:math:`V` and :math:`W`.

We use these values to run a simulation which then produces the observations
we need to feed into the Kalman filter.
The code block below will generate a list of values which can
be used as the observations for a run of a Kalman filtering algorithm.
Let

.. math::

   x = \begin{bmatrix}a \\ b\end{bmatrix}, \quad
   F = \begin{bmatrix} 0.85 &-.01 \\0.02 &0.65\end{bmatrix}, \quad
   H = \begin{bmatrix} 1& 0 \\ 0 & 1 \end{bmatrix},


.. math::

   V = \begin{bmatrix} 0.2 & 0.02 \\ 0.02 & 0.35 \end{bmatrix}, \quad
   W = \begin{bmatrix}  0.4 & 0.0 \\ 0.0 & 0.4  \end{bmatrix} .

.. math::

   \quad Gu_k = \begin{bmatrix}0.75\sin(0.5t_k) \\ 0.5\cos(0.5t_k) \end{bmatrix}, \quad
   x_0 = \begin{bmatrix} 0\\0\end{bmatrix}, \quad
   P_0 = \begin{bmatrix}0 & 0\\ 0&0\end{bmatrix}.

Assume that you want to simulate this on :math:`0 \leq t \leq 10` with 200 subdivisions.  
The includes ...

.. code-block:: julia

        using Random, Distributions
        using Plots, LinearAlgebra


The simulation variables ...  (note :math:`t = 0.05*k`)


.. code-block:: julia

       #  Create fake dataset for experiment
       N = 200
       t = range(0, 10, length = N)  # for control input
       u1 = 0.75*sin.(0.5*t)
       u2 = 0.5*cos.(0.5*t)
       mu1 = [0.0, 0.0]
       mu2 = [0.0, 0.0]
       x_sim = zeros(2, N)
       z_sim = zeros(2, N)
       F = Float64[0.85 -0.01; 0.02 0.65]
       FT = transpose(F)
       G = transpose(Float64[u1 u2;])

The filter variables

.. code-block:: julia

       H = Float64[1 0; 0 1]
       HT = transpose(H)
       V = Float64[0.2 0.02; 0.02 0.35]
       W = Float64[0.4 0.0; 0.0 0.4]
       P = zeros(2,2)
       x_estimate = zeros(2,N)


In practice you will simplify some of the expressions before coding them up.  For example, the matrix :math:`H` is the identity matrix and you can just remove it.   It is left in so you can see the full process just like it appears in the original algorithm.  Also, if you cat/paste the code, you have it.

The simulation ...

.. code-block:: julia

       for k = 2:N
         process_noise = rand(MvNormal(mu1, V), 1)
         observation_noise = rand(MvNormal(mu2, W), 1)
         x_sim[:,k] = (F * x_sim[:, k-1]) + G[:,k] + process_noise
         z_sim[:,k] = (H * x_sim[:, k]) + observation_noise
        end
        # done with fake data

The code block above provides the array z which is then piped into the
Kalman Filter

.. code-block:: julia

   for k = 2:N
     x_process_update = F * x_estimate[:,k-1] + G[:,k]
     P_variance_update = F * (P * FT) + V
     innovation = z_sim[:,k] - H * x_process_update
     Innovation_covariance = H * (P_variance_update * HT) + W
     Kal_gain = P_variance_update * (HT * inv(Innovation_covariance))
     x_estimate[:,k] = x_process_update + Kal_gain * innovation
     P = P_variance_update - Kal_gain * (H * P_variance_update )
   end


.. code-block:: julia


   time = range(0, N, length=N)
   p1 = plot(time, x_sim[1,:], xlabel = "k", ylabel = "x1", legend = false)
   scatter!(time,z_sim[1,:])
   plot!(time,x_estimate[1,:])
   display(p1)
   savefig("kalmandemo2_x.svg")

   p2 = plot(time, x_sim[2,:], xlabel = "k", ylabel = "x2", legend = false)
   scatter!(time,z_sim[2,:])
   plot!(time,x_estimate[2,:])
   display(p2)
   savefig("kalmandemo2_y.svg")


The blue dots are a graph of :math:`(x_1)_k`, the red dots are the
observations :math:`z_k`, and the green dots are the Kalman estimate of
the state.

.. figure:: AdvFilteringFigures/kalmandemo2_x.*
   :width: 75%
   :align: center

The blue dots are a graph of :math:`(x_2)_k`, and the green dots are the
Kalman estimate of the state.

.. figure:: AdvFilteringFigures/kalmandemo2_y.*
   :width: 75%
   :align: center


What happens if we can only observe one state?  What if

.. math::

    H = \begin{bmatrix} 1& 0  \end{bmatrix}

In this example, we will also make the simplications in the code that you would do in practice.  For example, when you see :math:`Hx` you would just code up :math:`x_1`.

.. code-block:: julia

        using Random, Distributions
        using Plots, LinearAlgebra


The simulation variables ...


.. code-block:: julia

       #  Create fake dataset for experiment
       N = 200
       t = range(0, 10, length = N)  # for control input
       u1 = 0.75*sin.(0.5*t)
       u2 = 0.5*cos.(0.5*t)
       mu1 = [0.0, 0.0]
       mu2 = 0.0
       x_sim = zeros(2, N)
       z_sim = zeros(N)
       F = Float64[0.85 -0.01; 0.02 0.65]
       FT = transpose(F)
       G = transpose(Float64[u1 u2;])

The filter variables

.. code-block:: julia

       V = Float64[0.2 0.02; 0.02 0.35]
       W = 0.4
       P = zeros(2,2)
       x_estimate = zeros(2,N)


The simulation ...

.. code-block:: julia

       for k = 2:N
         process_noise = rand(MvNormal(mu1, V), 1)
         observation_noise = rand(Normal(mu2, W[1]))
         x_sim[:,k] = (F * x_sim[:, k-1]) + G[:,k] + process_noise
         z_sim[k] = x_sim[1, k] + observation_noise
        end
        # done with fake data

The code block above provides the array z which is then piped into the
Kalman Filter

.. code-block:: julia

   for k = 2:N
     x_process_update = F * x_estimate[:,k-1] + G[:,k]
     P_variance_update = F * (P * FT) + V
     innovation = z_sim[k] - x_process_update[1]
     Innovation_covariance = P_variance_update[1,1] + W
     Kal_gain = (1.0/Innovation_covariance) .* P_variance_update[:,1]
     x_estimate[:,k] = x_process_update + innovation .* Kal_gain
     P = P_variance_update - Kal_gain * transpose(P_variance_update[1,:] )
   end


.. code-block:: julia

    time = range(0, N, length=N)
    p1 = plot(time, x_sim[1,:], xlabel = "k", ylabel = "x1", legend = false)
    scatter!(time,z_sim)
    plot!(time,x_estimate[1,:])
    display(p1)
    savefig("kalmandemo3_x.svg")

    p2 = plot(time, x_sim[2,:], xlabel = "k", ylabel = "x2", legend = false)
    plot!(time,x_estimate[2,:])
    display(p2)
    savefig("kalmandemo3_y.svg")



The blue line is the graph of :math:`(x_1)_k`, the red dots are the
observations :math:`z_k`, and the green line is the Kalman estimate of
the state.

.. figure:: AdvFilteringFigures/kalmandemo3_x.*
   :width: 75%
   :align: center

The blue line is the graph of :math:`(x_2)_k`, and the red line are the
Kalman estimate of the state.

.. figure:: AdvFilteringFigures/kalmandemo3_y.*
   :width: 75%
   :align: center

Injecting noise
~~~~~~~~~~~~~~~~~~~~

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


.. code-block:: julia

        t = range(0,6.28,length=N)
        c = cos.(t)
        n = rand(Normal(0, 0.25), 200)
        a1 = c + n
        b1 = 20 * c + n
        a2 = n .* c
        b2 = 20 * n .* c
        a1sub = a1 - c
        b1sub = b1 - 20 * c
        a2sub = a2 - c
        b2sub = b2 - 20 * c
        std(a1sub)
        std(b1sub)
        std(a2sub)
        std(b2sub)

The output

.. code-block:: julia

   julia> std(a1sub)
   0.24006607075812747

   julia> std(b1sub)
   0.24006607075812744

   julia> std(a2sub)
   0.7324916532871306

   julia> std(b2sub)
   14.649833065742614

The multiplication by the signal will amplify the noise by the signal
strength and this changes the effective standard deviation. We will for
this text focus on adding noise via addition.  At this point, if you look at what we just did, it is actually pretty obvious.  In the additive case we have

.. math::

   (x + v) - x = v


One issue we will address
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

      = \begin{bmatrix}1 & 0.5 \\ 0 & 1\end{bmatrix}
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


The Kalman Gain
~~~~~~~~~~~~~~~~~~~

The Kalman Gain selects the amount of process update to be used compared to
the amount of observation to be used.   It is weighting each one to produce
the best possible estimate of state based on the current understanding of
the errors on both.


The Kalman Gain can be written as

.. math::
   K_k = P_{k}H_k^\text{T}\left( H_k P_{k} H_k^\text{T} + W_k \right)^{-1}.


If all of these variables were *scalars*, we can get a feel for the bounds on the
Kalman Gain:

.. math::
   K_k = P_{k}H_k / \left( H_k^2 P_{k} + W_k \right)

When :math:`W_k = 0` then :math:`K_k = 1/H_k` and as :math:`W_k \to \infty`
then :math:`K_k = 0`, so :math:`0 < K_k < 1/H_k`.

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
operational if the sensor is giving readings that don’t make sense
given the physical model, so the Kalman gain will move to where only the
physical model is used.

.. math:: K_k = P_kH_k^TS_k^{-1} \to 0

This can be a problem for sensors that have drift or some type of
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
fusion algorithm given in :numref:`multivariatesensorfusion`.

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


.. rubric:: Footnotes

.. [#f2] The numbers were selected to help visualize the process.
