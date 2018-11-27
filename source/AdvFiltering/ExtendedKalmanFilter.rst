:index:`Extended Kalman Filter`
--------------------------------

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

   F_k = \partial f / \partial x = \displaystyle
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

This is a :index:`Taylor expansion` approach to dealing with nonlinear mappings.
For more information about :index:`Jacobians`,
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

Summed up into a procedure, we have:

.. _extkalmanfilteralg:
.. topic::  EFK Algorithm

   | **Input** :math:`x_0`, :math:`P_0`
   | **Output** Estimates of :math:`x_k`, :math:`P_k`
   | :math:`k=0`
   | **while** (not terminated) **do**
   |    :math:`k=k+1`
   |    :math:`x_k = f(\hat{x}_{k-1|k-1}, u_{k})`
   |    :math:`F_k =  \mbox{Jacobian}(f)|_{x_k}`
   |    :math:`P_{k} = F_{k} P_{k-1} F_{k}^{T} + V_{k}`
   |    :math:`H_k = \mbox{Jacobian}(h)|_{x_k}`
   |    :math:`y_k = z_k - H_k x_{k}`
   |    :math:`S_k = H_k P_{k} H_k^\text{T} + W_k`
   |    :math:`K_k = P_{k}H_k^\text{T}S_k^{-1}`
   |    :math:`x_k =   x_{k} + K_k y_k`
   |    :math:`P_{k} = (I - K_k H_k) P_{k}`
   | **end while**


Short Example
~~~~~~~~~~~~~

What is the Extended Kalman Filter formulation of the motion model:

.. math:: \begin{array}{l}\dot{x} = y \\\dot{y} = -\cos(x) + 0.4\sin(t)\end{array},

and observation

.. math:: h(x,y) = \begin{bmatrix}x \\ y\end{bmatrix}

with step size :math:`\Delta t = 0.1,` and noise

.. math:: V = \begin{bmatrix} 0.1&0.01\\0.01& 0.1\end{bmatrix}, \quad , W = \begin{bmatrix} 0.05&0\\0& 0.05\end{bmatrix}.

Assume that the initial values, :math:`t_0 = 0`,

.. math:: x_0 = \begin{pmatrix} 1 \\ 1 \end{pmatrix}

and we have a reasonably accurate start

.. math:: P_0 = \begin{pmatrix} 0.5 & 0 \\ 0 & 0.5   \end{pmatrix}

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

In Terms Chapter, we derived the equations
for the motion of the differential drive robot. In that chapter we also
simulated the motion of the robot based on wheel velocity data. Small
amounts of noise in the wheel velocity data could cause significant
errors in position estimation. Using the Extended Kalman Filter, we can
improve the location estimate as well as gain estimates for the
uncertainty of the location. :numref:`Fig:DDagain`
recalls the variables and equations that were derived.

.. _`Fig:DDagain`:
.. figure:: AdvFilteringFigures/dddim.*
   :width: 20%
   :align: center

   The variables used in the DD model.

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

.. math::

   t_k \equiv k\Delta t, \quad t_{k+1} = (k+1)\Delta t

.. math::

   x_k \equiv x(t_k), \quad y_k \equiv y(t_k)

.. math::

   \omega_{1, k}\equiv \dot{\phi}_{1}(t_k),  \quad
   \omega_{2, k}\equiv \dot{\phi}_{2}(t_k)

The discrete approximations to the differential drive equations are:

.. math::

   \begin{array}{l}
   \displaystyle x_{k+1} = x_k + \frac{r\Delta t}{2} (\omega_{1, k}+\omega_{2, k})\cos(\theta_k) \\[2mm]
   \displaystyle y_{k+1} = y_k + \frac{r\Delta t}{2} (\omega_{1, k}+\omega_{2, k})\sin(\theta_k) \\[2mm]
   \displaystyle \theta_{k+1} = \theta_k + \frac{r\Delta t}{2L} (\omega_{1, k}-\omega_{2, k})
   \end{array}

The next step is to linearize the process dynamics. This means that we
must compute the matrix :math:`F` from the nonlinear model :math:`f`.

.. math::

   x_k = \begin{bmatrix} x_k \\ y_k \\ \theta_k \end{bmatrix}, \quad
   u_k = \begin{bmatrix} \omega_{1, k} \\ \omega_{2, k}\end{bmatrix},

.. math::

   f(x_k,u_k) = \begin{bmatrix}
                  x_k + \frac{r\Delta t}{2} (\omega_{1, k}+\omega_{2, k})\cos(\theta_k) \\[3mm]
   y_k + \frac{r\Delta t}{2} (\omega_{1, k}+\omega_{2, k})\sin(\theta_k) \\[3mm]
   \theta_k + \frac{r\Delta t}{2L} (\omega_{1, k}-\omega_{2, k})
                \end{bmatrix}

.. math::

   \displaystyle F_k =
   \begin{bmatrix} \displaystyle  \frac{\partial f_1}{\partial x_1} & \displaystyle  \frac{\partial f_1}{\partial x_2}  &
   \displaystyle \frac{\partial f_1}{\partial x_3}  \\[5pt]
   \displaystyle  \frac{\partial f_2}{\partial x_1} & \displaystyle \frac{\partial f_2}{\partial x_2}  &
   \displaystyle \frac{\partial f_2}{\partial x_3}  \\[5pt]
   \displaystyle  \frac{\partial f_3}{\partial x_1} & \displaystyle \frac{\partial f_3}{\partial x_2}  &
   \displaystyle \frac{\partial f_3}{\partial x_3}  \end{bmatrix}
   \displaystyle  = \begin{bmatrix} 1 & 0  &
   -\frac{r\Delta t}{2} (\omega_{1, k}+\omega_{2, k})\sin(\theta_k)  \\[5pt]
   0 & 1  &
   \frac{r\Delta t}{2} (\omega_{1, k}+\omega_{2, k})\cos(\theta_k)  \\[5pt]
   0 & 0  & 1  \end{bmatrix}

Assume that you start the robot with pose :math:`[0,0,0]` and you know
this is exact so

.. math:: P_{0|0} = \begin{bmatrix} 0 & 0 & 0\\ 0 & 0 & 0 \\ 0 & 0 & 0 \end{bmatrix}.

Let the process noise and measurement noise covariances be

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

Note that the steps above ONLY apply to when you can observe all three
variables making :math:`H` the identity matrix.

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

so ...

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

So,

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

EKF Python Example
~~~~~~~~~~~~~~~~~~~~

We will take a similar setup as before, with a few values modified, and
generate the Python code required. For this simulation, we place the
noise only in the process equations and the observation. It is also
reasonable to consider placing the noise in the control inputs as well.
Assume that you start the robot with pose :math:`[0,0,0]` and you know
this is exact so

.. math:: P_{0|0} = \begin{bmatrix} 0 & 0 & 0\\ 0 & 0 & 0 \\ 0 & 0 & 0 \end{bmatrix}.

Let the process noise and measurement noise covariances be

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



.. figure:: AdvFilteringFigures/extendedkalmanfilter1.*
   :width: 60%
   :align: center

   The Extended Kalman Filter applied to the motion of a differential
   drive robot. Domain axis is time and vertical axis are the state
   variables. The simulation pose is given by the blue line, the
   observation of the pose given by the red dots and the pose estimate
   is given by the green dots.


.. figure:: AdvFilteringFigures/extendedkalmanfilter2.*
   :width: 60%
   :align: center

   The Extended Kalman Filter applied to the motion of a differential
   drive robot. This figure plots the :math:`y` state variable against
   the :math:`x` state variable with :math:`\theta` ignored. The
   simulation pose is given by the blue line, the observation of the
   pose given by the red dots and the pose estimate is given by the
   green dots.

Mecanum EKF Example
~~~~~~~~~~~~~~~~~~~

Developing the Extended Kalman Filter for the Mecanum drive is
basically the same process. The only thing to derive is the matrix
:math:`F`. Recalling   :eq:`meccanumDFK`:


  .. math::

     \begin{bmatrix} x_{k+1}\\[3mm] y_{k+1}\\[3mm] \theta_{k+1} \end{bmatrix}
     =   \begin{bmatrix} x_{k}\\[3mm] y_{k}\\[3mm] \theta_{k} \end{bmatrix} +
     \frac{ r\Delta t }{4} \begin{bmatrix} A\cos(\theta_{k})  - B \sin(\theta_{k})   \\[3mm]
     A\sin(\theta_{k})  + B \cos(\theta_{k})                     \\[3mm]
                                 \frac{2}{(L_1+L_2) } C
              \end{bmatrix}

where
:math:`A = \left( \omega_{FL,k} + \omega_{FR,k} + \omega_{BL,k} + \omega_{BR,k} \right)`,
:math:`B = \left(-\omega_{FL,k} + \omega_{FR,k} + \omega_{BL,k} - \omega_{BR,k}  \right)`,
and
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
