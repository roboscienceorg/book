
Kalman Derivation
-----------------

Motivating Example
~~~~~~~~~~~~~~~~~~

Consider a truck on perfectly frictionless, infinitely long straight
rails. Initially the truck is stationary at position 0, but it is
buffeted this way and that by random acceleration. We measure the
position of the truck every :math:`\Delta t` seconds using a radio
triangulation system, but these measurements are imprecise; we want to
maintain a model of where the truck is and what its velocity is. We show
here how we derive the model from which we create our Kalman filter.

The position and velocity of the truck is described by the linear state
space

.. math:: {x}_{k} = \begin{bmatrix} x \\ \dot{x} \end{bmatrix}

 where :math:`\dot{x}` is the velocity, that is, the derivative of
position with respect to time.

We assume that between the :math:`(k-1)^{th}` and :math:`k^{th}`
timestep the truck undergoes a constant acceleration of :math:`a_k` that
is normally distributed, with mean 0 and standard deviation
:math:`\sigma_a`.

From Newton’s laws of motion we conclude that

.. math:: {x}_{k} = {A} {x}_{k-1} + {G}a_{k}

 (note that there is no :math:`{B}u` term since we have no known control
inputs) where

.. math:: {A} = \begin{bmatrix} 1 & \Delta t \\ 0 & 1 \end{bmatrix}

 and

.. math::

   {G} = \begin{bmatrix} \frac{\Delta t^{2}}{2} \\ \Delta
         t \end{bmatrix}

 so that

.. math:: {x}_{k} = {A} {x}_{k-1} + {w}_{k}

 where :math:`{w}_{k} \sim N(0, {R})` and

.. math:: {R}={G}{G}^{\text{T}}\sigma_a^2 =\begin{bmatrix} \frac{\Delta t^4}{4} & \frac{\Delta t^3}{2} \\ \frac{\Delta t^3}{2} & \Delta t^2 \end{bmatrix}\sigma_a^2.

At each time step, a noisy measurement of the true position of the truck
is made. Let us suppose the measurement noise :math:`v_k` is also
normally distributed, with mean 0 and standard deviation
:math:`\sigma_z`.

.. math:: {z}_{k} = {H x}_{k} + {v}_{k}

 where

.. math:: {H} = \begin{bmatrix} 1 & 0 \end{bmatrix}

 and

.. math::

   {Q} = \textrm{E}[{v}_k {v}_k^{\text{T}}]
       = \begin{bmatrix} \sigma_z^2 \end{bmatrix}

We know the initial starting state of the truck with perfect precision,
so we initialize

.. math:: \hat{{x}}_{0|0} = \begin{bmatrix} 0 \\ 0 \end{bmatrix}

 and to tell the filter that we know the exact position, we give it a
zero covariance matrix:

.. math:: {P}_{0|0} = \begin{bmatrix} 0 & 0 \\ 0 & 0 \end{bmatrix}

 If the initial position and velocity are not known perfectly the
covariance matrix should be initialized with a suitably large number,
say L, on its diagonal.

.. math:: {P}_{0|0} = \begin{bmatrix} L & 0 \\ 0 & L \end{bmatrix}

 The filter will then prefer the information from the first measurements
over the information already in the model.

The Kalman Process Update
~~~~~~~~~~~~~~~~~~~~~~~~~

The mean of the process update is used for the state prediction:

.. math:: E(x_k) = E(Fx_{k-1} + Gu_k + v_k) \to \hat{x}_k = F\hat{x}_{k-1} + Gu_k

 Using this as the process update we have,

.. math:: \hat{x}_{k|k-1} = F_{k}\hat{x}_{k-1|k-1} + G_{k} u_{k}

 The covariance update is found from the formula

.. math:: P_{k|k-1} = E[(x_k - \hat{x}_{k|k-1})(x_k - \hat{x}_{k|k-1})^T]

 From :math:`x_k = F_{k}x_{k-1} + G_{k}u_k + v_k` and
:math:`\hat{x}_{k|k-1} = F_{k}\hat{x}_{k-1|k-1} + G_{k} u_{k}`

.. math::

   P_{k|k-1} =
     E[F_{k}(x_{k-1} - \hat{x}_{k-1|k-1})(x_{k-1} - \hat{x}_{k-1|k-1})^TF_{k}^T

.. math::

   + 2F(x_{k-1}-
    \hat{x}_{k-1|k-1})v_k^T + v_kv_k^T]

 Note that :math:`E(v_k) =0` we have

.. math::

   P_{k|k-1}= F_{k}E[(x_{k-1} - \hat{x}_{k-1|k-1})(x_{k} - \hat{x}_{k-1|k-1})^T]F_{k}^T
    + E[v_kv_k^T] ,

 so

.. math:: P_{k|k-1} = F_{k}P_{k-1|k-1}F_{k}^T + V_k

The increment
~~~~~~~~~~~~~

The output of the measurement :math:`z_k` lives on

.. math:: \Omega = \{ x\in \mathbb{R}^n | Hx = z_k\}

where :math:`z_k` is a fixed value. This is the space of all solutions
:math:`x` to the linear system. The idea is to select
:math:`\hat{x}_{k|k}` to be the closest object on :math:`\Omega` to the
prediction :math:`\hat{x}_{k|k-1}`.

|image|

Define :math:`\Delta x = \hat{x}_{k|k} - \hat{x}_{k|k-1}` note that the
shortest distance makes :math:`\Delta x \perp \Omega`. Facts about
:math:`\Omega`, :math:`\Delta x` and :math:`H`:

-  Rows of :math:`H` are orthogonal to :math:`\Omega`.

-  Rows must span the remainder of :math:`\mathbb{R}^n` (column space).

-  :math:`\Delta x` is in the row space of :math:`H`.

-  :math:`\Delta x` is in the column space of :math:`H^T`.

|image|

Innovation
~~~~~~~~~~

Define the innovation error: :math:`\nu`

.. math:: \nu = z_k - H\hat{x}_{k|k-1}

 which is the difference between what the sensors reported and what the
sensors would report if the prediction was true.

|image|

Relating innovation to update
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

We need a way to use the innovation :math:`\nu` to produce the update
:math:`\Delta x`.

|image|

From :math:`z_k = H\hat{x}_{k|k}` we have

.. math:: z_k = H(\hat{x}_{k|k-1}+\Delta x) = H\hat{x}_{k|k-1}+H\Delta x

.. math:: H\Delta x = z_k - H\hat{x}_{k|k-1}= \nu

 apply the right pseudo-inverse and we obtain

.. math:: \Delta x = H^T(HH^T)^{-1}(z_k - H\hat{x}_{k|k-1})

The update formula - recursive least squares
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The problem: we don’t know :math:`z_k` exactly. It comes from a
distribution. We would like to form an update and select
:math:`\hat{x}_{k|k}` which is closest to the set :math:`\Omega`. We no
longer have this set since :math:`\Omega` is now a distribution due the
the noise :math:`w_k`. The predicted output is

.. math:: \hat{z}_k = H_k\hat{x}_{k|k-1},

but the most likely output is :math:`z_k^*` a different value.

| :math:`z_k` - measured
| :math:`\hat{z}_k` - computed from process update
| :math:`z^*_{k}` - best estimate of :math:`z_k`

|image|

The update formula - recursive least squares
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| :math:`\hat{x}_{k-1|k-1} \to \hat{x}_{k|k-1}` - process update
| :math:`\hat{z}_k \equiv H\hat{x}_{k|k-1}`
| :math:`z_k` - measured

|image|

| :math:`\nu = z_k - \hat{z}_k`
| Project :math:`\nu` onto the orthogonal subspace
| of :math:`z^*_k=Hx`

|image|

Using a difference of Gaussian formula we have

.. math::

   z_k^* = \hat{z}_k + H_kP_{k|k-1}H_k^T \left( H_kP_{k|k-1}H_k^T + W_k\right)^{-1}
   (z_k - \hat{z}_k)

 Using this we can apply it to the weighted update:

.. math:: \Delta x = P_{k|k-1} H^T (HP_{k|k-1} H^T)^{-1} (z_k^* - H_k\hat{x}_{k|k-1})

 and with some algebraic effort have

.. math:: \Delta x = P_{k|k-1} H^T (HP_{k|k-1} H^T + W_k)^{-1} (z_k - H_k\hat{x}_{k|k-1})

Kalman Measurement Update
~~~~~~~~~~~~~~~~~~~~~~~~~

Thus we have

.. math::

   \hat{x}_{k|k} = \hat{x}_{k|k-1} +  \underbrace{P_{k|k-1}H_k^T
   \left(H_k P_{k|k-1} H_k^T + W_k\right)^{-1}}_{\text{Kalman Gain}}
   \underbrace{\left( z_k - H_k\hat{x}_{k|k-1} \right)}_{\text{Innovation}}

 The covariance of the observation is found in a similar manner that was
done with the least squares estimate

.. math:: P_{k|k} = P_{k|k-1} - P_{k|k-1} H_k^T\left(H_k P_{k|k-1} H_k^T + W_k\right)^{-1} H_K P_{k|k-1}
