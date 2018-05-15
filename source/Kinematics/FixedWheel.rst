Fixed Wheel
-----------

We begin with the axle and wheel. Let P be the chassis center, L is the
distance from P to the wheel contact point. Define :math:`\alpha` as the
angle between the wheel axle and :math:`X_R` and :math:`\beta` as the
angle between the wheel axis :math:`A` and the axle. We also define
:math:`A` as the vector in the wheel axle direction and :math:`v` as the
orthogonal vector (the vector in the travel direction of the wheel). See
Figure \ `[wheelconfig] <#wheelconfig>`__.


.. figure:: KinematicsFigures/fixedwheel.*
   :width: 40%
   :align: center

   Basic wheel and axle configuration.[wheelconfig]


.. math::

   \label{eq:axledirection}
   A = \left\langle \cos(\alpha+\beta) , \sin(\alpha+\beta) \right\rangle

.. math::

   \label{eq:wheeldirection}
   v = \left\langle \sin(\alpha+\beta) , -\cos(\alpha+\beta) \right\rangle

We will examine the linear and rotational aspects of motion separately.
They can be combined in the following manner:

.. math::

   \left\langle \dot{x}_I , \dot{y}_I , 0 \right\rangle + \left\langle 0 , 0 , \dot{\theta} \right\rangle =
   \left\langle \dot{x}_I , \dot{y}_I , \dot{\theta} \right\rangle = \dot{\xi}_I

These equations are in the inertial or global coordinate system. Our
derivations will start in the robot or local coordinate system and so
the velocity vectors will need to be rotated into the robot coordinates:
:math:`\dot{\xi}_R = R(\theta)^{-1}\dot{\xi}_I`.

There are two basic constraints we will apply to derive the kinematic
equations. The first is *no slip* which will mean that the wheel has
perfect traction and the distance around the wheel translates into the
linear distance. This condition is broken when we lose traction on
surfaces with ice, snow or mud. The next constraint is the *no slide*
constraint. This constraint means that we always move in the direction
the wheel is pointed and not in the axle direction. This constraint can
be violated on slick surfaces where the rotational motion of the craft
causes the vehicle to slide sideways. For this section we will assume
that we have no slip and no slide conditions enforced for the basic
wheel.

We begin with the no slip constraint. All of the motion of the vehicle
must be accounted for by wheel motion. This means that the total motion
of the craft must be equal to the wheel travel and in the wheel
direction. This seems obvious and can be translated into a mathematical
constraint using the no slip condition. First, we resolve the linear
motion of the craft and then address the rotational aspect. Recall that
magnitude of the linear motion from the wheel ( in the direction of the
wheel) is :math:`r\dot{\phi}`. The direction of motion is given by
:math:`v`. Since we have both linear and rotational motion built into
:math:`\dot{\xi}`, and to keep the dimensions matching up, the vector
:math:`v` is extended to :math:`\langle v_1, v_2, 0 \rangle`. If
:math:`v` is a unit vector, :math:`\| v \| = 1`, then the projection
onto :math:`v` is

.. math:: P^1_v (u) = \frac{v\cdot u}{\| v\|^2} = v \cdot u = \left\langle \sin(\alpha+\beta) , -\cos(\alpha+\beta) , 0 \right\rangle \cdot u.


.. figure:: KinematicsFigures/fixedwheel2.*
   :width: 40%
   :align: center

   Motion in the angular direction is shown by the vector :math:`w`.
   [fig:angularradialmotion]

For angular motion, we can break the motion of the wheel vector
(:math:`v`) into radial and angular components,
Figure \ `[fig:angularradialmotion] <#fig:angularradialmotion>`__. The
radial component is in the direction of the :math:`L` vector. The
angular component is :math:`w`. It is the angular and not the radial
component which will contribute to :math:`\dot{\theta}`. The angular
component must have :math:`-L \dot{\theta}` for the angular speed in the
:math:`w` direction. [The negative comes from the direction of
:math:`w`.] Projecting that speed onto :math:`v` gives
:math:`-L\cos(\beta) \dot{\theta}` which means our projection component
is :math:`P^2_v = \left\langle 0 , 0 ,  -L\cos(\beta) \right\rangle`.
Combining the projections :math:`P_v = P^1_v + P^2_v`:

.. math:: P_v =  \left\langle \sin(\alpha+\beta) , -\cos(\alpha+\beta), -L\cos(\beta) \right\rangle

and recall

.. math:: P_v [\dot{\xi}_R]  = P_v [R(\theta)^{-1}\dot{\xi}_I] .

| So we obtain:
| :math:`P_v [R(\theta)^{-1}\dot{\xi}_I]`

.. math::

   \label{wheelprojection}
    = \left\langle \sin(\alpha+\beta) , -\cos(\alpha+\beta), -L\cos(\beta) \right\rangle
   \cdot R(\theta)^{-1}\left\langle \dot{x}_I , \dot{y}_I , \dot{\theta} \right\rangle .

For *No Slip* we have:

.. math:: P_v [R(\theta)^{-1}\dot{\xi}_I] =r\dot{\phi}

.. math::

   \Rightarrow  \left\langle \sin(\alpha+\beta) , -\cos(\alpha+\beta), -L\cos(\beta) \right\rangle
   R(\theta)^{-1}\dot{\xi}_I = r\dot{\phi}

For *No Slide*, we want the projection in the direction of A and L to be
zero (a similar derivation as above):

.. math:: P_A [R(\theta)^{-1}\dot{\xi}_I]= 0

.. math::

   \Rightarrow  \left\langle \cos(\alpha+\beta) , \sin(\alpha+\beta), L\sin(\beta) \right\rangle
   \cdot R(\theta)^{-1}\dot{\xi}_I= 0

Steered Wheel
~~~~~~~~~~~~~

| The only difference for steered wheels compared to fixed wheels is
  that the angle :math:`\beta` varies over time. This does not have an
  effect on the form of the equations at an instanteous time, but will
  when integrated over time.
| For *No Slip* we have:
| :math:`P_v [R(\theta)^{-1}\dot{\xi}_I]`

  .. math::

     =  \left\langle \sin(\alpha+\beta(t)) , -\cos(\alpha+\beta(t)), -L\cos(\beta(t)) \right\rangle
     R(\theta)^{-1}\dot{\xi}_I = r\dot{\phi}

| For *No Slide*, as before we want the projection to be zero:
| :math:`P_A R(\theta)^{-1}\dot{\xi}_I`

  .. math::

     =  \left\langle \cos(\alpha+\beta(t)) , \sin(\alpha+\beta(t)), L\sin(\beta(t)) \right\rangle
     \cdot R(\theta)^{-1}\dot{\xi}_I= 0

Castor Wheel
~~~~~~~~~~~~

| For the castor wheel, the no slip condition is the same (as the castor
  offset, d, plays no role in the motion in the direction of the wheel).
  The offset, d, does change the equations in the no slide aspect.
| For *No Slip*:

  .. math::

     \left\langle \sin(\alpha+\beta(t)) , -\cos(\alpha+\beta(t)), -L\cos(\beta(t)) \right\rangle
     R(\theta)^{-1}\dot{\xi}_I = r\dot{\phi}

For *No Slide*:

.. math::

   \left\langle \cos(\alpha+\beta(t)) , \sin(\alpha+\beta(t)), d + L\sin(\beta(t)) \right\rangle
   \cdot R(\theta)^{-1}\dot{\xi}_I + d\dot{\beta} = 0


.. figure:: KinematicsFigures/castorwheel.*
   :width: 40%
   :align: center

   Castor Wheel

Omni, Swedish, or Mecanum Wheels
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


.. figure:: KinematicsFigures/swedish_angle.*
   :width: 15%
   :align: center

   Swedish Wheel

| Let :math:`\gamma` be the angle between the roller axis and wheel
  plane (plane orthogonal to the wheel axis) For *No Slip*:

  .. math::

     \left\langle \sin(\alpha+\beta+\gamma) , -\cos(\alpha+\beta+\gamma), -L\cos(\beta +\gamma) \right\rangle
     R(\theta)^{-1}\dot{\xi}_I

  \ :math:`= r\dot{\phi}\cos(\gamma)`
| For *No Slide*:

  .. math::

     \left\langle \cos(\alpha+\beta +\gamma) , \sin(\alpha+\beta+\gamma),  L\sin(\beta + \gamma) \right\rangle
     \cdot R(\theta)^{-1}\dot{\xi}_I

  \ :math:`= r\dot{\phi}\sin(\gamma) + r_{sw}\dot{\phi}_{sw}`
| Note that since :math:`\phi_{sw}` is free (to spin), the no slide
  condition is not a constraint in the same manner as the fixed or
  steered wheels.
