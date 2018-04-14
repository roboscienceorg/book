Multiple Wheel Model and Matrix Formulation
-------------------------------------------

Since nearly all the robots we will work with have three or more wheels.
The equations we derived above can be combined to build a complete
kinematic model. We begin with some basic variables that define the
system.

-  Let :math:`N` denote the total number of wheels

-  Let :math:`N_f` denote the number of fixed wheels

-  Let :math:`N_s` denote the number of steerable wheels

-  Let :math:`\phi_f(t)` and :math:`\beta_f` be the fixed wheel angular
   velocity and wheel position.

-  Let :math:`\phi_s(t)` and :math:`\beta_s(t)` be the steerable wheel
   angular velocity and wheel position.

We bundle the latter two values in a vector for notational ease:

.. math::

   \phi (t) = ( \phi_{f,1}(t), 
   \phi_{f,2}(t), \phi_{f,3}(t), ..., \phi_{s,1}(t), \phi_{s,2}(t), ...)

.. math::

   \beta (t) = ( \beta_{f,1}(t), 
   \beta_{f,2}(t), \beta_{f,3}(t), ..., \beta_{s,1}(t), \beta_{s,2}(t), ...)

Next we collect the no slip constraints, the equations derived above for
the various drive types and place them in a matrix:

.. math:: J_1 R(\theta)^{-1}\dot{\xi}_I = \begin{bmatrix} J_{1f} \\ J_{1s}\end{bmatrix} R(\theta)^{-1} \dot{\xi}_I= J_2 \dot{\phi}

where :math:`J_1` is the matrix with rows made up of the rolling
constraints and :math:`J_2` is a diagonal matrix made from wheel
diameters. In a similar manner we can bundle up the no slide constraints
(fixed and steered):

.. math:: C_1 R(\theta)^{-1}\dot{\xi}_I = \begin{bmatrix} C_{1f} \\ C_{1s}\end{bmatrix} R(\theta)^{-1} \dot{\xi}_I = 0.

This is matrix shorthand to address the kinematic models for a variety
of systems.

.. math:: \begin{bmatrix} J_1 \\ C_1 \end{bmatrix} R(\theta)^{-1} \dot{\xi}_I = \begin{bmatrix} J_2 \\ 0\end{bmatrix} \dot{\phi}
