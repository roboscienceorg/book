Navigation Planner
------------------

Even if we can avoid local minimums, what does one do about a
correctly located minimum? Meaning, how can one assure that the
minimum of the potential function, once the repulsive potentials are
included, stays at the goal location? One approach is to no longer sum
potentials, but form products and powers of potential functions. For
simplicity, we assume that that the space and the obstacles are
spheres. Then you can construct a navigation potential function which
is a potential function used for navigation and is a Morse function.

Define

  .. math:: \beta_0(q) = -d^2(q,q_0) + r^2_0

  .. math:: \beta_i(q) = d^2(q,q_0) - r^2_i

   and

  .. math:: \beta(q) = \prod_{i=0}^n \beta_i(q)

   This works as the repulsive potential.

For the attractive potential, define

.. math:: \gamma_\kappa (q) = (d(q,q_\text{goal}))^{2\kappa}

 Normally one might combine the two potentials by

.. math:: \frac{\gamma_\kappa(q)}{\beta(q)}

 but this function goes to infinity as you approach the obstacle. A way
to combine the two and not have numerical issues (keep the values
bounded)

.. math::

   s(q,\lambda) = \frac{\gamma_\kappa}{\lambda \beta + \gamma_\kappa}
   = \frac{(d(q,q_\text{goal}))^{2\kappa}}{\lambda \beta + (d(q,q_\text{goal}))^{2\kappa}}

 Finally, this is sharpened by
:math:`\phi(q) = \left( s(q,\lambda)\right)^{1/\kappa}`.

This can be assembled and we have for the navigation potential function:

.. math::

   \phi (q) =
   \frac{d^2(q,q_\text{goal})}{\left(d^{2\kappa}(q,q_\text{goal}) + \lambda \beta
   \right)^{1/\kappa}}

.. raw:: latex

   \centering

|image|

.. raw:: latex

   \FloatBarrier
