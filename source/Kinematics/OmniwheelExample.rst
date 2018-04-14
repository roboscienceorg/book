Omniwheel Example
-----------------

For this example we look at a Swedish three wheel robot, Figure
`[Fig:Tribot] <#Fig:Tribot>`__. We use an unsteered :math:`90^\circ`
Swedish wheel, so :math:`\beta_i =0` and :math:`\gamma_i = 0` for all
:math:`i`. Going counterclockwise in the figure, we have
:math:`\alpha_1 = \pi/3`, :math:`\alpha_2 = \pi` and
:math:`\alpha_3 = -\pi/3`. You will note that the :math:`C_1` matrix is
of zero rank and so the sliding constraint does not contribute to (nor
is needed for) the model. The equations for motion then are

.. math:: \dot{\xi}_I = R(\theta) J^{-1}_{1f}J_2\dot{\phi}

\ where

.. math::

   J_{1f} = \begin{bmatrix} \sqrt{3}/2 & -1/2 & -L \\ 0 & 1 & -L \\ -\sqrt{3}/2 & -1/2 & -L \end{bmatrix},
    \quad 
   J_2 = \begin{bmatrix} r & 0 & 0 \\ 0 & r & 0 \\ 0 & 0 & r \end{bmatrix}

.. raw:: latex

   \centering

.. figure:: motion/tribot
   :alt: The Omniwheel can be configured in a three wheel [Fig:Tribot]

   The Omniwheel can be configured in a three wheel [Fig:Tribot]

.. _subsec:twoaxle:
