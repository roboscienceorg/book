Differential Drive - Rederivation
---------------------------------

To get a feel for the more general approach above, it is worthwhile to
rederive the equations for the differential drive robot. This allows us
to check our result as well as see how the matrices are constructed.
From :numref:`ddrive_rederivation` we have
for the left wheel: :math:`\alpha = \pi/2`, :math:`\beta = 0`; and for
the right wheel: :math:`\alpha = -\pi/2`, :math:`\beta = \pi` (to be
consistent with the coordinate system).

.. _`ddrive_rederivation`:
.. figure:: KinematicsFigures/ddexample.*
   :width: 40%
   :align: center

   The differential drive robot dimensions and variables.

Recall the left wheel rolling constraint is given by

.. math::

   \left\langle \sin(\alpha+\beta) , -\cos(\alpha+\beta), -L\cos(\beta) \right\rangle =
   \left\langle 1 , 0, -L \right\rangle

and the right wheel rolling constraint is

.. math::

   \left\langle \sin(\alpha+\beta) , -\cos(\alpha+\beta), -L\cos(\beta) \right\rangle =
   \left\langle 1 , 0, L \right\rangle .

From these two equations we can form the rolling constraint matrix:

.. math:: J_1 = \begin{bmatrix} 1 & 0 & -L \\ 1 & 0 & L \end{bmatrix}

In a similar manner, recall that the left wheel sliding constraint is

.. math::

   \left\langle \cos(\alpha+\beta) , \sin(\alpha+\beta), L\sin(\beta) \right\rangle =
   \left\langle 0 , 1, 0 \right\rangle ,

and the right wheel sliding constraint is

.. math::

   \left\langle \cos(\alpha+\beta) , \sin(\alpha+\beta), L\sin(\beta) \right\rangle
   = \left\langle 0 , 1, 0 \right\rangle  .

Again in a similar manner we can form the sliding constraint matrix:

.. math:: C_1 = \begin{bmatrix} 0 & 1 & 0 \\ 0 & 1 & 0 \end{bmatrix}.

Since the two rows are linearly dependent, we only need to keep one row,
so the matrix looks like

.. math:: C_1 = \begin{bmatrix} 0 & 1 & 0 \end{bmatrix}

The two matrices are stacked to form a single matrix model:

.. math:: \begin{bmatrix}  J_1 \\[4mm] C_1 \end{bmatrix} =  \begin{bmatrix} 1 & 0 & -L \\ 1 & 0 & L \\ 0 & 1 & 0 \end{bmatrix}.

The same can be done with the right hand side arrays

.. math:: \begin{bmatrix} J_2 \\ 0\end{bmatrix} = \begin{bmatrix} r & 0 \\ 0 & r \\0& 0\end{bmatrix}.

The resulting motion model is

.. math::

   \begin{bmatrix} 1 & 0 & -L \\ 1 & 0 & L \\ 0 & 1 & 0 \end{bmatrix} R(\theta)^{-1} \dot{\xi}_I
   = \begin{bmatrix} r & 0 \\ 0 & r \\0& 0\end{bmatrix} \dot{\phi}

Expanding

.. math::

   \begin{bmatrix} 1 & 0 & -L \\ 1 & 0 & L \\ 0 & 1 & 0 \end{bmatrix}
    \begin{bmatrix} \cos \theta & \sin \theta & 0 \\ -\sin \theta &
   \cos \theta & 0 \\  0 & 0 & 1  \end{bmatrix}
    \dot{\xi}_I
   = \begin{bmatrix} r & 0 \\ 0 & r \\0& 0\end{bmatrix}
   \begin{bmatrix}\dot{\phi}_2 \\ \dot{\phi}_1\end{bmatrix}

| To be consistent with the previous example, we had the left wheel as
  (2) and the right wheel as (1) - hence the reverse ordering on the
  :math:`\phi` terms.
| This is the system to solve. Invert the left hand array first, then
  invert the rotation matrix.

Working out the details:

.. math::

   \begin{bmatrix} \cos \theta & \sin \theta & 0 \\ -\sin \theta &
   \cos \theta & 0 \\  0 & 0 & 1  \end{bmatrix}
    \dot{\xi}_I
   = \begin{bmatrix} 1 & 0 & -L \\ 1 & 0 & L \\ 0 & 1 & 0 \end{bmatrix}^{-1}
   \begin{bmatrix} r & 0 \\ 0 & r \\0& 0\end{bmatrix}
   \begin{bmatrix}\dot{\phi}_2 \\ \dot{\phi}_1\end{bmatrix}

.. math::

   \dot{\xi}_I =  \begin{bmatrix}\dot{x} \\ \dot{y} \\ \dot{\theta} \end{bmatrix}
   = \begin{bmatrix} \cos \theta & \sin \theta & 0 \\ -\sin \theta &
   \cos \theta & 0 \\  0 & 0 & 1  \end{bmatrix}^{-1}
   \begin{bmatrix} 1 & 0 & -L \\ 1 & 0 & L \\ 0 & 1 & 0 \end{bmatrix}^{-1}
   \begin{bmatrix} r & 0 \\ 0 & r \\0& 0\end{bmatrix}
   \begin{bmatrix}\dot{\phi}_2 \\ \dot{\phi}_1\end{bmatrix}

.. math::

   \begin{bmatrix}\dot{x} \\ \dot{y} \\ \dot{\theta} \end{bmatrix}
   = \begin{bmatrix} \cos \theta & -\sin \theta & 0 \\ \sin \theta &
   \cos \theta & 0 \\  0 & 0 & 1  \end{bmatrix}
   \begin{bmatrix} 1/2 & 1/2 & 0 \\ 0 & 0 & 1 \\ -1/(2L) & 1/(2L) & 0 \end{bmatrix}
   \begin{bmatrix} r\dot{\phi}_2 \\ r\dot{\phi}_1 \\ 0\end{bmatrix}

and finally ....

.. math::

   \begin{bmatrix}\dot{x} \\ \dot{y} \\ \dot{\theta} \end{bmatrix}
   = \begin{bmatrix} \cos \theta & -\sin \theta & 0 \\ \sin \theta &
   \cos \theta & 0 \\  0 & 0 & 1  \end{bmatrix}
    \begin{bmatrix} \frac{r}{2}\dot{\phi}_1 + \frac{r}{2}\dot{\phi}_2 \\ 0   \\
    -\frac{r}{2L}\dot{\phi}_2 + \frac{r}{2L}\dot{\phi}_1  \end{bmatrix}

.. math::

   = \begin{bmatrix}  \frac{r}{2}\left(\dot{\phi}_1 + \dot{\phi}_2\right)\cos \theta \\
   \frac{r}{2}\left(\dot{\phi}_1 + \dot{\phi}_2\right)\sin \theta \\
   \frac{r}{2L}\left(\dot{\phi}_1 -\dot{\phi}_2\right) \end{bmatrix}

(and you didn’t think this was going to work out, did you.) You may
apply this machinery to other systems as well.


Omniwheel Example
-----------------

For this example we look at a Swedish three wheel robot,
:numref:`Fig:Tribot`. We use an unsteered :math:`90^\circ`
Swedish wheel, so :math:`\beta_i =0` and :math:`\gamma_i = 0` for all
:math:`i`. Going counterclockwise in the figure, we have
:math:`\alpha_1 = \pi/3`, :math:`\alpha_2 = \pi` and
:math:`\alpha_3 = -\pi/3`. You will note that the :math:`C_1` matrix is
of zero rank and so the sliding constraint does not contribute to (nor
is needed for) the model. The equations for motion then are

.. math:: \dot{\xi}_I = R(\theta) J^{-1}_{1f}J_2\dot{\phi}

where

.. math::

   J_{1f} = \begin{bmatrix} \sqrt{3}/2 & -1/2 & -L \\ 0 & 1 & -L \\ -\sqrt{3}/2 & -1/2 & -L \end{bmatrix},
    \quad
   J_2 = \begin{bmatrix} r & 0 & 0 \\ 0 & r & 0 \\ 0 & 0 & r \end{bmatrix}

.. _`Fig:Tribot`:
.. figure:: KinematicsFigures/tribot.*
   :width: 40%
   :align: center

   The Omniwheel can be configured in a three wheel

.. _subsec:twoaxle:
