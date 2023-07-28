Problems
--------


#. Basic Kalman Filter. Let

   .. math:: x = \begin{bmatrix}x_1 \\ x_2\end{bmatrix}, \quad F = \begin{bmatrix} 0 &0.1 \\-0.02 &0.2\end{bmatrix}, \quad G_k u_k= \begin{bmatrix} 0\\ 2*\sin(k/25)\end{bmatrix},

   .. math::

      H = \begin{bmatrix} 1& 0 \end{bmatrix},
      \quad V = \begin{bmatrix} 0.05^2&0\\0.& 0.05^2\end{bmatrix}, \quad W = 0.25^2,

   .. math:: x(0) = \begin{bmatrix} 0.025\\0.1\end{bmatrix}, \quad P(0) = \begin{bmatrix}0 & 0\\ 0&0\end{bmatrix}.

   Apply the Kalman Filter process to compute 100 iterations and plot
   them. Hint: run the simulation to create your observation data :math:`z`
   and then run your Kalman Filter.

#. Assume that one has three different measurements for the location of
   some object. The three measurements with the covariances are

   .. math::

      (10.5, 18.2), \quad \left(\begin{array}{cc} 0.1 & 0.01 \\ 0.01 & 0.15
        \end{array}\right); \quad
      (10.75, 18.0), \quad \left(\begin{array}{cc} 0.05 & 0.005 \\ 0.005 & 0.05
          \end{array}\right);

   .. math::

      (9.9, 19.1), \quad \left(\begin{array}{cc} 0.2 & 0.05 \\ 0.05 & 0.25
      \end{array}\right).

   Fuse this data into one measurement and provide an estimate of the
   covariance.

#. Run a simulation on

   .. math:: \begin{array}{l}\dot{x} = y \\\dot{y} = -\cos(x) + 0.5\sin(t)\end{array}

   adding noise to the :math:`x` and :math:`y` components (with variance
   = 0.2 on each). Let :math:`\Delta t = 0.1`. Assume that you can observe
   the first variable, :math:`x`, with variance :math:`0.25`. Record the
   observations. Write a program to run the EKF on the observed data and
   compare the state estimate to the original values.

#. Differential Drive - EKF. The motion equations for a differential drive
   robot are given below. Assume that the wheels are 5cm in radius and the
   wheelbase is 12cm. Recall that the kinematics for this is (r = radius, L
   = wheelbase):

   .. math::

      \begin{array}{l}
       \dot{x} = \frac{r}{2} (\dot{\phi_1}+\dot{\phi_2})\cos(\theta) \\[5mm]
      \dot{y} = \frac{r}{2} (\dot{\phi_1}+\dot{\phi_2})\sin(\theta) \\[5mm]
      \dot{\theta} = \frac{r}{2L} (\dot{\phi_1}-\dot{\phi_2})
      \end{array}

   Select :math:`\Delta t = 0.2` (time increment) and convert to discrete equations.
   After conversion, assume the  covariance of the state transition is :math:`V`.
   Also assume that you
   have a local GPS system that gives :math:`(x,y)` data subject to
   Gaussian noise with covariance :math:`W`. The units on the noise are
   given in cm. If you want to use meters then you will need to divide your
   noise by 100.

   a. Starting at :math:`t=0`, :math:`x=0`, :math:`y=0`, :math:`\theta=0`,
      predict location when wheel velocities are:

      ::

                  t=0 -> 5:  omega1 = omega2 = 3 (rads/time),
                  t=5 -> 6:  omega1 = - omega2 = 1,
                  t=6 -> 10: omega1 = omega2 = 3,
                  t=10 -> 11:  - omega1 = omega2 = 1,
                  t=11 -> 16: omega1 =  omega2 = 3,


      assuming that you have Gaussian noise in the process that is described by:

      .. math::

         `V = \begin{bmatrix}.05 &  .02 & 0.01\\.02& .05& 0.01\\ 0.01& 0.01& .1\end{bmatrix}`


   #. Write out the formulas for the Extended Kalman Filter.

   #. Apply an Extended Kalman filter to the motion simulation above to
      track the location of the vehicle. Observations can be simulated by
      using previous simulation data as actual data, i.e. use this as the
      observed data (:math:`z_k`). Parameters:

      .. math:: x_{0|0} = (0,0,0), \quad V = \begin{bmatrix}.05 &  .02 & 0.01\\.02& .05& 0.01\\ 0.01& 0.01& .1\end{bmatrix},

      .. math:: W= \begin{bmatrix} .08& .02 \\.02&  .07\end{bmatrix}, \quad P_{0|0} = \begin{bmatrix}2 &0& 0\\0 &1& 0\\0& 0& 0.5\end{bmatrix}.

   #. Output the x-y locations on a 0.5 sec grid and compare in a plot.

   #. The covariance matrix P gives the uncertainly ellipse for the
      location of the robot. Plot 5 ellipses along the path. This ellipse
      has major and minor axes given by the eigenvectors of P and the axes
      lengths are given by the associated eigenvalues. Matplotlib can plot
      an ellipse, `click
      here. <https://matplotlib.org/api/_as_gen/matplotlib.patches.Ellipse.html#matplotlib.patches.Ellipse>`__

#. Assume that you have a differential drive robot located in a lab with
   special landmarks placed in various locations around the lab. Also
   assume that your robot has a forward looking stereo vision system which
   can determine the distance and angle off the forward direction (relative
   to the robot) of the landmark. You don’t know the locations of the
   landmarks and the stereo system can only see the landmarks if the angle
   off of the front is less than 75\ :math:`^{\circ}`.

   a. Write the equations for the apriori EKF step (:math:`f_k`) for some
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
