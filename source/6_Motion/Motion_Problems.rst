Problems
--------

#. Write a Python function to compute wheel angles in the Ackerman system
   given the desired vehicle turn angle and frame parameters.
   This the function should have arguments `(theta, l1, l2)` and function should
   return the two wheel angles `(theta_l, theta_r)`.

#. What are the motion equations for the Ackerman drive? [Meaning forward
   and angular velocity as a function of wheel speed.] Assume wheel radius
   is :math:`r`.

#. A dual Ackerman drive would steer both front and rear wheels using an
   Ackerman steering approach. What would the pros and cons for this system
   compared to a single Ackerman drive?


#. Assume that you have a rectangular Mechanum robot with
   :math:`L_1 = 0.30`\ m, :math:`L_2 = 0.20`\ m and :math:`r=0.08`\ m. Find
   the path of the robot for the given wheel rotations:
   :math:`\dot{\phi}_1 = 0.75*\cos(t/3.0)`,
   :math:`\dot{\phi}_2 = 1.5*cos(t/3.0)`, :math:`\dot{\phi}_3 = -1.0`,
   :math:`\dot{\phi}_4 = 0.5`. Start with :math:`x, y, \theta = 0` and set
   :math:`t=0`, :math:`\Delta t = 0.05`. Run the simulation for 200
   iterations (or for 10 seconds). Keeping the x and y locations in an
   array is an easy way to generate a plot of the robot’s path. If x, y are
   arrays of x-y locations then try

   ::

       import pylab as plt
       plt.plot(x,y,'b.')
       plt.show()

   Showing the orientation takes a bit more work. Matplotlib provides a
   vector plotting method. You need to hand it the location of the vector
   and the vector to be plotted, :math:`(x,y,u,v)`, where :math:`(x,y)` s
   the vector location and :math:`(u,v)` are the x and y components of the
   vector. You can extract those from :math:`\theta` using
   :math:`u = s*\cos(\theta)` and :math:`v = s*\sin(\theta)` where
   :math:`s` is a scale factor (to give a good length for the image, e.g.
   0.075). The vector plot commands are then

   ::

       plt.quiver(u,v,c,s,scale=1.25,units='xy',color='g')
       plt.savefig('mecanumpath.pdf')
       plt.show()



#. Real motion and measurement involves error and this problem will
   introduce the concepts. Assume that you have a differential drive robot
   with wheels that are 20cm in radius and L is 12cm. Using the
   differential drive code (forward kinematics) from the text, develop code
   to simulate the robot motion when the wheel velocities are
   :math:`\dot{\phi}_1 = 0.25t^2`, :math:`\dot{\phi}_2 = 0.5t`. The
   starting location is [0,0] with :math:`\theta = 0`.

   a. Plot the path of the robot on :math:`0\leq t \leq 5`. It should end
      up somewhere near [50,60].

   #. Assume that you have Gaussian noise added to the omegas each time you
      evaluate the velocity (each time step). Test with :math:`\mu = 0` and
      :math:`\sigma = 0.3`. Write the final location (x,y) to a file and
      repeat for 100 simulations. Hint:

      ::

           mu, sigma = 0.0, 0.3
           xerr = np.random.normal(mu,sigma, NumP)
           yerr = np.random.normal(mu,sigma, NumP)

   #. Generate a plot that includes the noise free robot path and the final
      locations for the simulations with noise. Hint:

      ::

          import numpy as np
          import pylab as plt
          ...
          plt.plot(xpath,ypath, 'b-', x,y, 'r.')
          plt.xlim(-10, 90)
          plt.ylim(-20, 80)
          plt.show()

   #. Find the location means and 2x2 covariance matrix for this data set,
      and compute the eigenvalues and eigenvectors of the matrix. Find the
      ellipse that these generate. [The major and minor axes directions are
      given by the eigenvectors. Show the point cloud of final locations
      and the ellipse in a graphic (plot the data and the ellipse). Hint:

      ::

          from scipy import linalg
          from matplotlib.patches import Ellipse
          s = 2.447651936039926
          #  assume final locations are in x & y
          mat = np.array([x,y])
          #  find covariance matrix
          cmat = np.cov(mat)
          # compute eigenvals and eigenvects of covariance
          eval, evec = linalg.eigh(cmat)
          r1 = 2*s*sqrt(evals[0])
          r2 = 2*s*sqrt(evals[1])
          #  find ellipse rotation angle
          angle = 180*atan2(evec[0,1],evec[0,0])/np.pi
          # create ellipse
          ell = Ellipse((np.mean(x),np.mean(y)),r1,r2,angle)
          #  make the ellipse subplot
          a = plt.subplot(111, aspect='equal')
          ell.set_alpha(0.1)    #  make the ellipse lighter
          a.add_artist(ell)   #  add this to the plot

#. Describe the different styles of Swedish wheel.


#. Find the analytic wheel velocities and initial pose for a Mecanum robot
   tasked to follow (:math:`r=3`, :math:`L_1 = 10`, :math:`L_2=10` all in
   cm) the given paths (path units in m). Plot the paths and compare to the
   actual functions to verify.

   a. :math:`y=(3/2)x + 5/2`

   #. :math:`y = x^{2/3}`

#. What are the wheel velocity formulas for a four wheel Mechanum robot,
   (:math:`r=3`, :math:`L_1 = 10`, :math:`L_2=10` all in cm) which
   drives in the circular path :math:`(x-3)^2/16 + (y-2)^2/9 = 1` and always
   faces the center of the circle.

#. In Veranda, drive a Mecanum robot along a square with corners (0,0),
   (10,0), (10,10), (0,10), :math:`L_1 = 0.30`, :math:`L_2 = 0.20` and
   :math:`r=0.08`. You should stop and “turn” at a corner, but keep the
   robot faced in the x-axis direction. Drive the edges at unit speed. Use
   a video screen capture program to record the results.

#. In Veranda, drive the Mecanum robot in an infinity (:math:`\infty`) shape.
   Use a video screen capture program to record the results.
