Problems
--------


#. An object is displaced by :math:`<10,5,15>` and then by :math:`<-5, 20, 0>`.  What is the total displacement?

#. What is the angle between :math:`<1,2,3>` and :math:`<3,2,1>`?

#. Given (1,2), (2,1), (3,2), (4,4).

   a.  Using the pseudoinverse, find the least squares linear interpolant of the data points:  :math:`y = a_1x + a_0`.
   #.  Plot the data and the resulting line.

#. Given :math:`x+y = 1`, :math:`x+2y = 0`, :math:`2x+y = 1`.

   a.  Write this as a linear system and find the left pseudoinverse of the matrix.  [Note: label the matrix as A, right hand side as b and unknown as x.]
   #.  Find the least squares solution for :math:`(x_1, x_2)`.
   #.  What is the error :math:`\| Ax - b\|`?

#. Given :math:`x+2y = 2`, :math:`2x+4y = 4`.

   a.  Frame this as a 2x2 matrix problem.
   #.  Can you solve this system?  Why or why not?
   #.  Plot the columns of the matrix and the right hand side (as vectors).
   #.  Plot :math:`A^Tb`.
   #.  What is the least squares solution for x?

#. Find the 3x3 rotation matrix that will rotate

   a.  about the x axis by :math:`30^\circ`.
   #.  about the z axis by :math:`120^\circ`.


#. Analytically show that the inverse rotation matrix is the same matrix as replacing :math:`\theta` by :math:`-\theta`.

#.  In homogeneous coordinates, is a rotation followed by a displacement the same as the displacement followed by the rotation?  Prove if this is true or provide a counter-example.  Note that proof does not mean give examples.

#. Find the 3x3 transformation matrix that rotates a vector about y by :math:`15^\circ` and then about x by :math:`25^\circ`.

#.  Find the 4x4 homogeneous coordinate transformation matrix that rotates a vector about z by :math:`45^\circ`, then translates by :math:`<1,2,3>` and then rotates about y by :math:`30^\circ`.

#.  Show that one of the eigenvectors for a rotation matrix points in the direction of the axis of rotation.  What is the eigenvalue?

#.  Assume that you have a wheel rolling on a circular path (or track).  Let the radius of the path be 10m, the radius of the wheel be 30cm.  Assume the wheel is turning at 3 revolutions per minute.  What is the revolutions per minute around the path?
