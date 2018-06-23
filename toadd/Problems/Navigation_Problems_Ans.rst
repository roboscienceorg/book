Problems
--------

.. raw:: latex

   \setcounter{Exc}{0}

[Planning_ans]

Write a Python algorithm to perform boundary following on a grid domain.

Write a boundary following routine for the DD robot in Gazebo using the
Lidar. Use a video screen capture program to record the results.

Assume that you have a finite number of convex solid obstacles (solid
means you are not starting inside). Prove or provide a counter-example.

#. Will Bug 1 succeed in navigating from any start point to any goal
   point?

#. Will Bug 2 succeed in navigating from any start point to any goal
   point?

#. Will Tangent Bug succeed in navigating from any start point to any
   goal point?

Sketch equations `[LidarRangeEq] <#LidarRangeEq>`__ and
`[ObsConstrEq] <#ObsConstrEq>`__.

Assume that you have a grid map of the type found in the left image of
Figure \ `[coarsemap] <#coarsemap>`__ which was stored in an array. If
the start point was an interior cell, implement the Bug 1 algorithm to
find the sequence of cells which describe an escape path if one exists.

Is it possible to have a single non-convex obstacle trap Bug 1 or Bug 2?

Assume that you have a grid domain and the obstacles are represented in
the grid map. Write a Python program to implement:

#. Bug 1

#. Bug 2

#. Bug 3

#. Tangent Bug

Implement the following in Gazebo:

#. Bug 1

#. Bug 2

#. Bug 3

#. Tangent Bug

Use a video screen capture program to record the results.

[ex:A*] Write a Python program to implement :math:`A^*` to find a route
on a map. Your initial map should be a black and white bitmap. Use the
GIMP/NetPBM approach to construct a binary discrete map for the
environment.

Write a Python program to implement the Wavefront algorithm.

#. Demonstrate on a map with multiple obstacles.

#. Compare to the :math:`A^*` approach in
   Exercise \ `[ex:A*] <#ex:A*>`__.

Let the domain be the rectangle :math:`0\leq x \leq 15` and
:math:`0 \leq y \leq 10`. Place the start position at (0,5). Place the
end position at (15,5). Assume you have a disk centered at (6,4) with
radius 2 and a disk centered at (8,6) with radius 3. Find a potential
function which can navigate the robot from the start to the end
position.

#. Plot the resulting path in Python with obstacles included in the map.

#. Using STDR, move a robot along the path. Use a video screen capture
   program to record the results.

#. Compare to the Wavefront approach.

Rework the previous problem with a Poisson navigation function.

Write a Python program to implement the Brushfire algorithm and extract
the equidistance pixels. Display the equidistance pixel map. Demonstrate
on a map with multiple obstacles.

.. raw:: latex

   \Closesolutionfile{Answer}

.. [1]
   These need not be the same. For example certain paths may be
   traversed at different speeds depending on location and path
   geometry.

.. [2]
   I am being a bit sloppy. I have ignored the thickness of the wall in
   a couple of the distance computations and so will round up here.

.. [3]
   if((i!=0)||(j!=0))

.. [4]
   Known as zero “almost everywhere".

.. [5]
   This is due to the switching on and off a large repulsive potential.

.. |a) A more disceptive obstacle. This provides the basic obstacle shape and relative pose. b) Extending the difference in the obstacle shape to increase the path difference between Bug 1 and Bug2. [complicatedobstacle]| image:: path/complicated_obst0
.. |a) A more disceptive obstacle. This provides the basic obstacle shape and relative pose. b) Extending the difference in the obstacle shape to increase the path difference between Bug 1 and Bug2. [complicatedobstacle]| image:: path/complicated_obst
.. |Bug1 can outperform Bug2. [bug1vsbug2]| image:: path/complicated_obst_b1
.. |Bug1 can outperform Bug2. [bug1vsbug2]| image:: path/complicated_obst_b2
.. |Obstacles producing discontituities in the range map. Assume that one can determine discontinuities in the distance function :math:`\rho_R`.[discontrange]| image:: path/range
.. |Obstacles producing discontituities in the range map. Assume that one can determine discontinuities in the distance function :math:`\rho_R`.[discontrange]| image:: path/rangefunction
.. |(a) Points of discontinuity: :math:`O_1`, :math:`O_2`, ..., :math:`O_n`. (b) Object ambiguity. [discontinuitypoints]| image:: path/discont
.. |(a) Points of discontinuity: :math:`O_1`, :math:`O_2`, ..., :math:`O_n`. (b) Object ambiguity. [discontinuitypoints]| image:: planning/singleVSdouble
.. |a) We assume that the boundary is a smooth function. b) The normal and tangent directions to the offset curve.[offsetcurve]| image:: path/offset0
.. |a) We assume that the boundary is a smooth function. b) The normal and tangent directions to the offset curve.[offsetcurve]| image:: path/offset
.. |image| image:: netpbm/imagecoords
.. |image| image:: netpbm/neighbors
.. |a) A very simple maze. b) A more complicated maze. [maze]| image:: turtle/maze0
.. |a) A very simple maze. b) A more complicated maze. [maze]| image:: turtle/maze2
.. |a) Wall following (right hand) to solve the maze. b) Connecting the outside to make a circle. [mazesolwall]| image:: turtle/maze_sol_wall
.. |a) Wall following (right hand) to solve the maze. b) Connecting the outside to make a circle. [mazesolwall]| image:: turtle/maze_sol_wall_circle
.. |a) Wall path extracted from the maze. b) Moving the nodes on the path to show the circle. [mazesolcircle]| image:: turtle/maze_sol_wall_circle1
.. |a) Wall path extracted from the maze. b) Moving the nodes on the path to show the circle. [mazesolcircle]| image:: turtle/maze_sol_wall_circle2
.. |The coarsening of the grid map for a maze and the construction of the graph representation. Left side image is a maze on a finer grid. The right side image is a coarser grid with graph drawn. [coarsemap]| image:: path/finemaze
.. |The coarsening of the grid map for a maze and the construction of the graph representation. Left side image is a maze on a finer grid. The right side image is a coarser grid with graph drawn. [coarsemap]| image:: path/coarsemaze
.. |Wavefront will apply to maze and unstructured domains.[fig:struct_unstruct]| image:: planning/complmaze
.. |Wavefront will apply to maze and unstructured domains.[fig:struct_unstruct]| image:: planning/cave
.. |Wavefront algorithm progress. [fig:wavefrontprogress]| image:: path/finemaze_numbered_2
.. |Wavefront algorithm progress. [fig:wavefrontprogress]| image:: path/finemaze_numbered_3
.. |Wavefront algorithm progress. [fig:wavefrontprogress]| image:: path/finemaze_numbered_4
.. |Wavefront algorithm progress. [fig:wavefrontprogress]| image:: path/finemaze_numbered_path
.. |Wavefront fill example. [fig:wavefillexample]| image:: path/obsmaparray
.. |Wavefront fill example. [fig:wavefillexample]| image:: path/initmaparray
.. |Wavefront fill example. [fig:wavefillexample]| image:: path/goalmaparray
.. |Wavefront fill example. [fig:wavefillexample]| image:: path/fillmaparray
.. |Wavefront fill example. [fig:wavefillexample]| image:: path/startbtmaparray
.. |Wavefront fill example. [fig:wavefillexample]| image:: path/btmaparray
.. |image| image:: potential/gradient_figure_1_crop
.. |image| image:: potential/gradient_figure_2_crop
.. |image| image:: potential/gradient_figure_3_crop
.. |image| image:: potential/gradient_figure_1a_crop
.. |image| image:: potential/gradient_figure_2a_crop
.. |image| image:: potential/gradient_figure_3a_crop
.. |A. Attractive potential function. B. Repulsive potential function.[example2potential]| image:: potential/potential1
.. |A. Attractive potential function. B. Repulsive potential function.[example2potential]| image:: potential/potential2
.. |image| image:: potential/potential3
.. |image| image:: potential/potentialavoid1a
.. |Left: Example obstacle overlap. Right: An example of four and eight point connectivity.| image:: potential/brushfire
.. |Left: Example obstacle overlap. Right: An example of four and eight point connectivity.| image:: path/neighbors2
.. |Sample Brushfire Fill 1| image:: potential/brushfire1
.. |Sample Brushfire Fill 1| image:: potential/brushfire2
.. |Sample Brushfire Fill 2| image:: potential/brushfire3
.. |Sample Brushfire Fill 2| image:: potential/brushfire4
.. |Sample Brushfire Fill 3| image:: potential/brushfire5
.. |Sample Brushfire Fill 3| image:: potential/brushfire6
.. |image| image:: planning/piecewise_const
.. |image| image:: potential/navipot.png

