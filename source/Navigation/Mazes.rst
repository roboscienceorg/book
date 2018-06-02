Mazes
-----

A very common demonstration of mobile robot problem solving is a maze
escape. Even though a maze escape routine is standard fair for a data
structures course, it is still impressive to see it implemented with a
robot in a maze.

.. _`maze0`:
.. figure:: NavigationFigures/maze0.*
   :width: 45%
   :align: center

   A very simple maze.

.. _`maze2`:
.. figure:: NavigationFigures/maze2.*
   :width: 45%
   :align: center

   A more complicated maze.

.. _`mazesol`:
.. figure:: NavigationFigures/maze_sol.*
   :width: 40%
   :align: center

   Solution path through a maze.

:numref:`mazesol` shows a solution path through a maze.
The random mouse algorithm is one approach to finding a route. The
algorithm has the “mouse” travel straight until a wall is encountered.
Then the “mouse” randomly selects a new direction to follow. This
approach is a form of random search which eventually finds a route,
although rather slowly.

Wall Following
^^^^^^^^^^^^^^

The best known method to traverse a maze is the wall following method.
The idea is to place your left or right hand on the wall as you traverse
the maze. If the maze is simply connected, the method is proven to
provide a path out of the maze. By looking at
:numref:`mazesol`, the solution path partitions the
maze. A simply connected maze is partitioned into two objects which are
deformable to a disk. To see this, focus on the right (or in the figure
the lower) part of the separated maze. Tracing the path,
:numref:`mazesolwall`, we record our motion through
the maze. This path can be extracted,
:numref:`mazesolcircle` to see that it is indeed a
circle. The topology as not changed.

.. _`mazesolwall0`:
.. figure:: NavigationFigures/maze_sol_wall.*
   :width: 40%
   :align: center

   Wall following (right hand) to solve the maze.

.. _`mazesolwall`:
.. figure:: NavigationFigures/maze_sol_wall_circle.*
   :width: 40%
   :align: center

   Connecting the outside to make a circle.

.. _`mazesolcircle1`:
.. figure:: NavigationFigures/maze_sol_wall_circle1.*
   :width: 40%
   :align: center

   Wall path extracted from the maze.

.. _`mazesolcircle2`:
.. figure:: NavigationFigures/maze_sol_wall_circle2.*
   :width: 40%
   :align: center

   Moving the nodes on the path to show the circle.


Since the path is a circle, then the algorithm will transport the robot
between any two points on the circle. Not having a simply connected maze
or having interior starting/finishing points can break this method -
which does not mean it will necessarily fail.


.. _`maze_notsimple`:
.. figure:: NavigationFigures/maze_notsimple.*
   :width: 40%
   :align: center

   A maze for which wall following can fail.

The Pledge algorithm is designed to address the problem of exiting a
maze which has non-simply connected components. This algorithm does not
work in reverse, meaning that it can escape a maze, but not enter one.



.. _`PledgeAlgorithm`:
.. topic::  Pledge Algorithm

   | **Input** A point robot with a tactile sensor
   | **Output** A path to the :math:`q_{\text{goal}}` or a conclusion no such path exists.
   | Set arbitrary heading.
   | **while** No obstacle in front **do**
   |   **repeat**
   |     Move forward
   |   **end while**
   | Select right or left side and place that side against the obstacle.
   | **while** Note original heading and sum of turns not zero **do**
   |   **repeat**
   |     Move along obstacle while keeping "hand" on obstacle
   |     Sum turn angles
   | **end while**


.. _`maze_notsimple_pledge`:
.. figure:: NavigationFigures/maze_notsimple_pledge.*
   :width: 30%
   :align: center

   The Pledge Algorithm.

The final escape algorithm presented here is Trémaux’s Algorithm. This
is a form of a recursive backtracker. From Wikipedia:

    Trémaux’s algorithm, invented by Charles Pierre Trémaux, is an
    efficient method to find the way out of a maze that requires drawing
    lines on the floor to mark a path, and is guaranteed to work for all
    mazes that have well-defined passages. A path is either unvisited,
    marked once or marked twice. Every time a direction is chosen it is
    marked by drawing a line on the floor (from junction to junction).
    In the beginning a random direction is chosen (if there is more than
    one). On arriving at a junction that has not been visited before (no
    other marks), pick a random direction (and mark the path). When
    arriving at a marked junction and if your current path is marked
    only once then turn around and walk back (and mark the path a second
    time). If this is not the case, pick the direction with the fewest
    marks (and mark it, as always). When you finally reach the solution,
    paths marked exactly once will indicate a direct way back to the
    start. If there is no exit, this method will take you back to the
    start where all paths are marked twice. In this case each path is
    walked down exactly twice, once in each direction. The resulting
    walk is called a bidirectional double-tracing.

In most maze solving applications, the maze is represented by a graph.
If you have seen some basic graph search algorithms you will recognize
this as a type of Depth First Search (DFS). For the robot however, there
is more than the DFS maze solving code. There is also the details of
navigating corridors and turns. Using only bump sensors this can be a
challenge, one we will address with ranging sensors later in this
chapter. However, without good sensors, using the algorithms like
Trémaux’s algorithm might not work out. Without the ability to drop and
sense breadcrumbs, the recursive backtracker will fail. One way to
approach this problem is to create a map of the maze as you work your
way through it. Acting on the map means you are working on existing
trails and this is just another way of marking the domain.

The robot is running on a more complicated lanscape than the just
operating in the maze. Working on a solution to the maze in the Pledge
Algorithm or Trémaux’s algorithm is simply working along the abstracted
paths. We are neglecting all the issues relevant to a robot such as
driving straight down the corridor, detecting walls, keeping distance
from walls, navigating turns, etc. All of this low level navigation is
ignored in the maze algorithms above and they focus on the higher level
aspect of maze escape. This makes sense in that separating the levels
helps to separate tasks leading to better code design.

To reduce the complexity we separate the maps for the robot, the
landscape map, which will have a precision set by the sensors and the
map or graph required by the maze, maze map. The maze map can use a grid
with larger cells. Large cells would mean lower precision but smaller
arrays. However, this is not a problem since the low level routines are
doing the positioning on the high resolution map leaving the high level
routines to navigate.

The maze map can be thought of as a low resolution version of the
landscape map. Each cell can still be an occupancy map, but with large
cells. In this case it is useful to take the cell as large as possible
so that corridors or walls are one cell wide. Using the centers of
unoccupied cells, these are nodes. Adjacent free cells can have their
center nodes connected. This builds a graph representation, see
:numref:`coarsemap`. So, now we have a high resolution
grid map and the corresponding graph representation of free space. This
concept will be used later in more advanced path planniing algorithms.
For now we employ a simple path planner.

.. _`coarsemap`:
.. figure:: NavigationFigures/finemazecoarse.*
   :width: 90%
   :align: center

   The coarsening of the grid map for a maze and the construction of the graph representation.  Left side image is a maze on a finer grid.  The right side image is a coarser grid with graph drawn.


One of the simpliest planners is the flood fill approach. Begin at the
endpoint and run a flood fill algorithm. If the flood fill paints the
starting point then a path has been discovered. You can run the flood
fill algorithm on the landscape map, the reduced maze map or the maze
graph. For illustration, we focus on the second one.

There is a fundamental difference between exploring the domain and a
route, and having a map available to discover a route. If the entire
domain is known and the question is simply to find the route, there are
routing tools available. The route can be found before exploration. We
will see later that flood fill approaches can help even in partially
explored (or mapped) domains.

The maze is a high regular and artificial structure. We don’t have
anything like them in nature and few things in our day to day
surroundings really resemble a maze. So, why discuss them? The maze has
setup some fundamental approaches which we will employ next. First, we
see that it makes sense to approach an obstacle, like a wall, and then
follow the obstacle. This is the “place a hand on the wall" idea. We see
that that approach is not sufficient from more complicated mazes and we
also need to know when and where to break free of the obstacle. We have
learned that seeing the domain in terms of a graph is useful in that we
can apply algorithms designed for graphs, such as a depth first search.
We see that certain solutions are comprehensive in how they solve the
problem and others are not. The maze is then the launching point for
planners which live in unstructured worlds.

Wave-front Planner
------------------

In this section we introduce the Wavefront planner. This planner is a
breadth first search algorithm applied to the grid map domain. The
implementation is similar to a flood fill algorithm. The Wavefront
Algorithm searches for the minimal path from start to goal in structured
and unstructured domains,
:numref:`fig:struct_unstruct`. Just like a
flood fill, Wavefront is rather simple. Assume that free space is
represented by white and occupied space is red or black (colored). Zoom
in so you can see the actual pixels as shown in
:numref:`fig:struct_unstruct`.

.. _`fig:struct_unstruct`:
.. figure:: NavigationFigures/floodfillenvs.*
   :width: 85%
   :align: center

   Wavefront will apply to maze and unstructured domains.

The process to find the path through the maze is simple. It is completed
in two stages. Stage one fills the map with distance numbers from the
goal. Stage two steps down the distances until the goal is reached.
Tracking the steps generates the path. So, we have two parts. First is
an algorithm called “Fill” which is like a flood fill in your paint
program. The second part is the “Descent” algorithm. Think of the fill
algorithm as building a hill where the start is at the top and the end
is at the bottom. All we do is walk downhill.


.. _`fig:finemaze`:
.. figure:: NavigationFigures/finemaze.*
   :width: 40%
   :align: center

   Initial Maze.

The Fill algorithm is easy to state. Label the goal pixel “1". Next,
label all unlabeled neighbors of the “1" pixel the number “2". Then
label all of the unlabeled neighbors of the “2" pixel the number “3".
You repeat this process by labeling all of the unlabeled neighbors of
the pixels with the label “k" the number “k+1". Do this until you run
out of unlabeled pixels.


.. _`fig:wavefrontprogress`:
.. figure:: NavigationFigures/finemaze_numbered.png
   :width: 90%
   :align: center

   Wavefront algorithm progress.



.. _`wavefrontfind`:
.. topic::  Wavefront Descent

   |  Begin at start pixel.
   | **repeat**
   |     Pick the neighbor pixel with the smallest label (or value).
   |     Step to that pixel.
   | **until** you arrive at goal pixel.



The first three images in
:numref:`fig:wavefrontprogress` give you a
few snapshots of the process on a maze. You may note that these numbers
are just the number of pixel steps from your current location to the
goal. It is a travel distance. Next is the Descent algorithm. Starting
at the start point, look around for the pixel with the smallest label or
value. Step there and repeat the process. Continue stepping downhill
until you reach the goal pixel.

.. _`fig:wavefillex`:
.. figure:: NavigationFigures/obsmaparray.*
   :width: 40%
   :align: center

   Wave front progression (0)

.. _`fig:wavefillex1`:
.. figure:: NavigationFigures/initmaparray.*
   :width: 40%
   :align: center

   Wave front progression (1)

.. _`fig:wavefillex2`:
.. figure:: NavigationFigures/goalmaparray.*
   :width: 40%
   :align: center

   Wave front progression (2)

.. _`fig:wavefillex3`:
.. figure:: NavigationFigures/fillmaparray.*
   :width: 40%
   :align: center

   Wave front progression (3)

.. _`fig:wavefillex4`:
.. figure:: NavigationFigures/startbtmaparray.*
   :width: 40%
   :align: center

   Wave front progression (4)

.. _`fig:wavefillex5`:
.. figure:: NavigationFigures/btmaparray.*
   :width: 40%
   :align: center

   Wavefront fill example complete.
