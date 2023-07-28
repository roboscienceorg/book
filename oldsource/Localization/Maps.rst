Maps
------------

.. Note:: This section under development.

The representation of the environment is reflected in the map. We have
discussed several methods earlier such as metric or continuous
descriptions, discrete or grid descriptions and topological
descriptions. Each choice has advantages and disadvantages.

Continuous descriptions can have high accuracy and lower data storage
requirements. The trade-off is the increased complexity of
representation of obstacles. Conversely discrete descriptions have high
storage requirements but reduced complexity of obstacle representation.
Discrete maps have limits on accuracy imposed by the basic grid cell
size. The precision of the map influences how precisely one can store
map features and is a factor in the computational complexity of the
algorithms using the map.

-  **Uniform** Cells based on a uniform grid. In 2d these are often
   called pixels. In 3d some call them voxels. Simple storage arrays.
   Not adaptive and can be storage intensive.

-  **Tree** Use of a tree representation for space. Hierarchical
   decomposition of space. In 2d, one uses a quadtree and in 3d one uses
   an octree. Can save storage if most of space is not partitioned.

-  **BSP** Binary Space Partitioning tree. Each cell is divided in half
   with some condition about area or partition line.

Much of the machinery used are the tools from computer graphics. Items
like points, cells, voxels, polyhedra, splines, patches, etc make up the
geometric primitives.

A map will then consist of the representation of space and the objects
inside that space. The maps we have discussed so far are usually known
as metric maps, ones which have an absolute reference frame and
numerical values as to object location.

Another successful approach is using *topological* representations. Here
space is represented as a graph. Connectivity between objects (vertices)
is done via the edges. The edges can have length as well as be oriented.
A vertex represents a unique landmark and can hold orientation data (wrt
to the graph).

For unexplored areas of the map one should place “Here there be
dragons”...
