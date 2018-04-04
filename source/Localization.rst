Localization and Mapping
========================

This chapter focuses on determining the location of a robot on a known
map, the converse problem of building a map given a location, and when
neither the location or map are involved known as SLAM - Simultaneous
Localization and Mapping.

What is localization?
---------------------

*Using sensory information to locate the robot in its environment is the
most fundamental problem to providing a mobile robot with autonomous
capabilities.* [Cox 1991]

Given a map of the environment, a sequence of actions and sensor
measurements, our goal is to estimate the robot’s position and error of
the position estimate. There are two problem classes or types for
localization we will study: **Position Tracking** and **Global
Localization**. They emphasize two different but important aspects of
localization.

Position tracking is one of the most common issues facing the beginning
roboticist. Also called ego-motion determination is the task which given
a known starting location, keeps track of the path or position of the
robot over time. This can be a local method in that the absolute
location of the robot may not be known and only the relative position
from the starting point can be determined. Methods may employ motion
sensing such as wheel encoders, inertial navigation sensors and optical
flow algorithms. The focus can be purely on relative changes in position
and not at all related to a map.

Global localization is the task of determining the position in a global
coordinate system. This can arise from determination of initial pose for
the position tracking problem or in recovery from localization system
failure (also known as the kidnapped robot problem). Global localization
may want to place the robot on a known map or begin the mapping process
with a known location.

As we learned in the Filtering chapter, measurements of pose data is
error prone. The system can be uncertain about global landmarks. [We
observe we are 30 meters down the hall, but don’t know which hallway.]
All of this introduces error into both the position tracking and the
global position of a robot. This requires some type of error reduction
or filtering approach to be employed. In addition to noise, the
landscape can change. One must take into account the dynamic environment
and any changes that occur due to the robot.

Some of the localization approaches interact with maps. The structure of
the map and the information stored in the map can strongly influence the
results. Some approaches are metric in that they track pose data for all
the actors. Other approaches are feature based which may only know the
robot is near something. One may instrument the environment with tags
and beacons. Or one may just learn interesting features and infer
location.

Localization may employ active or passive approaches. Passive approaches
just observes where active approaches means the robot moves to minimize
some variable of the pose and goal. Multiple robots can be used to
enhance localization by exchange of information as well as use the other
robots for relative positioning.

Localization Belief
-------------------

This section under development.

The process of localization is stating a belief that the robot is found
at a given location. There is ambiguity in our knowledge which implies
this belief can be represented as a probability distribution. We can
arbitrarily decide that this belief, the probability distribution, is a
normal distribution or something more exotic. Having a single “bump”
means we have one hypothesis of location, a *single hypothesis*. Having
a distribution with multiple bumps means we have *multiple hypotheses*
of the robot location.

|Single and Multiple Hypotheses.|   |Single and Multiple Hypotheses.|

Having multiple hypotheses seems a bit odd at first, but actually
arises. Imagine you have a Starbucks map - a map of a city that just
shows Starbucks. Also assume you drive up to a Starbucks. Now compare to
the map. You can now isolate your position to one of the :math:`n`
Starbucks locations on the map. This is an example of multiple
hypotheses. Only until you receive additional information are you able
to break the ambiguity. With the reduced information available to a
robot, this situation arises when faced with vision system that use
corners and walls to generate landmarks.

Map Representation
------------------

This section under development.

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

Decomposition
-------------

This section under development.

We have discussed in earlier chapters three different ways to represent
the environment: metric or continuous, discrete or grid and topological.

Map Building
------------

This section under development.

Review some of the issue around building maps.

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

Correlation-Based Models
~~~~~~~~~~~~~~~~~~~~~~~~

Given a global map, one type of sensor model is to correlate a local
sensor based map with a global map. *Map Matching*. It is tied with map
building and localization. This is described later in detail.
Preliminaries

-  Assume that you have a occupancy grid map :math:`m`.

-  Assume that this is a simple map with grid cells marked as occupied
   or not - binary map.

-  Store the map in an array :math:`m[i][j]`.

-  Let :math:`x_t = (x,y,\theta)` be the robot’s pose.

-  Let :math:`z_t^k` be the range value of a sensor reading.

-  Let :math:`x_{k,\mbox{sens}}, y_{k,\mbox{sens}}` be the location of
   the sensor in the local coordinates.

-  Let :math:`\theta_{k,\mbox{sens}}` be the angle of the beam from the
   local (robot) coordinate system.

-  Use sensors to build a local map :math:`m_{\mbox{local}}[i_L][j_L]`

-  Correlate local and global coordinate systems:
   :math:`\begin{pmatrix}x & y & \theta\end{pmatrix}^T`

.. figure:: sensor/coords.png
   :alt: Coordinate transforms to relate observed obstacle to global
   map.

   Coordinate transforms to relate observed obstacle to global map.

.. math::

   \begin{pmatrix} x_{z_t^k}\\y_{z_t^k} \end{pmatrix} =
     \begin{pmatrix}x \\ y \end{pmatrix} + \begin{pmatrix} \cos\theta &
       -\sin\theta \\ \sin\theta & \cos\theta\end{pmatrix}
        \begin{pmatrix}x_{k,\mbox{sens}}\\y_{k,\mbox{sens}}\end{pmatrix}
        + z_t^k \begin{pmatrix}\cos (\theta + \theta_{k,\mbox{sens}}) \\
        \sin (\theta + \theta_{k,\mbox{sens}})\end{pmatrix}

-  Find the correlation between the two spatially aligned maps on the
   common regions of definition.

   -  List out the map as a vector :math:`v[k] = m[i][j]` where
      :math:`k=n*j+i`.

   -  Plot them as vectors and compare, how close?

   -  Using the average of the two, show you can get a better comparison
      by subtracting off the average.

   -  Find the angle between the two differenced vectors.

   -  Thus :math:`\overline{m} = \frac{1}{2N} \sum \left( m[i][j] +
        m_{\mbox{local}}[i][j]\right)`.

   -  Define

      .. math:: \rho = \frac{(m - \overline{m})\cdot (m_{\mbox{local}} -\overline{m})}{\|m - \overline{m}\| \| m_{\mbox{local}} -\overline{m}\|}

   -  Define :math:`p(m_{\mbox{local}}|x_t,m) = \max \{\rho , 0\}`

Can you do template matching on this? How about ICP?

.. raw:: latex

   \centering

|image|

.. raw:: latex

   \centering

Where does this fit: |image|

**Extract features from measurements.**

This is similar to what is done in computer vision.

-  Identify features which correspond to distinct objects, call them
   landmarks.

-  Assume you can obtain a range and bearing for the landmark.

-  Call the unique identifier for a landmark, a signature.

-  For the :math:`i^{th}` measurement at time :math:`t`, denote range by
   :math:`r^i_t`, bearing :math:`\phi^i_t` and signature :math:`s^i_t`.

*Feature based map*: :math:`m = \{ m_1, m_2, \dots \}`. The
:math:`j^{th}` map feature be defined by
:math:`m_j = (m_{j,x}, m_{j,y}, s_j)^T`. The :math:`i^{th}`

feature then can be correlated to the :math:`j^{th}` landmark.

Let the robot pose given by :math:`x_t = (x, y, \theta)^T`. Then we
have:

.. math::

   \begin{array}{l} r^i_t = \sqrt{(m_{j,x}-x)^2+(m_{j,y}-y)^2} +
   \epsilon_{\sigma_r^2}\\[8pt] \displaystyle \phi^i_t =
   \tan^{-1}\frac{m_{j,y}-y}{m_{j,x}-x}-\theta +
   \epsilon_{\sigma_{\phi}^2}\\[8pt] s^i_t = s_j + \epsilon_{\sigma_s^2}
   \end{array}

*Data association problem* A key problem is the association of features
to landmarks.

-  Introduce a *correspondence variable* between feature :math:`f_t^i`
   and landmark :math:`m_j`: :math:`c^i_t \in \{ 1, 2, 3,
   \dots , N+1\}` where :math:`N` is the number of landmarks in the map.

-  If :math:`c^i_t = j \leq N` then the :math:`i^{th}` feature observed
   at time :math:`t`

   corresponds to the :math:`j^{th}` landmark in the map. [:math:`c^i_t`
   is the true identity.]

-  If :math:`c^i_t = N+1` then the feature does not correspond to a
   landmark in the map.

To compute the probability of a feature corresponding to known landmark:

#. :math:`j=c^i_t`

#. :math:`\hat{r}^i_t = \sqrt{(m_{j,x}-x)^2+(m_{j,y}-y)^2}`

#. :math:`\hat{\phi}^i_t =\displaystyle\mbox{atan}\left(\frac{m_{j,y}-y}{m_{j,x}-x}\right) - \theta`

#. :math:`q = \mbox{Gauss}(r^i_t-\hat{r})\mbox{Gauss}(\phi^i_t-\hat{\phi}) \mbox{Gauss}(s^i_t-\hat{s})`

.. |Single and Multiple Hypotheses.| image:: localization/singlehypothesis
.. |Single and Multiple Hypotheses.| image:: localization/multihypothesis
.. |image| image:: roadmap/map5.png
.. |image| image:: roadmap/map5_cut.png

