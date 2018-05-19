Stereo Cameras
--------------

We can build cameras that are much more accurate than the eye, however,
understanding that the image belongs to a particular object is a much
harder task. Implicit in the process is our stereo vision. We have the
ability to reconstruct the 3D world through our eyes; an ability for
which significant effort has been expended to duplicate in computer
science. A branch of modern computer vision, stereo vision, uses
multiple cameras and algorithms to reconstruct and understand the
environment. Some of the tools developed to do this have been widely
distributed. For example, the automatic stitching of images that cameras
perform when building panoramas.

This is successfully employed using pairs of cameras which do a good job
at reconstructing the 3D world we live in. The process requires one to
determine the position shift in pixels of a fixed object. The difficult
part is automatically identifying common points in the image.
Significant effort has been invested in the determination of common
features between two images. Several well known algorithms such as SIFT
and SURF are available now to simplify this process. Once that is done,
it is easy to triangulate the depth of the point, :numref:`intro-stereo1`.

.. _`intro-stereo1`:
.. figure:: VisionFigures/stereo1.*
   :width: 80%
   :align: center

   Seeing in three dimensions with a pair of calibrated cameras:
   determining depth using basic Trigonometry.


.. figure:: VisionFigures/stereo1a.*
   :width: 50%
   :align: center

   Seeing in three dimensions with a pair of calibrated cameras:
   determining depth using basic Trigonometry.

Define a coordinate system where the horizontal axis is :math:`x` and
the vertical axis is :math:`z`. Let the focal point of the left camera
be at the origin of the :math:`x-z` coordinate system. Using both
cameras, we would like to find the coordinates :math:`(x,z)` for the
point :math:`w`. Assume that we are given the focal depth :math:`f`
(positive value) and pixel offsets in image sensor :math:`v_1`,
:math:`v_2` all as *unsigned* (positive) quantities. Then

.. math::

   \left(\frac{z}{x}\right) = \left(\frac{f}{v_1}\right),\quad\quad
   \left(\frac{z}{b-x}\right)  = \left(\frac{f}{v_2}\right)

Cross multiply and set equal to common fraction; then remove fractions:

.. math::

   \left(\frac{v_1}{x}\right) = \left(\frac{f}{z}\right) = \left(\frac{v_2}{b-x}\right)
   \quad \Rightarrow \quad v_2 x = v_1(b-x) = v_1 b - v_1 x \Rightarrow  (v_1+v_2) x = v_1b

Solving for :math:`x`, we obtain the equation below. Plugging this into
:math:`z = fx / v_1` we obtain the equation for :math:`z`.

.. math::
   :label: intro:stereodistance

   x = \frac{v_1b}{v_1+v_2}, \quad
   z = \frac{fb}{v_1+v_2}


.. _`fig:seeing3d`:
.. figure:: VisionFigures/disparitya.*
   :width: 50%
   :align: center

   Seeing in three dimensions with a pair of calibrated cameras: building
   a disparity (depth) map.

.. _`fig:seeing3d-a`:
.. figure:: VisionFigures/disparityb.*
   :width: 50%
   :align: center

   Seeing in three dimensions with a pair of calibrated cameras:
   reconstructing the 3D world.


Once depth for the collection of feature points are known, depth for
surrounding points is inferred. This allows the construction of a
disparity map which maps grey scale values to pixel. It is a depth map
which is shown in Figure \ `[intro-stereo2] <#intro-stereo2>`__. A depth
map is a useful tool in object identification. The depth map can be used
for segmentation, the process by which we separate an image into
distinct components or objects. Once we have the object segmented, then
we may lookup in a shape database to determine what the object is, known
as object recognition. The depth map is one of several ways to perform
object recognition and is a useful tool if we have already computed the
depth map.

.. _`intro-stereo2`:
.. figure:: VisionFigures/rgbdslam.jpg
   :width: 50%
   :align: center

   Seeing in three dimensions with RGBD sensors.

Once a depth map is made, reconstruction of the environment can follow.
Essentially a 3D CAD type representation of the world surrounding the
robot. Thus the environment is mapped in 3D. This is useful for robots
which perform remote reconnaissance as well as for robots which need to
navigate through the environment according to some plan. The map
building process normally places the robot in the map, known as
localization. Thus we can compute optimal paths and safe trajectories.

Depth Sensing Cameras
~~~~~~~~~~~~~~~~~~~~~

We also have a choice of sensors which can directly measure the depth of
field. These are known as time of flight cameras or 3D cameras. The
Microsoft Kinect is a common example. The units range significantly in
cost depending on accuracy, range and speed of the device. These devices
directly provide depth without having to compute a disparity map or some
other intermediate data set. They are very helpful in doing 3D
reconstructions of the environment.


.. figure:: VisionFigures/3dcamera.*
   :width: 50%
   :align: center

   3D Camera
