Vision and Mapping
------------------

For many of us our dominant sense is vision and we have readily
available sensors - the camera. Cameras can be much more sensitive than
our eyes as they can deal with a greater intensity and frequency range.
For all of the improvements in digital imaging, processing all of that
data into a meaningful information is still a significant challenge. One
of the major goals in computer vision is to develop vision systems
modeled after our own capability.

0.32 |For humans (and I suppose animals), it is very easy to distinguish
apples, tomatoes and PT balls, but not as easy for machine vision
systems|

0.32 |For humans (and I suppose animals), it is very easy to distinguish
apples, tomatoes and PT balls, but not as easy for machine vision
systems|

0.32 |For humans (and I suppose animals), it is very easy to distinguish
apples, tomatoes and PT balls, but not as easy for machine vision
systems|

0.49 |It is easy for a human but hard for a computer to track the road
in a variety of lighting conditions and road types.|

0.49 |It is easy for a human but hard for a computer to track the road
in a variety of lighting conditions and road types.|

With the rise of convolutional neural networks (since 2012), we have
witnessed dramatic improvements in computer vision. The field is
commonly known as deep learning and is addressing some fundamental
problems in vision as well as a host of other applications. Advances in
deep learning are starting to impact robotics and will significantly as
times goes.

**Mapping**, in robotics, is the building of a representation of the
robot’s environment. The assumption often made is that either a map is
available or not required. In some cases a map is required, but not
available. If the application is surveying, the map is the goal. When
reasonable localization is present, mapping just follows from the
onboard sensors. If range sensors are available, then a map can be
produced by knowing the location of the sensor (we assume the relation
between the robot and its sensors are known) and the range data to
objects. A map can be produced as the robot moves about the environment
collecting the data. Again, the details on how this is done is dependent
on the environmental representation (such as metric versus grid maps).
The details are also affected by the accuracy and resolution of the
sensing system.

If location is not known, but the sensors do provide some metric or
range information, then mapping is still possible. SLAM, Simultaneous
Localization and Mapping, is the process to determine the local map as
well as the robot’s location on the map. We will discuss SLAM later on
in the text.

An interesting *chicken and egg* problem arises. Map building requires
knowledge about localization. Conversely, localizing a robot on a map
requires a map.

...then I can figure out my location from landmarks.

...then I can build a map.

...then....?

.. figure:: slam/path_todest.png
   :alt: SLAM: Simultaneous Localization and Mapping[intro-slam]

   SLAM: Simultaneous Localization and Mapping[intro-slam]

When a robot enters an unknown environment, neither the map of the
environment for the location of the robot on the map are understood.
These two processes must occur together, simultaneous localization and
mapping. This is done often enough that it has a name: SLAM
([intro-slam]). The 2D SLAM problem has been well addressed for interior
environments, however 3D SLAM is an active area of research.

0.45 |Localization and Routing|

0.45 |Localization and Routing|

There are limits of course. It is possible to confuse any SLAM system.
Generally, if humans cannot map or localize, then expect the robot
cannot either. Consider highly repetitive environments or featureless
environments, Figure ([intro-slam-problem]); it is easy to see how a
vision system could get confused. These are special cases where there is
very little information available however and we don’t expect the vision
system to perform without adequate data.

0.35 |Compare the structure of a maze to that of a forest scene. Very
simple robots can plan a route and escape a maze. Routing through random
obstacles in three dimensions is still very difficult for a
robot.[mazeforest]|

0.45 |Compare the structure of a maze to that of a forest scene. Very
simple robots can plan a route and escape a maze. Routing through random
obstacles in three dimensions is still very difficult for a
robot.[mazeforest]|

If the robot knows the environment, either from a successful application
of a SLAM algorithm or predetermined in the case of industrial robots
with structured workspaces, then it is reasonable to ask about planning
motion which is optimal in some sense. The field of planning is
interested in deriving motion paths for articulator arms or mobile
robots, Figure ([planning-problem]). The environment will have
obstacles, the robot will have constraints, and the task will have
certain goals. Based on these requirements, the system attempts to
compute a path in the environment or working space that satisfies the
goals.

It is interesting to note that tasks which are easy for humans can be
hard for robots and tasks which are hard for human may be easy for
robots. Meaning tasks with lots of structure and rigid environments, the
robot can succeed and maybe succeed better than a human. Other tasks
which lack structure for which humans are quite adept, the robot may not
succeed at all.
