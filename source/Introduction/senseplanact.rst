Sense, Plan and Act
-------------------

Robin Murphy in her text *AI
Robotics* :raw-latex:`\cite{Murphy:2000:IAR}`, discusses the fundamental
processes that robots must have. Sensors gather information about the
environment surrounding the robot. A control system takes in the sensed
information, merges it with knowledge contained within and plans a
response, then acts accordingly.

The sense, plan, act architecture was the obvious first attempt at
control. Sensory data is received and processed into sensory
information. For example a laser ranging device returns raw data. This
raw data is processed into a distance map. The distance map might be
corollated with an existing environment map. Using the map information
the system can plan a response. This could be a trajectory for the robot
or robot manipulator. Once the response is decided, the system would
determine how to engage the actuators to gain the desired response using
the kinematics models for the system.

Few, if any, biological systems operate this way. Biological systems
react to stimulus more directly. There is a sense-act architecture that
is in place. For a particular sensory input, a predetermined action is
defined. This reflex system can be fast and effective. The limitations
are obvious. The responses to the environment need to be predetermined.
General purpose robots or robots in new environments cannot use this
approach. Often the situation requires more complex responses which need
planning that takes into account local data.

Hybrid approaches can be built from the sense-act architecture. Murphy
describes a plan, sense-act approach. The robot will plan out the task
and then embark on the task. During execution, the robot will work in a
sense-act reactive mode while carrying out the plan. These ideas are
abstractions and we will have oppotunity to see how each can play out in
detail when we look at more complicated tasks.

Bugs, bats and rats
~~~~~~~~~~~~~~~~~~~

The natural world is simply amazing. It is filled with incredible
solutions to some very difficult challenges and the engineering has
often looked at the natural world for ideas and insipration. An ant is a
very simple creature which manages to survive all around the world in a
vast array of environments. Ants can navigate large habitats with local
sensing only (that we are aware of currently). We can use these small
creatures as a model for some basic path planning and navigation. It is
not our goal to imitate the natural world and so we make no attempt at
an accurate insect model. For our purposes a generic “bug” will suffice.

All of us have watched ants wander the landscape. I often wonder how
they actually manage to cover such large distances and return to the
nest. Ants have three very important senses - touch, smell and sight
(ants can sense sound through sensing the vibrations and so we lump this
into touch). The sense of smell is very important for ants. They use
pheromones to leave markers. In a sense, ants are instrumenting the
landscape. As we will see this is similar to the northstar style
navigation systems used in many commercial systems.

Touch based navigation is the most elementary approach to sensory
system. It can be used in conjunction with chemical detection or taste.
Although possible for robotics, we will not discuss chemical detection
sensors here. Another approach is to use sound. We use sound in a
subconscious manner as a way to feel the room. It is an extension of
touch. We infer hard or soft surfaces as well as room size. This is a
passive use. An active use would be listening to our own voice. The
feedback gained again give us some information about our surroundings.
The most active use commonly illustrated is by bats and dolphins. They
use sonar which gives them obstacle avoidance when vision is inadequate.
Sensing using sound is easier than using light or radio waves due to the
slower propagation speeds. Basic distance sensors using sound are
inexpensive and readily available so many robots have successfully
employed them for use.

We tend to use animals as models for robot capability. Placing a rodent
in a maze was done early on to test memory and learning skills. It gives
a benchmark to compare robot and animal capability, as well as providing
a comparison.
