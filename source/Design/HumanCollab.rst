Human Collaboration
-------------------

The NSF [3]_ announced the second NRI, National Robotics Initiative,
Ubiquitous Collaborative Robots (NRI-2.0). The first line of the NSF
proposal call reads *The goal of the National Robotics Initiative (NRI)
is to support fundamental research that will accelerate the development
and use of robots in the United States that work beside or cooperatively
with people.* This is not a new research line, but is a very public
acceleration of a trend. The goal of the research and expectation for
the future is that humans and robots will be working together in a
collaborative manner. It should be noted that collaboration is more than
remote control. The intent is to have autonomous robots working in
collaboration with people, but not fully controlled by those
individuals.

To have this collaboration, we need to remove the cages. The system
needs to be safe and people need to trust the system. Trust is earned,
not just announced. So how can one develop a trusted and safe robotic
partner?

Environmental Awareness
~~~~~~~~~~~~~~~~~~~~~~~

Robots need full awareness of their surroundings. They especially need
to know if and where the humans are. This requires a number of sensors.
These sensors need to be redundant and differentiated. By this we mean
that we need multiple sensors which can confirm sensor readings. A
particular vantage point might be blocked or subject to interference.
For example, reflected light could produce a bogus reading on a light
sensor pointed in a particular direction. Noise or other disturbances
can produce erroneous readings for vibration or pressure sensors. So
redundancy is important for correct errors and filling in gaps.

By differentiation we are asking for diversity of sensor types. We might
want to know that we sense something at a particular range, we sense
noises and detect heat. The three together gives us more information
than having just one sensor type in which each one would be fooled by
the same erroneous signal. It is much less likely that three different
erroneous signals would occur to fool three different types of sensors.

For a human to work safely and comfortably around the machine, they need
to know the machine is aware of their location with the idea that the
machine can and will adapt to the human presence. In this case, the
robot must avoid collisions. Limiting movement areas, limiting movement
speed and force reduction are things that can be employed to enhance
safety and confidence. Newer systems will have a zone for which the
robot must enforce predetermined limits. Clearly implementation of these
policies requires constant environmental awareness. More than just
limits, feedback from the robot to the individuals is needed. Using
lights, sounds and motions are all approaches that can be used to let
the people working around the system know whether or not the system is
aware of their presence.

Direct Communication
~~~~~~~~~~~~~~~~~~~~

Work can involve lots of concurrent activities. Humans are easily
distracted and this can cause someone to not pay attention to a robot or
miss the warning light. Systems need to have very direct and clear
communications to avoid potential harm. You see in plenty of industrial
machines the system of warning lights and sirens indicating machine
activity. In collaborative robotics intended to be with humans, the
cages and sirens are not an option. The recent expectation is for a much
higher level of cognition in robots we work with. So, we can expect that
robots speak to us letting us know what they sense, what they plan, and
what they are attempting. Robots with eyes or faces can turn towards the
users which helps in gaining human attention and communication.

Indirect Communication
~~~~~~~~~~~~~~~~~~~~~~

You commonly hear people say things like “non-verbals makes up 93% of
communication” [4]_ (which is now seen as more urban legend or myth).
However, non-verbal communication is an important manner in which humans
communicate. Giving robots the ability to both understand and present
non-verbal elements helps in the overall goal of safety and confidence.
Non-verbals such as gestures and expressions can be added in the overall
system design at with current technology. Understanding human gestures
and expressions is under development seeing advances based on the newer
deep learning (neural networks) systems.

It is clear that the non-verbal will go a long way in making people more
comfortable around the robots. It is one of the ways we can
anthropomorphize the robot leading to better human adoption. Although
taking this too far can lead to other problems as we will discuss below.

The real world
~~~~~~~~~~~~~~

One of the reasons we have robots is that they can repeat an action over
and over again, exactly the same each time. This is what industrial
machines do. Generally people don’t like repetitive jobs, are not good
at it and can cause injury. We live in a world of imprecision and
randomness. Repeatability and uniformity are abstract concepts not often
found in nature. Once the robot leaves the confines of the production
line, the environment gets much more complicated. Robots must be
tolerant of variation in all dimensions. It must be able to handle this
from the outside as well as itself. It must be able to respond to
failing sensors, software and actuators in addition to unexpected
external events.

Fault tolerance will be an increasingly important aspect to human-robot
collaboration. Systems which constantly assess the state of the robot,
the progress of the task and the environment are needed to be successful
in the dynamic and varied human environment. At first, humans will
accept changing their behavior to work with the robot, but for the robot
to be accepted it needs to meet the person halfway and not require that
behavioral changes are required for the human collaborator; especially
those due to safety concerns.

Consider power failure in the robot. The first industrial robot the
author saw was one who had a bad reputation. It was in a research lab
tended by graduate students. When powered on the arm would jump from its
resting position to some ready or home position. This action was very
fast. When power was removed, the forces used to keep the arm in place
were released and the arm would release in another unexpected manner.
One of those power cycles caused the arm to strike a student. The
student was injured but otherwise was ok.

Power changes can be very scary. If the power hits all systems at once,
the servos will receive power at the same time as all of the
electronics. Computers and microcontrollers take time to power up or
boot. So, this leaves the servos at the mercy of the random signals on
the communication channels. Mobile robots jump, jerk and drift. Arms can
swing and gantries can move. It is essential that a safe power on
process is developed. A well designed system does not allow the random
bits on the controllers and buses to cause robot motion. Only when the
CPU is up and has verified its state, sensor inputs, executed safe state
protocols will it move the robot. The system must validate that only
those commands will initiate motion.

Power down is another dangerous time. A heavy robot arm can fall or drop
a heavy load. Mechanical locks or resistance must be used to prevent
people getting hurt by robot components falling to a rest position. The
power up / down problem is part of a larger issue of behavioral
expectations and consistency. Software is said to be secure if it
behaves like you expect it to. [5]_ The author believes this is a good
definition of a secure or trustworthy robot. One that behaves like you
expect it to behave. This is how individuals learn to trust each other.

We started this chapter with three accounts of horrible industrial
accidents. The fatality was due to collision between the operator and
the robot manipulator. Not all robots working in close proximity will be
sufficiently powerful to injure a person, but that is not the point. One
must design the system with the assumption that human robot collisions
will happen. Having touch sensing through the robot helps the system
know when contact is made. This must trigger an interruption of the
current task. The task can be stopped, the articulator moved to a safe
distance and then wait for human direction.

Having power limits in the robot may also help. So if the robot
manipulator collides with a human, it cannot do any real damage. There
is current research in soft (flexible) robotic systems. One of the goals
is to increase the safety by limiting the possible power delivered to
any obstacle. Responding to a collision is important even in these low
power cases since there is probably an issue, and it is annoying to get
struck by the robot.

Close interactions
~~~~~~~~~~~~~~~~~~

For a robot to work with people, it needs to act like people. A concept
of personal space needs to be enforced. Beyond awareness its
surroundings and of individuals near it, the robot needs to respond like
humans do in respecting personal space. Path planning needs to route
around heads and limbs. Just like we do when working together. When the
path planner can not do this it needs to tell the human in a polite way
to adjust. Equally useful will be the ability to understand the human
through gestures and verbal commands that the robot needs to adjust.
There are times that the robot and the human will need to be in physical
contact to perform a task. Careful visual and audio feedback is required
to be an effective partner in the collaboration. To be fair, this is a
skill that many people struggle with.

Appearance
~~~~~~~~~~

With the innate human tendency towards anthropomorphism, we can build on
it by providing the robot with humanlike features. Eyes, faces and arms
all work at a psychological level to make the machine seem more human.
However, there are clear limits to this increasing humanization which
can be seen in our psychological response to certain systems. Take
Actroid, Figure \ `[Figure:actroid] <#Figure:actroid>`__, which is
designed to replace a human receptionist. It has been built to look as
human as possible. The idea expressed by Japanese roboticist Masahiro
Mori in 1970 is that the more human-like a machine appears, the more
endearing it will be. This is not the case, however. As the design
becomes more and more similar to the human or animal it is attempting to
model, we have a negative response. We use terms like “creepy” or
“wrong”. It makes us uncomfortable. This is known as *uncanny valley*.
Our acceptance of, or comfort with, the machine drops as the design
approaches lifelike accuracy. All cultures (that the author is aware of)
exhibit this, but varies greatly in the exact boundary of their limits.

.. raw:: latex

   \centering

.. figure:: robots/uncanny
   :alt: Uncanny Valley, the drop in the comfort graph as a function of
   human likeness.[fig:uncannyvwalley]

   Uncanny Valley, the drop in the comfort graph as a function of human
   likeness.[fig:uncannyvwalley]

A completely different view
~~~~~~~~~~~~~~~~~~~~~~~~~~~

Evan Selinger, a Philosophy Professor at RIT, has a completely different
take on the utility of anthropomorphic design. He argues that bots,
robots and the like should strive to be less or appear less human. That
because we have this innate tendency towards anthropomorphism, we make
assumptions and mistakes based on those assumptions. Take Siri for
example, Siri is based on speech recognition and machine learning
technologies. Siri uses a female voice and human speech patterns to
present the guise of humanity. Although sophisticated, Siri and Alexa
and the like are far from human. Machine Learning is still a
mathematical pattern matching tool and not a self-conscience cognitive
entity. Placing this technology in a robot, does not then transform the
robot into more human than molding it into a human form.

Dr. Selinger argues that the designers should do the opposite. Have the
system constantly let everyone know it is a robot; voice its
limitations. The system needs drop a gender in the voice or at least
vary the one used. By continually providing feedback that separates the
robot or system from anthropomorphism, the system is better able to
assist the user since the context is clear. Robotic systems are created
to assist us with tasks. Making them increasing human does not
necessarily make them better assistants. For example, fidelity to human
speech patterns means that, as Dr. Selinger puts it, the “key-board
shortcuts" are not available.

To build on this idea, one can argue don’t need to create robots that
are a partial or substandard human. We have plenty of people on the
planet and many are underemployed. We need the robots to focus on the
tasks in which we do want to replace human labor. We also know that
humans are generalists. We are not the fastest or the strongest or the
most robust. We do many many things and in some cases just well enough.
Our robots should be tuned and exceptional for the task at hand. They
should be specialists and as such not strive to look or act or be like
humans.
