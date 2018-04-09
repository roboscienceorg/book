Introduction[Chap:Introduction]
===============================

Growing up in the modern age means many things to many people. For those
of us reading (or writing) a book on robotics, it means getting a
healthy dose of technology. Hollywood has raised us on spaceships,
cloning, alien worlds and intelligent machines. This was, however, not
incredibly unrealistic as progress in science and technology has been
advancing at an ever increasing rate. We have come to expect something
new, maybe even dramatic, every single year...and for the most part
haven’t been disappointed! Over the years we have seen significant
developments in medicine, space, electronics, communications and
materials. There has always been excitement regarding the latest
development. Even though the world around us has been struggling with
war, poverty and disease, science and technology offers us a reprieve.
It is an optimistic view that things can and will get better. In full
disclosure - we love technology! It is the magic of our age and learning
how the magic works only makes it more fun. In no way does the author
claim that technology is the answer to our problems. That clearly lies
with our willingness to look beyond our differences with acceptance,
compassion and grace. If technology can bring us together, then it has
succeeded in helping us far beyond our wildest dreams.

Robotics is a shining example of technical optimism - a belief that the
human condition can be improved through sufficiently advanced
technology. It is the premise that a machine can engage in the
difficult, the tedious, and the dangerous; leaving humans out of harms
way. Technology is a fancy word for tool utilization. Even though
biologists have long shown that humans were not alone in their usage of
tools, we are indisputably the master tool users on the planet. Tools
extend our grasp, our strength and our speed. We know of no technology
that aims to extend human capability like robotics does.

Robotics is an engineering discipline stemming from the fusion of
science and art. Humans have an insatiable curiosity, a drive to create,
and a considerable amount of self-interest. Building machines which look
like us, act like us and do things like us was the engineering manifest
destiny. Although we have succeeded in building machines that do
complicated tasks, we really place the value in what we learn about
ourselves in the process. A process we embark on here.

What exactly is a robot?
------------------------

0.31 |FANUC 6-axis welding robot. :raw-latex:`\cite{wiki:fanuc}`|

0.55 |FANUC 6-axis welding robot. :raw-latex:`\cite{wiki:fanuc}`|

[Figure:robotimagesA]

Definition
~~~~~~~~~~

What is a robot? This is actually a complicated question. Wikipedia
defines a robot in the following manner: *A robot is a mechanical
intelligent agent which can perform tasks on its own, or with guidance;
usually an electro-mechanical machine which is guided by computer and
electronic programming.* (There are plenty of opinions on Wikipedia. I
find that it is pretty good for math, science and engineering quick
reference but not always an expository presentation. It is also good at
reflecting opinions, which in this case is useful.) Merriam-Webster, on
the other hand, says a robot is *a real or imaginary machine that is
controlled by a computer and is often made to look like a human or
animal.* According to the Encyclopaedia Britannica, a robot is *any
automatically operated machine that replaces human effort, though it may
not resemble human beings in appearance or perform functions in a
humanlike manner*.

0.4 |[Figure:robotimagesB]|

0.55 |[Figure:robotimagesB]|

This latter definition includes washing machines, bread makers, and
other devices not generally seen as a robot. However, as we will argue,
that does not matter! A definition of robot that uses form or motion is
flawed. What if we made the statement broader? **A robot is seen as a
sophisticated machine that, as stated above, replaces human effort**.
Nothing else really defines robotics as well.

Regardless of definition, these machines surround us. Today we can see
them used in from manufacturing to exploration, from assistive
technologies and medicine to entertainment, from research to education,
and much more.

0.35 |[Figure:robotimagesC]|

0.35 |[Figure:robotimagesC]|

There is no consensus on which machines qualify as robots. However,
there *is* a general agreement that robots exhibit behaviors which mimic
humans or animals - that is, *behavior which seems intelligent.* We
expect the robot to interact with its environment and the objects within
that environment. Most of us may expect that the robot performs this
interaction through movement and sensation.

0.4 |[Figure:robotimagesD]|

0.49 |[Figure:robotimagesD]|

Many may expect the robot to perform complex tasks or deal with harsh,
unforgiving environments. Some may expect a robot to be an extension of
themselves through teleoperation or remote control, while others expect
it to be a fully autonomous device.

We can boil down our notion of robot abilities to three things:

**Perception:**
    sensing the environment and to a limited degree understanding the
    sensory information.

**Cognition:**
    ability to make decisions and responses based on the sensory
    information and not acting in a pre-programmed manner.

**Actuation:**
    full mobility of the machine or control of a tool through a
    manipulator.

0.45 |[Figure:robotimagesE]|

0.45 |[Figure:robotimagesE]|

One interesting phenomenon that could be influencing the lack of a solid
definition for the term is that what we label a “robot" varies with
time. When a new capability arises, one that was previously considered
to be solely in the domain of humans and animals, we tend to label it a
robot. As soon as that capability becomes routine, the device is thought
of a mechatronic device.

0.45 |[Figure:robotimagesF]|

0.45 |[Figure:robotimagesF]|

Robots embody technological magic. So, it is natural that some
previously unseen ability programmed into a machine will have a magic
quality for humans, thus making that machine more of a robot. But with
time, we get accustomed to it, and the magic gets replaced with
expectation.

0.475

0.475

|

0.475

It can be argued that there is nothing new in the subject of robotics -
that all we are doing is building machines. Nothing different than what
engineers have been doing all along. The term robotics has more to do
with our ego and psychology than anything to do with science and
technology. However, there is a body of knowledge related to building
machines that interact in human or physical environments. This is what
we will consider robotics.

0.475

0.475

A brief history
~~~~~~~~~~~~~~~

1023 BC
'''''''

In ancient China, a curious account on automata is found in the Lie Zi
text, written in the 3rd century BC. Within it there is a description of
an encounter between King Mu of Zhou (1023-957 BC) and a mechanical
engineer known as Yan Shi, who was an ’artificer’. According to the
text, the artificer proudly presented the king with a life-size,
human-shaped figure of mechanical handiwork which could sing and move in
a life-like manner.

205 BC
''''''

In ancient Greece, an orrery known as the Antikythera Mechanism is
developed. This device is credited as being the first analog computer.

.. figure:: robots/antikytheramachine.jpg
   :alt: Antikythera Mechanism. :raw-latex:`\cite{wiki:antik}`

   Antikythera Mechanism. :raw-latex:`\cite{wiki:antik}`

270 BC
''''''

The Greek engineer Ctesibius (c. 270 BC) applies a knowledge of
pneumatics and hydraulics to produce the first organ and water clocks
with moving figures.

1088 AD
'''''''

The Cosmic Engine, a 10-meter (33 ft) clock tower built by Su Song in
Kaifeng, China. It featured mechanical mannequins that chimed the hours,
ringing gongs or bells among other devices.[6][7]

1206 AD
'''''''

Al-Jazari (1136-1206), an Arab Muslim inventor during the Artuqid
dynasty, designed and constructed a number of automatic machines,
including kitchen appliances, musical automata powered by water, and the
first programmable humanoid robot in 1206. Al-Jazari’s robot was a boat
with four automatic musicians that floated on a lake to entertain guests
at royal drinking parties. His mechanism had a programmable drum machine
with pegs (cams) that bump into little levers that operate the
percussion. The drummer could be made to play different rhythms and
different drum patterns by moving the pegs to different locations.[8]

.. figure:: robots/Al-Jazari.jpg
   :alt: Al-Jazari’s Mechanical Musical Boat.
   :raw-latex:`\cite{wiki:aljazari}`

   Al-Jazari’s Mechanical Musical Boat.
   :raw-latex:`\cite{wiki:aljazari}`

1495
''''

Leonardo da Vinci draws plans for a mechanical knight.

1922
''''

The word *robot* is introduced to the English language through the play
Rossum’s Universal Robots by the Czech writer Karel Čapek. The play is
centered around a factory staffed by intelligent cyborgs. The English
term robot comes from the Slavic word *robota* which roughly translates
as work or labor. Credit for the term goes to Karel’s brother Josef.

1954
''''

Following World War II, efforts in automation increased. Early advances
were seen in teleoperation and computer numerically controlled (CNC)
machining. General Electric produced machines that had a master slave
approach where the master manipulator would control the slave. The CNC
machines gained popularity in the aircraft industry by milling high
performance parts in lower volumes. The merger of these two technologies
produced the first programmed articulated device by George Devol in
1954. He replaced the master manipulator with CNC technology. Joseph
Engelberger purchased the rights and founded Unimation in 1956.
Unimation placed its first robot arm in a General Motors plant in 1961.

1969
''''

The 1960’s saw significant experimentation with manipulator designs,
feedback systems and actuator types. One such example of a robotic
manipulator is the Stanford Hydrolic Arm and Stanford Manipulator,
designed in 1969 by Victor Scheinman, a Mechanical Engineering student
working in the Stanford Artificial Intelligence Lab (SAIL).

1973
''''

The Cincinnati Milacron :math:`T^3` is released. It was a heavy lift
assembly line manipulator. In 1978, Unimation introduced the PUMA,
(Programmable Universal Machine for Assembly) and JPL started a research
program to develop space based teleoperated manipulators. By the late
1970’s, applications for industrial robots grew quickly and robots in
industry became established.

The history for mobile robots is much more recent. The challenges for
mobile robots, as we will see later on, are fundamentally different than
industrial automation. An early example is the Johns Hopkins *Beast*. It
was a simple autonomous mobile system that navigated using touch sensors
and could recharge itself. This system required an instrumented
environment. A notable development is *Shakey*, by the Stanford Research
Institute (SRI) from 1966-72. This robot implemented computer vision and
natural language processing and is responsible for the development of
the A\* search algorithm, the Hough transform, and visibility graphs.

Robots in the news
~~~~~~~~~~~~~~~~~~

Items are hyperlinked to web pages.

2017
^^^^

-  `Tertill (Franklin Robotics) - Fully autonomous weeding
   robot. <http://www.franklinrobotics.com/>`__

-  `Minitaur (Ghost Robotics) - Legged version of the Rhex but with
   enhanced obstacle response. <https://www.ghostrobotics.io/>`__

-  `Fast Foward. Autonomous delivery robot.
   Paggio. <http://piaggiofastforward.com/>`__

-  `Cobalt Indoor Security Robots. Collaboratory security
   robots. <https://www.cobaltrobotics.com/>`__

-  `Ekso GT, exoskeleton to assist paraplegics. Ekso
   Bionics <http://eksobionics.com/>`__

-  `Kuri. Home “social" robot. Mayfield
   Robotics. <http://www.mayfieldrobotics.com/>`__

2016
^^^^

-  `SpotMini, a compact version of Boston Dynamics’ Spot
   robot. <http://spectrum.ieee.org/automaton/robotics/home-robots/boston-dynamics-spotmini/>`__

-  `Pleurobot - experiments in salamander motion through
   robotics. <http://spectrum.ieee.org/automaton/robotics/robotics-hardware/how-epfl-made-pleurobot/>`__

-  `Vyo - Different approach to social domestic
   robots. <http://spectrum.ieee.org/automaton/robotics/home-robots/vyo-robotic-smart-home-assistant/>`__

2015
^^^^

-  `DRC Hubo - UNLV finished 8th place in the
   DRC. <http://www.drc-hubo.com/>`__

-  `Momaro - experimentation in rescue
   robots. <http://www.ais.uni-bonn.de/nimbro/Rescue/>`__

-  `iCub - The iCub is the humanoid robot developed at IIT as part of
   the EU project RobotCub. <http://www.icub.org/>`__

-  `Walkman Robot - EU humanoid. <https://www.walk-man.eu/>`__

-  `Deepfield Robotics targeting
   agriculture. <http://spectrum.ieee.org/automaton/robotics/industrial-robots/bosch-deepfield-robotics-weed-control/>`__

2014
^^^^

-  `Robocup 2014: Goal! Although the human team was not really
   aggressive, the goal was well setup and the defender did try to block
   the shot. <https://www.youtube.com/watch?v=fbDBlXJ5CE8>`__

-  `Pronking. RHex is used to experiment with new gaits. Pronking is
   commonly known with the African Springbok and is used to understand
   very dynamic
   locomotion. <https://www.youtube.com/watch?v=rDwV2RWq0LY>`__

-  `Boston Dynamic’s descendent of Big Dog is LS3. LS3 is getting field
   testing for use as ground support for
   Marines. <http://www.bostondynamics.com/robot_ls3.html>`__

-  `CMU’s Biorobotics lab has a new generation of robotic snakes. This
   one uses elastic actuators for smooth
   motion. <https://www.youtube.com/watch?v=lZUzwNbromY#t=122>`__

-  `Festo announces a robot kangaroo. Why? Well who wouldn’t want a
   kangaroo robot? <https://www.youtube.com/watch?v=mWiNlWk1Muw>`__

2013
^^^^

-  `Boston Dynamic’s BigDog gets an arm which can throw heavy
   objects. <https://www.youtube.com/watch?v=2jvLalY6ubc>`__ , Figure
   [bigdog]

-  Google’s robotic car gets a full test. Figure [googlecar]
   :raw-latex:`\cite{wiki:googlecar}`

-  `Watch Flying Robots Build a 6-Meter
   Tower. <http://spectrum.ieee.org/automaton/robotics/diy/video-watch-flying-robots-build-a-6-meter-tower>`__
   , Figure [quadswarm] :raw-latex:`\cite{wiki:quadswarm}`

0.49 |Quadrotors Building a Tower[quadswarm]|

0.49 |Quadrotors Building a Tower[quadswarm]|

|

1 |Quadrotors Building a Tower[quadswarm]|

[2013news]

2012
^^^^

-  `Boston Dynamics announces Legged Squad Support System (LS3) which is
   a militarized variant of Big
   Dog. <http://en.wikipedia.org/wiki/Legged_Squad_Support_System>`__

Our notions about robots are driven by literature, movies and
television. The nearly universal images of robots in fiction have driven
our expectations and to some degree affected the robots we currently
have. The stories present robots in a vast array of situations with a
range of technologies. These robots offer a canvas that opens
exploration of themes where the characters can have dramatically
different abilities or views than human agents. It allows the author to
ask big questions about what it means to be human and that of friendship
or relationships. It also allows the author to suspend all reality by
painting robotics characters as pure evil or immensely powerful giving a
backdrop for character growth. But how is this important? It is because
the role fiction has played, it, as much as the needs of society and
economic forces, influences what we do in robotics.

An Overview
-----------

Robotics as a discipline is often described as an interdisciplinary
field constructed from Mechanical Engineering, Electrical Engineering,
Industrial Engineering and Computer Science. It is fairly new as an
academic area and mostly grew out of Mechanical or Electrical
Engineering programs. Previously, various aspects of the robotics trade
was found in subjects such as kinematics, dynamics, controls,
mechatronics, embedded systems, sensing, signal processing,
communications, algorithms and planning.

.. figure:: overview
   :alt: Robotics is a blend of mechatronics, embedded systems,
   controls, sensing, signal processing, kinematics, dynamics,
   communications, algorithms and planning.

   Robotics is a blend of mechatronics, embedded systems, controls,
   sensing, signal processing, kinematics, dynamics, communications,
   algorithms and planning.

Application domains for robotics is a quickly growing list. We are quite
used to seeing robots in large industrial settings like automotive
manufacturing and palletizing. They have made a name in welding,
painting, inspection, product loading, parts placement and a variety of
other industrial tasks. Hazardous environments (space, underwater,
chemical/nuclear, military) are a significant growth area for mobile
robotics. Applications that manipulated radioactive materials, toxic
chemicals and other hazards have been prime choices for teleoperated
systems since the human operator can be kept safely away. During flu
season, the workplace can be considered a hazardous area and
telepresence can address the issue. A recent version of a standard
teleoperated robot is the surgical robot. This device can follow human
motion but scale it down to be effective in regions where human motor
control is too crude and dangerous. It is like having a gear reduction
in motion leading to more precise and accurate manipulation.

Roboticists often view robots as systems comprised of three components:
**Sensors, Software and Effectors**. In other words, there is
**perception, cognition and actuation**. One could break a text down
into those three major components. Although it has a certain taxonomic
appeal, the reality is that these aspects are intertwined and should be
studied together.

A simplistic taxonomy
~~~~~~~~~~~~~~~~~~~~~

To get started, we use a rather crude taxonomy of robots: **mobile
robots** and **industrial robots**. The mobile systems are best known
through examples like the NASA Rovers and the IED detecting robots of
our armed forces. Industrial robots have been in use for a half of a
century and are well known in manufacturing and more recently with
surgical robots. Typical examples are shown in
Figure [fig:fixedvmobile].

0.49

0.49 |[fig:fixedvmobile]Mobile vs Manufacturing Robots|

It is important to note that partitioning these machines into two
categories ignores the full spectrum of systems available. As the
application areas grow, this distinction will vanish. However, it is
useful at the moment to illustrate some concepts. Useful in that we are
able to isolate various challenges and technologies in existence. Later
we will dismiss the artificial categories and look at mobile autonomous
systems in unified manner.

Robotics built a name in manufacturing. The ability to repeat a task
exactly for thousands or hundreds of thousands of times is essential to
take advantage of scale. It enables a market advantage by keeping
assembly costs down. This may be due to human labor costs, human speed,
human error, human environmental restrictions or some combination.

Thus industrial systems grew out of the need to do a specific task
quickly, accurately and cheaply. These systems live in an instrumented
and structured environment. The task, the interaction between robot and
objects, is understood and predetermined. Highly accurate positioning
for tools, exact tool paths and application of specific tool forces
dominated the designs.

Contrast this view with the mobile machine. By its very intent, this
device leaves the confines of the lab or shop. It moves into new and
possibly unexpected environments. Lack of instrumentation outside the
lab and lack of pre-determined structure removes any possibility of
predetermined interactions. They must be novel and thus requiring a
great deal more from the system. The possible types of interactions are
enormous and as such the machine must not be specifically programmed,
but must be a generalist. Although the precision of interaction and
speed of task may be greatly reduced, the increase in complexity for the
system in the new untamed world is much more complex. It requires
behaviors that mimic intelligence. It is in this arena that computer
scientists can contribute best. The contrasting elements are given in
Table [table-fixedvmobile].

2tableLineOne .. tableLineTwo = ^3mm\_2mm

| to 0.7 Manufacturing & Mobile machines
| Dedicated & General
| Fixed environment & Changing environment
| Predetermined tasks & Adapting tasks
| Fixed interactions & Novel interactions

A less simplistic view
~~~~~~~~~~~~~~~~~~~~~~

The *industrial robot* verses *mobile robot* is one way to partition up
the robot design space, but is one that really does not do justice to
the vast array of creative designs which have emerged. Robots are
machines which help reduce human effort in some manner. We create them
to assist us. Understanding robots in terms of how they are used or how
we interact with them, although rather human centered, is another way to
classify these machines. It is also a way to classify newer systems that
don’t really fit into one of the two boxes described above.

Take, for example, the new surgical robots. These systems are not
mobile. They share many attributes of the industrial robotics designs.
However, these systems operate (pun intended) in a vastly dynamic
environment since no human is the same. These systems are not performing
repetitive tasks but are carefully controlled by the surgeon. A similar
issue arises when you examine the current class of telepresence robots.
They are not autonomous and are confined to simple office environments.
So how should we understand these systems as robots. Or are they?

Let’s try a thought experiment. Say you are a surgeon. The scalpel is
directly controlled by the surgeon’s hands and eyes. That instrument can
be placed on a rod to access difficult regions. Maybe a long linked or
flexible rod. To see in the hard to access regions, we can place a small
video camera. We bundle and run the camera and scalpel through linked
rods and cables. Instead of controlling the position of these
instruments by hand, we decide to control using servos. Because we are
not using our hands to control, we have lost the “feel” of the
instrument interacting with the tissue, so we add some types of feedback
in the grips. We now have a surgical robot. But where did it cease being
a tool and become a robot?

Surgical robots, telepresence robots, and remotely piloted drones all
extend human capability. They extend our reach and our senses. They can
operate autonomously in the limited sense of physical separation from
the human, but not without constant direction. Although they can be very
sophisticated, they are automatons or appliances. We will use robotic
appliance to describe this class of robots which is an extension of us
and not worry so much as to their construction or mobility. Simply that
they are not collaborators with us; merely extensions of the pilot. The
classic industrial robots, cleaning robots and 3D printers easily fall
in this category. Pre-programmed systems extend our work hours by
replicating the programmer’s first successful (remote) run.

The efforts you see with the PR2 or the Baxter show a different trend.
These are robots that are collaborators. They work with us, maybe beside
us, but semi-autonomously. This means that they are not simply
reflecting our directions, but are adding something to create a team and
ultimately something greater than the sum of the parts. These robots are
agents acting independently to some degree. Home care robots and
autonomous vehicles are two such examples. The rise of robot agents is
strictly due to the recent successes in machine learning. It is the new
forms of artificial intelligence that are making robotic agents a
reality, and appears to be in a rapid growth phase.

The value of classifying is to help one understand the landscape. Its
utility ceases the moment it restricts innovation. So we will leave the
classifications behind us and refer to them only when required.

Electronic components of a small mobile robot
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

It is useful exercise to open up a small mobile robot and become
familiar with the hardware. There has been an explosion in options for
robotics. Low cost microcontrollers are immensely powerful. There is an
ever growing list of sensors, actuators and support electronics. This is
very helpful for the computer scientist since one no longer needs custom
equipment to get a mobile system operational. Using USB interfaces, it
is possible to connect the various systems just like we do with Legos.
(Later we will note that USB may not be the best choice due to
electromagnetic interference.) Before we get any further, however, lets
go over the basic terms we need to know for this section.

End Effector

the movable part of the robot, often this is the robotic arm.

the end of the manipulator.

the motor, servo or other device which translates commands into motion.

any device that takes in environmental information and translates it to
a signal for the computer such as cameras, switches, ultrasonic ranges,
etc.

can refer to the hardware or software system that provides low level
control of a physical device (mostly meaning positioning control), but
may also refer to the robot control overall.

the cpu that controls the system. There may be multiple cpus and
controllers or just one unit overall.

all of the code required to make the system operate.

Figure [intro-components] shows the basic hardware elements of a typical
low cost small mobile robot. We can see sensors, software and effectors
in this unit. There are two sensing systems described in Figure
[intro-components]. The familiar sensor is the Microsoft Kinect. The
Kinect is a type of sensor known as a ranger which is any device that
provides distance or range information. It also has a built in camera
which is integrated with unit. The depth sensor returns an array of
distances that are registered with the the pixels in the camera image.
This is very useful because you then have a distance approximation for
features seen in the image and have both 3D reconstruction and color
mapping for a scene.

The second sensor found on this unit is the LIDAR. This is a laser
ranging unit. It does a horizontal sweep (the pictured unit sweeps
roughly :math:`240^\circ` arc) and returns the distances along the arc.
The LIDAR only returns depth information along the arc so can only give
a cross-section of the scene. Placing the LIDAR on a pan or tilt system
then can scan a region if required. Many human environments are just
extensions of a 2D floor plan into 3D by extending the vertical
direction and so a LIDAR is a very useful ranging device.

A camera can be a useful sensor and paired with a second camera the pair
can provide depth of field. Stereo vision for robots works on the same
principles as stereo vision in humans. Since the Kinect does not operate
in sunlight, a stereo camera setup is a cost effective alternate to more
expensive ranging equipment. Other inexpensive approaches use a type of
sonar. An ultrasonic transducer can send a chirp. Knowing the speed of
sound one can determine the distance of an object in front of the sonar
unit.

Simple sensing systems can detect touch or impact (bump sensors for
example). Sensors are available to measure pressure and force. These are
important in manipulation where the object is fragile relative to the
robot gripper. There is a vast array of sensors available measure light,
radiation, heat, humidity, magnetic fields, acceleration, spin, etc.

Autonomy and SAE
~~~~~~~~~~~~~~~~

Autonomy or Autonomous appears quite often in the current press. What
does this mean? Dictionary.com will define this as “acting independently
or having the freedom to do so”. We should be careful to distinguish
autonomous (and probably autonomy) from automated. The root meaning of
autonomy is self-governance verses the idea of automated which is “to
make automatic". Although similar in sound, automatic carries the sense
of preprogrammed or pre-sequenced. The difference being that autonomy
hints at using information from the environment, making decisions to
arrive at some goal, but not programmed in a fixed set of actions.

In common usage, we see autonomous and unmanned as inter-changeable.
Whether or not a person is involved, the idea is that the system can
operate successfully without human guidance. However, a self-driving car
is a significant challenge and the industry is looking at partial levels
of autonomy as achievable goals in the near term. SAE has released
definitions of levels of autonomy for automobiles. This is strictly a
characterization for commercially available ground vehicles. These are
intended to provide a common set of definitions for the industry. A
description of these levels can be found at the NHTSA
(https://www.nhtsa.gov/technology-innovation/automated-vehicles-safety).

+-----------+--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| Level 0   | The human driver does all the driving.                                                                                                                                                                                                                                                                                                           |
+-----------+--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| Level 1   | An advanced driver assistance system (ADAS) on the vehicle can sometimes assist the human driver with either steering or braking/accelerating, but not both simultaneously.                                                                                                                                                                      |
+-----------+--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| Level 2   | An advanced driver assistance system (ADAS) on the vehicle can itself actually control both steering and braking/accelerating simultaneously under some circumstances. The human driver must continue to pay full attention (“monitor the driving environment”) at all times and perform the rest of the driving task.                           |
+-----------+--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| Level 3   | An Automated Driving System (ADS) on the vehicle can itself perform all aspects of the driving task under some circumstances. In those circumstances, the human driver must be ready to take back control at any time when the ADS requests the human driver to do so. In all other circumstances, the human driver performs the driving task.   |
+-----------+--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| Level 4   | An Automated Driving System (ADS) on the vehicle can itself perform all driving tasks and monitor the driving environment - essentially, do all the driving - in certain circumstances. The human need not pay attention in those circumstances.                                                                                                 |
+-----------+--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| Level 5   | An Automated Driving System (ADS) on the vehicle can do all the driving in all circumstances. The human occupants are just passengers and need never be involved in driving.                                                                                                                                                                     |
+-----------+--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+

.. |FANUC 6-axis welding robot. :raw-latex:`\cite{wiki:fanuc}`| image:: robots/HONDA_ASIMO.jpg
.. |FANUC 6-axis welding robot. :raw-latex:`\cite{wiki:fanuc}`| image:: robots/FANUC.jpg
.. |[Figure:robotimagesB]| image:: robots/Actroid.jpg
.. |[Figure:robotimagesB]| image:: robots/IED_detonator.jpg
.. |[Figure:robotimagesC]| image:: robots/Roomba.jpg
.. |[Figure:robotimagesC]| image:: robots/hexapod2.jpg
.. |[Figure:robotimagesD]| image:: robots/KeeponTophatNextfest2007.jpg
.. |[Figure:robotimagesD]| image:: robots/lloyd.jpg
.. |[Figure:robotimagesE]| image:: robots/Laproscopic.jpg
.. |[Figure:robotimagesE]| image:: robots/358px-PR2_robot_at_RoboGames.jpg
.. |[Figure:robotimagesF]| image:: robots/RUNSWift_Naos_2010.jpg
.. |[Figure:robotimagesF]| image:: robots/robonaut.jpg
.. |Quadrotors Building a Tower[quadswarm]| image:: robots/bigdog_arm.jpg
.. |Quadrotors Building a Tower[quadswarm]| image:: robots/google_car.jpg
.. |Quadrotors Building a Tower[quadswarm]| image:: robots/constructioncopter.jpg
.. |[fig:fixedvmobile]Mobile vs Manufacturing Robots| image:: robots/arm.jpg




Touching on the fundamental challenges
======================================

There are plenty of very interesting developments in new materials, new
mechanical systems and electrical systems. Recently the options for
mechanical and electrical components has increased to the point that for
many designs, off-the-shelf options are available. This allows for very
rapid prototyping. A system can be assembled quickly so that developers
may focus on the software and it allows much more time on the software
aspect enabling contribution by software engineers. The control systems
are very mature and are done at the lowest levels. This allows the
developers to move to the highest levels of the software. The
interesting questions from a computer science perspective relate to
robot autonomy.

Autonomy is a significant challenge for those who work in robotics and
artificial intelligence. Sensors can easily provide immense amounts of
data. Understanding this data is a completely different and formidable
issue. Thus we arrive at the fundamental distinction between syntax and
semantics. Autonomous systems need to perceive the world, recognize
objects, know their location and plan their
activities ([intro-autonomy]). Perception of the world around requires
sufficient sensory data to reconstruct the world, but also requires a
conceptualization of the world leading to understanding. Recognition of
objects is essentially the same issue, again requiring
conceptualization. Conceptualization requires a model or framework. A
model is needed for localization and activity planning. Having robust
and flexible models that operate in realtime is a complex task; a task
that we will touch on in detail later in this text.

= [diamond, draw, fill=blue!20, text width=4.5em, text badly centered,
node distance=3cm, inner sep=0pt] = [rectangle, draw, fill=blue!20, text
width=5em, text centered, rounded corners, minimum height=4em] = [draw,
-latex’] = [draw, ellipse,fill=red!20, node distance=3cm, minimum
height=2em] (init) perceive the world; (recognize) recognize objects;
(loc) know location; (plan) plan activity; (init) – (recognize);
(recognize) – (loc); (loc) – (plan);

**Requirement**

-  Have a model of the environment

-  Perceive and analyze the environment

-  Find its position within the environment

-  Plan and execute the movement

**Implementation**

-  Maps and Sensor Data

-  Data filtering and Sensor Fusion

-  Localization, Mapping, Navigation

-  Path planning and Optimal paths

.. figure:: robots/RUNSWift_AIBOS.jpg
   :alt: `Robots in
   RoboCup. <http://upload.wikimedia.org/wikipedia/commons/f/f2/RUNSWift_AIBOS.jpg>`__

   `Robots in
   RoboCup. <http://upload.wikimedia.org/wikipedia/commons/f/f2/RUNSWift_AIBOS.jpg>`__

Autonomy presents additional challenges. The environment is very
dynamic. Objects can enter, leave and change shape. The landscape
changes, location and orientation are unsure. However there are more
subtle issues. Think about how the day progresses. The light changes as
with the angle of the sun. There might be changes in natural versus
artificial light. As the robot moves, the perspective on objects change.
For example, look at your coffee cup (or tea cup ...). As you rotate the
cup, the handle can slip out of view. Now we see a cylinder and not a
mug. Without higher order cognitive functions like object permanence,
the object has changed type.

Modeling the environment is difficult. There are no simple ways to do
this. You may have a compact representation, but the enormous storage
requirements brings large computational complexity. For example, you
might decide to use a simple grid system to mark areas of occupied or
free space. Say the grid is a cube 4 inches on a side. In a typical
warehouse which is 20,000 sq ft by 15 ft high gives us 2.7 million grid
points to filter through. Larger outdoor domains are not possible with
grid based object referencing and so other more complicated storage
approaches are needed.

Another aspect which makes autonomy challenging is the multitude of
sources of uncertainty. Sensors are noisy devices. At times they seem
more like random number generators than physical sensors. From moment to
moment, the picture that an autonomous system has changes due to the
noise of the sensors. The noise needs to be filtered out while keeping
relevant data and doing so quickly.

Navigation and Localization
---------------------------

Navigation is the process of routing the robot through the environment.
Localization is the process of determining where the robot is in the
environment. Most of the robots we imagine can move around. So, we
expect that a mobile robot can navigate its environment. This really
seems pretty simple. After all, worms and insects can do it, so machines
should have no problem. Right? Navigation in three dimensions requires
that the robot have a full understanding of the obstacles in the
environment as well as the size and shape of the robot. Determining a
path through the environment may also come with constraints on the path
or robot pose. Typically to route a robot to some location, the current
location is needed. Clearly just moving and avoiding obstacles does not
require any knowledge of location, but there are plenty of times where
the routing and localization problem are intertwined.

0.49

0.49

Navigation requires sensory information. The availability and type of
information is critical to how effectively the robot can navigate or
localize. Having only sensors that measure wheel location makes
localization difficult and path planning impossible. Dead Reckoning is
the method of determining the speed and run times for the motors. Then
repeating this in different combinations in order to navigate the
course. Essentially this is the game that you memorize your steps and
turns and then try to retrace them with a blindfold. Modifying the
environment allows for much better control of the robot but with the
added costs of environment modification, see Figure [environmentmods].
Dead reckoning normally has very poor results due to normal variations
in motors. Environmental instrumentation can be very successful if
available.

.. figure:: slam/localization
   :alt: Localization can be very difficult. In this example, a LIDAR
   scan is compared to a known map to deduce the location of the robot.

   Localization can be very difficult. In this example, a LIDAR scan is
   compared to a known map to deduce the location of the robot.

The approaches and algorithms are based on the underlying
representations of space. We can represent space as a grid, or a
continuum or an abstract system, Figure [fig:maptypes]. Each method will
determine the way we index the object (integers or floating point
values), the resolution on location and the algorithm for accessing the
object. We could also represent space in a discrete manner. This makes
grid based approaches available. Space could also have a graph
structure. The algorithms to navigate then will use or exploit these
different ways space is represented. The differences give rise to
different performance, accuracy, and results.

0.3 |An example of different map types.[fig:maptypes]|

0.3 |An example of different map types.[fig:maptypes]|

0.35 |An example of different map types.[fig:maptypes]|

Although challenging, navigation is a core skill in mobile robotics.
Autonomous navigation is a focus for many industries. Farming is looking
at conversion to autonomous machines as well as autopilot systems for
automobiles. Of great current interest is a vision based autopilot
system, Figure [fig:visionautopilot]. This is an active area of research
and we touch on it in the next section.

.. figure:: vision/bosch.jpg
   :alt: Vision based driver assist system (Bosch).
   [fig:visionautopilot]

   Vision based driver assist system (Bosch). [fig:visionautopilot]

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

Robot Control
-------------

Assume that you want to build a robot that can deliver mail to the
residents in a elder care facility. This is akin to the drug delivery
robots in hospitals. The halls are straight and corners are 90 degrees.
The layout does not change much over time and the build plans are
available before the robot goes into service. The first temptation would
be to try a form of dead reckoning. Of course it is clear that the
wheels and motors are not ideal or identical. Drift will occur. The dead
reckoning approach, meaning an approach which does not take in position
information is known as open loop control. The open refers to not having
feedback. Open loop control has problems with drift.

To address this problem the system will gather information from sensors
and use this information to update the position. Meaning it will correct
for drift. Not that we are completely stopping the drift since error
will creep in and we cannot eliminate this. However we can adjust the
system and hopefully compensate enough to navigate successfully. Using
the feedback is known as closed loop. It is more complicated than open
loop control but necessary for real world position control.

When one designs and builds a robot, it is natural to focus on the
intended abilities. We think about having the robot perform some set of
tasks. After laying out what the robot should do and what sensory data
it needs, then we tend to think about how we will coordinate those
activities. The coordination of the activities is an important element
in the system design. Much of the way the robot behaves can be traced to
the coordination approach used. There are two ways to proceed here; one
based on a classical artificial intelligence approach and ones based on
newer methods in artificial intelligence.

.. figure:: slam/Control
   :alt: Control system for a simple navigation system which fuses
   odometry and sonar.[odosonarslam]

   Control system for a simple navigation system which fuses odometry
   and sonar.[odosonarslam]

For the classical methods we need complete modeling of the system in the
environment. Typically this is a complete mathematical model of the
different ways that the robot moves: kinematic model, control inputs,
environment description, etc. The approach is then function based and
follows a sequential decomposition of the tasks, see
Figure [robotcontrolclassical]. Independent of how things operate “under
the hood”, we tend to view these systems as interacting with the
environment using a four stage conceptual framework [fourstage].

0.4 |A more traditional approach to robot control.
[robotcontrolclassical]|

0.59 |A more traditional approach to robot control.
[robotcontrolclassical]|

Most of the time the developer will want to code up the robot behaviors.
This may involve a set of actions or reactions to events. They can be
simple rules sets, or finite state machines or very complicated expert
systems. The goal is to impart the robot with enough machine
intelligence so that it can operate in the environment which it is
deployed but keep the code simple enough to run on the onboard
processors. For example, a number of years ago, one of the authors used
a state machine for a simple exploration robot. [intro-statemachine]. In
this case the decision process is completely defined before the robot is
sent out.

.. figure:: slam/StateMachine
   :alt: A finite state machine for an exploration
   robot.[intro-statemachine]

   A finite state machine for an exploration robot.[intro-statemachine]

The Prussian general Helmuth von Moltke the Elder has been paraphrased
in “No battle plan ever survives contact with the enemy.” This is
certainly true for the preprogrammed robots. Unless in the situation of
an industrial robot which has as consistent environmental presentation,
the issues in the natural world are overwhelming. Beyond things like
noise and drift are unexpected objects or events in the world around the
robot. The programmer is hard pressed to anticipate, design and program
for all the contingencies. The sensors can be inconsistent or
unreliable. All of this leads to difficulties in obtaining accurate
position/orientation estimates.

From early in its history, engineers have been dealing with the vast
separate of the perfect world in one’s mind and the messy dirty world we
live in. Tools such as digital signal filters like the Kalman Filter
aimed at cleaning up sensor input or higher fidelity motor encoders to
increase accuracy and resolution have been, and still are, widely
embraced. Fuzzy logic or Bayesian based algorithms gave some measure of
robustness, with the latter being exceptionally effective at dealing
with uncertainty. Recent state of the art systems are a bundle of Sigma
Point Kalman filters, Markov localization algorithms, motion planning
and goal determination routines, actuator control codes, glued together
by some type of interprocess communication. All of this is supported by
some modern OS and middleware.

For fully autonomous mobile robots, such as seen in planetary
exploration, it is impossible for the system designer to anticipate all
of the situations the robot will find itself in. Even when we can
anticipate, we tend to think and use language with significant lack of
precision. This allows us to say things like “drive to the gas station,
turn left and head up until you see ...”, which are easy to say but very
hard to program. Increases in data, mission scope, environment means the
computational task increases at a geometric rate. To address this, we
turn to lessons learned in the biological world. Clearly evolution has
solved these problems in nature and so we engage the tools of natural
computing to solve the problems in robotics. It has been said that the
killer app in artificial intelligence is robotics. Although I believe
this to be true, given the difficulty in defining a robot, the statement
is mostly a catchy one liner.

Google, Nvidia, Amazon, Facebook all have embraced some form of machine
learning as critical to their futures. Some of these approaches are
statistical, but many are biologically motivated. For example,
convolutional neural networks and reinforcement learning are two very
current popular approaches in machine learning. Neither is new, but has
benefited from years of research in algorithmic tuning and massive
increases in hardware performance. The connectionist approaches tend to
be highly parallelizable and see dramatic improvements in performance on
GPUs, FPGAs and DSP hardware (TPUs). Thus modern robot control
architectures see a parallel decomposition of the elements in the
sensing, cognitive and actuation stages of the control algorithm and
reflect the biological roots, see Figure [robotcontrolnewer].

.. figure:: slam/newAIcontrol
   :alt: Newer approaches parallelize the control architecture. The
   details of the final fusion step are discussed later.
   [robotcontrolnewer]

   Newer approaches parallelize the control architecture. The details of
   the final fusion step are discussed later. [robotcontrolnewer]

The strengths of these new machine learning tools are in the ability to
learn, the robustness to faults and errors, as well as a much reduced
human design. Rules or patterns are not programmed in. Cases, especially
edge cases need not to be defined. Kinematic models can be dispensed.
Digital Signal Filters and sensor fusion models may be removed. Having a
system which can learn can by orders of magnitude reduce development
hours for a specific system. The machine learning methods we will
examine in later chapters will mostly be based on biology, specifically
on neural networks or behavioral learning theories.

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

A Few Last Words...
===================

This text aims to survey the subject of robotics. However, that is
complete fantasy. Robotics is a huge field and it is not possible to
really touch on all the different areas, delve into some of them and
keep this text under several thousand pages (and a university course
lasting one semester) as well as keeping your interest. So, we must
compromise. This text will focus more on mobile systems and the
technology to implement them than it will on manipulators (robotic arms
and industrial assembly systems) - but not exclusively.

We will approach the subject from a computer science point of view and
write for a computer science audience. This does not imply that
mechanical or electrical engineers should set this down, just that the
presentation will have a distinctly software orientation. Our coverage
will balance more on higher level systems, machine intelligence,
communications and algorithms with less time towards hardware,
controllers, control systems, mechanics, electrical and materials. In
essence we will see the robot as a type of distributed computing system
but one which is aware of how the input data is gathered (sensory
devices) and how the computational results are used (motors, etc).

As a computer scientist, what do you need to know to get started in
Robotics? The list below provides an overview of the topics we will
touch on.

-  Simulation and Mathematics

-  Behaviors and Motion

-  Mechanics, Kinematics and Controls

-  Electronics, Signals and Power

-  Embedded Systems and Communications

-  Distributed Systems

-  Sensing, Vision and Ranging

-  Planning, Routing, Localization and Navigation

-  Mathematics

So, we begin our course in mobile robotics fundamentals. Robotics
combines mechanical, electrical and software systems and some of these
systems you need to understand as they fundamentally impact each other.
The goal of this course is to develop sufficient background and
understanding in the subject of mobile autonomous robotics so that you
may become involved with a very dynamic growing industry. As Murphy’s
text will indicate, we will break the subject down into three aspects:
perception or sensing, cognition or planning, navigation, localization,
object recognition, and actuation or motion. Perception, cognition and
actuation (sense, plan, act) is a basic theme for this course. However,
there is a bit of the “chicken and egg” problem. The subjects are tied
together. Each one can affect the other. It does not make sense to march
entirely through planning, then through sensing and finish with
actuation, no more than it would make sense to give you all of the lines
of one actor in a play, followed by the next actor, and so forth.

The development of the subject has been a bit of a conversation between
engineers and nature. Writing this book in complete historical accuracy
is an interesting idea, but I bet it would become tedious after a couple
of chapters. Our approach here is to give you a taste of a concept and
put it into practice; then relate it to other concepts. Later we return
and go into more detail, put that into practice and relate it to more
involved concepts. This process will cycle through the sense, plan, act
aspects - just as a real robotic system would. In short, I am applying
Agile development to you. You are being rapid prototyped into a
roboticist.

Supplementary Reading
---------------------

There are many very good books on robotics. The field is well served by
individuals who want to share their knowledge at many different levels
and viewpoints. The important differences are the goals of the books.
Some texts will focus on presenting the mathematics of articulated
manipulators. Some will want to focus on mobile robot path planning.
Others will want to talk about robot controllers using biological
models. All of these points of view are important.

Below is a list of some texts with a brief description on the focus and
audience:

alligator

- This is a great book that focuses on the algorithms behind autonomy.
It presents a more theoretical treatment of mobile systems and does not
spend much time on the classic kinematic tools like the D-H formalism.
For most schools, this would be a graduate level text based on the
mathematics used in the book although this could be used as an elective
in a senior course if topics were carefully chosen.
:raw-latex:`\cite{Choset:2005:PRM}`

- This is a good shorter book which restricts itself to exactly what the
title implies. The text you are reading follows the outline setout by
Siegwart & Nourbakhsh and both are heavily influenced by Choset’s text.
The material on wheels and the associated kinematics is more in depth
than other subjects in the text. Vision, Navigation, Localization and
Mapping are briefly touched upon but supplementary material is probably
warranted. :raw-latex:`\cite{Siegwart:2004:IAM}`

- This text is similar in topics and level to Autonomous Mobile Robots.
Selection between the two would be based on specific topics of interest.
:raw-latex:`\cite{Dudek:2000:CPM}`

- Braunl’s book surveys the field at a level that a junior in most
engineering programs could easily understand. It has a wealth of
information based on the author’s personal experiences. It describes
many projects and systems at a high level but does not delve deeply into
the topics. If the hardware discussed in the text were more mainstream
or current (Arduino, Raspberry Pi, etc), it would make the text much
more approachable.

- Niku’s text is a great text for the more mechanical side of robotics.
There is a wealth of material on kinematic models, inverse kinematics,
and control. There are well done examples for basic kinematics as well.
:raw-latex:`\cite{niku2010introduction}`

Problems
========

[Introduction\_ans]

How would you define a robot?

What are the two main types of robots as presented by the text?

The text has broken robots into fixed or industrial manipulators and
mobile machines. Often the industrial manipulator is performing repeated
tasks or is remotely operated. Although many mobile systems are also
remotely operated, we don’t consider them doing repeated tasks.

Can you think of another way to classify robotic systems? What are the
strengths and weaknesses of the classification?

What problems does a mobile robot face that a stationary robot does not?
What about the other way around?

A stationary robot does not encounter new or novel environments. The
workspace for a stationary robot is relatively fixed. It’s tasks are
predetermined for the most part. A mobile robot constantly moves into
new environments. Even if the task is to be repeated, in a new
environment the details of performing the task will be different.

What are the differences between robots that are considered Mobile
Machines and those that are considered to be Manufacturing Machines?

Do you think the *Robotic Appliance* and *Robotic Agent* partitioning is
a more effective way to classify robots? Why or why not?

List several approaches that industry has used so robots can navigate in
an environment; mentioning an advantage and disadvantage of each.

Describe the information gathered by a RGBD sensor such as the Microsoft
Kinect.

Describe the information gathered by a stereo camera pair.

List out different ways one could assist a robot in navigating around a
building or inside a building when GPS is not an option. [Think
sensors.]

What is an FPGA? What is a GPU? What is a TPU? What are their strengths
and weaknesses compared to traditional CPUs?

What are Isaac Asimov’s Three Laws of Robotics? What do they mean? Are
they complete, meaning are they a sufficient set of rules?

Work in robotics can replace people with machines. This results in job
loss. Discuss the ethics of working in the robotics industry.

Military robotics is a growing industry. Although many systems have a
high degree of autonomy, use of deadly force is left for the human.
Discuss the ethical issues in allowing the robot to make these
decisions.

If an autonomous system by design or error causes an accident, who is
liable?

List a few ways biology has inspired robotics.

.. |An example of different map types.[fig:maptypes]| image:: slam/discretemap
.. |An example of different map types.[fig:maptypes]| image:: slam/metricmap
.. |An example of different map types.[fig:maptypes]| image:: slam/topomap
.. |For humans (and I suppose animals), it is very easy to distinguish apples, tomatoes and PT balls, but not as easy for machine vision systems| image:: vision/apple3.jpg
.. |For humans (and I suppose animals), it is very easy to distinguish apples, tomatoes and PT balls, but not as easy for machine vision systems| image:: vision/tomato.jpg
.. |For humans (and I suppose animals), it is very easy to distinguish apples, tomatoes and PT balls, but not as easy for machine vision systems| image:: vision/redball.jpg
.. |It is easy for a human but hard for a computer to track the road in a variety of lighting conditions and road types.| image:: vision/road.jpg
.. |It is easy for a human but hard for a computer to track the road in a variety of lighting conditions and road types.| image:: vision/road2.png
.. |Localization and Routing| image:: robots/hallway.jpg
.. |Localization and Routing| image:: slam/route.jpg
.. |Compare the structure of a maze to that of a forest scene. Very simple robots can plan a route and escape a maze. Routing through random obstacles in three dimensions is still very difficult for a robot.[mazeforest]| image:: robots/maze.png
.. |Compare the structure of a maze to that of a forest scene. Very simple robots can plan a route and escape a maze. Routing through random obstacles in three dimensions is still very difficult for a robot.[mazeforest]| image:: robots/Forest.jpg
.. |A more traditional approach to robot control. [robotcontrolclassical]| image:: slam/classicAIcontrol
.. |A more traditional approach to robot control. [robotcontrolclassical]| image:: slam/new_old_AI_blend
