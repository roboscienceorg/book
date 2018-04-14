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
