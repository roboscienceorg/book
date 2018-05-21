Cybersecurity Issues
--------------------

Robotics software is complicated. Current design approaches use multiple
cores and cpus. Interprocess communication is done via buses or sockets.
Effectively a robot is a collection of networked nodes. As such it is
prone to all of the security issues found in any distributed system. It
is in effect the IOT (Internet of Things) security problem.

Using Garfinkle and Spafford’s definition of :index:`security`, that the computer
should behave as expected, there are a number of security issues. [#f1]_
The problems listed above in this chapter are specific examples of
security problems.

The most advanced robotics software is based on ROS, the Robot Operating
System, which is covered in detail in the next chapter. ROS is not an
operating system, but a middleware layer and software collection. ROS
manages socket communication over multiple nodes. ROS currently only
runs on Linux. Unix and thus Linux, was not designed with security in
mind. It was designed for ease of use, flexibility, extensiblity and an
expectation of user sophistication. Much of Unix was developed by
students and researchers to facilitate projects and not as production
(engineered) code.

So this means that ROS inherits any security issues found in Linux
(Ubuntu). The steps, processes and procedures that any security
professional working on a distributed network of Linux systems would
take are steps that need to be considered in any ROS based system. The
principles are similar,
Table :numref:`tab:securityplanning` outlines the
steps.



.. _`tab:securityplanning`:
.. table::  Security Planning
   :align:  center
   :widths: auto

   +----------------------+----------------------+
   | Aspects of Planning  | Creating Policies    |
   +----------------------+----------------------+
   | Risk Assessment      | Implementation       |
   +----------------------+----------------------+
   | Cost-Benefit Analysis| Validation           |
   +----------------------+----------------------+

The first design stage is :index:`security planning`. Have a complete
understanding of the goals or tasks for the robot. This normally means
having something like a CONOPS document (concept of operations
document). This will allow you to determine the overall planning
process. Yes, I just suggested that you plan your planning. The idea is
that you don’t want to miss any aspect of the system level view. Having
system engineering expertise on the team can be quite valuable here.

Based on a concept of use, the next step is to perform a thorough risk
assessment. We have discussed a few risks above. One must look at
various scenarios such as loss of the robot, loss of data, loss of
control, etc. Depending on application, we may be concerned about
harming individuals or loss of collected data or lost revenue due to
down time. For surveillance robots, loss of data integrity or real time
feeds can shut down a mission, but for deep space robots there might not
be any expectation of having a continuous communication. You may want
the data to be confidential as with military robots or continuously
available interaction. There are some core questions you should ask:

-  | What am I trying to protect?
   |  [Human life and limb, data, control, expensive hardware, ...]

-  | What do I need to protect against?
   |  [The elements, environmental variation, hostile humans or
     software, ...]

-  | How much money, time and effort am I willing to spend to obtain
     adequate protection?
   |  [Cost of loss vs cost of protection.]

Immediately after you can perform the following steps:

#. Identify assets

#. Identify threats

#. Calculate risks

Threats come in many forms. Most of them are not related to malicious
humans, Table :numref:`tab:systemfailures`. The
media will latch onto a DEF CON report about “hacking” into the
bluetooth on a tire pressure monitor and then accessing some of the
car’s control system. This leads the media to report that hackers can
break into your car and drive you over a cliff. Although this is a very
real concern in the future, currently there are much more pressing
issues. The likely causes for robot failures for the near future will be
lack of risk analysis and poorly tested software.


.. _`tab:systemfailures`:
.. table::  System Failures
   :align:  center
   :widths: auto

   +----------------------------------------+--------------------------------------+
   | Power loss or surges and battery life  | Equipment failure, EMF noise, static |
   +----------------------------------------+--------------------------------------+
   | Sensor failure or obstruction          | Upgrades to underlying software      |
   +----------------------------------------+--------------------------------------+
   | Loss of network service                | Viruses and poorly tested software   |
   +----------------------------------------+--------------------------------------+
   | Loss of human input                    | Third party (crackers)               |
   +----------------------------------------+--------------------------------------+
   | Water, dust and chemical damage        | Misconfigured software               |
   +----------------------------------------+--------------------------------------+


You may be able to remove a risk by changing the way a feature is
implemented but in some cases it requires removing a feature (or ability
in the case of robots). Maybe it is the decision on physical barriers
instead of the sensors and controls required to work safely around the
robot. Keep in mind that engineering is not about providing the coolest
and newest technology. Good engineering like good design is about
solving problems. Sometimes the best solution is not the highest tech
solution.

Cost-Benefit analysis takes the risks and converts them to cost by
estimating the cost of the threat if it occurs. In the cases you can
estimate the cost and compare this to the cost of building a solution
which avoids that particular threat,
Table :numref:`tab:costbene`. Clearly some of the numbers
are very rough. Estimating the time and parts to build some system comes
from experience. If multiple systems are built then the design and
testing costs can be prorated over all of the production units leaving
just the parts and assembly costs for the units. If development costs
are :math:`x` and per unit cost is :math:`y` then the cost of the threat
mitigation is :math:`x + Ny` where the number of units is :math:`N`. The
threat risk is multiplied over the number of deployed units, :math:`N`.
If the threat probability per unit for the lifetime of the device is
:math:`p` and the cost of that threat is :math:`z`, then you are
comparing :math:`Npz` to :math:`x+Ny`. This is the most simplistic way
to view the analysis and more detailed studies should be done.


.. _`tab:costbene`:
.. table::  Cost Benefit Analysis
   :align:  center
   :widths: auto

   +------------------------------------------+--------------------------------------+
   |  *Cost of loss*                          |  *Cost of prevention*                |
   +------------------------------------------+--------------------------------------+
   | Short/Long term lack of availability     | Additional design and testing        |
   +------------------------------------------+--------------------------------------+
   | Permanent loss (accidental or deliberate)|  Equipment (hardware and software)   |
   +------------------------------------------+--------------------------------------+
   | Unauthorized disclosure (to some or all) |  User training                       |
   +------------------------------------------+--------------------------------------+
   | Replacement or recovery cost             | Performance                          |
   +------------------------------------------+--------------------------------------+



Although companies will assign a cost to loss of life and limb (based on
litigation and settlement amounts), we will assume this cost is higher
than the cost to prevent or avoid the risk. In this case you have hard
limits on the requirements that need to be enforced. Once the
cost-benefit analysis is complete, you will have an updated set of
requirements. In addition you can set guidelines for how the software
system will be designed and managed. Some of this will be implemented in
a set of security policies. Often these are very simple tasks like
making sure software is configured correctly. The last stages of the
planning process involve a careful design with clear test cases at each
stage to validate the design.

Network Security
~~~~~~~~~~~~~~~~

ROS based robots are a collection of networked nodes. Many systems have
wifi or bluetooth access. This opens the door for unauthorized access.
We strongly suggest getting a network security expert to advise the team
on design before the system goes to production. This is not a security
text, but the issues you are addressing are common security problems.
There are two types of access one can have: passive and active. Passive
access is worried about intercepting data. Active access is about
modifying machine behavior and is a direct host attack.

Passive:

-  Network wiretapping

-  Port scans and Idle scans

Active:

-  Denial-of-service attack

-  Spoofing

-  Man in the middle

-  ARP spoofing

-  Smurf attack

-  Buffer overflow

-  Heap overflow

-  SQL injection

Careful design, attention to details and good testing can go a long way
to prevent security issues. In many cases it is just a matter of just
getting it on the “to do” list and not difficult or expensive.

Adversarial Machine Learning and other attacks
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

New robots will need to address a whole new generation of attacks. These
attacks will be presented against the sensors and software in novel
ways. Several possible attacks are outlined below to illustrate the vast
array of issues the roboticist must address.

Insecure Embedded Devices
^^^^^^^^^^^^^^^^^^^^^^^^^

In 2008, the National Highway Transportation Safety Administration
mandated direct tire pressure monitoring. Indirect systems measure the
rotation speed of the wheel. Direct monitors have a pressure sensor
built into the wheel and transmit a tire pressure to the vehicle
electronics. In 2010, it was demonstrated that it was possible to hack
into the tire pressure monitor system for automobile tires. The study
showed that from this entry point, vehicle systems could be disrupted or
even controlled. Examples of shutting down brakes selectively, stopping
the engine and other hacks were described.

Like many IOT or other embedded devices, security is not implemented.
Classically for embedded devices it made sense. Embedded systems are
were isolated from other systems. But with the advent of bluetooth, wifi
and other wireless communications appearing on embedded hardware, they
become open to intrusion and manipulation. To address this, all wireless
communications should be encrypted. Even simple systems like an outside
temperature monitor. The point there is that the sensor engineer cannot
predict how the data will be used in an autonomous system. The hackers
may find that the right combination of false sensor readings causes the
vehicle software to make a catastrophic decision. The encryption will
also help in terms of a direct attack to load malicious code into any
vehicle system in a manner similar to the cyberattacks discussed above.

Computer Vision Vulnerabilities
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Computer vision is an active area of research which has shown great
progress in the last decade. Since 2012, we are seeing the transition of
computer vision systems from feature based approaches to deep learning
approaches. Deep learning (or machine learning) algorithms are not well
understood. In 2016, CMU showed they could defeat state of the art face
recognition algorithms. It is clear that the neural network based vision
system could be confused or mislead by correctly constructed patterns.
Neural network approaches are trained in a manner that means the
resulting decision system is not transparent. Testing is harder, often
statistically based, and systems can be shipped with significant issues
in vision accuracy or object recognition.

Sensor Compromise
^^^^^^^^^^^^^^^^^

In addition to vision, many autonomous vehicles currently use lidar and
gps. Lidar, or laser ranging, uses reflected laser light to determine
the distance of objects. Interference from other light sources can
causes errors in distance estimation. Use of laser pointers or other
sources overlapping the same frequency as the lidar could blind the
device. GPS spoofing can be done by sending false signals to the GPS
satellite receivers. Currently spoofed signals are hard to detect and so
false readings for position (and so velocity) are possible.

Motivation
^^^^^^^^^^

Who are the actors? Consider the fear and anger with the vision of the
future that eliminates so many jobs. Autonomous delivery vehicles,
autonomous long distance trucking and transport all have very real
economic consequences for a number of people. [#f2]_ Angry over job loss
has in the past led some to strike out at employers. Fear of a new
technology can lead to preemptive strikes. Bored kids or anti-technology
zealots as well as all forms of terrorists can find ways to exploit the
autonomous systems. The angry unemployed Teamster can cause financial
harm to a company by wrecking some of the fleet. The Luddite can cause
vehicles to go astray to make robotics tech seem dangerous in an attempt
to sway public opinion. The terrorist can take over the navigation
remotely and drive the truck into the crowd; even coordinating a fleet
for a large impact and very deadly attack.

It is important that robotics organizations provide options and
retraining for displaced workers. Public education on the Luddite
fallacy is important. [#f3]_ It is easy for politicians to vilify groups
for their own gain and so countering this behavior will require constant
effort for the near future. The root cause in many cases is inequity in
economics, corruption and unemployment. Addressing these issues will go
a long way in solving the security problems as well as many problems
facing us.

.. [#f1] The term security has become associated with preventing system
   cracking but secure really means that you can trust the system. You
   may not care about intrusion or data exposure, but you do care that
   the system operates the way you need it to.

.. [#f2] There are roughly 1.7 million trucking jobs in the U.S.

.. [#f3] The fallacy is that new technology eliminates jobs overall.
   New tech just displaces jobs to new sectors.
