Robotics Design Elements [Chap:Design]
======================================

Robotics Frameworks
-------------------

A *Robotics Framework* is currently a “catch-all” term. To most
roboticists it means a collection of tools to support robotics software
development. Typically a framework will provide some form of
interprocess communication and a collection of hardware drivers.
Interprocess communication is either shared memory and semaphore
wrappers or TCP/IP socket support. [1]_ There are many simulation
systems available. These range from fairly simplistic 2D single robot
with a few obstacles to very sophisticated 3D full physics engine
support systems. It is similar to what is seen in computer gaming. We
will discuss a few of the more popular approaches below.

MS Robotics Developer Studio
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Microsoft Robotics Developers Studio, MSRDS, is a full featured robotics
development environment. It provides support tools for developing
applications, supporting communications, visual authoring and
simulation. MSRDS is a commercial application. The tool includes an
asynchronous runtime environment which supports threading and
interprocess communication. VPL is the Visual Programming Language which
is in the spirit of Visual Studio and Visual Basic. This tool provides a
drag and drop GUI for application development as well as export to C#.
DSSME is a configuration editor to support application configuration and
distribution. VSE, Visual Simulation Environment provides 3D simulation
with physics. Robotics control software may be developed, simulated and
tested without hardware. MSRDS is an active project. It can found at
http://www.microsoft.com/robotics/.

Webots
~~~~~~

Like MSRDS, is a full featured robotics development and simulation
environment as well. It is a commercial application and is more oriented
to instruction/simulation than the others described here. This tool
provides a large choice of simulated sensors and hardware. Robotic
control code can be prototyped in simulation and then ported to hardware
for tuning. The goal is to provide a realistic simulation to reduce
development time using their Model, Program, Simulate, Transfer
approach. Unlike MSRDS, Player and ROS; Webots is more of a real physics
engine, with collision detection and dynamics simulation and less of a
robot OS/communications framework. It can be found at
http://www.cyberbotics.com.

Player-Stage
~~~~~~~~~~~~

Player is a robotics framework. It provides communications and robot
control interfacing. This is open source freely available software.
Player is one of the leaders in the distributed approach to robotic
control software. It provides a network interface to a variety of
hardware devices and systems. Using a client-server approach, it gives
the ability to control any device from any location. This allows
multiple languages and multiple platforms to be used as a single robot
control system; as long as they support sockets (TCP). It is especially
useful in research when the low level software is in C, the sensor
package is in Java and the behavior system is written in Python,
allowing the best tool for the job to be used. Player is still
maintained, but development ceased in 2010 (mostly due to ROS).

Stage is the simulation system that is loosely coupled with Player. They
are separate but have been extensively used together. Stage is consider
a 2.5D (more than 2D but less than 3D) simulation environment. Stage is
oriented towards a world which is described by a two dimensional map of
objects with some height. Fine details in the :math:`z` direction are
not modeled so the tool is not designed for simulation of grasping or
manipulation. Stage is a very popular tool for modeling ground robots
(and multiple ground robots). It has options to be compiled with control
code or communicate with Player via the network interface. In this case,
your control code would talk to Player which interfaces with Stage. The
concept is that you develop your control software interacting with
Stage. Then when ready to deploy, you disconnect Player-Stage and
connect Player to the real hardware.

Player-Stage is a great idea. Getting it to compile and run is rather
difficult. Since development has slowed and many developers have moved
on, finding the right combination of Player version, Stage version, OS
version and library collection can be frustrating. When it compiles and
runs, it is a great tool. It can be found at
http://playerstage.sourceforge.net.

ROS-Gazebo
~~~~~~~~~~

ROS, the Robot Operating System, is an open source robotics framework.
This project grew out of Player and many of the lessons learned with
Player are found in ROS. ROS provides the communication system, a
filesystem, distribution system and several thousand packages for device
support, robot control, machine vision, sensor fusion, mapping,
localization, etc. ROS was supported by Willow Garage (robotics company
out of Stanford), but now maintained by the

ROS is able to connect to Stage however the current focus is on Gazebo.
Gazebo is an open source 3D simulation envirnoment for robotics (which
began life with Player). It includes full physics simulation with the

ROS and Gazebo are extensions in some sense to Player-Stage. The idea of
developing code in simulation then redirecting to real hardware is
essentially the same outside the differences in interface syntax.

Robotics Programming Languages
------------------------------

There is not a “best” robotics programming language just as there is not
a best programming language in general. Arguments about a best language
are left to novices attempting to justify the language they most
recently learned. Programming languages are tools like pliers,
screwdrivers and hammers. It depends on what you want to do, what
resources you have available and your personal skill set. Some languages
are more popular however, like C and C++. The C family is used heavily
since it has a small footprint (C fits on microcontrollers) and is very
efficient. C++ provides the object oriented approach to a code base and
is widely adopted in industry. Recent languages like Java and C# are
popular when the robot has a full computer available as a controller.
One can even find older languages like BASIC and FORTH as well. In this
text we will focus on two: C/C++ and Python. [2]_

Why C/C++ and Python? C is the major language for embedded systems. It
can compile down to very compact code to run on a variety of
microprocessors. C++ is the object oriented extension to C and both
remain in the most popular programming language lists even though they
have been around for some time. Python is an object oriented scripting
language and is very popular as well, especially with regards to
programming education. Another simple reason is that these are the two
languages supported by ROS, the Robot Operating System. The bulk of the
examples in this text are written in Python.

Also, for this text, we will assume that you are running a relatively
current version of Ubuntu (possibly in a virtual machine). Python, C and
C++ are part of the standard Ubuntu distribution. Normally it is not
critical which version of Python is used (Python 2 versus 3).

Software Architectures
----------------------

Consider a typical robot or the architecture of the mobile robot
described in Figure \ `[intro-components] <#intro-components>`__. There
is only one computer involved in the sample system. How could
distributed computing be part of the equation? It is plausible that
parallel computing could be involved, but at first blush we might
suppose that this would be a multi-core issue which is managed by the
operating system. So again, why would distributed systems be of
interest?

At a very low level, we have basic sensor hardware, motor and servo
hardware and communications hardware. These live in a real time world
and have dedicated hardware. These are embedded devices which can then
communicate with the main system through standard interfaces such as USB
or i2c. Each one runs at different frequencies with different control
systems. It is interesting to watch the development of current robot
infrastructures. In some ways it wants to follow the path the
traditional operating systems did over the last 50 years. In other ways
this development benefits from the last half century.

Think for a moment about traditional program design and execution. The
process lives in an address space which is defined as all of the memory
addresses for which the program resides and accesses,
Fig \ `[fig:address_space] <#fig:address_space>`__.

.. raw:: latex

   \centering

.. figure:: os/address_space
   :alt: Address space for a process. [fig:address_space]

   Address space for a process. [fig:address_space]

The underlying operating system will go to great lengths to contain the
process in it’s allocated section of memory. Separate processes are run
in separate contexts where the hardware isolates the code. In addition
to the address space, processes no longer share registers, stack, files,
etc. The OS does this to protect code blocks from each other. Errors in
one system do not corrupt other systems (such as a faulty end condition
on a loop writing to memory). Unintended interactions are greatly
reduced. This is the design philosophy of any modern operating system.
The OS manages the resources and provides the programs the illusion that
they live alone on the computer. Each program runs as if it owns the
entire machine. It appears to the program that it gets the entire memory
system, disk system and networking. The OS does all the heavy lifting.

The other thing the OS is doing is giving the program a common interface
to the hardware below. Meaning that the program (well, the programmer)
need not worry about if the storage device is from vendor A or B. You
don’t need to know anything about the device specifics. There are
general interfaces for the memory system, file system, devices, etc.
This provides portability with software and really reduces the
programming effort. So the OS provides the illusion of an abstracted
computer or a virtual
computer \ `[fig:os-abstract] <#fig:os-abstract>`__.

.. raw:: latex

   \centering

.. figure:: os/abstract
   :alt: The fundamental machine abstraction. [fig:os-abstract]

   The fundamental machine abstraction. [fig:os-abstract]

The common interface is implemented by a series of system calls. These
are very special functions which allow access to the hardware. They are
not traditional function calls since the thread of execution is moved
over to the operating system. The kernel manages access and permissions,
performs the requested function or returns error codes, and the process
execution resumes. A current OS attempts to provide an abstract machine
for a process. Low level OS routines (drivers and modules) are expected
to translate the specifics of talking to a particular piece of hardware
to the general abstracted interface. This means the programmer just
interacts with a generic storage device and is not concerned about the
specific details of that device. Actually, the interface makes one not
even worry about the type of technology, for example magnetic vs solid
state. This same approach is needed in the robotics world. The USB
interface has helped with modular hardware in a limited sense. This
makes the development and maintenance of software much easier. It also
makes the system much more secure and robust. Being able to program
using a fixed set of system calls makes the developer’s job easier which
in turn reduces errors. It means that the tricky part of accessing the
hardware is done by individuals experienced in that domain. The
collection of system calls really defines the OS. Not so much the
collection of software shipped or the choice of desktop GUI.

It begs the question, if the operating system is really designed to
separate processes, then how do they communicate. Processes must have
communication. So various types of interprocess communication have been
devised to support the model of breaking computation into multiple
execution contexts, but still providing a way for the processes to
communicate and coordinate.

For the moment, assume you are going to write your robot control code.
Your code is a large sequence of sensing, planning and moving. The
planning code probably runs on the CPU and at megahertz speeds. The
sensing at kilohertz speeds and the movement at hertz speeds. As
mentioned above you have lots of different activities at different
speeds. We should take a page from the CS history books. We need modular
code. We need code that is interrupt driven. We need to separate the
different components.

Just like with desktop processing, it is neither possible or desirable
to place all of the code into a single address space running on a single
event loop. Even if we could place all of the sensor/actuator driver
routines into the same program, good design demands modular code. It is
essential to break the software into components. Separate them. This is
done for ease of design, maintenance, security, robustness, and fault
tolerance. At times you don’t even have a choice about modularity. The
current state of robotics development is that no single vendor builds
all of the parts for the robot. You must assemble the hardware from
different systems. The drivers for the components are provided. Robotics
systems are too large to write from scratch. They live on top of
existing traditional computing devices. What does this mean?

The first thing we want to address is the separation of data. This is
often approached by data encapsulation approaches found in object
oriented programming. Robotics has grown out of an embedded world
focused on controls. These were real time systems with hard constraints
on response times. By design the real time operating system and the
underlying hardware was not running full operating systems on high
performance computing hardware. So object oriented programming may not
have been viable due to lack of system support. However, now one can get
very powerful machines and full featured operating systems on postage
stamp sized systems. OOP provides ways to limit access of data and deal
with the complexities of large code installations.

We also want to separate the different functional blocks into different
execution blocks. Again OOP support can support the programmer in moving
to concurrent execution of methods. At a lower level, concurrency is
supported by the notion of threads. A thread is an execution context.
This means that the thread has a program counter, registers and a stack,
but may share the address space which contains the data. Multithreaded
programming gives the developer concurrency, but possibly at great cost.
Some of the most subtle and difficult errors can arise when multiple
threads are working on a common data block. Constructs such as
semaphores have been created to manage access to common data regions.
However, semaphores can cause deadlocking or process starvation.

Experience in both OOP and shared memory programming is important to
avoid disastrous results. Another issue is the pace of robotics
software. Systems have become increasing complicated over time.
Expertise in all areas is hard to find. The ability to use external
routines for certain aspects of the system - especially in development
is critical. Having a large collection of functionally distinct modules
makes the software akin to the building blocks found in hardware. Just
as hardware systems are separate but use common interfaces (such as
common pinouts in Arduino, or interfaces such as USB), software systems
need to do the same thing to realize their potential.

Programs then must communicate with other programs using standard
communication channels. One approach is to build each program as a
function in a library or a class. Pushing code into a library can be a
software engineering trap. Development is challenging enough when you
have a huge interconnected codebase and then add hardware uncertainty.
There are a thousand variations to a robot due to the number of sensors,
actuators, and software libraries. One does not want to rebuild the
system each time an update is released. A class will help with
encapsulation. Still, this metaphor is one of single address space
programming (yes, threads can help). Shared memory has been a favorite
due to it’s speed. Even so, it is fraught with danger. A course in
operating systems shows you how shared memory programming can lead to
problems far worse than low performance with the ability to completely
deadlock a system. Another issue is that there are probably multiple
processing units involved which don’t share memory and so threaded
models do not apply.

Multithreaded computation or shared memory programming is not the only
way to proceed. Another form of interprocess communication is known as
message passing. Data and computation requests are actively managed.
Data is packaged and sent off to remote processes; processes which do
not share the address space. These processes can be on different
machines with different operating systems. This is increasingly
important since the sensors and controllers are requiring their own
cpus. Message passing is a way to address the interprocess communication
need and also support multiple CPUs which do not share memory.

To support message passing interprocess communication, we need a way to
send a packet of data to a remote host. The Unix world developed sockets
as a method to send packaged data. Sockets and their supporting
infrastructure are the backbone of the internet. Network sockets are the
foundation of the internet which is probably the largest distributed
system on the planet. Using message passing interprocess communication
built over network sockets, we can build our collaborating process
groups. Sockets allow us to define a standard interface for
communication and then indirectly for computation. Building our software
components on a message passing architecture built on TCP/IP simplifies
the software engineering process. It embraces the robot as a distributed
system from the start. Asynchronous concurrent computing can proceed in
this environment. Scaling the number of devices is easier. Moving to
swarms of robots and having them act as a single system is a natural
outgrowth.

Robotics software followed some of the development seen in the general
computing world. Microcontrollers without an operating system running
programs resident in a single memory space. Adding functions, hardware
and external devices pushed for having more complicated operating system
support. Real time operating systems and desktop operating systems found
their way into robot hardware. As more demands on motion planning
occurred, increasingly powerful machines entered. This was made possible
by the increasing power and shrinking size of the cpu.

Operating Systems development saw large monolithic kernels like
unix, Figure-\ `[fig:os-monolithic] <#fig:os-monolithic>`__. They were
powerful, provided sufficient performance and were complicated.
Protection of resources and program portability became common. A
complicated system call interface was produced to support the separation
of user program from hardware. However, difficulties in development and
debugging lead to layered OS designs such as early NT and
OS/2, Figure-\ `[fig:os-layered] <#fig:os-layered>`__.

.. raw:: latex

   \centering

.. figure:: os/monolithic
   :alt: Monolithic[fig:os-monolithic]

   Monolithic[fig:os-monolithic]

.. figure:: os/layered
   :alt: Layered[fig:os-layered]

   Layered[fig:os-layered]

Separation of code blocks is not complete in either of the previous
designs and so experiments to build a minimal kernel, one which used
message passing to support interprocess communication, was created.
These were known as microkernels since the design promoted moving all
but the bare minimum out of the kernel leaving a very small kernel code
base.

.. raw:: latex

   \centering

.. figure:: os/mkernel
   :alt: Microkernel architecture. [os-microkernel]

   Microkernel architecture. [os-microkernel]

The concept of a micro-kernel is very appealing. So much so that the
Mach and NT kernels adopted the approach. The downfall was performance.
As we embark on robotics development we cannot forget past experience.
Performance drove many systems back to a monolithic design. Certainly
the real time systems that run the hardware need real time code. Linux
and Solaris decided against a microkernel approach and went with
loadable modules. For an operating system, performance or speed is
critical.

So, should we follow the OS path? Is the situation the same? There are
two important differences in robotics. First is the domain of operation
and the second is the measure of performance. The domain for a robot is
the physical world. Mechanical systems operate in the millisecond range.
The gigahertz range is well beyond what can be expected from mechatronic
systems. Any code that interacts with the mechatronic system does not
take the performance hit like what is seen with CPU process groups and
so the benefits of this design stand out. The other aspect is the
measure of performance. Once the processor can respond in time for a
request, speeding it up may have no impact on the operation. Our measure
now turns to the effectiveness of the robot in the task, development
ease, security issues, cost, etc. So, again, we can see the benefit of
message passing architectures.

However, processing sensor data or the planning operations could require
considerable resources and partitioning the code into separate processes
must be done with care. A careful study of data flow and data
dependencies is required. This allows one to exploit available
concurrency. Then the design decisions can be made regarding how to
handle selection of the hardware and the resulting interprocess
communication.

Computer vision can lead to massive amounts of concurrent simple
arithmetic operations. A CPU may not be the best choice. Not that it
cannot be done since most of the time it is. However, we know that
specialized hardware can vastly outperform CPUs when confronted with
structured operations. Use of FPGAs and GPUs are two great examples of
different architectures that have been applied. This type of asymmetric
computing can greatly enhance the performance of a robot which is
simultaneously running vision, navigation and mapping. A system that is
able to distribute different types of computation over asymmetric
processors is now entering the distributed computing realm.

Consider a couple of applications of robotics. One is teleoperation and
another is telepresence (arguably related, but are good examples). One
of the driving forces in robotics is to remove people from dangerous and
harmful situations. To this end, we require that the user is some
distance away. Both applications require local and remote processing,
and both require very robust communication.

A generalized communication system is needed. Something that provides
uniform interfaces and is not dependent on specific hardware; a system
that allows for modules to reside in separate address spaces and even
separate processing units connected over a LAN. This system must be able
to operate in an asynchronous fashion and be tolerant of faults (such as
restarting a module).

Distributed Computation and Communications
------------------------------------------

Sockets provide a bidirectional channel between two processes. Although
one side was setup like a server and one side like a client, this was
basically a point to point type of communication. With only two
processes one could call this peer to peer or client server as well,
however, in this case it is strictly one process to one process. The
socket mechanism underneath is used to implement client server
architectures. This allows many processes to connect into a single
process. So a client server architecture is immediately available.

Robotics is evolving from having completely integrated monolithic
control systems to modular distributed architectures. As the hardware
becomes more powerful and the goals more sophisticated, the complexity
of the control system increases. It is increasing in a superlinear
manner. We may view the workings of robotics software as a collection of
interconnected computations and view the collection in a graph. Nodes
would represent computational blocks, specifically processes.
Interprocess communication is represented by the edges connecting the
nodes in the graph. The connection between two nodes is the point to
point communication we discussed above. A single robot could have many
nodes. Some that control low level aspects like drive motors or wheel
encoder data. Others higher level like processing data for computer
vision algorithms, estimating position or routing the robot over the
landscape.

Many of the nodes will be producing data for other nodes. Some nodes are
producers, some consumers and some are both. The underlying client
server architecture appears to be required. For a particular node that
produces data for several other nodes, it needs to be a server to those
client nodes. With multiple servers running and each delivering a
different service, how should we manage this? The IP system connects to
a host:port combination. So, one would need to know the host:port pair
apriori. The host name might be known, but what about ports? A system
could have external software that uses any particular port range. Having
the vast collection of sensors and user contributed computational nodes
means that a port numbering and classification system needs to be
devised. Unix systems used to have remote procedures bound to port
numbers. This works when there is a limited list. When the list gets
long we need something like the Dewey Decimal system in the library. Of
course we know that as the scale of node types increases, the
predetermined mapping will eventually break.

One might think to run everything through a central server. Of course
this produces a significant bottleneck and will not scale at all. A
centralized system will not work. The system needs to be dynamic and
configurable. However, we need a way to allow the data producer to
connect with the data consumer. A peer to peer connection is desired to
avoid bottlenecks and other network issues related to a single central
server. We also need a way to dynamically map hosts and ports as the
system needs. This means that a database is required. The information
can be centralized or distributed. If scale allows, a centralized system
will have better response bounds since we know exactly how long it will
take to find the required data. A distributed database may require
several requests to get the information.

When a service starts up, it should register itself in a publicly
available database. It would register with a central server and record
that a particular service may be found at host:port pair. When the
client is ready, it can query the central repository, request the
service location and then connect to the correct server. This main
server or master is a nameserver. Having only the job of handing out
names at the start of the service, it does not affect the communications
later on. We will say a particular node with data ( the service) will
publish this data. This means that it registers with the name server and
accepts connections. A client requiring the data will subscript to the
data by requested the publishing node from the nameserver and then
requesting a connection to that node.

Having a specific service with multiple clients can complicate matters.
The point of the service is to produce something, not worry about
communications. So, to address this, a publish-subscribe mechanism can
be built that treats the data as a topic. That topic is available on a
type of message bus. The publisher and subscriber should be separated
and not know about each other. This way one can deal with issues of
scale, broken connections, reconnections and other real world issues
without disturbing either the publisher or subscriber. Of course this
will eliminate request-reply types of communication which should be
addressed using a direct point to point type of channel. These ideas
will be fully developed when we cover the Robot Operating System,
Chapter \ `[ROSChapter] <#ROSChapter>`__.

Before we proceed with building robots, we need to discuss safety, human
interaction and human environments. Robots can be very helpful, capable
even lifesaving devices. However they can pose serious risks which need
to be recognized and addressed. In addition, there are complexities in
working with humans and in human environments which need to be addressed
as well. In this short chapter we examine some of the issues.

Safety
------

Robert Williams, an American Auto Worker.
    Mr. Williams worked at a Ford Motor Company factory in Flat Rock,
    Michigan. He was working on January 25, 1979 with a parts-retrieval
    system that moved material from one part of the factory to another.
    When the robot began running slowly, Williams reportedly climbed
    into the storage rack to retrieve parts manually. He was struck in
    the head by the arm of a 1-ton production-line robot as he was
    gathering parts and killed instantly. He would go down in history as
    the first recorded human death by a robot. William’s family
    successfully sued the manufacturers of the robot, Litton Industries,
    and was awarded $10 million dollars. The court concluded that there
    were insufficient safety measures in place to prevent such an
    accident from happening.

Kenji Urada, a Japanese maintenance engineer.
    Urada worked at the Akashi Kawasaki Heavy Industries plant. On July
    4, 1981, Urada was checking on a malfunctioning robot. He leaped
    over the protective fence and accidentally hit the on-switch,
    resulting in the robot pushing him into a grinding machine with its
    hydraulic arm and crushing him to death. Mr. Urada is the second
    individual to be listed as a death by a robot.

Wanda Holbrook, an American technician.
    In July 2015, Wanda Holbrook, a maintenance technician performing
    routine duties on an assembly line at Ventra Ionia Main, an
    auto-parts maker in Ionia, Michigan, was “trapped by robotic
    machinery” and crushed to death.

When OSHA was founded in 1971, there was an estimated 14,000 job related
fatalities every year. This is roughly 38 deaths per day. In 2017, the
number has dropped to around 12 per day. The vast majority of these
deaths are impact related. The deaths are from falling or impact (struck
by vehicles or other machinery). Next in line are workplace violence,
electrocutions and drowning. Careful design, planning and implementation
of the workspace can prevent injuries and fatalities as well as save
considerable finances.

Large robots have become common in industrial environments and are now
starting to penetrate other markets. Since 1971, OSHA has documented
over 300,000 work related US fatalities. The fatality number for
industrial robotics is much lower at roughly 30 deaths for a 30 year
period, see Table \ `[tab:deathstats] <#tab:deathstats>`__. [However,
this is not a fair comparison since the overall number of workers around
industrial robots is much less that the general work population.] A good
argument can be made that a number of robotic systems place the robot in
the higher risk situation and they have most likely saved more lives
than were lost. The point here, though, is that shipments of robots are
currently exponentially increasing and as these machines move out of
heavy industrial settings, the potential for human injury is
exponentially increasing.

Industrial robots deployed in the automotive sector are powerful
machines. They can strike a human with great force causing fatal
injuries. Even smaller or lower powered systems can cause significant
injury. The power-up process can produce unpredictable positioning and
movement. This has prompted a series of guidelines for the setup and use
of industrial robots. These systems are placed in cages or in blocked
off areas. Fenced regions are setup so that opening the fence door shuts
down power. Isolation gates are common practice to keep people safe. A
standard power-down procedure is required for physical access to the
robot and robot workspace.

An examination of OSHA records and supported by studies in Sweden and
Japan show that accidents don’t occur during the normal operation of the
robot. During normal operation, training and safety barriers protect the
people working around the machines. Accidents occur during programming,
program touch-up or refinement, maintenance, repair, testing, setup, or
adjustment. Problems that arise occur when something unexpected has
happened. No training procedures for the current problem may exist and
the work staff is forced outside their training and expertise. The
situation may be confusing with multiple distractions. Human error
occurs often which has resulted in terrible accidents.

The reports show multiple instances of individuals circumventing safety
systems in place. By jumping fences or crossing safety barriers,
individuals placed themselves at grave risk. Accidents occur during
maintenance when systems are activated while individuals are inside the
robot workspace. Poor decisions to save time, by troubleshooting the
system without proper shutdown caused numerous fatalities and injuries.
Businesses that place extreme pressure to keep on schedule or not stop
the line, setup the culture of skipping normative practices leading to
unsafe decisions.

Robotics Industries Association
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Founded in 1974, RIA is the only trade group in North America organized
specifically to serve the robotics industry. Member companies include
leading robot manufacturers, users, system integrators, component
suppliers, research groups, and consulting firms.
https://www.robotics.org/

Safety standards by the RIA are used as the industry standard. The
national standard ISO 10218 is based on the RIA standard R15.06. This
standard covers hazard identification, risk assessment, actuation
control, speed control, stopping control, operation modes, axis limiting
and all other aspects of robot design and operation.

Once the decision is made to bring in robots, a full hazard
identification and analysis is required. The standard identifies the
following aspects:

#. the intended operations at the robot, including teaching,
   maintenance, setting and cleaning;

#. unexpected start-up;

#. access by personnel from all directions;

#. reasonably foreseeable misuse of the robot;

#. the effect of failure in the control system; and

#. where necessary, the hazards associated with the specific robot
   application.

Industrial robots need to be physically separated from people and the
standard defines the needed infrastructure. These include covering
gears, links, toolheads and electrical systems with panels and when not
possible placing in physical barriers between human work areas and the
robotics hardware. Robots need to have emergency stops and accessible
power-off panels. The physical barriers should automatically stop the
robot when anyone enters the workspace. The system needs to have speed
controls and workspace limit controls. Good software and good interfaces
are needed.

There is no substitute for good training and good policy. Many of the
accidents could have been avoided if workers followed the access rules.
Some accidents arose due to insufficient barriers, markers or space. All
of these threats can be addressed by a careful hazard analysis.

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

Human Environments
------------------

Human and outdoor spaces are messy. They are random, complicated and
dynamic. Operating there is more challenging than in a designed and
predictable assembly line. To complete a variety of tasks, robots need
to understand their location and orientation in space. They need to
sense and understand landmarks, obstacles and free space. In order to do
this in the past, the operating environment needed to be augmented or
instrumented. For example, lines painted on the floor or conduit in the
concrete would be used for directing the robot along paths and hallways.
IR sources, RFID tags or other systems are used for landmarks and by
using stored maps, landmarks would be used for localization. Orientation
could be inferred from the landmarks or if possible a compass.

Systems up to now would instrument the environment to help the robot in
the small confines of rooms and hallways found indoors. Outside the
system might access GPS which can give a rough estimate but lacks the
fidelity needed for indoor navigation. Modifying the environment can be
expensive and intrusive. It might not even be possible for some
locations. Until robots have a very clear understanding for their
surroundings, systems must rely on changing the environment.

To have an effective home robot, the homeowner needs to accept the
augmentation costs or not use the robot. Modern deep learning systems
may bring changes where it is no longer necessary to instrument the
region. Until then, design decisions must include environmental
augmentation.

Cybersecurity Issues
--------------------

Robotics software is complicated. Current design approaches use multiple
cores and cpus. Interprocess communication is done via buses or sockets.
Effectively a robot is a collection of networked nodes. As such it is
prone to all of the security issues found in any distributed system. It
is in effect the IOT (Internet of Things) security problem.

Using Garfinkle and Spafford’s definition of security, that the computer
should behave as expected, there are a number of security issues. [6]_
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
Table \ `[tab:securityplanning] <#tab:securityplanning>`__ outlines the
steps.

.. raw:: latex

   \vspace{-3mm}

2

#. Aspects of Planning

#. Risk Assessment

#. Cost-Benefit Analysis

#. Creating Policies

#. Implementation

#. Validation

The first design stage is security planning. Have a complete
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
humans, Table \ `[tab:systemfailures] <#tab:systemfailures>`__. The
media will latch onto a DEF CON report about “hacking” into the
bluetooth on a tire pressure monitor and then accessing some of the
car’s control system. This leads the media to report that hackers can
break into your car and drive you over a cliff. Although this is a very
real concern in the future, currently there are much more pressing
issues. The likely causes for robot failures for the near future will be
lack of risk analysis and poorly tested software.

.. raw:: latex

   \vspace{-3mm}

2

-  Power loss or surges and battery life

-  Sensor failure or obstruction

-  Loss of network service

-  Loss of human input

-  Water, dust and chemical damage

-  Equipment failure, EMF noise, static

-  Upgrades to underlying software

-  Viruses and poorly tested software

-  Third party (crackers)

-  Misconfigured software

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
Table \ `[tab:costbene] <#tab:costbene>`__. Clearly some of the numbers
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

.. raw:: latex

   \vspace{-3mm}

2 *Cost of loss*

-  Short/Long term lack of availability

-  Permanent loss (accidental or deliberate)

-  Unauthorized disclosure (to some or all)

-  Replacement or recovery cost

*Cost of prevention*

-  Additional design and testing

-  Equipment (hardware and software)

-  User training

-  Performance

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

2

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
economic consequences for a number of people. [7]_ Angry over job loss
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
fallacy is important. [8]_ It is easy for politicians to vilify groups
for their own gain and so countering this behavior will require constant
effort for the near future. The root cause in many cases is inequity in
economics, corruption and unemployment. Addressing these issues will go
a long way in solving the security problems as well as many problems
facing us.

.. raw:: latex

   \FloatBarrier

Problems
--------

.. raw:: latex

   \setcounter{Exc}{0}

]

What are the pros and cons to sharing address space?

How does a system call differ from a function call?

How can a message passing system be used to coordinate two processes?
Show a pseudo-code example.

What are some pros and cons with centralized server architectures for
coordinating multiprocess communication?

Many industrial systems have fences or cages that contain the robot
which prevents human-robot collisions. The cage system is designed to
shut power if the cage door is open. Some of these systems have a key
and lock that prevents restarting. This has been defeated by a worker
closing the cage door after entry, not taking the key and another worker
by accident powering up the robot. Describe some additions to this
system which could prevent this breach of safe operation.

Name two very simple things that can be done to make industrial robots
safer.

List some periods of robot operation that are very risky for humans.
What can be done to mitigate the risk?

What needs to be addressed for humans to comfortably work with robots?

What is the cause of uncanny valley?

Provide an example not in the text of an exploit of a robotics system.

.. raw:: latex

   \Closesolutionfile{Answer}

.. [1]
   If these terms don’t sound familiar, we will discuss them later in
   the text.

.. [2]
   Please don’t send me email telling me that this is three since C and
   C++ are *actually* different languages. Gnu g++ compiles both of
   them.

.. [3]
   National Science Foundation

.. [4]
   In 1971, Albert Mehrabian published a book, Silent Messages,
   asserting these numbers.

.. [5]
   This is a definition by Garfinkle and Spafford, Practical Unix &
   Internet Security.

.. [6]
   The term security has become associated with preventing system
   cracking but secure really means that you can trust the system. You
   may not care about intrusion or data exposure, but you do care that
   the system operates the way you need it to.

.. [7]
   There are roughly 1.7 million trucking jobs in the U.S.

.. [8]
   The fallacy is that new technology eliminates jobs overall. New tech
   just displaces jobs to new sectors.
