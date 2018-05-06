ROS - The Robot Operating System
================================

| The official ROS website defines ROS as follows:
| *ROS (Robot Operating System) provides libraries and tools to help
  software developers create robot applications. It provides hardware
  abstraction, device drivers, libraries, visualizers, message-passing,
  package management, and more.*

We present a brief summary of ROS. It should be noted that it fails to
convey the sheer power and complexity of the tasks that it performs, and
the ways in which it goes about doing so. The reader is strongly
encouraged to look at some of the very good recent texts on
ROS:raw-latex:`\cite{okane:2014:GIR, quigley:2015:PRR}`.

It is slightly misleading that ROS includes the phrase “operating
system” in the title. ROS itself is not an operating system in the
traditional sense, but it is much more than just a piece of software.
The many components combine to form an ecosystem of software which
warrants its title but is best thought of as middleware. While on the
“not” topic, ROS is not a programming language, not an IDE (integrated
development environment) and not just a library of robotics codes.

As mentioned, package management and hardware abstraction are just a
couple of features under the ROS umbrella, which support the
communication framework around which ROS is based. The intent is to
create a universal system which promotes code reuse and sharing among
multiple robotic platforms, operating systems, and applications as well
as small program footprints and efficient scaling. These pillars form
the core goals of ROS as a whole.

.. toctree::
   :maxdepth: 2

   Origins
   ROSInstallation
   FundamentalROS
   ROSCommunication
   Problems
