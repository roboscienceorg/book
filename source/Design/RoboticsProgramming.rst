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
