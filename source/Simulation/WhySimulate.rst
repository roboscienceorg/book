Why simulate?
-------------

Learning how to operate any robotic system can be rather rough on your
budget. A sufficiently robust robot that can support the standard array
of sensors, processing and drive system can run well into the thousands
of dollars. And there is nothing more annoying than having a silly
little software error send the robot tumbling down a flight of stairs.
Based on cost and possible system damage, many researchers and
instructors elect to run the robot in simulation. Physical robots can
take a long time to build, configure and get operational. They require
all the details to be just right or it will not power up.

Compare this to simulation. Getting the initial simulation software
running can be very time consuming, but once this is done, changes to
simulation software can be fast. Realistic simulations are very hard
however. It requires that all of the physical parts are carefully
modeled in the code. So the robot design or geometry with masses and
components needs to be included. Next the physics needs to be carefully
entered. All of the servo or motor dynamics, friction, torque,
acceleration, etc involved must be modeled. In many cases, building the
robot is faster than building a very accurate simulation.

So why would one simulate? One answer is that one can focus on a part of
the system and not worry about the entire design to be competed. The
simulation does not need all the components of the robot included and
you can focus on parts of the system. You can swap out parts quickly and
simulate components that don’t actually exist. A simulation can act as a
proof of concept leading to a new design. It can be cheaper than
prototyping. Simulations don’t wear out or run out of battery power.

Of course, your design might, by accident, optimize aspects unique to
the simulation and not work in the real world. The conclusion of many
roboticists is that simulation is a valuable tool in the initial
prototyping stage, but there is no substitute for a physical robot. Much
of this applies to a first course in robotics. Every reader can run
simulations at no cost (well assuming that we are using an open source
package). The simulated robot cannot be broken. It won’t run out of
power. The most valuable aspect is that the simulation can be restricted
to just a few elements making it much simpler. This avoids overwhelming
the novice and so will be our approach.

There are some really great motion simulation packages available. Often
these codes fall under more general motion simulation codes and overlap
gaming systems, kinematics and dynamics codes. These use physics engines
like the

For the roboticist interested in simulating a robot, it is very nice not
to have to look at the simulation code and focus on the control code.
This allows more time devoted to the robotics control problem. However,
if we have a single code base, the code must be designed to be
“compiled" together. This is a challenge in the face of shared
resources. By separating the code into different programs which
communicate via messages, we achieve data encapsulation and security,
modularity, module language independence, and location independence. ROS
will allow us to do just that.

But, looking back over the code, is it really worth go to the trouble of
having separate programs control the robot? Most of us just sit at one
computer and running multiple computers can be challenging. There are
several reasons to consider. Large blocks of code are hard to design,
develop and, mostly, debug. Good software practice would have us develop
classes or modules that address specific functions in the software. So,
separation of the graphics from the control code is good design. Of
course, we can do this without using our socket based design.

One reason for this design is that we can select the best environment
and programming language for the windowing and then the best environment
and language for the control code. The graphics might be written in C++
or Java. The control code could be written in Python.

The windowing code and the driving code are fundamentally different.
They run concurrently yet they operate on different schedules and
interact asynchronously. As we incorporate more detail into the graphics
side, it will require additional resources. It makes sense to assign
these different tasks to different cpus. The graphics side will want to
add maps, simulated sensors and other simulated hardware which needs to
be separate from the control code. When multiple developers become
involved, the interfaces between the device code must be established -
otherwise code chaos will emerge.
