Why simulate?
--------------

Learning how to operate any robotic system can be rather rough on your
budget. A sufficiently robust robot that can support the standard array
of sensors, processing and drive system can run well into the thousands
of dollars. And there is nothing more annoying than having a silly
little software error send the robot tumbling down a flight of stairs.
Based on cost and possible system damage, many researchers and
instructors elect to run the robot in simulation. Physical robots can
take a long time to build, configure and get operational. They require
all the details to be completed or it will not power up.

Compare this to simulation. Getting the initial simulation software
running can be time consuming, but once this is done, changes to
simulation software can be fast. Approximations of the system and the 
environment can be done in a  reasonable time frame with the possibility
of increasing accuracy later.  However, high fidelity simulations are hard
to do, it requires that all of the physical parts are carefully
modeled in the code. So the robot design or geometry with masses and
components needs to be included. Next the physics needs to be carefully
entered. All of the servo or motor dynamics, friction, torque,
acceleration, etc involved must be modeled. When physical accuracy is needed, building the
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
like the Open Dynamics Engine which can simulate the kinematics, forces,
collisions and other physical phenomenon.  Before we get to far in this 
discussion, we should clarify the language used in this chapter.  
You will hear terms like modeling, simulation, enulation and animation.
In this text, **modeling** will refer to a mathematical model of the physics.
This can refer to the kinematic constraints or equations of motion.   
Modeling can be used to refer to 3D solid object models.  For this
text, 3D modeling is infrequent enough that we will mention in explicitly.
**Simulation** will refer to the numerical computation of the model 
with specific values for parameters.   It is purely computation and does 
not imply any graphical aspect although in most cases we will want a 
plot or diagram to represent the results.  The goal of simulation is
to produce a numerical (again often accompanied by graphical output)
result that matches what one would observe with the physical system.
Emulation is a term used currently in software to go one step 
further.   Instead of just performing a computation at the mathematical
level, you simulation the underlying instruction set of the foreign hardware (that would be computing
the formula).   An emulation does not model or simulate the phyics of the hardware.   
Since we are working with the physics, simulation is the term used.
**Animation** will be reserved when we want to show the user some
behavior.  It may be the result of the modeling and simulatio or
it can be purely graphical and not physically realistic.   


For the roboticist interested in simulating a robot, it is very nice not
to have to look at the simulation code and focus on the control code.
This allows more time devoted to the robotics control problem. However,
if we have a single code base, the code must be designed to be
“compiled" together. This is a challenge in the face of shared
resources. By separating the code into different programs which
communicate via messages, we achieve data encapsulation and security,
modularity, module language independence, and location independence. 
Message passing architectures 
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
and programming language for the simulation, the best language for the control code and also for the 
windowing code. The graphics might be written in Java. The control code could be written in Python, and the numerical
solvers written in Julia.
The windowing code and the driving code are fundamentally different.
They run concurrently yet they operate on different schedules and
interact asynchronously. As we incorporate more detail into the graphics
side, it will require additional resources. It makes sense to assign
these different tasks to different cpus. The graphics side will want to
add maps, simulated sensors and other simulated hardware which needs to
be separate from the control code. When multiple developers become
involved, the interfaces between the device code must be established -
otherwise code chaos will emerge.


As stated before, producing motion in a real robot is not difficult.
Deciding on the
correct actions and controlling the system are the more challenging
aspects. The first is known as *Motion Planning* or just *Planning* and
the second is called *Controls*. To get started we will borrow
algorithms from nature since we see so many successful autonomous agents
in the biological world. Worms and insects are very successful animals.
They can sense the world and move around in it. We can borrow from
notions in physics and chemistry when we see simple systems moving in
constrained manners. The simplest solution is the best solution. It is
best to use no more components or technology than necessary. Beyond
basic elegance is the fact that the more components something has, the
greater probability the system will fail. This is true in our
simulations as well.

We return to the two examples in the previous section, the Two Link
Manipulator and the Mobile Disk Robot. Using these two systems, we will
introduce methods to simulate motion. These very basic systems can be
used as the prototypes for developing a simulation and for the simple
motion planning algorithms.   
