
Locomotion
----------

The word locomotion means “The act of moving from place to
place.” :raw-latex:`\cite{dictionary}` from Latin combining location and
motion. Biology has explored a variety of very interesting ways to move
around. Animals can be carried on currents, swim, crawl, slide, walk,
run, jump, and fly.

Most of the locomotion solutions found in nature are hard to imitate
with machines. Although legs are the base for human locomotion, humans
have valued wheels in their motion solutions. The earliest recorded
appearance of wheels is in Mesopotamia (Sumerian) at the mid-4th
millenium BC. The is evidence of independent discovery in the new world,
the lack of domesticated large animals prevented any development beyond
children’s toys. :raw-latex:`\cite{wiki:wheel}`.

Rolling is very efficient, especially compared to dragging or carrying
materials. At the macroscopic scale, nature has not developed wheels.
This is not surprising since the wheel needs to be disconnected from the
rest of the system for free rotation reasons, but would then not have
the required nutrient supply (blood or something similar) to grow,
develop and maintain the structure. Although nature did not evolve large
wheels, human motion has some similarities to rolling - a rolling
polygon when motion models are examined.

We are no longer bound to wheels as the only choice for motion. We can
implement a number of nature inspired motion solutions. The type of
motion used by a robot is often the most notable aspect of the machine.
Certainly a great deal of interest and entertainment can be found in
implementing novel locomotion into a robot.

As a robotics engineer, you may be asked to choose the type of motion,
meaning you must choose “fly, hop, swim, walk, roll ...” Often the
environment decides for you, for example, if you must operate in the air
or water, or in very rough terrain. You may have other constraints
involved like power consumption, weight or robustness. These constraints
will normally push the design towards one locomotion system.

When looking at a wheel verses an articulator there are some standard
issues that must be addressed. Articulation is much more complicated in
design, control and expense (both energy and financial). Legs
(articulators) require more actuators and thus more control components.
The control system is more complicated than with a wheeled design.
Another consideration is energy. Articulators move their center of mass
for locomotion and may have to move a significant amount of hardware.
Thus there is internal mass movement along with the external mass (the
vehicle). Wheels by their very design keep the center of mass at a
constant distance from the ground. This reduces power usage.

The tradeoff for the efficiency gain is that articulated motion has the
possibility of enhanced environmental robustness. This is clear when
watching any household spider run across a textured ceiling. The most
efficient wheel, the rail wheel is also the least robust in that it does
not operated outside an instrumented environment. Cars use wheels that
can run on a road or flat ground. To go beyond this we need to bring the
rail or road along with - the idea behind tracks. Track systems can
operate in a larger set of environments, but at cost of energy.


.. figure:: MotionFigures/energyusegraph.*
   :width: 60%
   :align: center

   The relations between energy, speed and motion
   type.[motionenergyspeed]
