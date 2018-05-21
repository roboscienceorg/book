Electric Motors
---------------

Although motors are actuation and not sensing devices, since they are
electrically powered, we address them here. An electric motor is in
concept a very simple device. Any time current is flowing through a
wire, a magnetic field is generated around (orthogonal to) the current
flow direction. By wrapping the wire into a coil, the field lines
overlap and intensify the magnetic field. This is the basis of an
electromagnet. The electromagnet will generate a force on a permanent
magnet with the direction of the force depending on the magnet pole and
current direction on the electromagnet. Placing the electromagnet on a
pivoting or lever arm as shown in
:numref:`electricmotor`, a rotational force can be generated.

.. _`electricmotor`:
.. figure:: ElectricalFigures/electricmotor.*
   :width: 70%
   :align: center

   Basic electric motor.

As described, the electromagnet will just align in the permanent magnet
field and oscillate to a stop. The magic is to switch the current
direction right as the moving electromagnet lines up. Then what was an
attractive force switches to a repulsive force. The momentum of the arm
will push past the alignment and the repulsive force will accelerate the
arm towards the opposite alignment. Then one switches the current again
repeating the process. The rotating arm will accelerate until the
frictional forces balance with the magnetic forces.

There are many types of electric motors and what was presented is a
simple inductive motor. Earlier designs would switch the current flow
using contacts on the rotating shaft. Now electronic switching can be
used. Motors which have this mechanical switching are referred to as
brushed motors (the contact is a metal “brush”) and ones that don’t use
them are called brushless motors. Motors can be run off of direct
current as described above, a DC motor, and can run off of alternating
current, an AC motor. Different designs provide motors with different
speeds (revolutions per minute), power requirements and different torque
properties.

A servo is an electric motor, some electronics and some gears. A signal
is sent to the servo, normally a PWM signal. This signal is modified for
the particulars of the servo operation and the specific motor. Normally
this means that the PWM encodes a servo angle and this needs to be
translated into the correct signals to position the motor. Motor
position in low cost servos is read by a potentiometer (a variable
resistor) which then by using some control logic adjusts the position
according to the servo signal. A rough schematic is given in
:numref:`servointernals`.

.. _`servointernals`:
.. figure:: ElectricalFigures/servo.png
   :width: 70%
   :align: center

   Servo internals.
