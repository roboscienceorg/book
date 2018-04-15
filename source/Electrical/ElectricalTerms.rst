Electrical Terms
----------------

**Voltage** is the electrical pressure. If we use a water pipe as an
analogy, you can think of water pressure in the pipe as the voltage. The
electrical pressure is measured in Volts. The symbol used in computation
is :math:`v`. **Current** is the flow of electrons along a conductor.
Again, using the water pipe analogy, think of water flow in a pipe. The
current flow is measured in Amps and the symbol used is :math:`i`.
**Resistance** is the measure of the difficulty to pass an electric
current through that element. It is denoted by :math:`R` and measured in
units of Ohms. The power in an electrical circuit is measured in Watts
and Watts = Volts \* Amps. About 770 Watts makes up one horsepower. The
fundamental components are given in
Figure \ `[fundamentalcomponents] <#fundamentalcomponents>`__.

| Resistor (Ohms): resists the current flow, :math:`V = iR` (think of a
  narrowing in a pipe)
| |Resistors.[fig:resistors]| |Resistors.[fig:resistors]|.

| Capacitor (Farads): stores energy in an electrical field,
  :math:`i = \displaystyle C\frac{dV}{dt}` (think of a storage tank)
| |Capacitors.[fig:capacitors]| |Capacitors.[fig:capacitors]|

| Inductor (Henrys): stores energy in a magnetic field,
  :math:`V = \displaystyle L\frac{di}{dt}` (think of a flywheel in the
  pipe.)
| |Inductos.[fig:inductors]| |Inductos.[fig:inductors]|

The fundamental law in circuits is Ohm’s Law,
Figure \ `[circuitsohmslaw] <#circuitsohmslaw>`__:

.. math:: V = iR

 where :math:`V` is in volts, :math:`i` is in amps, :math:`R` is in
Ohms.

|Ohms Law. Note the direction of current flow is the opposite electron
flow.[circuitsohmslaw]| |Ohms Law. Note the direction of current flow is
the opposite electron flow.[circuitsohmslaw]|

Current flow can be in one direction or vary in direction. These are
known as direct current (DC) and alternating current (AC).

|[circuitcurrent] Direct and alternating current.| |[circuitcurrent]
Direct and alternating current.|

Electronic devices run on direct current and this is the type of power
delivered by batteries. Large scale power distribution is most
efficiently done using alternating current (and at much higher
voltages). So the power that enters our homes is AC. To get alternating
current down from the high voltage levels that are used in transmission
lines to an outlet, a transformer is used. You have often heard them as
they make that characteristic hum. To convert from AC to DC, another
approach is used. A device called a diode has the property that it
allows current to flow one way, in essence it is an electrical one way
valve, Figure \ `[circuitdiode] <#circuitdiode>`__.

|Diode.[circuitdiode]| |Diode.[circuitdiode]|

A clever connection of four diodes known as a diode bridge reroutes
current so that it flows in one direction only (will still vary, but at
least stay the same direction),
Figure \ `[circuitdiodebridge] <#circuitdiodebridge>`__. This bridge can
also be used to protect inputs to electronic devices in case positive
and negative lines get reversed.

|[circuitdiodebridge] A combination of diodes known as a bridge to
convert alternating current into positive current.|
|[circuitdiodebridge] A combination of diodes known as a bridge to
convert alternating current into positive current.|

The current headed out of the diode bridge flows in one direction, but
the voltage is still fluctuating. Another device is employed, a
capacitor. Using the water analogy, think of the capacitor as a storage
tank. It will smooth out the voltage fluctuations like a pond smooths
out stream flow. These basic circuit devices are used in a common
household circuits such as a power supply,
Figure \ `[powersupply] <#powersupply>`__.

.. figure:: circuit/power1
   :alt: The power supply circuit.[powersupply]

   The power supply circuit.[powersupply]

In this circuit, wall power (alternating current at 115 volts) is fed
into the left side. S1 is the symbol for the on/off switch. The next
device is a 3 Amp (3A) fuse. The high voltage AC is fed into the
transformer (T1) and dropped down to 24 volts (still AC). Next comes the
bridge circuit which re-routes the current flow so we have rectified (or
unidirectional) current flow. Following the bridge is a large capacitor
that will smooth the flow. It still has ripples in the flow (and they
can be large). So the current is fed into a voltage regulator which
significantly smooths the voltage level. The resistors and capacitors
surrounding the regulator (LM317) select the output voltage level. Now
you understand what is inside those bricks that charge your laptop,
phone, camera, etc.