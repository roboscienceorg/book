Problems
--------

.. raw:: latex

   \setcounter{Exc}{0}

[Electrical_ans]

Provide the name and circuit diagram for the following:

#. The circuit element that allows current to flow in one direction.

#. The device that can boost or reduce AC voltage sources.

#. The basic elements that store energy in electrical or magnetic
   fields.

#. The device that reroutes current from ac to dc.

#. The device that prevents oscillations in mechanical switch circuits.

Provide a labeled circuit/hardware diagram for a system that has a
microcontroller driving a brushed DC motor (based on motor encoder
output so that the system can control the actual speed and direction of
the motor using a simple feedback based control loop). Assume that the
stall current of the motor and the motor operating voltages are well in
excess of what the microcontroller can source. [Note: Your
microcontroller has PWM, GPIO, I2C, UART, lines for this application.]
Explain each part of the diagram.

Assume that you can provide input for a motor controller in terms of
percent of duty cycle (0-100): u. Also assume that at a 10 Hz rate you
get a reading from an encoder that provides the output rpm of the wheel.
Write a function that controls the rpm (range is 0 to 350), based on the
value in u.

NA

What is a PWM signal?

Can you think of a circuit to accept DC power which could hook up to the
batteries either way. [Meaning that if the user hooks up the wires
backwards, it automatically still works].

In an H bridge, are there switch combinations that cause problems? Why
or why not?

When robots are rolling down a hill, the electric motors can act as
generators. The current generated may damage the motor controllers. Is
there a design that might be able to route the generated power to an
on-board battery charger? Provide a circuit.

.. |Resistors.[fig:resistors]| image:: circuit/Resistor.jpg
.. |Resistors.[fig:resistors]| image:: circuit/resistor
.. |Capacitors.[fig:capacitors]| image:: circuit/Capacitor.jpg
.. |Capacitors.[fig:capacitors]| image:: circuit/capacitor
.. |Inductos.[fig:inductors]| image:: circuit/Inductor.jpg
.. |Inductos.[fig:inductors]| image:: circuit/inductor
.. |Ohms Law. Note the direction of current flow is the opposite electron flow.[circuitsohmslaw]| image:: circuit/Ohms
.. |Ohms Law. Note the direction of current flow is the opposite electron flow.[circuitsohmslaw]| image:: circuit/ohms-law-illustrated
.. |[circuitcurrent] Direct and alternating current.| image:: circuit/dc
.. |[circuitcurrent] Direct and alternating current.| image:: circuit/ac
.. |Diode.[circuitdiode]| image:: circuit/diode
.. |Diode.[circuitdiode]| image:: circuit/diode-rect
.. |[circuitdiodebridge] A combination of diodes known as a bridge to convert alternating current into positive current.| image:: circuit/diodebridge
.. |[circuitdiodebridge] A combination of diodes known as a bridge to convert alternating current into positive current.| image:: circuit/acdc
.. |a) Lead-Acid cell. b) Li-Po c) Battery circuit symbol. [leadacidlipo]| image:: robots/battery3.jpg
.. |a) Lead-Acid cell. b) Li-Po c) Battery circuit symbol. [leadacidlipo]| image:: robots/battery.jpg
.. |a) Lead-Acid cell. b) Li-Po c) Battery circuit symbol. [leadacidlipo]| image:: circuit/battery_symbol
.. |Basic power delivery.[basicpower]| image:: pwm/battery
.. |Basic power delivery.[basicpower]| image:: pwm/batteryswitch
.. |(a) Resistor to limit current flow and drop voltage. (b) A voltage divider circuit.[voltagedivider]| image:: pwm/batteryresistor
.. |(a) Resistor to limit current flow and drop voltage. (b) A voltage divider circuit.[voltagedivider]| image:: circuit/vdivider

