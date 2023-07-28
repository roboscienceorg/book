Basic Power Delivery and Motors
--------------------------------

PWM
^^^^

Say that you have a basic electric light circuit (like a flashlight),
:numref:`basicpower`-(a). We are able to turn this on
and off using a switch, :numref:`basicpower`-(b/c).

.. _`basicpower`:
.. figure:: MotionFigures/basicpower0.*
   :width: 80%
   :align: center

   Basic power delivery a) Direct connection  b) Open switch  c) Closed switch.

This is a great system until someone asks to dim the light. Now we are
faced with how to reduce the voltage across the light. One might think
that a transformer could be used. The problem is that the transformer is
an AC not a DC device. So, the next idea is to limit current by placing
a device along the flow of current that will resist the current flow. We
can use the component described above called a resistor.

.. _`voltagedivider`:
.. figure:: MotionFigures/basicpower1.*
   :width: 60%
   :align: center

   Power control. (a) Resistor to limit current flow and drop voltage.
   (b) A :index:`voltage divider` circuit.

The problem with this design is that some of the energy is wasted as
heat in the resistor. For low power circuits, this may not be a problem,
but for higher power devices like for electric motors, considerable
energy is wasted as heat. Current through the resistors in
:numref:`voltagedivider`-(b) is

.. math:: \displaystyle i = \frac{V}{R_1+R_2}.

Voltage drop across :math:`R_1` in
:numref:`voltagedivider`-(b) is

.. math:: \displaystyle V_{R_1} = \left(\frac{R_1}{R_1+R_2}\right)V .

Power is

.. math:: \displaystyle W = i*V_{R_1} = R_1\left(\frac{V}{R_1+R_2}\right)^2

**A quick example:**


Assume we are using a 12V power source and we want to use a voltage
divider to provide 9V, :numref:`divider12to9`.

.. _`divider12to9`:
.. figure:: MotionFigures/vdivider2.*
   :width: 15%
   :align: center

   Voltage divider to drop 12V to 9V.

Assume that the load is a simple resistor with resistance 10 ohms. Since
:math:`R_2` is in parallel with the load, we get an effective resistor
for the parallel combination of the load and :math:`R_2`:

.. math:: \displaystyle  R_p = \frac{R_2R_L }{(R_2 + R_L)}= \frac{10R_2 }{(R_2 + 10)}.

 The total resistance is

.. math:: R =  R_1 + R_p = R_1 + \frac{10R_2 }{(R_2 + 10)}.

The voltage drop across :math:`R_1` is :math:`(12-9)=3` volts and the
current is given by

.. math:: i = V/R = \displaystyle \frac{12}{R_1 + \frac{10R_2 }{(R_2 + 10)}} = 3/R_1

 so,

.. math:: \displaystyle \frac{1}{4} = \left( \frac{R_1}{R_1 + \frac{10R_2}{(R_2 + 10)}}\right)

and after some algebra,

.. math:: R_1 =\displaystyle \frac{5R_2}{(R_2 + 10)}.

If :math:`R_2 = 10` Ohms, then :math:`R_1 = 2.5`. The load uses:
:math:`W_L = iV = (9/10)9 = 8.1` Watts. The whole circuit uses

.. math::

   W = V^2/R = \displaystyle\frac{12^2}{R_1 + \frac{10R_2}{(R_2 + 10)}} = \displaystyle
    \frac{12^2}{2.5 + \frac{100}{(20)}} = 19.2

A waste of 19.2 - 8.1 = 11.1 Watts. For circuits that power larger
motors, this can be a significant problem as it can be very difficult to
remove the heat. The system will be at risk due to the high
temperatures, for example burned components and melted solder, or even
fire. For battery based circuits, this approach significantly reduces
battery life. Another approach is needed.

One solution is to switch on and off the power very quickly, known as
:index:`Pulse Width Modulation`, :index:`PWM`. To see what we mean,
:numref:`circuitpwm` here is a graph of the voltage
though time.

.. _`circuitpwm`:
.. figure:: MotionFigures/pwm.*
   :width: 60%
   :align: center

   Switching power on and off.

.. _`circuitpwmduty`:
.. figure:: MotionFigures/pwm_duty.*
   :width: 60%
   :align: center

   On-Off pulsing known as Pulse Width Modulation - PWM.

The amount of time the pulse is high compared to low is the :index:`duty cycle`.
Duty cycle is often expressed as a percent of the pulse length which is
called the period. Why does this matter? By this method, we deliver a
fraction of the energy which then makes light dimmer. It does not have
the energy waste as compared to using a resistor. If we run the on and
off fast enough, our eyes will not see the flicker and it will just
appear dimmer.

.. _`pwmcontrol`:
.. figure:: MotionFigures/pwm_motor.*
   :width: 60%
   :align: center

   PWM control of an electric motor.

This is also the method by which we control an electric motor. The
frequency of this waveform does not change (because the duration of a
single waveform is unchanged). The time that the voltage is high
compared to the voltage is low does change. During the high part of the
waveform an electric motor will start to increase in speed. During the
low part the motor will coast and slow down.

You may ask how we switch the power on and off really fast. It is not
like we have a little light switch and 87 cups of coffee. Hard for us,
trivial for a computer. In fact, this is the basic way computers
operate. They switch lines on and off millions or even billions of times
per second. A program can be used to switch on and off an output line at
a variety of frequencies and duty cycles.

If a computer generates the signal, the computer electronics is probably
limited to 0.1 Amps or less. Certainly not enough to drive a large
electric motor which might want to draw many amps. Using a pwm to drive
a power transistor is the way to get power delivered. One minor problem
is that this only runs one way. An :index:`H-bridge` is a clever way to provide a
reverse current, :numref:`hbridgeswitches`. By
closing S1 and S4, current will flow from left to right,
:numref:`hbridgeswitchesclosed`. By
closing S3 and S2, current will flow from right to left. Replacing the
switches with transistors will provide the switching speed required for
PWM operation.

.. _`pwmfet`:
.. figure:: MotionFigures/transistor-motor.*
   :width: 20%
   :align: center

   Using a transistor to control power.

.. _`hbridgeswitches`:
.. figure:: MotionFigures/H_bridge.*
   :width: 40%
   :align: center

   H-Bridge, a way to select the direction of current
   flow.

.. _`hbridgeswitchesclosed`:
.. figure:: MotionFigures/H_bridge_operating.*
   :width: 80%
   :align: center

   Selecting current direction.


Electric Motors
^^^^^^^^^^^^^^^^^

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
.. figure:: MotionFigures/electricmotor.*
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
.. figure:: MotionFigures/servo.*
   :width: 70%
   :align: center

   Servo internals.
