Serial and Parallel Chain Manipulators
--------------------------------------

Manufacturing robots typically work in a predefined and restricted
space. They usually have very precise proprioception (the knowledge of
relative position and forces) within the space. It is common to name the
design class after the coordinate system which the robot naturally
operates in. For example, a cartesian design (similar to gantry systems)
is found with many mills and routers, heavy lift systems, 3D Printers
and so forth, see Figure \ `[gantrysample-a] <#gantrysample-a>`__.
Actuation occurs in the coordinate directions and is described by
variable length linear segments (links) or variable positioning along a
segment. This greatly simplifies the mathematical model of the machine
and allows efficient computation of machine configurations.

In two dimensions, one can rotate a linear actuator about a common
center producing a radial design which would use a polar coordinate
description. Adding a linear actuator on the :math:`z` axis gives a
cylindrical coordinate description,
Figure \ `[gantrysample-b] <#gantrysample-b>`__.



.. figure:: TermsFigures/cartesian.png
   :width: 70%
   :align: center

   Cartesian design [gantrysample-a] Muhammad Furqan grabcad.com


.. figure:: TermsFigures/weldingarm.png
   :width: 70%
   :align: center

   Cylindrical design [gantrysample-b]   Mark Dunn  grabcad.com

A serial chain manipulator is the most common design in industrial
robots. It is built as a sequence of links connect by actuated joints
(normally seen as a sequence starting from an attached base and
terminating at the end-effector. By relating the links to segments and
joints as nodes, we see that serial link manipulators can be seen as
graphs with no loops or cycles. The classical robot arm is an example of
a serial chain manipulator, Figure \ `[armsample-a] <#armsample-a>`__.
Robot arms normally employ fixed length links and use rotary joints.
This are often called articulated robots or the arm is called an
articulator. Very general tools exist to construct mathematical
descriptions of arm configuration as a function of joint angles. A
formalism developed by Denavit and Hartenberg can be used to obtain the
equations for position.


.. figure:: TermsFigures/ABB_IRB4600.jpg
   :width: 70%
   :align: center

   Articulated [armsample-a]  Ivo Jardim  grabcad.com

.. figure:: TermsFigures/SpaceClaim103.jpg
   :width: 70%
   :align: center

   Delta Design [armsample-b]  Ivan Volpe  grabcad.com

Another popular approach is the parallel chain manipulator, which uses
multiple serial chains to control the end-effector. An example of one,
called a Delta Robot, can be seen in
Figure \ `[armsample-b] <#armsample-b>`__.
