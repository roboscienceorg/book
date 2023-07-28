Embedded Approaches
---------------------


Complementary Filter
~~~~~~~~~~~~~~~~~~~~

Assume that you have two different sensors (measurements from two
different sources) in which one sensor has high frequency noise and the
other sensor has low frequency noise. A complementary filter exploits
this situation by applying a low pass filter to the first sensor data
and a high pass filter to the second sensor. The two signals
“complement” each other in terms of information.

.. _`fig:complementary`:
.. figure:: AdvFilteringFigures/complementary.*
   :width: 50%
   :align: center

   Complementary Filter
