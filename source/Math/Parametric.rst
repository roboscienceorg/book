Parametric Form
---------------

Say you want to traverse a path :math:`C`, shown in
Figure \ `[Fig:intro-path] <#Fig:intro-path>`__.

.. raw:: latex

   \centering

.. figure:: control/path1
   :alt: A path for an explicitly defined function.[Fig:intro-path1]

   A path for an explicitly defined function.[Fig:intro-path1]

.. figure:: control/path2
   :alt: A path for a parametric function.[Fig:intro-path2]

   A path for a parametric function.[Fig:intro-path2]

The path :math:`C` often will come from some function description of the
curve :math:`y = f(x)`. This type of description will work for many
paths, but fails for a great number of interesting paths like circles:
:math:`x^2 + y^2 = 1`. We want to be able to wander around in the plan
crossing our own path which certainly is not the graph of a function.
So, we must move to a parametric description of the path (actually a
piecewise parametric description). You want to prescribe
:math:`x(t), y(t)` and obtain :math:`\dot{\phi_1},\dot{\phi_2}`. Clearly
if you have :math:`x(t), y(t)`, differentiation will yield
:math:`\dot{x}(t), \dot{y}(t)`, so we may assume that we know
:math:`\dot{x}(t), \dot{y}(t)`. Using :math:`\dot{x}` and
:math:`\dot{y}` we may drive the robot along the curve of interest. How
does one follow an arbitrary curve?

The first step is to write in parametric form: :math:`x(t)`,
:math:`y(t)`. Example: convert :math:`y=x^2` to parametric

.. math:: \mbox{Let } x = t  \quad \to \quad y = x^2 = t^2

| Note that there are an infinite number of choices :
| Let

  .. math::

     \begin{array}{l}
     x = 2t  \quad \to \quad y = x^2 = 4t^2 \\
     x = e^t  \quad  \to \quad y = x^2 = e^{2t} \\
     x = \tan(t) \quad \to \quad y = x^2 = \tan^2(t)
     \end{array}

and so forth.

All the parametric forms provide the same curve, same shape, same
geometry. They vary in the speed. Think of the function form telling you
the shape, like the shape of a road, but not the velocity. The
parametric form gives you both path shape and velocity. We will assume
that you can find parametric functions :math:`x = \phi(t)` and
:math:`y = \psi(t)` such that the graph is :math:`y=f(x)` which
generates the path :math:`C` of interest.

Example Functions
^^^^^^^^^^^^^^^^^

Some examples of parametric forms may help in getting good at writing
these down.

Line
    :math:`x(t) = t`, :math:`y(t) = mt + b`, where :math:`m` is the
    slope and :math:`b` is the intercept.

Circle
    :math:`x(t) = R \cos(t) + h`, :math:`y(t) = R \sin(t) + k`, where
    the radius is :math:`R` and the center is :math:`(h,k)`.

Ellipse
    :math:`x(t) = A \cos(t) + h`, :math:`y(t) = B \sin(t) + k`, where
    :math:`A` and :math:`B` describe the major and minor axes and the
    center is :math:`(h,k)`.

Lissajous
    :math:`x(t) = A\sin(at)`, :math:`y(t) = B \sin(bt)`
    (Figure `[Fig:intro-path2] <#Fig:intro-path2>`__ :math:`A=1`,
    :math:`B=1`, :math:`a=3`, :math:`b=4`). Infinity: :math:`A=1`,
    :math:`B=0.25`, :math:`a=1`, :math:`b=2`

Root
    :math:`x(t) =  t^2`, :math:`y(t) = t`.

Heart
    :math:`x(t) = 16\sin^3(t)`,
    :math:`y(t) = 13\cos(t) - 5\cos(2t) -2\cos(3t) - \cos(4t)`

