
SciPy and Mathematics
---------------------

The following material assumes that you are familiar with Python. Python
reads like pseudocode and so it is possible to follow along without a
background in Python if you have seen some other programming language. A
quick introduction is given in the Appendices for those who want to ramp
up before reading on.


SciPy, is a collection of open-source packages for Scientific
Computing. One of the packages, redundantly named, SciPy library is a
collection of numerical methods including special functions,
integration, optimization, linear algebra, interpolation, and other
standard mathematics routines. NumPy is an open-source Python package
supporting data structures and low level algorithms for scientific
computing which is used by SciPy. [#f1]_ The main data structure of
interest to us from numpy is an array type and efficient methods to
access array elements.

Many of the implementations of iPython load NumPy and SciPy (as well as
the plotting package matlibplot) automatically. The idea is that most
users of iPython are going to use these. To use the NumPy or SciPy
libraries you need to import them. Since the scientific libraries are
large, we don’t want to drop them into the main namespace. The Python
community now uses the standard names for the namespaces:

::

    >>> import numpy as np
    >>> import scipy as sp
    >>> import matplotlib as mpl
    >>> import matplotlib.pyplot as plt

Scipy sub-packages need to be imported separately, for example:

::

    >>> from scipy import linalg, optimize

As stated above, some versions of iPython (some IDEs) will import
certain libraries for you. Say you are tired of typing the five import
lines above each time you run iPython. There is a full configuration
system available. To find the location of the config files, type at the
command prompt

.. code:: bash

    $ ipython locate
    /home/yourloginname/.ipython

    $ ipython locate profile foo
    /home/yourloginname/.ipython/profile_foo

You may create a profile for each of the different iPython activities.
We will stick with the default which is profile_default. The startup
files, files that get run when you start iPython, are located in the
startup subdirectory. In my case this is:

| /Users/jmcgough/.ipython/profile_default/startup

Inside the startup directory, I created a file: 05-early.py containing

::

    import numpy as np
    import scipy as sp
    import matplotlib as mpl
    import matplotlib.pyplot as plt

which then runs those import commands each time iPython is invoked. In
this next section, we will review some needed mathematics and introduce
SciPy as we proceed.

.. rubric:: Footnotes

.. [#f1] Thanks to the NumPy and SciPy online tutorials for great examples.
