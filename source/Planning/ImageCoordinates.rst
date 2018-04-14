Image coordinates, array coordinates and math matrix coordinates
----------------------------------------------------------------

Images are simply two dimensional arrays of integers. Much like matrices
in your math courses. There are a couple of differences you need to
know. First, the coordinate system for an image has the y coordinate
increasing as you head down. Second, the origin is the top left pixel.
The graphic below indicates this coordinate system. The way we store two
dimensional arrays is Array[row][col]. Increasing row will increase in
the y direction downwards. So the two dimensional array is consistent
with the image coordinate system. We will call neighbor pixels the eight
pixels surrounding the center pixel. The graphic below shows the
standard mathematical notation for plots and graphs :math:`(x,y)` and
the array notation for the pixels.

|image| |image|

C++ easy access of neighbor pixels
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Assume that you have an image stored in the two dimensional array map.
Many algorithms require you to access all eight neighbor pixels. You can
write these out by hand or you can do a two dimensional loop. The
following code accesses all eight neighbors and the point itself (nine
pixels):

::

    for(i=-1;i<=1;i++) {
         for(j=-1;j<=1;j++) {
               value = map[row+i][col+j];
          }
    }

Most likely your implementation will not care about the center point.
But if you explicitly want to skip it, try adding a conditional [3]_. Be
very careful about stepping outside map array bounds. Either you need to
check for stepping outside the array or your loops need to stay inside
the array. For example, instead of running :math:`i=0` to :math:`i=n`
you run from :math:`i=1` to :math:`i=n-1`. The outer layer of pixels are
the “walls" and you don’t touch them. This is why we suggest having a
layer or two of black pixels around the map.

Impacts in grid environments
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The last issue that needs to be addressed is object interaction. How
should we handle an impact? Earlier in the chapter,
Figure \ `[sizematters] <#sizematters>`__, we saw that for circular
robots, we could just add the radius of the robot to the obstacle and
then treat the robot as a point mass. For path planning of circular
robots we can then inflate the obstacle using a truncated flood fill
algorithm and proceed with path planning using just a point as the
robot. The flood fill algorithm will be discussed later on in this
chapter. We can then assume that all of the obstacle maps have been
preprocessed and just focus on the planning aspect.

Detecting a collision is now very easy. The robot is a point and so
impact is determined if the point is adjacent to an obstacle. Assume
that the robot is at location and the obstacle map is obstMap[i,j]. Also
assume that empty space is represented by 0 in the array, filled space
is represented by 1, and the boolean variable impact records impact or
not.

::

    if (obstMap[i+1,j] == 1) or (obstMap[i-1,j] == 1) or \
            (obstMap[i,j+1]== 1)  or (obstMap[i,j-1]== 1) :
        impact = 1

This will work to determine if is adjacent to a filled pixel. The
problem that arises is with the array boundaries. For example if i = 0
then the comparison obstMap[i-1,j] == 1 falls out of the array bounds.
The literature has two standard approaches for this issue. One way to
proceed is to treat the four sides and four corners of the array as
special cases with code lines of the form if i == 0 then omit the left
neighbor check. One must do this for top and bottom, left and right
boundaries.

The second common approach in the literature is known as ghost points.
The idea is to inflate the array by one pixel on each boundary. Say that
the obstacle map is 800 wide x 600 high. Normally your array runs i =
0..799 and j = 0..599. Declare the storage array to be 802 x 602. Then
place the obstacle map in i = 1..800 x j = 1..600. We define an open
landscape as no solide boundary on the edges of the obstacle map
(meaning no walls around the region). A closed landscape will have
walls. For an open landscape set the arrray entries for , [i = 801,j] ,
[i, j=0] , [i,j = 601] equal to zero. For a closed landscape set those
values to 1. The boundaries no longer generate out of array errors and
the need for special boundary cases is eliminated. The code above will
work as is.

The simplest approach is to flood fill about the obstacle the full
radius plus one. This means that when the center of the robot overlaps
the obstacle on the configuration space, the physical robot is adjacent
in the physical workspace. It neither requires a list of comparisons or
an inflated array. In this case the code is very simple:

::

    if (obstMap[i,j] == 1):
        impact = 1

It is now time to put everything together. We first list the server code
example. As above, a few lines have a backslash continuation character
which are for typesetting here and are not needed in the code. For
simplicity the obstacle map will use 0 for occupied and not 0 for open.
These just follows the image where black is 0 and which is 255. The code
first sets up the Turtle canvas. It places the robot at (-300,0) and
selects not to draw the path. Then we take a break and setup the
sockets. The program will block until a socket is established (recall
the discussion on event loops). Finally the progam enters the turtle
loop. It reads a comment on the socket and then issues that command to
the Turtle. The client program discussed above is used to communicate
with the turtle server.

The impact aspect is not really robust. The focus is on planning, not on
physics. We make no attempt to stop the robot and allow it to pass
through walls. It is the responsibility of the planner to stop, backup,
turn and move around.
