.. _`section:imagemaps`:

NetPBM - Working with Image Maps
---------------------------------------------------

It can get very tedious creating maps by filling in cells in code. The
answer is to draw a map by hand in some bitmap program (paint program)
and then export for use in your own code. However, in general, image
formats can be complicated. One option is to use a very simple
uncompressed image format known as NetPBM format. This is a collection
of half a dozen or so formats. We will use the PPM (portable pixmap)
ascii format. It is very simple and easy to use, although not space
efficient. Other formats like PNG and GIF are compressed and use MUCH
less space, they are significantly harder to work with. There is a wiki
page on the formats: http://en.wikipedia.org/wiki/Netpbm_format.

The first line of the image file contains the file type (we generally
don’t believe the file extensions and store the file type inside the
file). For PPM ASCII files the file code is P3. Often the second line is
a comment line with file information (for humans not programs). The
comment character is the hash symbol, #, and begins the line. Your
program will ignore comment lines. The next line in the ppm file
contains two numbers: the image width (columns) in pixels and the image
height (rows) in pixels.

| Following the image dimensions is a line containing the color depth.
  This is the largest value used to store color information. For this
  assignment we will use one byte which gives us 255. The remaining
  lines are the pixels. They are listed as R G B triples per line all
  written in a long column. The example from Wikipedia is useful,
  http://en.wikipedia.org/wiki/Netpbm_format :

(0,0) (400,-100)|image|

::

    P3
    # The P3 means colors are in ASCII, then 3 columns and 2 rows,
    # then 255 for max color, then RGB triplets
    3 2
    255
    255   0   0
    0  255   0
    0   0  255
    255  255   0
    255  255  255
    0   0   0

You may note that the online documentation shows RGB triplets written
out in rows and columns. This does not matter since the number of
columns is known and it is easier to place one pixel per line.

Creating and viewing the PPM map files
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

To create and view the PPM files, we suggest you use GIMP - Gnu Image
Manipulation Tool: `www.gimp.org <www.gimp.org>`__. Think of a free
version of Photoshop. The download page: http://www.gimp.org/downloads/.
Once you have downloaded GIMP, test the installation by running the
application.

Making a map
~~~~~~~~~~~~

-  Open GIMP.

-  | From file select New
   | You will be asked for the image size. This is the map size. Start
     with smaller maps where you don’t exceed :math:`300 \times 300`.
     You can make larger maps later. Larger maps take longer and might
     be harder to debug.

-  Double click on the pencil icon. This will allow you to set the draw
   size.

-  Draw a simple map.

-  The map needs to have a boundary that is a couple of pixels thick.

-  Selecting save will save a GIMP format file (for later editing).

-  Export this to PPM. Select Export As and change the file extension to
   \*.ppm. GIMP should automatically detect and save as PPM.

-  It will ask binary or ASCII. Pick ASCII.

This map can be read into your program.

Reading a map with GIMP
~~~~~~~~~~~~~~~~~~~~~~~

-  Double click on the \*.ppm file that you generated. It should open
   GIMP and display the file.

-  Alternatively, you can run GIMP and under file, select the open menu.
   Open the file you want.

| **PPM Images**
| All ppm images consist of the following:

-  1st line will have a magic number identifying the file type, P3 or
   P6.

-  2nd line could be a comment. [If the line starts with a # it is a
   comment. There may be multiple lines that contain comments.]

-  After the comments come the width then height in ascii format

-  Next line contains the maximum color value the image can contain.

-  The image data follows.

| **P3 image data (Ascii data)**
| Row 1 then row 2 then row 3 � row n of the image. Each row of the
  image contains a pixel for each column of the image. A pixel for the
  image contains 3 color values in the order of red green then blue with
  a whitespace character that separates each of the color values

::

    P3
    #P3 data is in ascii, 3 columns-2 rows, 255 for max color, then RGB triplets
    3 2
    255
    255 0 0
    0 255 0
    0 0 255
    255 255 0
    255 255 255
    0 0 0

| **P6 image data (Binary data)**
| Data is in the same order but instead of using the ascii format it is
  in binary. 3 bytes make up pixel. The first byte is red, then green,
  then blue.

::

    P6
    #any comment string
    3 2
    255
    !@#$%^&*()_+|{}:"<

| **PGM Image file format**
| All pgm images are formatted with the following:

-  1st line contains the magic number identifying the file type �P2� or
   �P5�

-  2nd line on could contain comments. If the line starts with a # it is
   a comment

-  Next line contains the width whitespace and height in ascii format

-  The following line contains the maximum grayscale value in ascii.

-  The image data.

| **P2 image data (Ascii data)**
| The data will come in rows with each row containing a value for a
  pixel. Since this is grayscale, each pixel will be one byte in size
  and contain a value between 0 and 255. No line should exceed 70
  characters.

::

    P2
    # feep.pgm 24 columns, 7 rows max value of 15 (ours will always be 255)
    24 7
    15
    0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0
    0  3  3  3  3  0  0  7  7  7  7  0  0 11 11 11 11  0  0 15 15 15 15  0
    0  3  0  0  0  0  0  7  0  0  0  0  0 11  0  0  0  0  0 15  0  0 15  0
    0  3  3  3  0  0  0  7  7  7  0  0  0 11 11 11  0  0  0 15 15 15 15  0
    0  3  0  0  0  0  0  7  0  0  0  0  0 11  0  0  0  0  0 15  0  0  0  0
    0  3  0  0  0  0  0  7  7  7  7  0  0 11 11 11 11  0  0 15  0  0  0  0
    0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0

| **P5 image data (Binary data)**
| Data is organized into row 1, row 2, row 3, � row n. Each row n has
  enough bytes written to fill the columns within the row. Each column
  uses one byte and it is written in binary form.

::

    P5
    # feep.pgm 24 columns, 7 rows max value of 15 (ours will always be 255)
    24 7
    15
    !@#$%^&*()_+|{}:"<$%^&*(!@#$%^&*()_+|{}:"<$%^&*(!@#$%^&*()_+|{}:"<
    $%^&*(!@#$%^&*()_+|{}:"<$%^&*(!@#$%^&*()_+|{}:"<$%^&*(!@#$%^&*()
    _+|{}:"<$%^&*(!@#$%^&*()_+|{}:"<$%^&*(
