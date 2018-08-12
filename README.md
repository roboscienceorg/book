# Book

This is main source for the Roboscience materials.

The document source is in the source directory.  You then need to run a make html to build the target sphinx web tree.  This creates a build directory and the web tree is contained within.  You will need the Anaconda distribution installed for the build.  

To build the html:   make html
To build the pdf:    make latexpdf

The work flow will be to move content from the latex source in the Robotics_Text repo over to this one.

The metabook.pdf file describes the structure that I have in mind.

Initial Tasks:

1.  Produce a folder for each text chapter.   Inside the folder there will be a figures folder which will contain the images for that chapter.   ONLY the images used in the text will be copied over.  Please do not bulk copy over of images.    Eventually we will have a slides folder as well, but this will be addressed after the text is complete (don't add yet).
2.  The main rst file will be a file that includes the sections for the specific chapter (assuming that we don't have a huge number of sections - really small sections can be merged).
3.  Convert over the *.tex to *.rst.  Remove the problem section for conversion later.  Remove specific sections that cause issues like algorithms or latex/tikz build images for later conversion.
4.  Copy the referenced images over.   To avoid name collisions - how about we name the images:  chapname_imagename.pdf/png  - unless you think we can mod sphinx to preserve folders....
5.  Fix math errors.
6.  Fix ref and numbering errors.
7.  Fix image/caption errors.
8.  Address footnotes and citations (bib).
9.  Address the latex algorithms environment and convert tikz elements to pdf.
10.  Address code sections.  [Meaning - do we keep in text or move to another document such as a lab document.]

