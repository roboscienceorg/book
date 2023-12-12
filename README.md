# Book

This is main source for the Roboscience text materials.  

**Note:  Significant rework of the text right now (9/1/2021 ... 12/23/2021)  - I broke latexpdf.**   

I am reworking this text to meet the needs of several courses.   The current push is to get an HTML version updated and working.  THe graphics for the html version is formatted in SVG.  Latex does not like SVG and so those I have used PDF.   I don't have all of the graphics in both SVG and PDF (with the latter missing in some cases).  This is causing problems.  It is an easy fix - I just need to spend the time.  I hope to have it addressed by early Oct (2021).    **End note**

The document source is in the source directory.  You then need to run a make html to build the target sphinx web tree.  This creates a build directory and the web tree is contained within.  You will need the Anaconda distribution installed for the build.  

To build the html:   make html

To build the pdf:    make latexpdf

Our goal is to build a useful and modular curriculum.   So, instead of just forking the distro in silence, how about you contact us.  We might be able to add the content you want and end up with a better product overall.  We might also be able to setup for custom builds.  Thanks.

## Steps to take to get the book up and running
- Modify the latex conversion script to:
  - convert all image files to one format that Tex can handle
  - remove ".*" in the text
  - Add a header and closing document tag at the end of every rst file
- Look at image size and scaling
- Sections that need some love:
  - 2: Preliminaries
  - 4: Simulation
  - 10: Planning
  - 11: Control
  - 13: Localization
  - 14: Machine Leanring
- General clean up and updating images
- ZeroMQ or ROS?
- Finding/making a pretty LaTex template
- Sections to possibly write:
  - DeepCV
  - Reinforcement Learning Control
  - Advanced Controls
  - Advanced Planning
  - Exploration
  - SystemID
  - UAVs
  - Spacecraft
  - UUVs
  

