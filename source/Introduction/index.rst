Introduction[Chap:Introduction]
===============================

Growing up in the modern age means many things to many people. For those
of us reading (or writing) a book on robotics, it means getting a
healthy dose of technology. Hollywood has raised us on spaceships,
cloning, alien worlds and intelligent machines. This was, however, not
incredibly unrealistic as progress in science and technology has been
advancing at an ever increasing rate. We have come to expect something
new, maybe even dramatic, every single year...and for the most part
haven’t been disappointed! Over the years we have seen significant
developments in medicine, space, electronics, communications and
materials. There has always been excitement regarding the latest
development. Even though the world around us has been struggling with
war, poverty and disease, science and technology offers us a reprieve.
It is an optimistic view that things can and will get better. In full
disclosure - we love technology! It is the magic of our age and learning
how the magic works only makes it more fun. In no way does the author
claim that technology is the answer to our problems. That clearly lies
with our willingness to look beyond our differences with acceptance,
compassion and grace. If technology can bring us together, then it has
succeeded in helping us far beyond our wildest dreams.

Robotics is a shining example of technical optimism - a belief that the
human condition can be improved through sufficiently advanced
technology. It is the premise that a machine can engage in the
difficult, the tedious, and the dangerous; leaving humans out of harms
way. Technology is a fancy word for tool utilization. Even though
biologists have long shown that humans were not alone in their usage of
tools, we are indisputably the master tool users on the planet. Tools
extend our grasp, our strength and our speed. We know of no technology
that aims to extend human capability like robotics does.

Robotics is an engineering discipline stemming from the fusion of
science and art. Humans have an insatiable curiosity, a drive to create,
and a considerable amount of self-interest. Building machines which look
like us, act like us and do things like us was the engineering manifest
destiny. Although we have succeeded in building machines that do
complicated tasks, we really place the value in what we learn about
ourselves in the process. A process we embark on here.

.. toctree::
   :maxdepth: 2

   whatisarobot
   overview
   navigation
   vision
   robotcontrol
   senseplanact
   FewLastWords
   supplementary





Problems
--------

[Introduction\_ans]

How would you define a robot?

What are the two main types of robots as presented by the text?

The text has broken robots into fixed or industrial manipulators and
mobile machines. Often the industrial manipulator is performing repeated
tasks or is remotely operated. Although many mobile systems are also
remotely operated, we don’t consider them doing repeated tasks.

Can you think of another way to classify robotic systems? What are the
strengths and weaknesses of the classification?

What problems does a mobile robot face that a stationary robot does not?
What about the other way around?

A stationary robot does not encounter new or novel environments. The
workspace for a stationary robot is relatively fixed. It’s tasks are
predetermined for the most part. A mobile robot constantly moves into
new environments. Even if the task is to be repeated, in a new
environment the details of performing the task will be different.

What are the differences between robots that are considered Mobile
Machines and those that are considered to be Manufacturing Machines?

Do you think the *Robotic Appliance* and *Robotic Agent* partitioning is
a more effective way to classify robots? Why or why not?

List several approaches that industry has used so robots can navigate in
an environment; mentioning an advantage and disadvantage of each.

Describe the information gathered by a RGBD sensor such as the Microsoft
Kinect.

Describe the information gathered by a stereo camera pair.

List out different ways one could assist a robot in navigating around a
building or inside a building when GPS is not an option. [Think
sensors.]

What is an FPGA? What is a GPU? What is a TPU? What are their strengths
and weaknesses compared to traditional CPUs?

What are Isaac Asimov’s Three Laws of Robotics? What do they mean? Are
they complete, meaning are they a sufficient set of rules?

Work in robotics can replace people with machines. This results in job
loss. Discuss the ethics of working in the robotics industry.

Military robotics is a growing industry. Although many systems have a
high degree of autonomy, use of deadly force is left for the human.
Discuss the ethical issues in allowing the robot to make these
decisions.

If an autonomous system by design or error causes an accident, who is
liable?

List a few ways biology has inspired robotics.

.. |An example of different map types.[fig:maptypes]| image:: slam/discretemap
.. |An example of different map types.[fig:maptypes]| image:: slam/metricmap
.. |An example of different map types.[fig:maptypes]| image:: slam/topomap
.. |For humans (and I suppose animals), it is very easy to distinguish apples, tomatoes and PT balls, but not as easy for machine vision systems| image:: vision/apple3.jpg
.. |For humans (and I suppose animals), it is very easy to distinguish apples, tomatoes and PT balls, but not as easy for machine vision systems| image:: vision/tomato.jpg
.. |For humans (and I suppose animals), it is very easy to distinguish apples, tomatoes and PT balls, but not as easy for machine vision systems| image:: vision/redball.jpg
.. |It is easy for a human but hard for a computer to track the road in a variety of lighting conditions and road types.| image:: vision/road.jpg
.. |It is easy for a human but hard for a computer to track the road in a variety of lighting conditions and road types.| image:: vision/road2.png
.. |Localization and Routing| image:: robots/hallway.jpg
.. |Localization and Routing| image:: slam/route.jpg
.. |Compare the structure of a maze to that of a forest scene. Very simple robots can plan a route and escape a maze. Routing through random obstacles in three dimensions is still very difficult for a robot.[mazeforest]| image:: robots/maze.png
.. |Compare the structure of a maze to that of a forest scene. Very simple robots can plan a route and escape a maze. Routing through random obstacles in three dimensions is still very difficult for a robot.[mazeforest]| image:: robots/Forest.jpg
.. |A more traditional approach to robot control. [robotcontrolclassical]| image:: slam/classicAIcontrol
.. |A more traditional approach to robot control. [robotcontrolclassical]| image:: slam/new_old_AI_blend
