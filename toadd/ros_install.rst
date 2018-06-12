[Enter the following commands in a terminal window]

Setup your computer to accept software from packages.ros.org
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

::

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

Set up your keys
^^^^^^^^^^^^^^^^

Before you can install ROS packages, you must get and add their package
authentication key.

::

    sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

Update all of your repositories (not just those for ROS).

::

    sudo apt-get update

Install ROS (Desktop Full version)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

For this text, we strongly suggest the Full Desktop version.

::

    sudo apt-get install ros-kinetic-desktop-full

User Account
^^^^^^^^^^^^

Before use, you need to setup the user account. Note that the first
command requires root and the second is done as the user.

::

    sudo rosdep init
    rosdep update

Set up your environment
^^^^^^^^^^^^^^^^^^^^^^^

ROS uses the shell environment in order to run properly on a system. The
shell is an interface for a user to access operating system services and
programs. In order to start ROS programs in the terminal, the user needs
to source the setup.bash file to add ROS to the environment. The
environment variables allow the OS to find the executable programs and
for ROS programs to find other ROS progams installed on the system.

::

    echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
    source ~/.bashrc

Create a Catkin Workspace:
~~~~~~~~~~~~~~~~~~~~~~~~~~

Catkin is the build system that ROS uses. It is an extension of CMake
and some Python code. Cmake and Python allow for portability to any
system that supports them. A catkin workspace is a folder where the
programmer will modify, build, and install catkin packages. You will do
all of your programming for ROS inside of this workspace.

Create a new directory for the workspace:
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

::

    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws
    catkin_make
    echo $ROS_PACKAGE_PATH

| This should build the workspace and then echo out
| ``/home/youruser/catkin_ws/src:/opt/ros/kinetic/share``.

STDR Simulator Installation
~~~~~~~~~~~~~~~~~~~~~~~~~~~

Everything is up in the Bitbucket repo it should be public to view.
https://bitbucket.org/stdr_simulator/stdr_simulator So here are the
instructions.

Install git (if not already installed):
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

::

    sudo apt-get install git

.. _create-a-catkin-workspace-1:

Create a catkin workspace
^^^^^^^^^^^^^^^^^^^^^^^^^

::

    cd <your_catkin_ws>/src
    git clone https://bitbucket.org/stdr_simulator/stdr_simulator.git
    cd ..
    rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO
    catkin_make
    source devel/setup.bash

Test Simulator Install
^^^^^^^^^^^^^^^^^^^^^^

| Now everything should be installed that is required for the launch
  files as well. For the python example, see the code in
| ``stdr_simulator/stdr_samples/scripts.speedcntl.py``

#. In one terminal

   ::

       roslaunch stdr_launchers server_with_map_and_gui_plus_robot.launch

#. Then in another terminal

   ::

       python stdr_simulator/stdr_samples/scripts/speedcntl.py

The control GUI should pop up. The left slider is the forward velocity
and the right slider is robot orientation. Try driving the robot through
the map for a short distance. If this works, you have succeeded in
showing that ROS and the STDR Simulator are working. The inner workings
will be described in detail later. For those who want to jump in, the
"sim_with_joy"
launch file has some comments in it which are useful in
getting up to speed.

Joystick Control Example
^^^^^^^^^^^^^^^^^^^^^^^^

For a joystick controller, here is an example: (ps3 controller)

::

    sudo chmod a+rw /dev/input/js0
    roslaunch stdr_launchers sim_with_joy

Note the select button has to be held and left joystick controls.


------------------



You should see the number appear on the listener. You now have a fairly
complicated connection between three processes. We can express the data
communication in a data flow graph. The processes are the nodes in the
graph and the topics are the edges. ROS can generate this for you using:

::

    rqt_graph

.. _`fig:rosgraph`:
.. figure:: ROSFigures/rosgraph.png
   :width: 75%
   :align: center

   The graph of nodes and topics for the current ROS
   session.

:numref:`fig:rosgraph` shows the resulting graph.
ROS’s Publish/Subscribe architecture is a many-to-many communication
protocol. This means that a publisher can talk to many different
subscribers. Multiple publishers can be on a single topic. It can get
complicated and ``rqt_graph`` might not resolve it well graphically as
you see that it did not show the multiple topics between the publisher
(talker) and the subscriber (listener2).


You can get information on one of the topics:

::

    jmcgough@ubuntu:~$ rostopic info /chatter
    Type: std_msgs/String

    Publishers:
     * /talker_25024_1505313174390 (http://ubuntu:36647/)

    Subscribers:
     * /listener_25288_1505313198989 (http://ubuntu:41441/)
