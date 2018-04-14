Simple Example
--------------

Control Model
~~~~~~~~~~~~~

Models that are not static rely on physics engine. These models can be
controlled during simulation, in a realistic manner. In this tutorial,
we will create a box with a ray sensor and change its velocity along the
x axis in proportion to the sensor reading. First create a work
directory:

mkdir :math:`\sim`/gazebo_control_model; cd
:math:`\sim`/gazebo_control_model

The world file we will be usign is shown in figure 1. Notice that the
box is defined as NOT static. This tells Gazebo to simulate the model in
the physics engine. The box model also contains a laser sensor on top of
it. Sensors must be defined as a child to a link node, and the link node
must be a child of the model node. Information about how to define valid
SFD nodes can be found in the SDF refrence depending upon which SDF
version you are using. Notice that <visualize>true</visualize>element
allows us to see the laser scan in blue.

The world also contains another box, named wall_model. You can seee the
laser bounce against it.

The world file will not run at this point because we have not yet
compiled and installed the plugin specified in the world file.

Create a world file: gedit
:math:`\sim`/gazebo_control_modelmodel_control.world.

Copy and paste the following xml code into it:

Control model Plugin
^^^^^^^^^^^^^^^^^^^^

The plugin must be attatched to a model (as in the above world file).
When it is loaded, the plugin instance’s Load method is called by
Gazebo. The code looks for “raysensor”, a refrence to a RaySensor
Instance and stores the refrence into a raysensor member variable, as
well as the refrence to the model instance.

A connection is then created to subscribe to plugin to the
WorldUpdateStart events via the OnUpdate callback method.

Create the plugin file: gedit :math:`\sim`/gazebo_control_model
control_model.cc

Copy and paste the code block below into it:

Build
^^^^^

Copy the contents of this CMake script into a file called CMakeLists.txt

gedit :math:`\sim`/gazebo_control_model/CMakeLists.txt

and copy and paste the following contents into it:

Create a build directory and cd into it and compile it: mkdir
:math:`\sim`/gazebo_control_model/build cd
:math:`\sim`/gazebo_control_model/build cmake .. make

Run Tutorial
^^^^^^^^^^^^

Add the current directory to the shared library path and run gazebo with
the world we just created. In a terminal cd
:math:`\sim`/gazebo_control_modelbuild export
GAZEBO_PLUGIN_PATH=‘pwd’:$GAZEBO_PLUGIN_PATH

Control TurtleBot
~~~~~~~~~~~~~~~~~

Now we will slightly change the steps and code in the previous tutorial
to control the turtlebot Go to the model_control.world in the previous
tutorial and replace the code with the following:

Control TurtleBot plugin
^^^^^^^^^^^^^^^^^^^^^^^^

Go to the control_model.cc in the previous tutorial and replace the code
with the following code:

.. _build-1:

Build
^^^^^

compile the code once again. cd :math:`\sim`/gazebo_control_model/build
make

.. _run-tutorial-1:

Run Tutorial
^^^^^^^^^^^^

cd :math:`\sim`/gazebo_control_model/build gazebo
:math:`\sim`/gazebo_control_modelmodel_control.world

You should see the turtlebot moving linearly

Controlling a mobile robot
~~~~~~~~~~~~~~~~~~~~~~~~~~

This tutorial describes the process of writing a model plugin based
controller for a wheeled mobile robot to dynamically manipulate the
robot.

Make a Mobile Robot
~~~~~~~~~~~~~~~~~~~

Setup your model directory
^^^^^^^^^^^^^^^^^^^^^^^^^^

| 1. Create a model directory
| cd :math:`\sim`/.gazebo; mkdir -p models/my_robot; cd models/my_robot
| 2. Create a :math:`sim`/.gazebo/models/model.config file:
| gedit :math:`\sim`/.gazebo/models/my_robot/model.config
| 3. Paste the following contents in model.config 4. Create a
  :math:`\sim`/.gazebo/models/my_robot/model.sdf
| gedit :math:`\sim`/.gazebo/models/my_robot/model.sdf
| 5. Paste in the following

Setup your workspace
^^^^^^^^^^^^^^^^^^^^

mkdir :math:`\sim`/my_plugin; cd :math:`\sim`/my_plugin

Setup cmake
^^^^^^^^^^^

gedit  /my_plugin/CMakeLists.txt

Copy this content into it:

Create your plugin code
^^^^^^^^^^^^^^^^^^^^^^^

| Create a file named :math:`\sim`/my_plugin/my_plugin.cc. This plugin
  code overrides the Load method to keep a refrence to the model and the
  two wheel hinges. It then creates a callback tied to the world update
  start event. During the update, a force of 0.2 Newton is injected in
  each hinge, causing the robot to spin.
| gedit my_plugin.cc Copy and paste the following code

Compiling your plugin
^^^^^^^^^^^^^^^^^^^^^

mkdir :math:`\sim`/my_plugin/build; cd :math:`\sim`\ my_plugin/build
cmake ..; make

Running the simulation
^^^^^^^^^^^^^^^^^^^^^^

Setup your gazebo plugin paths so it contains: cd  /my_plugin/build
export GAZEBO_PLUGIN_PATH=’pwd’:\ :math:`\sim`\ GAZEBO_PLUGIN_PATH

| Start gazebo (restart if it’s already running) in the same terminal
  you ran the ‘export GAZEBO_PLUGIN_PATH form.
| gazebo

Insert my_robot.. You should see the robot spin.

Controlling a mobile TurtleBot
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This tutorial describes the process of writing a mobile plugin based
controller for a turtlebot to dynamically manipulate the turtlebot. For
this first of all download the model repository of Gazebo from
‘http://gazebosim.org/models/’. Then simulate irobot create and
turtlebot in the gazebo so that the models are avilable in
.gazebo/models folder.

.. _setup-your-workspace-1:

Setup your workspace
^^^^^^^^^^^^^^^^^^^^

mkdir :math:`\sim`/my_turtlebot_plugin; cd
:math:`\sim`/my_turtlebot_plugin

.. _setup-cmake-1:

Setup cmake
^^^^^^^^^^^

gedit :math:`\sim`/my_turtlebot_plugin/CMakeLists.txt

Copy this content into it

.. _create-your-plugin-code-1:

Create your plugin code
^^^^^^^^^^^^^^^^^^^^^^^

Create a file named
:math:`\sim`/my_turtlebot_plugin/my_turtlebot_plugin.cc. This plugin
code overrides the Load method to keep a refrence to the model and the
two wheel higes. It then creates a callback tied to the world update
start event. During the update, a force of 0.4 Newton is injected in
each hinge, causing the robot to spin.

gedit my_turtlebot_plugin.cc

.. raw:: latex

   \verbatiminput{forgazebo/controlling_a_mobile_turtlebot/my_turtlebot_plugin.cc}

.. _compiling-your-plugin-1:

Compiling your plugin
^^^^^^^^^^^^^^^^^^^^^

mkdir :math:`\sim`/my_turtlebot_plugin/build; cd
:math:`\sim`/my_turtlebot_plugin/build cmake ..; make

Edit your model to include the model plugin
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Open the robot model file
:math:`\sim`/.gazebo/models/turtlebot/model.sdf

Add the following lines directly before the </model> tag near the end of
the file: <plugin filename=“libmy_turtlebot_plugin.so”
name=“my_turtlebot_plugin”> <left_wheel >create::left_wheel
</left_wheel> <right_wheel >create::right_wheel </right_wheel> </plugin>

.. _running-the-simulation-1:

Running the simulation
^^^^^^^^^^^^^^^^^^^^^^

Setup your gazebo plugin paths so it contains: cd
:math:`\sim`/my_turtlebot_plugin/build export
GAZEBO_PLUGIN_PATH=’pwd’:\ :math:`\sim`/GAZEBO_PLUGIN_PATH

Start gazebo (restart if it’s already running) in the same terminal that
you ran ‘export GAZEBO_PLUGIN_PATH’

gazebo

Insert turtlebot You should see the turtlebot spin.
