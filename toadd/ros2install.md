# Installing ROS2 via Debian Packages

As of Beta 2 we are building Debian packages for Ubuntu Xenial.
They are in a temporary repository for testing.
The following links and instructions reference the latest release - currently ardent.

Resources:
 - [Jenkins Instance](http://build.ros2.org/)
 - [Repositories](http://repo.ros2.org)
 - Status Pages ([amd64](http://repo.ros2.org/status_page/ros_ardent_default.html), [arm64](http://repo.ros2.org/status_page/ros_ardent_uxv8.html))

## Setup Sources

To install the Debian packages you will need to add our Debian repository to your apt sources.
First you will need to authorize our gpg key with apt like this:

```
sudo apt update && sudo apt install curl
curl http://repo.ros2.org/repos.key | sudo apt-key add -
```

And then add the repository to your sources list:

```
sudo sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main xenial main" > /etc/apt/sources.list.d/ros2-latest.list'
```

## Install ROS 2 packages

The following commands install all `ros-ardent-*` package except `ros-ardent-ros1-bridge` and `ros-ardent-turtlebot2-*` since they require ROS 1 dependencies.
See below for how to also install those.

```
sudo apt update
sudo apt install `apt list "ros-ardent-*" 2> /dev/null | grep "/" | awk -F/ '{print $1}' | grep -v -e ros-ardent-ros1-bridge -e ros-ardent-turtlebot2- | tr "\n" " "`
```

## Environment setup

```
source /opt/ros/ardent/setup.bash
```

If you have installed the Python package `argcomplete` (version 0.8.5 or higher, see below for Xenial instructions) you can source the following file to get completion for command line tools like `ros2`:

```
source /opt/ros/ardent/share/ros2cli/environment/ros2-argcomplete.bash
```

### (optional) Install argcomplete >= 0.8.5 on Ubuntu 16.04

If you need to install `argcomplete` on Ubuntu 16.04 (Xenial), you'll need to use pip, because the version available through `apt-get` will not work due to a bug in that version of `argcomplete`:

```
sudo apt install python3-pip
sudo pip3 install argcomplete
```

## Choose RMW implementation

By default the RMW implementation `FastRTPS` is being used.
By setting the environment variable `RMW_IMPLEMENTATION=rmw_opensplice_cpp` you can switch to use OpenSplice instead.

## Additional packages using ROS 1 packages

The `ros1_bridge` as well as the TurtleBot demos are using ROS 1 packages.
To be able to install them please start by adding the ROS 1 sources as documented [here](http://wiki.ros.org/Installation/Ubuntu?distro=kinetic)

If you're using Docker for isolation you can start with the image `ros:kinetic` or `osrf/ros:kinetic-desktop`
This will also avoid the need to setup the ROS sources as they will already be integrated.

Now you can install the remaining packages:

```
sudo apt update
sudo apt install ros-ardent-ros1-bridge ros-ardent-turtlebot2-*
```
