In this document we will install ROS on Raspberry Pi

**Description:** This instruction covers the installation of ROS Melodic on the [Raspberry Pi](http://www.raspberrypi.org/) with Raspbian Buster. However as final repositories are available now, today it is faster and easier to use Ubuntu Mate 18.04 (Bionic, [here](https://www.raspberrypi.org/downloads/)) together with the standard ARM installation instructions [here](http://wiki.ros.org/melodic/Installation/Ubuntu).

**Keywords:** [RaspberryPi](http://www.raspberrypi.org/), Setup, Melodic, Buster

**Tutorial Level:** BEGINNER

## Introduction

This tutorial explains how to install ROS Melodic from source on the Raspberry Pi. The instructions is based on the [ROSberryPi Kinetic installation](http://wiki.ros.org/ROSberryPi/Installing%2520ROS%2520Kinetic%2520on%2520the%2520Raspberry%2520Pi).

![/!\](http://wiki.ros.org/moin_static197/rostheme/img/alert.png "/!\") **Note:** If you're using the Raspberry Pi 2 or 3 it is faster and easier to use the standard ARM installation instructions [here](http://wiki.ros.org/melodic/Installation/Ubuntu).

## Prerequisites

These instructions assume that Raspbian Buster is being used as the OS on the Raspberry Pi. The download page for current images of Raspbian is [https://www.raspberrypi.org/downloads/](https://www.raspberrypi.org/downloads/).

### Setup ROS Repositories

First install repository key:

$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb\_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

Now, make sure your Debian package index is up-to-date:

$ sudo apt-get update
$ sudo apt-get upgrade

### Install Bootstrap Dependencies

$ sudo apt install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential cmake

### Initializing rosdep

$ sudo rosdep init
$ rosdep update

## Installation

Now, we will download and build ROS Melodic.

### Create a catkin Workspace

In order to build the core packages, you will need a catkin workspace. Create one now:

$ mkdir -p ~/ros\_catkin\_ws
$ cd ~/ros\_catkin\_ws

Next we will want to fetch the core packages so we can build them. We will use wstool for this. Select the wstool command for the particular variant you want to install:

**ROS-Comm: (recommended)** ROS package, build, and communication libraries. No GUI tools.

-   $ rosinstall\_generator ros\_comm --rosdistro melodic --deps --wet-only --tar > melodic-ros\_comm-wet.rosinstall
    $ wstool init src melodic-ros\_comm-wet.rosinstall
    

**Desktop:** ROS, [rqt](http://wiki.ros.org/rqt), [rviz](http://wiki.ros.org/rviz), and robot-generic libraries

-   $ rosinstall\_generator desktop --rosdistro melodic --deps --wet-only --tar > melodic-desktop-wet.rosinstall
    $ wstool init src melodic-desktop-wet.rosinstall
    

This will add all of the catkin or wet packages in the given variant and then fetch the sources into the ~/ros\_catkin\_ws/src directory. The command will take a few minutes to download all of the core ROS packages into the src folder. The \-j8 option downloads 8 packages in parallel.

So far, only the ROS-Comm variant has been tested on the Raspberry Pi in Melodic; however, more are defined in [REP 131](http://ros.org/reps/rep-0131.html#variants) such as robot, perception, etc. Just change the package path to the one you want, e.g., for robot do:

$ rosinstall\_generator robot --rosdistro melodic --deps --wet-only --tar > melodic-robot-wet.rosinstall
$ wstool init src melodic-robot-wet.rosinstall

Please feel free to update these instructions as you test more variants.

If wstool init fails or is interrupted, you can resume the download by running:

wstool update -j4 -t src

### Resolve Dependencies

Before you can build your catkin workspace, you need to make sure that you have all the required dependencies. We use the rosdep tool for this.

#### Resolving Dependencies with rosdep

The dependencies should be resolved by running rosdep:

$ cd ~/ros\_catkin\_ws
$ rosdep install -y --from-paths src --ignore-src --rosdistro melodic -r --os=debian:buster

This will look at all of the packages in the src directory and find all of the dependencies they have. Then it will recursively install the dependencies.

The --from-paths option indicates we want to install the dependencies for an entire directory of packages, in this case src. The --ignore-src option indicates to rosdep that it shouldn't try to install any ROS packages in the src folder from the package manager, we don't need it to since we are building them ourselves. The --rosdistro option is required because we don't have a ROS environment setup yet, so we have to indicate to rosdep what version of ROS we are building for. Finally, the -y option indicates to rosdep that we don't want to be bothered by too many prompts from the package manager.

After a while rosdep will finish installing system dependencies and you can continue.

### Building the catkin Workspace

Once you have completed downloading the packages and have resolved the dependencies, you are ready to build the catkin packages.

Invoke catkin\_make\_isolated:

$ sudo ./src/catkin/bin/catkin\_make\_isolated --install -DCMAKE\_BUILD\_TYPE=Release --install-space /opt/ros/melodic

**Note:** This will install ROS in the equivalent file location to Ubuntu in /opt/ros/melodic however you can modify this as you wish.

With older raspberries it is recommended to increase the [add swap space](http://raspberrypimaker.com/adding-swap-to-the-raspberrypi/). Also recommended to decrease the compilation thread count with the -j1 or -j2 option instead of the default -j4 option:

$ sudo ./src/catkin/bin/catkin\_make\_isolated --install -DCMAKE\_BUILD\_TYPE=Release --install-space /opt/ros/melodic -j2

Now ROS should be installed! Remember to source the new installation:

$ source /opt/ros/melodic/setup.bash

Or optionally source the setup.bash in the ~/.bashrc, so that ROS environment variables are automatically added to your bash session every time a new shell is launched:

$ echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc

## Maintaining a Source Checkout

### Updating the Workspace

See the [Ubuntu source install instructions](http://wiki.ros.org/melodic/Installation/Source) for steps on updating the ros\_catkin\_ws workspace. The same steps should apply to the Raspberry Pi.

### Adding Released Packages

You may add additional packages to the installed ros workspace that have been released into the ros ecosystem. First, a new rosinstall file must be created including the new packages (Note, this can also be done at the initial install). For example, if we have installed ros\_comm, but want to add ros\_control and joystick\_drivers, the command would be:

$ cd ~/ros\_catkin\_ws
$ rosinstall\_generator ros\_comm ros\_control joystick\_drivers --rosdistro melodic --deps --wet-only --tar > melodic-custom\_ros.rosinstall

You may keep listing as many ROS packages as you'd like separated by spaces.

Next, update the workspace with wstool:

$ wstool merge -t src melodic-custom\_ros.rosinstall
$ wstool update -t src

After updating the workspace, you may want to run rosdep to install any new dependencies that are required:

$ rosdep install --from-paths src --ignore-src --rosdistro melodic -y -r --os=debian:buster

Finally, now that the workspace is up to date and dependencies are satisfied, rebuild the workspace:

$ sudo ./src/catkin/bin/catkin\_make\_isolated --install -DCMAKE\_BUILD\_TYPE=Release --install-space /opt/ros/melodic

## References

-   [http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Kinetic%20on%20the%20Raspberry%20Pi](http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Kinetic%20on%20the%20Raspberry%20Pi)
    

Wiki: ROSberryPi/Installing ROS Melodic on the Raspberry Pi (last edited 2020-04-19 09:47:27 by [GaborCsorvasi](http://wiki.ros.org/GaborCsorvasi "GaborCsorvasi @ catv-176-63-218-31.catv.broadband.hu[176.63.218.31]"))