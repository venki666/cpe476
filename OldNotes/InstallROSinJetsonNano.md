
### Getting Started with ROS on Jetson Nano

ROS is the natural choice when building a multi-sensory autonomous robot.  
After setting up the Jetson Nano with its JetPack image using our [Getting Started](https://www.stereolabs.com/blog/getting-started-with-jetson-nano/) guide, we are going to install the latest version of ROS that runs on Ubuntu 18 Bionic Beaver: [Melodic Morenia](http://wiki.ros.org/melodic).

#### Installation

Open a new terminal by pressing **Ctrl + Alt + t** or executing the “**Terminal**” application using the Ubuntu 18 launch system.

Set up the Jetson Nano to accept software from _packages.ros.org_:
```
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb\_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'  
```
Add a new apt key:
```
$ sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```
\[_**Note**: the ROS GPG key has changed due to a [security issue on the ROS build farm server](https://discourse.ros.org/t/security-issue-on-ros-build-farm/9342). If you configured your Jetson Nano for ROS following this guide before the **24th June 2019**, please follow_ [_this guide_](https://discourse.ros.org/t/new-gpg-keys-deployed-for-packages-ros-org/9454) _to replace the old key in the correct way_\]

Update the Debian packages index:
```
$ sudo apt update  
```
Install the ROS **Desktop** package, including support for `rqt`, `rviz`and other useful robotics packages:
```
$ sudo apt install ros-melodic-desktop
```
**Note**: “ROS Desktop Full” is a more complete package, however it is not recommended for an embedded platform; 2D/3D simulators will be installed with it and they take too much space on ROM, and are too computationally hungry to be used on the Nano.

Initialize **rosdep**. **rosdep** enables you to easily install system dependencies for source code you want to compile and is required to run some core components in ROS:
```
$ sudo rosdep init   
$ rosdep update
```
It is recommended to load the ROS environment variables automatically when you execute a new shell session. Update your `.bashrc` script:
```
$ echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc   
$ source ~/.bashrc
```
Now the Jetson Nano is ready to execute ROS packages and become the brain of your autonomous robot.

#### Configure a catkin workspace

To start running your own ROS packages or install other packages from source (such as the [ZED ROS wrapper](https://github.com/stereolabs/zed-ros-wrapper) for example), you must create and configure a `catkin` workspace.

Install the following dependencies:
```
$ sudo apt-get install cmake python-catkin-pkg python-empy python-nose python-setuptools libgtest-dev python-rosinstall python-rosinstall-generator python-wstool build-essential git  
```
Create the catkin root and source folders:
```
$ mkdir -p ~/catkin\_ws/src   
$ cd ~/catkin\_ws/
```
Configure the catkin workspace by issuing a first “empty” build command:
```
$ catkin\_make
```
Finally, update your `.bashrc` script with the information about the new workspace:
```
$ echo "source ~/catkin\_ws/devel/setup.bash" >> ~/.bashrc   
$ source ~/.bashrc
```
Your catkin workspace is now ready to compile your ROS packages from source directly onto the Jetson Nano.


