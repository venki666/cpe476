## ROS Autonomous SLAM using Rapidly Exploring Random Tree (RRT)

Implementation of the RRT algorithm for unknown environment explorations by the mobile robot Turtlebot3 with the help of ROS’s Navigation stack and Gazebo simulator

## RRT Algorithm

A Rapidly-exploring Random Tree (RRT) is a data structure and algorithm that is designed for efficiently searching nonconvex high-dimensional spaces. RRTs are constructed incrementally in a way that quickly reduces the expected distance of a randomly-chosen point to the tree. RRTs are particularly suited for path planning problems that involve obstacles and differential constraints (nonholonomic or kinodynamic). [source](http://lavalle.pl/rrt/about.html)

![](https://miro.medium.com/freeze/max/60/1*L7EGF1xlRNJY-YrPcauoWA.gif?q=20)

![](https://miro.medium.com/max/600/1*L7EGF1xlRNJY-YrPcauoWA.gif)

![](https://miro.medium.com/max/1200/1*L7EGF1xlRNJY-YrPcauoWA.gif)

RRT Algorithm Animation | Image by Author, generated using [source code](https://github.com/pbpf/RRT-2)

## RRT for SLAM Application

We will be using the RRT algorithm for the robot to path plan to all the reachable distant endpoints within the sensor’s vicinity aka frontier points which in return makes the robot map new regions continuously using SLAM as it tries to reach its new distant endpoints. Applying RRT to mobile robots in such a way enables us to create a self-exploring autonomous robot with no human interventions required. The nature of the algorithm tends to be biased towards unexplored regions which become very beneficial to environment exploring tasks. More in-depth information and flow of this strategy can found in this publication: [Autonomous robotic exploration based on multiple rapidly-exploring randomized trees.](https://ieeexplore.ieee.org/document/8202319)

![](https://miro.medium.com/max/60/1*yQT1NDL6GIzYQGSwrCpogg.jpeg?q=20)

![](https://miro.medium.com/max/700/1*yQT1NDL6GIzYQGSwrCpogg.jpeg)

![](https://miro.medium.com/max/1400/1*yQT1NDL6GIzYQGSwrCpogg.jpeg)

Propagation of RRT Algorithm for exploring unknown space | Image by Author

## [Gazebo Simulator](http://gazebosim.org/)

The Gazebo Simulator is a well-designed standalone robot simulator which can be used to rapidly test algorithms, design robots, perform regression testing, and train AI system using realistic scenarios. We will be using the Gazebo environment with a house model for our robot to explore and produce a map of the house.

![](https://miro.medium.com/max/60/0*AbtDsiY4mVyL7Au6.png?q=20)

![](https://miro.medium.com/max/700/0*AbtDsiY4mVyL7Au6.png)

![](https://miro.medium.com/max/1400/0*AbtDsiY4mVyL7Au6.png)

House Model in Gazebo Simulator | Image by Author

# Let's Dive into working with ROS

For this demo feel free to download my pre-built ROS package [**_ros\_autonomous\_slam_**](https://github.com/fazildgr8/ros_autonomous_slam) from my [**_Github_**](https://github.com/fazildgr8) repository.

This repository consists of a ROS package that uses the Navigation Stack to autonomously explore an unknown environment with help of GMAPPING and constructs a map of the explored environment. Finally, a path planning algorithm from the Navigation stack is used in the newly generated map to reach the goal. The Gazebo simulator is used for the simulation of the Turtlebot3 Waffle Pi robot. Various algorithms have been integrated for Autonomously exploring the region and constructing the map with help of the 360-degree Lidar sensor. Different environments can be swapped within launch files to generate the required map of the environment. The current most efficient algorithm used for autonomous exploration is the Rapidly Exploring Random Tree (RRT) algorithm. The RRT algorithm is implemented using the package from [rrt\_exploration](http://wiki.ros.org/rrt_exploration) which was created to support the Kobuki robots which I further modified the source files and built it for the Turtlebot3 robots in this package.

## There are three main steps to be executed in this project.

-   Step 1: Place the Robot in the Environment within Gazebo
-   Step 2: Perform Autonomous exploration of the environment and generate the Map
-   Step 3: Perform path planning and go to goal in the environment

# Prerequisites and set up for the Project

Before starting to execute the steps please make sure you have the prerequisites and setup for this project demo completed successfully. There are three setup parts (Gazebo ROS Installation, Turtlebot3 Packages, and Navigation Stack Installation).

# ROS Installation

I used Ubuntu 18 OS with ROS Melodic Version. Check the ROS official documentation for the Installation [ROS Installation](http://wiki.ros.org/melodic/Installation/Ubuntu)

## Gazebo ROS Installation

The main Gazebo Simulator which is a stand-alone application must be installed. Go through the documentation [Gazebo Installation](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install). Test the working of Gazebo and its version with
```
gazebo  
which gzserver  
which gzclient
```
After Installing the Gazebo, the Gazebo ROS Package must be installed separately

sudo apt-get install ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control

Replace `melodic` with your version of ROS everywhere in this tutorial.

## Turtlebot3 packages

The Turtlebot3 ROS packages can be either downloaded and built from source files in your workspace or else directly installed from the Linux terminal. Either way works, I would recommend doing both as it installs all the missing dependencies required automatically.

![](https://miro.medium.com/max/60/1*BtqglXezF87ty5kWtXVXFA.png?q=20)

![](https://miro.medium.com/max/700/1*BtqglXezF87ty5kWtXVXFA.png)

![](https://miro.medium.com/max/1400/1*BtqglXezF87ty5kWtXVXFA.png)

Turtlebot3 Waffle Pi in House Model within Gazebo | Image by Author

Direct Installation
````
source /opt/ros/melodic/setup.bash  
sudo apt-get install ros-melodic-turtlebot3-msgs  
sudo apt-get install ros-melodic-turtlebot3
```
Building the packages
```
cd catkin\_ws/src  
git clone -b melodic-devel https://github.com/ROBOTIS-GIT/turtlebot3  
git clone -b melodic-devel https://github.com/ROBOTIS-GIT/turtlebot3\_simulations  
cd ..  
catkin\_make  
source /devel/setup.bash
```
## Navigation Stack

The Navigation stack can also be downloaded as source files to your workspace and built.
```
sudo apt-get install ros-melodic-navigation  
cd catkin\_ws/src  
git clone -b melodic-devel https://github.com/ros-planning/navigation  
cd ..  
catkin\_make  
source /devel/setup.bash
```
# The Main Execution of the Autonomous SLAM Demo

# Step 1: Place the Robot in the Environment within Gazebo

Set your environment variable to the model robot to be used.
```
export TURTLEBOT3\_MODEL=waffle\_pi  
source ~/.bashrc
```
Execute the given launch to open Gazebo with the given world file and place the robot Turtlebot3 Waffle pi model in it.
```
roslaunch ros\_autonomous\_slam turtlebot3\_world.launch
```
Keep this process running always and execute other commands in a different terminal.

# Step 2: Perform Autonomous exploration of the environment and generate the Map
```
roslaunch ros\_autonomous\_slam autonomous\_explorer.launch
```
Run the Autonomous Explorer launch file which executes two tasks for us at the same time.

1.  It starts the SLAM node in the Navigation stack with a custom modified RVIZ file to monitor the mapping of the environment.
2.  It simultaneously starts the Autonomous explorer which is a Python-based controller to move around the robot grazing all the areas which help the SLAM Node to complete the mapping. The default algorithm used for exploration is the RRT algorithm. I have also created an explorer method that uses Bug Wall following algorithm for exploration which can be tested by adding `explorer` the argument to the launch which takes `[RRT,BUG_WALLFOLLOW]`.

# Setting Exploration region for RRT in RVIZ Window ([More Details](http://wiki.ros.org/rrt_exploration/Tutorials/singleRobot))

The RRT exploration requires a rectangular region around the robot to be defined in the RVIZ window using four points and a starting point for exploration within the known region of the robot. The total five points must be defined in the exact sequence given below using the RVIZ Publish Points option.

![](https://miro.medium.com/max/60/1*yPtywj_JnK6zdW6ZbE5P4Q.jpeg?q=20)

![](https://miro.medium.com/max/514/1*yPtywj_JnK6zdW6ZbE5P4Q.jpeg)

![](https://miro.medium.com/max/1028/1*yPtywj_JnK6zdW6ZbE5P4Q.jpeg)

Publish Points Sequence | Image by Author, more details from [rrt\_exploration Tutorials](http://wiki.ros.org/rrt_exploration/Tutorials/singleRobot)

Monitor the Mapping process in the RVIZ window and sit back and relax until our robot finishes mapping XD.

![](https://miro.medium.com/freeze/max/60/0*iEXIBPUNuoTuQmaJ.gif?q=20)

![](https://miro.medium.com/max/600/0*iEXIBPUNuoTuQmaJ.gif)

![](https://miro.medium.com/max/1200/0*iEXIBPUNuoTuQmaJ.gif)

Robot Mapping using RRT Algorithm | Image by Author

Once you are satisfied with the constructed map, Save the map.
```
rosrun map\_server map\_saver -f my\_map
```
The _my\_map.pgm_ and _my\_map.yaml_ gets saved in your home directory. Move these files to the package’s maps folder (catkin\_ws\\src\\ros\_autonomous\_slam\\maps).Now your new map which is basically an occupancy grid is constructed!

![](https://miro.medium.com/max/60/0*fOACNaoAWYFhLepF.png?q=20)

![](https://miro.medium.com/max/384/0*fOACNaoAWYFhLepF.png)

![](https://miro.medium.com/max/768/0*fOACNaoAWYFhLepF.png)

Constructed Map | Image by Author

**Incase of Autonomous Fails** you can manually control the robot in the environment using the keyboard with a separate launch execution given below. You can also manually explore and construct the map like a game.

roslaunch turtlebot3\_teleop turtlebot3\_teleop\_key.launch

![](https://miro.medium.com/freeze/max/60/0*FiZxO1WpbiJ6cwYY.gif?q=20)

![](https://miro.medium.com/max/600/0*FiZxO1WpbiJ6cwYY.gif)

![](https://miro.medium.com/max/1200/0*FiZxO1WpbiJ6cwYY.gif)

Manual Robot Mapping| Image by Author

# Step 3: Perform path planning and go to goal in the environment

We will be using the Navigation stack of the ROS to perform the path planning and go to goal using /move\_base/goal actions. The given blow launch execution opens up an RVIZ window which shows the Robot location within the previously constructed map.
```
roslaunch ros\_autonomous\_slam turtlebot3\_navigation.launch
```
The RVIZ Window shows the robot’s local map construction using its Laser sensors with respect to the Global Map previously constructed in Step 2 with help of a cost map.

# Setting Goal in the RVIZ Window

-   First, estimate the initial Pose i.e locating the real robot location with respect to the Map. This can be set in the RVIZ window itself using the 2D Pose Estimate and pointing and dragging the arrow in the current robot’s location and orientation.

![](https://miro.medium.com/max/60/0*Gc07pF5_BBNcS6jU.png?q=20)

![](https://miro.medium.com/max/700/0*Gc07pF5_BBNcS6jU.png)

![](https://miro.medium.com/max/1400/0*Gc07pF5_BBNcS6jU.png)

2D Estimate Tab

-   A GOAL point can be set in the RVIZ window itself using the 2D Nav Goal option which will be available in the top window tab. This allows you to set a goal point in the map within the RVIZ environment, then the robot automatically performs the path planning and starts to move in its path.

![](https://miro.medium.com/max/60/0*YteSKqfyJRw6GBzA.png?q=20)

![](https://miro.medium.com/max/700/0*YteSKqfyJRw6GBzA.png)

![](https://miro.medium.com/max/1400/0*YteSKqfyJRw6GBzA.png)

2D Nav Goal Tab

# Ros Navigation Stack Tuning Guide

ROS Navigation Stack requires tuning its parameters which work differently for different environment types to get the Optimal SLAM and Pathplanning performance. Here is ROS’s Navigation Stack parameter tuning guide for Turtlebot3. [Turtlebot3 Navigation Parameter Tuning Guide](https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/#tuning-guide)

![](https://miro.medium.com/freeze/max/60/0*Qtu_PJQDFhJ0wE9m.gif?q=20)

![](https://miro.medium.com/max/600/0*Qtu_PJQDFhJ0wE9m.gif)

![](https://miro.medium.com/max/1200/0*Qtu_PJQDFhJ0wE9m.gif)

Robot local path and global path planning | Image by Author

![](https://miro.medium.com/freeze/max/60/0*ib5nK3TdQi-h-ucI.gif?q=20)

![](https://miro.medium.com/max/600/0*ib5nK3TdQi-h-ucI.gif)

![](https://miro.medium.com/max/1200/0*ib5nK3TdQi-h-ucI.gif)

