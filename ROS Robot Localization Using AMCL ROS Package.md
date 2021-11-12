## ROS Robot Localization Using AMCL ROS Package

## Requirements

1.  Install necessary ROS packages.

```
apt-get update && apt-get install ros-kinetic-navigation ros-kinetic-map-server ros-kinetic-move-base
rospack profile
apt-get install ros-kinetic-amcl
```

## How To Use

1.  Launch the gazebo world and the robot.

```
roslaunch udacity_bot udacity_world.launch
```

2.  Launch the navigation stack.

```
roslaunch udacity_bot amcl.launch
```

3.  Set the goal position.

```
rosrun udacity_bot navigation_goal
```

## Default topics

-   Image Topic: /udacity\_bot/camera1/image\_raw
-   Image Info Topic:/udacity\_bot/camera1/camera\_info
-   Laser Scan Topic: /udacity\_bot/laser/scan
-   Odometry Topic: /odom
-   Movement Commands: /cmd\_vel

To spaw the robot into a running Gazebo simulation with a custom Odometry Topic append an argument like this:  
`$ roslaunch udacity_bot spawn_udacity_bot.launch odometryTopic:=odom_perfect`

## Repository architecture

### Directories

-   **urdf/** : (required) contains the files that generate the robot model and provide simulated actuators and sensors
-   **meshes/** : (required) contains the mesh files of the laser sensor
-   **config/** : (optional) contains YAML files that store the Navigation Stack configuration files for the robot
-   **rviz/** : (optional) contains Rviz configuration settings for displaying the robot model
-   **launch/** : (optional) contains launch files for starting the simulation / running nodes
-   **worlds/** : (optional) contains scene/environment files for Gazebo
-   **maps/** : (optional) contains the occupancy grid based maps required for navigation

### Robot model files

-   **udacity\_bot.xacro** : the xacro file that generates the urdf description file of the robot
-   **udacity\_bot.gazebo** : contains the Gazebo plugins that provide an interface to control the robot wheels and simulate the laser sensor

## Direct usage

-   Clone this repository into a ROS catkin workspace
-   Build and source the workspace
-   To view this robot model on an empty Gazebo world: `$ roslaunch udacity_bot empty_world.launch`
-   To launch this package including the Jackal Race Gazebo world and Rviz: `$ roslaunch udacity_bot udacity_world.launch use_rviz:=true`  
    or:
-   To spawn the robot into another already opened Gazebo world:  
    `$ roslaunch udacity_bot spawn_udacity_bot.launch`

If you want to move the robot using a keyboard you will also need to start a teleop node.  
To run the AMCL localization node and use the robot with the Navigation Stack type in a new window: `roslaunch udacity_bot amcl.launch`

To view raw images on the topic /camera/rgb/image\_raw, use:  
`$ rosrun image_view image_view image:=/udacity_bot/camera1/image_raw`

## Mapping

First check that you satisfy all dependencies by running: `$ rospack find gmapping`  
You can use this robot to build a map since it includes the required odometry and laser sensor. The inluded launch file `mapping.launch` will start Gazebo, load a predetermined world, and start the package **gmapping** properly configured, along with all other nodes required, including Rviz.  
To move the robot around a telop node is also required, for instance you can use:  
`$ rosrun rqt_robot_steering rqt_robot_steering`  
At the beginning there could be no map in Rviz, you may need to wait few second until it is generated.


### Robot chassis + wheels + camera sensor + laser finder sensor:

[![](https://github.com/mkhuthir/RoboND-Robot-Localization-Project/raw/master/misc/udacity_bot_4.jpg)](https://github.com/mkhuthir/RoboND-Robot-Localization-Project/blob/master/misc/udacity_bot_4.jpg)

### Full robot model with material colors in RViz:

[![](https://github.com/mkhuthir/RoboND-Robot-Localization-Project/raw/master/misc/udacity_bot_5.jpg)](https://github.com/mkhuthir/RoboND-Robot-Localization-Project/blob/master/misc/udacity_bot_5.jpg)


## Launching muth\_bot Packages

-   if you want to launch empty world with only the robot use the following:

$ roslaunch muth\_bot muth\_empty\_world.launch

-   to launch the simulated robot along with amcl package in jackal race world, run the following commands each in a separate terminal window.

To launch jackal race world

$ roslaunch muth\_bot muth\_world.launch

To start amcl node

$ roslaunch muth\_bot amcl.launch

To send 2d navigation goal

$ rosrun muth\_bot goto\_goal

## muth\_bot creation steps

To create this bot package from scatch you can follow same steps as explained above with some differences in gazebo plugins and config files.

### Robot chassis + wheels + camera sensor + laser finder sensor:

[![](https://github.com/mkhuthir/RoboND-Robot-Localization-Project/raw/master/misc/muth_bot_4.jpg)](https://github.com/mkhuthir/RoboND-Robot-Localization-Project/blob/master/misc/muth_bot_4.jpg)

### Full robot model with material colors in RViz:

[![](https://github.com/mkhuthir/RoboND-Robot-Localization-Project/raw/master/misc/muth_bot_5.jpg)](https://github.com/mkhuthir/RoboND-Robot-Localization-Project/blob/master/misc/muth_bot_5.jpg)



