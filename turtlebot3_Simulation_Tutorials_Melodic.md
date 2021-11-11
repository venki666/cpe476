
### [Install Dependent ROS Packages](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#install-dependent-ros-packages)[](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#install-dependent-ros-packages)

```
$ sudo apt-get install ros-kinetic-joy ros-kinetic-teleop-twist-joy \
  ros-kinetic-teleop-twist-keyboard ros-kinetic-laser-proc \
  ros-kinetic-rgbd-launch ros-kinetic-depthimage-to-laserscan \
  ros-kinetic-rosserial-arduino ros-kinetic-rosserial-python \
  ros-kinetic-rosserial-server ros-kinetic-rosserial-client \
  ros-kinetic-rosserial-msgs ros-kinetic-amcl ros-kinetic-map-server \
  ros-kinetic-move-base ros-kinetic-urdf ros-kinetic-xacro \
  ros-kinetic-compressed-image-transport ros-kinetic-rqt* \
  ros-kinetic-gmapping ros-kinetic-navigation ros-kinetic-interactive-markers
```

### [Install TurtleBot3 Packages](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#install-turtlebot3-packages)[](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#install-turtlebot3-packages)

Install TurtleBot3 via Debian Packages.

```
$ sudo apt-get install ros-kinetic-dynamixel-sdk
$ sudo apt-get install ros-kinetic-turtlebot3-msgs
$ sudo apt-get install ros-kinetic-turtlebot3
```

### [Set TurtleBot3 Model Name](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#set-turtlebot3-model-name)[](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#set-turtlebot3-model-name)

Set the default `TURTLEBOT3_MODEL` name to your model. Enter the below command to a terminal.

-   In case of TurtleBot3 Burger
    
    ```
    $ echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
    ```
    
-   In case of TurtleBot3 Waffle Pi
    
    ```
    $ echo "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc
    ```
    

-   **Gazebo Tutorials** : [http://gazebosim.org/tutorials](http://gazebosim.org/tutorials)

## [Gazebo Simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation)[](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation)

<iframe src="https://www.youtube.com/embed/UzOoJ6a_mOg" allow="autoplay; encrypted-media" allowfullscreen="" width="640" height="358" frameborder="0"></iframe>

The contents in e-Manual can be updated without a prior notice and video contents could be outdated.

This Gazebo Simulation uses **ROS Gazebo package**, therefore, proper Gazebo version for ROS1 Kinetic has to be installed before running this instruction.

### [Install Simulation Package](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#install-simulation-package)[](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#install-simulation-package)

The **TurtleBot3 Simulation Package** requires `turtlebot3` and `turtlebot3_msgs` packages as prerequisite. Without these prerequisite packages, the Simulation cannot be launched.  
Please follow the [PC Setup](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/) instructions if you did not install required packages and dependent packages.

```
$ cd ~/catkin_ws/src/
$ git clone -b kinetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
$ cd ~/catkin_ws && catkin_make
```

### [Launch Simulation World](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#launch-simulation-world)[](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#launch-simulation-world)

Three simulation environments are prepared for TurtleBot3. Please select one of these environments to launch Gazebo.

**Please make sure to completely terminate other Simulation world before launching a new world.**

1.  Empty World  
    ![](https://emanual.robotis.com/assets/images/platform/turtlebot3/simulation/turtlebot3_empty_world.png)
    
    ```
    $ export TURTLEBOT3_MODEL=burger
    $ roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
    ```
    
2.  TurtleBot3 World  
    ![](https://emanual.robotis.com/assets/images/platform/turtlebot3/simulation/turtlebot3_world_bugger.png)
    
    ```
    $ export TURTLEBOT3_MODEL=waffle
    $ roslaunch turtlebot3_gazebo turtlebot3_world.launch
    ```
    
3.  TurtleBot3 House  
    ![](https://emanual.robotis.com/assets/images/platform/turtlebot3/simulation/turtlebot3_house.png)
    
    ```
    $ export TURTLEBOT3_MODEL=waffle_pi
    $ roslaunch turtlebot3_gazebo turtlebot3_house.launch
    ```
    

**NOTE**: If TurtleBot3 House is launched for the first time, downloading the map may take more than a few minutes depending the network status.

### [Operate TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#operate-turtlebot3)[](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#operate-turtlebot3)

In order to teleoperate the TurtleBot3 with the keyboard, launch the teleoperation node with below command in a new terminal window.

```
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

 ![](https://emanual.robotis.com/assets/images/icon_unfold.png) Read more about **How to run Autonomous Collision Avoidance**

A simple collision avoidance node is prepared which keeps certain distance from obstacles and make turns to avoid collision.  
In order to autonomously drive a TurtleBot3 in the **TurtleBot3 world**, please follow the instruction below.

1.  Terminate the turtlebot3\_teleop\_key node by entering `Ctrl` + `C` to the terminal that runs the teleop node.
    
2.  Enter the below command to the terminal.
    
    ```
    $ roslaunch turtlebot3_gazebo turtlebot3_simulation.launch
    ``` ![](https://emanual.robotis.com/assets/images/icon_unfold.png) Read more about **How to visualize Simulation data(RViz)**

RViz visualizes published topics while simulation is running. You can launch RViz in a new terminal window by entering below command.

```
$ roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch
```

![](https://emanual.robotis.com/assets/images/platform/turtlebot3/simulation/turtlebot3_gazebo_rviz.png) 

**NOTE**

-   Please run the Simulation on **Remote PC**.
-   Launching the Simulation for the first time on the Remote PC may take a while to setup the environment.

 ![](https://emanual.robotis.com/assets/images/icon_unfold.png) Read more about **TurtleBot3 Simulation**

TurtleBot3 supports simulation development environment that can be programmed and developed with a virtual robot in the simulation. There are two development environments to do this, one is using a **fake node with 3D visualization tool RViz**, and the other is using the **3D robot simulator Gazebo**.

-   The **fake node** is suitable for testing with the robot model and movement, but it does not support sensors.
-   If you need to perform SLAM or Navigation, **Gazebo** would be a feasible solution as it supports sensors such as IMU, LDS, and camera.

In this instruction, Gazebo will be mainly introduced which is most widely used among ROS developers.

-   **Gazebo Tutorials** : [http://gazebosim.org/tutorials](http://gazebosim.org/tutorials)

## [Gazebo Simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation)[](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation-1)

<iframe src="https://www.youtube.com/embed/UzOoJ6a_mOg" allow="autoplay; encrypted-media" allowfullscreen="" width="640" height="358" frameborder="0"></iframe>

The contents in e-Manual can be updated without a prior notice and video contents could be outdated.

This Gazebo Simulation uses **ROS Gazebo package**, therefore, proper Gazebo version for ROS1 Melodic has to be installed before running this instruction.

### [Install Simulation Package](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#install-simulation-package)[](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#install-simulation-package-1)

The **TurtleBot3 Simulation Package** requires `turtlebot3` and `turtlebot3_msgs` packages as prerequisite. Without these prerequisite packages, the Simulation cannot be launched.  
Please follow the [PC Setup](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/) instructions if you did not install required packages and dependent packages.

```
$ cd ~/catkin_ws/src/
$ git clone -b melodic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
$ cd ~/catkin_ws && catkin_make
```

### [Launch Simulation World](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#launch-simulation-world)[](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#launch-simulation-world-1)

Three simulation environments are prepared for TurtleBot3. Please select one of these environments to launch Gazebo.

**Please make sure to completely terminate other Simulation world before launching a new world.**

1.  Empty World  
    ![](https://emanual.robotis.com/assets/images/platform/turtlebot3/simulation/turtlebot3_empty_world.png)
    
    ```
    $ export TURTLEBOT3_MODEL=burger
    $ roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
    ```
    
2.  TurtleBot3 World  
    ![](https://emanual.robotis.com/assets/images/platform/turtlebot3/simulation/turtlebot3_world_bugger.png)
    
    ```
    $ export TURTLEBOT3_MODEL=waffle
    $ roslaunch turtlebot3_gazebo turtlebot3_world.launch
    ```
    
3.  TurtleBot3 House  
    ![](https://emanual.robotis.com/assets/images/platform/turtlebot3/simulation/turtlebot3_house.png)
    
    ```
    $ export TURTLEBOT3_MODEL=waffle_pi
    $ roslaunch turtlebot3_gazebo turtlebot3_house.launch
    ```
    

**NOTE**: If TurtleBot3 House is launched for the first time, downloading the map may take more than a few minutes depending the network status.

### [Operate TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#operate-turtlebot3)[](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#operate-turtlebot3-1)

In order to teleoperate the TurtleBot3 with the keyboard, launch the teleoperation node with below command in a new terminal window.

```
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

 ![](https://emanual.robotis.com/assets/images/icon_unfold.png) Read more about **How to run Autonomous Collision Avoidance**

A simple collision avoidance node is prepared which keeps certain distance from obstacles and make turns to avoid collision.  
In order to autonomously drive a TurtleBot3 in the **TurtleBot3 world**, please follow the instruction below.

1.  Terminate the turtlebot3\_teleop\_key node by entering `Ctrl` + `C` to the terminal that runs the teleop node.
    
2.  Enter the below command to the terminal.
    
    ```
    $ roslaunch turtlebot3_gazebo turtlebot3_simulation.launch
    ``` ![](https://emanual.robotis.com/assets/images/icon_unfold.png) Read more about **How to visualize Simulation data(RViz)**

RViz visualizes published topics while simulation is running. You can launch RViz in a new terminal window by entering below command.

```
$ roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch
```

![](https://emanual.robotis.com/assets/images/platform/turtlebot3/simulation/turtlebot3_gazebo_rviz.png) 

**NOTE**

-   Please run the Simulation on **Remote PC**.
-   Launching the Simulation for the first time on the Remote PC may take a while to setup the environment.

 ![](https://emanual.robotis.com/assets/images/icon_unfold.png) Read more about **TurtleBot3 Simulation**

TurtleBot3 supports simulation development environment that can be programmed and developed with a virtual robot in the simulation. There are two development environments to do this, one is using a **fake node with 3D visualization tool RViz**, and the other is using the **3D robot simulator Gazebo**.

-   The **fake node** is suitable for testing with the robot model and movement, but it does not support sensors.
-   If you need to perform SLAM or Navigation, **Gazebo** would be a feasible solution as it supports sensors such as IMU, LDS, and camera.

In this instruction, Gazebo will be mainly introduced which is most widely used among ROS developers.

-   **Gazebo Tutorials** : [http://gazebosim.org/tutorials](http://gazebosim.org/tutorials)

## [Gazebo Simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation)[](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation-2)

<iframe src="https://www.youtube.com/embed/UzOoJ6a_mOg" allow="autoplay; encrypted-media" allowfullscreen="" width="640" height="358" frameborder="0"></iframe>

The contents in e-Manual can be updated without a prior notice and video contents could be outdated.

This Gazebo Simulation uses **ROS Gazebo package**, therefore, proper Gazebo version for ROS1 Noetic has to be installed before running this instruction.

### [Install Simulation Package](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#install-simulation-package)[](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#install-simulation-package-2)

The **TurtleBot3 Simulation Package** requires `turtlebot3` and `turtlebot3_msgs` packages as prerequisite. Without these prerequisite packages, the Simulation cannot be launched.  
Please follow the [PC Setup](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/) instructions if you did not install required packages and dependent packages.

```
$ cd ~/catkin_ws/src/
$ git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
$ cd ~/catkin_ws && catkin_make
```

### [Launch Simulation World](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#launch-simulation-world)[](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#launch-simulation-world-2)

Three simulation environments are prepared for TurtleBot3. Please select one of these environments to launch Gazebo.

**Please make sure to completely terminate other Simulation world before launching a new world.**

1.  Empty World  
    ![](https://emanual.robotis.com/assets/images/platform/turtlebot3/simulation/turtlebot3_empty_world.png)
    
    ```
    $ export TURTLEBOT3_MODEL=burger
    $ roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
    ```
    
2.  TurtleBot3 World  
    ![](https://emanual.robotis.com/assets/images/platform/turtlebot3/simulation/turtlebot3_world_bugger.png)
    
    ```
    $ export TURTLEBOT3_MODEL=waffle
    $ roslaunch turtlebot3_gazebo turtlebot3_world.launch
    ```
    
3.  TurtleBot3 House  
    ![](https://emanual.robotis.com/assets/images/platform/turtlebot3/simulation/turtlebot3_house.png)
    
    ```
    $ export TURTLEBOT3_MODEL=waffle_pi
    $ roslaunch turtlebot3_gazebo turtlebot3_house.launch
    ```
    

**NOTE**: If TurtleBot3 House is launched for the first time, downloading the map may take more than a few minutes depending the network status.

### [Operate TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#operate-turtlebot3)[](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#operate-turtlebot3-2)

In order to teleoperate the TurtleBot3 with the keyboard, launch the teleoperation node with below command in a new terminal window.

```
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

 ![](https://emanual.robotis.com/assets/images/icon_unfold.png) Read more about **How to run Autonomous Collision Avoidance**

A simple collision avoidance node is prepared which keeps certain distance from obstacles and make turns to avoid collision.  
In order to autonomously drive a TurtleBot3 in the **TurtleBot3 world**, please follow the instruction below.

1.  Terminate the turtlebot3\_teleop\_key node by entering `Ctrl` + `C` to the terminal that runs the teleop node.
    
2.  Enter the below command to the terminal.
    
    ```
    $ roslaunch turtlebot3_gazebo turtlebot3_simulation.launch
    ``` ![](https://emanual.robotis.com/assets/images/icon_unfold.png) Read more about **How to visualize Simulation data(RViz)**

RViz visualizes published topics while simulation is running. You can launch RViz in a new terminal window by entering below command.

```
$ roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch
```

![](https://emanual.robotis.com/assets/images/platform/turtlebot3/simulation/turtlebot3_gazebo_rviz.png) 

**NOTE**

-   Please run the Simulation on **Remote PC**.
-   Launching the Simulation for the first time on the Remote PC may take a while to setup the environment.

 ![](https://emanual.robotis.com/assets/images/icon_unfold.png) Read more about **TurtleBot3 Simulation**

TurtleBot3 supports simulation development environment that can be programmed and developed with a virtual robot in the simulation. There are two development environments to do this, one is using a **fake node with 3D visualization tool RViz**, and the other is using the **3D robot simulator Gazebo**.

-   The **fake node** is suitable for testing with the robot model and movement, but it does not support sensors.
-   If you need to perform SLAM or Navigation, **Gazebo** would be a feasible solution as it supports sensors such as IMU, LDS, and camera.

In this instruction, Gazebo will be mainly introduced which is most widely used among ROS developers.

-   **Gazebo Tutorials** : [http://gazebosim.org/tutorials](http://gazebosim.org/tutorials)

## [Gazebo Simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation)[](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation-3)

<iframe src="https://www.youtube.com/embed/UzOoJ6a_mOg" allow="autoplay; encrypted-media" allowfullscreen="" width="640" height="358" frameborder="0"></iframe>

The contents in e-Manual can be updated without a prior notice and video contents could be outdated.

This Gazebo Simulation uses **ROS Gazebo package**, therefore, proper Gazebo version for ROS2 Dashing has to be installed before running this instruction.

### [Install Simulation Package](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#install-simulation-package)[](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#install-simulation-package-3)

The **TurtleBot3 Simulation Package** requires `turtlebot3` and `turtlebot3_msgs` packages as prerequisite. Without these prerequisite packages, the Simulation cannot be launched.  
Please follow the [PC Setup](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/) instructions if you did not install required packages and dependent packages.

```
$ cd ~/turtlebot3_ws/src/
$ git clone -b dashing-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
$ cd ~/turtlebot3_ws && colcon build --symlink-install
```

The **GAZEBO\_MODEL\_PATH** parameter must be appended in the `.bashrc` file. Enter below command to add the information.

```
$ echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models' >> ~/.bashrc
$ source ~/.bashrc
```

### [Launch Simulation World](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#launch-simulation-world)[](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#launch-simulation-world-3)

Three simulation environments are prepared for TurtleBot3. Please select one of these environments to launch Gazebo.

**Please make sure to completely terminate other Simulation world before launching a new world.**

1.  Empty World  
    ![](https://emanual.robotis.com/assets/images/platform/turtlebot3/simulation/turtlebot3_empty_world.png)
    
    ```
    $ export TURTLEBOT3_MODEL=burger
    $ ros2 launch turtlebot3_gazebo empty_world.launch.py
    ```
    
2.  TurtleBot3 World  
    ![](https://emanual.robotis.com/assets/images/platform/turtlebot3/ros2/gazebo_world.png)
    
    ```
    $ export TURTLEBOT3_MODEL=waffle
    $ ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
    ```
    
3.  TurtleBot3 House  
    ![](https://emanual.robotis.com/assets/images/platform/turtlebot3/simulation/turtlebot3_house.png)
    
    ```
    $ export TURTLEBOT3_MODEL=waffle_pi
    $ ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
    ```
    

**NOTE**: If TurtleBot3 House is launched for the first time, downloading the map may take more than a few minutes depending the network status.

### [Operate TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#operate-turtlebot3)[](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#operate-turtlebot3-3)

In order to teleoperate the TurtleBot3 with the keyboard, launch the teleoperation node with below command in a new terminal window.

```
$ ros2 run turtlebot3_teleop teleop_keyboard
```

 ![](https://emanual.robotis.com/assets/images/icon_unfold.png) Read more about **How to run Autonomous Collision Avoidance**

A simple collision avoidance node is prepared which keeps certain distance from obstacles and make turns to avoid collision.  
In order to autonomously drive a TurtleBot3 in the **TurtleBot3 world**, please follow the instruction below.

1.  Terminate the turtlebot3\_teleop\_key node by entering `Ctrl` + `C` to the terminal that runs the teleop node.
    
2.  Enter the below command to the terminal.
    
    ```
    $ ros2 run turtlebot3_gazebo turtlebot3_drive
    ``` ![](https://emanual.robotis.com/assets/images/icon_unfold.png) Read more about **How to visualize Simulation data(RViz2)**

RViz2 visualizes published topics while simulation is running. You can launch RViz2 in a new terminal window by entering below command.

```
$ ros2 launch turtlebot3_bringup rviz2.launch.py
```

![](https://emanual.robotis.com/assets/images/platform/turtlebot3/simulation/turtlebot3_gazebo_rviz.png) 

**NOTE**

-   Please run the Simulation on **Remote PC**.
-   Launching the Simulation for the first time on the Remote PC may take a while to setup the environment.

 ![](https://emanual.robotis.com/assets/images/icon_unfold.png) Read more about **TurtleBot3 Simulation**

TurtleBot3 supports simulation development environment that can be programmed and developed with a virtual robot in the simulation. There are two development environments to do this, one is using a **fake node with 3D visualization tool RViz**, and the other is using the **3D robot simulator Gazebo**.

-   The **fake node** is suitable for testing with the robot model and movement, but it does not support sensors.
-   If you need to perform SLAM or Navigation, **Gazebo** would be a feasible solution as it supports sensors such as IMU, LDS, and camera.

In this instruction, Gazebo will be mainly introduced which is most widely used among ROS developers.

-   **Gazebo Tutorials** : [http://gazebosim.org/tutorials](http://gazebosim.org/tutorials)

## [Gazebo Simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation)[](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation-4)

<iframe src="https://www.youtube.com/embed/UzOoJ6a_mOg" allow="autoplay; encrypted-media" allowfullscreen="" width="640" height="358" frameborder="0"></iframe>

The contents in e-Manual can be updated without a prior notice and video contents could be outdated.

This Gazebo Simulation uses **ROS Gazebo package**, therefore, proper Gazebo version for ROS2 Foxy has to be installed before running this instruction.

### [Install Simulation Package](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#install-simulation-package)[](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#install-simulation-package-4)

The **TurtleBot3 Simulation Package** requires `turtlebot3` and `turtlebot3_msgs` packages as prerequisite. Without these prerequisite packages, the Simulation cannot be launched.  
Please follow the [PC Setup](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/) instructions if you did not install required packages and dependent packages.

```
$ cd ~/turtlebot3_ws/src/
$ git clone -b foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
$ cd ~/turtlebot3_ws && colcon build --symlink-install
```

### [Launch Simulation World](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#launch-simulation-world)[](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#launch-simulation-world-4)

Three simulation environments are prepared for TurtleBot3. Please select one of these environments to launch Gazebo.

**Please make sure to completely terminate other Simulation world before launching a new world.**

1.  Empty World  
    ![](https://emanual.robotis.com/assets/images/platform/turtlebot3/simulation/turtlebot3_empty_world.png)
    
    ```
    $ export TURTLEBOT3_MODEL=burger
    $ ros2 launch turtlebot3_gazebo empty_world.launch.py
    ```
    
2.  TurtleBot3 World  
    ![](https://emanual.robotis.com/assets/images/platform/turtlebot3/ros2/gazebo_world.png)
    
    ```
    $ export TURTLEBOT3_MODEL=waffle
    $ ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
    ```
    
3.  TurtleBot3 House  
    ![](https://emanual.robotis.com/assets/images/platform/turtlebot3/simulation/turtlebot3_house.png)
    
    ```
    $ export TURTLEBOT3_MODEL=waffle_pi
    $ ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
    ```
    

**NOTE**: If TurtleBot3 House is launched for the first time, downloading the map may take more than a few minutes depending the network status.

### [Operate TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#operate-turtlebot3)[](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#operate-turtlebot3-4)

In order to teleoperate the TurtleBot3 with the keyboard, launch the teleoperation node with below command in a new terminal window.

```
$ ros2 run turtlebot3_teleop teleop_keyboard
```

 ![](https://emanual.robotis.com/assets/images/icon_unfold.png) Read more about **How to run Autonomous Collision Avoidance**

A simple collision avoidance node is prepared which keeps certain distance from obstacles and make turns to avoid collision.  
In order to autonomously drive a TurtleBot3 in the **TurtleBot3 world**, please follow the instruction below.

1.  Terminate the turtlebot3\_teleop\_key node by entering `Ctrl` + `C` to the terminal that runs the teleop node.
    
2.  Enter the below command to the terminal.
    
    ```
    $ ros2 run turtlebot3_gazebo turtlebot3_drive
    ``` ![](https://emanual.robotis.com/assets/images/icon_unfold.png) Read more about **How to visualize Simulation data(RViz2)**

RViz2 visualizes published topics while simulation is running. You can launch RViz2 in a new terminal window by entering below command.

```
$ ros2 launch turtlebot3_bringup rviz2.launch.py
```

![](https://emanual.robotis.com/assets/images/platform/turtlebot3/simulation/turtlebot3_gazebo_rviz.png) 

**NOTE**

-   Please run the Simulation on **Remote PC**.
-   Launching the Simulation for the first time on the Remote PC may take a while to setup the environment.

 ![](https://emanual.robotis.com/assets/images/icon_unfold.png) Read more about **TurtleBot3 Simulation**

TurtleBot3 supports simulation development environment that can be programmed and developed with a virtual robot in the simulation. There are two development environments to do this, one is using a **fake node with 3D visualization tool RViz**, and the other is using the **3D robot simulator Gazebo**.

-   The **fake node** is suitable for testing with the robot model and movement, but it does not support sensors.
-   If you need to perform SLAM or Navigation, **Gazebo** would be a feasible solution as it supports sensors such as IMU, LDS, and camera.

In this instruction, Gazebo will be mainly introduced which is most widely used among ROS developers.

-   **Gazebo Tutorials** : [http://gazebosim.org/tutorials](http://gazebosim.org/tutorials)

## [Gazebo Simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation)[](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation-5)

<iframe src="https://www.youtube.com/embed/UzOoJ6a_mOg" allow="autoplay; encrypted-media" allowfullscreen="" width="640" height="358" frameborder="0"></iframe>

The contents in e-Manual can be updated without a prior notice and video contents could be outdated.

This Gazebo Simulation uses **ROS Gazebo package**, therefore, proper Gazebo version must to be installed before running this instruction.

### [Launch Simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#launch-simulation)[](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#launch-simulation)

If you are running ROS1 Noetic, please replace `melodic` with `noetic` in the command below.

1.  To start the simulation, open one elevated command prompt.
    
    ```
    > c:\opt\ros\melodic\x64\setup.bat
    > c:\ws\turtlebot3\devel\setup.bat
    > set TURTLEBOT3_MODEL=waffle
    > roslaunch turtlebot3_fake turtlebot3_fake.launch
    ```
    
2.  Then, open another elevated command prompt.
    
    ```
    c:\opt\ros\melodic\x64\setup.bat
    c:\ws\turtlebot3\devel\setup.bat
    set TURTLEBOT3_MODEL=waffle
    roslaunch turtlebot3_gazebo turtlebot3_simulation.launch
    ```

When SLAM in Gazebo simulator, you can select or create various environments and robot models in virtual world. Other than preparing simulation environment instead of bringing up the robot, SLAM Simulation is pretty similar to that of [SLAM](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/#slam) with the actual TurtleBot3.

The following instructions require prerequisites from the previous sections, so please review to the [Simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/) section first.

### Launch Simulation World[](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam_simulation/#launch-simulation-world)

Three Gazebo environments are prepared, but for creating a map with SLAM, it is recommended to use either **TurtleBot3 World** or **TurtleBot3 House**.  
Use one of the following commands to load the Gazebo environment.

In this instruction, TurtleBot3 World will be used.  
Please use the proper keyword among `burger`, `waffle`, `waffle_pi` for the `TURTLEBOT3_MODEL` parameter.

```
$ export TURTLEBOT3_MODEL=burger
$ roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

 ![](https://emanual.robotis.com/assets/images/icon_unfold.png) Read more about **How to load TurtleBot3 House**

```
$ export TURTLEBOT3_MODEL=burger
$ roslaunch turtlebot3_gazebo turtlebot3_house.launch
``` 

### Run SLAM Node[](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam_simulation/#run-slam-node)

Open a new terminal from Remote PC with `Ctrl` + `Alt` + `T` and run the SLAM node. Gmapping SLAM method is used by default.  
Please use the proper keyword among `burger`, `waffle`, `waffle_pi` for the `TURTLEBOT3_MODEL` parameter.

```
$ export TURTLEBOT3_MODEL=burger
$ roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
```

### Run Teleoperation Node[](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam_simulation/#run-teleoperation-node)

Open a new terminal from Remote PC with `Ctrl` + `Alt` + `T` and run the teleoperation node from the Remote PC.  
Please use the proper keyword among `burger`, `waffle`, `waffle_pi` for the `TURTLEBOT3_MODEL` parameter.

```
$ export TURTLEBOT3_MODEL=burger
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

 Control Your TurtleBot3!
 ---------------------------
 Moving around:
        w
   a    s    d
        x

 w/x : increase/decrease linear velocity
 a/d : increase/decrease angular velocity
 space key, s : force stop

 CTRL-C to quit
```

### Save Map[](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam_simulation/#save-map)

When the map is created successfully, open a new terminal from Remote PC with `Ctrl` + `Alt` + `T` and save the map.

![](https://emanual.robotis.com/assets/images/platform/turtlebot3/simulation/virtual_slam.png)

```
$ rosrun map_server map_saver -f ~/map
```

![](https://emanual.robotis.com/assets/images/platform/turtlebot3/simulation/map.png)

> The saved map.pgm file

 ![](https://emanual.robotis.com/assets/images/icon_unfold.png) Read more about **How to SLAM with multiple TurtleBot3**

In order to create a map with multiple robots, **multirobot-map-merge** package is required.  
Follow the instructions below instead of **Launching Simulation World** section of this page to operate multiple TurtleBot3.

1.  Install necessary package
    
    ```
    $ sudo apt-get install ros-kinetic-multirobot-map-merge
    ```
    
2.  Load multiple TurtleBot3 in TurtleBot3 House.  
    These loaded turtlebot3s are set initial position and orientation.
    
    ```
    $ roslaunch turtlebot3_gazebo multi_turtlebot3.launch
    ```
    
    ![](https://emanual.robotis.com/assets/images/platform/turtlebot3/simulation/turtlebot3_house_slam.png)
    
3.  Launch SLAM for each TurtleBot3
    
    ```
    $ ROS_NAMESPACE=tb3_0 roslaunch turtlebot3_slam turtlebot3_gmapping.launch set_base_frame:=tb3_0/base_footprint set_odom_frame:=tb3_0/odom set_map_frame:=tb3_0/map
    $ ROS_NAMESPACE=tb3_1 roslaunch turtlebot3_slam turtlebot3_gmapping.launch set_base_frame:=tb3_1/base_footprint set_odom_frame:=tb3_1/odom set_map_frame:=tb3_1/map
    $ ROS_NAMESPACE=tb3_2 roslaunch turtlebot3_slam turtlebot3_gmapping.launch set_base_frame:=tb3_2/base_footprint set_odom_frame:=tb3_2/odom set_map_frame:=tb3_2/map
    ```
    
4.  Merge map data from each TurtleBot3
    
    ```
    $ roslaunch turtlebot3_gazebo multi_map_merge.launch
    ```
    
5.  Launch RViz
    
    ```
    $ rosrun rviz rviz -d `rospack find turtlebot3_gazebo`/rviz/multi_turtlebot3_slam.rviz
    ```
    
6.  Operate each TurtleBot3
    
    ```
    $ ROS_NAMESPACE=tb3_0 rosrun turtlebot3_teleop turtlebot3_teleop_key
    $ ROS_NAMESPACE=tb3_1 rosrun turtlebot3_teleop turtlebot3_teleop_key
    $ ROS_NAMESPACE=tb3_2 rosrun turtlebot3_teleop turtlebot3_teleop_key
    ```
    
    ![](https://emanual.robotis.com/assets/images/platform/turtlebot3/simulation/turtlebot3_house_slam1.png)
    
7.  Save the Map
    
    ```
    $ rosrun map_server map_saver -f ~/map
    ``` 

When SLAM in Gazebo simulator, you can select or create various environments and robot models in virtual world. Other than preparing simulation environment instead of bringing up the robot, SLAM Simulation is pretty similar to that of [SLAM](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/#slam) with the actual TurtleBot3.

The following instructions require prerequisites from the previous sections, so please review to the [Simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/) section first.

### Launch Simulation World[](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam_simulation/#launch-simulation-world-1)

Three Gazebo environments are prepared, but for creating a map with SLAM, it is recommended to use either **TurtleBot3 World** or **TurtleBot3 House**.  
Use one of the following commands to load the Gazebo environment.

In this instruction, TurtleBot3 World will be used.  
Please use the proper keyword among `burger`, `waffle`, `waffle_pi` for the `TURTLEBOT3_MODEL` parameter.

```
$ export TURTLEBOT3_MODEL=burger
$ roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

 ![](https://emanual.robotis.com/assets/images/icon_unfold.png) Read more about **How to load TurtleBot3 House**

```
$ export TURTLEBOT3_MODEL=burger
$ roslaunch turtlebot3_gazebo turtlebot3_house.launch
``` 

### Run SLAM Node[](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam_simulation/#run-slam-node-1)

Open a new terminal from Remote PC with `Ctrl` + `Alt` + `T` and run the SLAM node. Gmapping SLAM method is used by default.  
Please use the proper keyword among `burger`, `waffle`, `waffle_pi` for the `TURTLEBOT3_MODEL` parameter.

```
$ export TURTLEBOT3_MODEL=burger
$ roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
```

### Run Teleoperation Node[](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam_simulation/#run-teleoperation-node-1)

Open a new terminal from Remote PC with `Ctrl` + `Alt` + `T` and run the teleoperation node from the Remote PC.  
Please use the proper keyword among `burger`, `waffle`, `waffle_pi` for the `TURTLEBOT3_MODEL` parameter.

```
$ export TURTLEBOT3_MODEL=burger
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

 Control Your TurtleBot3!
 ---------------------------
 Moving around:
        w
   a    s    d
        x

 w/x : increase/decrease linear velocity
 a/d : increase/decrease angular velocity
 space key, s : force stop

 CTRL-C to quit
```

### Save Map[](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam_simulation/#save-map-1)

When the map is created successfully, open a new terminal from Remote PC with `Ctrl` + `Alt` + `T` and save the map.

![](https://emanual.robotis.com/assets/images/platform/turtlebot3/simulation/virtual_slam.png)

```
$ rosrun map_server map_saver -f ~/map
```

![](https://emanual.robotis.com/assets/images/platform/turtlebot3/simulation/map.png)

> The saved map.pgm file

 ![](https://emanual.robotis.com/assets/images/icon_unfold.png) Read more about **How to SLAM with multiple TurtleBot3**

In order to create a map with multiple robots, **multirobot-map-merge** package is required.  
Follow the instructions below instead of **Launching Simulation World** section of this page to operate multiple TurtleBot3.

1.  Install necessary package
    
    ```
    $ sudo apt-get install ros-melodic-multirobot-map-merge
    ```
    
2.  Load multiple TurtleBot3 in TurtleBot3 House.  
    These loaded turtlebot3s are set initial position and orientation.
    
    ```
    $ roslaunch turtlebot3_gazebo multi_turtlebot3.launch
    ```
    
    ![](https://emanual.robotis.com/assets/images/platform/turtlebot3/simulation/turtlebot3_house_slam.png)
    
3.  Launch SLAM for each TurtleBot3
    
    ```
    $ ROS_NAMESPACE=tb3_0 roslaunch turtlebot3_slam turtlebot3_gmapping.launch set_base_frame:=tb3_0/base_footprint set_odom_frame:=tb3_0/odom set_map_frame:=tb3_0/map
    $ ROS_NAMESPACE=tb3_1 roslaunch turtlebot3_slam turtlebot3_gmapping.launch set_base_frame:=tb3_1/base_footprint set_odom_frame:=tb3_1/odom set_map_frame:=tb3_1/map
    $ ROS_NAMESPACE=tb3_2 roslaunch turtlebot3_slam turtlebot3_gmapping.launch set_base_frame:=tb3_2/base_footprint set_odom_frame:=tb3_2/odom set_map_frame:=tb3_2/map
    ```
    
4.  Merge map data from each TurtleBot3
    
    ```
    $ roslaunch turtlebot3_gazebo multi_map_merge.launch
    ```
    
5.  Launch RViz
    
    ```
    $ rosrun rviz rviz -d `rospack find turtlebot3_gazebo`/rviz/multi_turtlebot3_slam.rviz
    ```
    
6.  Operate each TurtleBot3
    
    ```
    $ ROS_NAMESPACE=tb3_0 rosrun turtlebot3_teleop turtlebot3_teleop_key
    $ ROS_NAMESPACE=tb3_1 rosrun turtlebot3_teleop turtlebot3_teleop_key
    $ ROS_NAMESPACE=tb3_2 rosrun turtlebot3_teleop turtlebot3_teleop_key
    ```
    
    ![](https://emanual.robotis.com/assets/images/platform/turtlebot3/simulation/turtlebot3_house_slam1.png)
    
7.  Save the Map
    
    ```
    $ rosrun map_server map_saver -f ~/map
    ``` 

When SLAM in Gazebo simulator, you can select or create various environments and robot models in virtual world. Other than preparing simulation environment instead of bringing up the robot, SLAM Simulation is pretty similar to that of [SLAM](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/#slam) with the actual TurtleBot3.

The following instructions require prerequisites from the previous sections, so please review to the [Simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/) section first.

### Launch Simulation World[](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam_simulation/#launch-simulation-world-2)

Three Gazebo environments are prepared, but for creating a map with SLAM, it is recommended to use either **TurtleBot3 World** or **TurtleBot3 House**.  
Use one of the following commands to load the Gazebo environment.

In this instruction, TurtleBot3 World will be used.  
Please use the proper keyword among `burger`, `waffle`, `waffle_pi` for the `TURTLEBOT3_MODEL` parameter.

```
$ export TURTLEBOT3_MODEL=burger
$ roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

 ![](https://emanual.robotis.com/assets/images/icon_unfold.png) Read more about **How to load TurtleBot3 House**

```
$ export TURTLEBOT3_MODEL=burger
$ roslaunch turtlebot3_gazebo turtlebot3_house.launch
``` 

### Run SLAM Node[](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam_simulation/#run-slam-node-2)

Open a new terminal from Remote PC with `Ctrl` + `Alt` + `T` and run the SLAM node. Gmapping SLAM method is used by default.  
Please use the proper keyword among `burger`, `waffle`, `waffle_pi` for the `TURTLEBOT3_MODEL` parameter.

```
$ export TURTLEBOT3_MODEL=burger
$ roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
```

### Run Teleoperation Node[](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam_simulation/#run-teleoperation-node-2)

Open a new terminal from Remote PC with `Ctrl` + `Alt` + `T` and run the teleoperation node from the Remote PC.  
Please use the proper keyword among `burger`, `waffle`, `waffle_pi` for the `TURTLEBOT3_MODEL` parameter.

```
$ export TURTLEBOT3_MODEL=burger
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

 Control Your TurtleBot3!
 ---------------------------
 Moving around:
        w
   a    s    d
        x

 w/x : increase/decrease linear velocity
 a/d : increase/decrease angular velocity
 space key, s : force stop

 CTRL-C to quit
```

### Save Map[](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam_simulation/#save-map-2)

When the map is created successfully, open a new terminal from Remote PC with `Ctrl` + `Alt` + `T` and save the map.

![](https://emanual.robotis.com/assets/images/platform/turtlebot3/simulation/virtual_slam.png)

```
$ rosrun map_server map_saver -f ~/map
```

![](https://emanual.robotis.com/assets/images/platform/turtlebot3/simulation/map.png)

> The saved map.pgm file

When SLAM in Gazebo simulator, you can select or create various environments and robot models in virtual world. Other than preparing simulation environment instead of bringing up the robot, SLAM Simulation is pretty similar to that of [SLAM](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/#slam) with the actual TurtleBot3.

The following instructions require prerequisites from the previous sections, so please review to the [Simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/) section first.

### Launch Simulation World[](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam_simulation/#launch-simulation-world-3)

Three Gazebo environments are prepared, but for creating a map with SLAM, it is recommended to use either **TurtleBot3 World** or **TurtleBot3 House**.  
Use one of the following commands to load the Gazebo environment.

In this instruction, TurtleBot3 World will be used.  
Please use the proper keyword among `burger`, `waffle`, `waffle_pi` for the `TURTLEBOT3_MODEL` parameter.

```
$ export TURTLEBOT3_MODEL=burger
$ ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

 ![](https://emanual.robotis.com/assets/images/icon_unfold.png) Read more about **How to load TurtleBot3 House**

```
$ export TURTLEBOT3_MODEL=burger
$ ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
``` 

### Run SLAM Node[](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam_simulation/#run-slam-node-3)

Open a new terminal from Remote PC with `Ctrl` + `Alt` + `T` and run the SLAM node. Cartographer SLAM method is used by default.

```
$ export TURTLEBOT3_MODEL=burger
$ ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

### Run Teleoperation Node[](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam_simulation/#run-teleoperation-node-3)

Open a new terminal from Remote PC with `Ctrl` + `Alt` + `T` and run the teleoperation node from the Remote PC.

```
$ export TURTLEBOT3_MODEL=burger
$ ros2 run turtlebot3_teleop teleop_keyboard

 Control Your TurtleBot3!
 ---------------------------
 Moving around:
        w
   a    s    d
        x

 w/x : increase/decrease linear velocity
 a/d : increase/decrease angular velocity
 space key, s : force stop

 CTRL-C to quit
```

### Save Map[](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam_simulation/#save-map-3)

When the map is created successfully, open a new terminal from Remote PC with `Ctrl` + `Alt` + `T` and save the map.

![](https://emanual.robotis.com/assets/images/platform/turtlebot3/simulation/virtual_slam.png)

```
$ ros2 run nav2_map_server map_saver -f ~/map
```

![](https://emanual.robotis.com/assets/images/platform/turtlebot3/simulation/map.png)

> The saved map.pgm file

When SLAM in Gazebo simulator, you can select or create various environments and robot models in virtual world. Other than preparing simulation environment instead of bringing up the robot, SLAM Simulation is pretty similar to that of [SLAM](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/#slam) with the actual TurtleBot3.

The following instructions require prerequisites from the previous sections, so please review to the [Simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/) section first.

### Launch Simulation World[](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam_simulation/#launch-simulation-world-4)

Three Gazebo environments are prepared, but for creating a map with SLAM, it is recommended to use either **TurtleBot3 World** or **TurtleBot3 House**.  
Use one of the following commands to load the Gazebo environment.

In this instruction, TurtleBot3 World will be used.  
Please use the proper keyword among `burger`, `waffle`, `waffle_pi` for the `TURTLEBOT3_MODEL` parameter.

```
$ export TURTLEBOT3_MODEL=burger
$ ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

 ![](https://emanual.robotis.com/assets/images/icon_unfold.png) Read more about **How to load TurtleBot3 House**

```
$ export TURTLEBOT3_MODEL=burger
$ ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
``` 

### Run SLAM Node[](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam_simulation/#run-slam-node-4)

Open a new terminal from Remote PC with `Ctrl` + `Alt` + `T` and run the SLAM node. Cartographer SLAM method is used by default.

```
$ export TURTLEBOT3_MODEL=burger
$ ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

### Run Teleoperation Node[](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam_simulation/#run-teleoperation-node-4)

Open a new terminal from Remote PC with `Ctrl` + `Alt` + `T` and run the teleoperation node from the Remote PC.

```
$ export TURTLEBOT3_MODEL=burger
$ ros2 run turtlebot3_teleop teleop_keyboard

 Control Your TurtleBot3!
 ---------------------------
 Moving around:
        w
   a    s    d
        x

 w/x : increase/decrease linear velocity
 a/d : increase/decrease angular velocity
 space key, s : force stop

 CTRL-C to quit
```

### Save Map[](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam_simulation/#save-map-4)

When the map is created successfully, open a new terminal from Remote PC with `Ctrl` + `Alt` + `T` and save the map.

![](https://emanual.robotis.com/assets/images/platform/turtlebot3/simulation/virtual_slam.png)

```
$ ros2 run nav2_map_server map_saver_cli -f ~/map
```

![](https://emanual.robotis.com/assets/images/platform/turtlebot3/simulation/map.png)

> The saved map.pgm file

When SLAM in Gazebo simulator, you can select or create various environments and robot models in virtual world. Other than preparing simulation environment instead of bringing up the robot, SLAM Simulation is pretty similar to that of [SLAM](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/#slam) with the actual TurtleBot3.

The following instructions require prerequisites from the previous sections, so please review to the [Simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/) section first.

### Launch SLAM Simulation[](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam_simulation/#launch-slam-simulation)

If you are running ROS1 Noetic, please replace `melodic` with `noetic` in the command below.

Please use the proper keyword among `burger`, `waffle`, `waffle_pi` for the `TURTLEBOT3_MODEL` parameter.

```
> c:\opt\ros\melodic\x64\setup.bat
> c:\ws\turtlebot3\devel\setup.bat
> set TURTLEBOT3_MODEL=waffle

> curl -o turtlebot3_demo.launch https://raw.githubusercontent.com/ms-iot/ROSOnWindows/master/docs/Turtlebot/turtlebot3_demo.launch
> roslaunch turtlebot3_demo.launch
```

After a few moments, you will see Gazebo running a simulated world with your simulated TurtleBot3, RViz running the mapping progress, and a simulation node to drive the TurtleBot3 random walking.

![](https://ms-iot.github.io/ROSOnWindows/Extras/Turtlebot3_Gazebo_SLAM.gif)

### Launch Simulation World[](https://emanual.robotis.com/docs/en/platform/turtlebot3/nav_simulation/#launch-simulation-world)

Terminate all applications with `Ctrl` + `C` that were launced in the previous sections.

In the previous [SLAM](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/#slam) section, TurtleBot3 World is used to creat a map. The same Gazebo environment will be used for Navigation.

Please use the proper keyword among `burger`, `waffle`, `waffle_pi` for the `TURTLEBOT3_MODEL` parameter.

```
$ export TURTLEBOT3_MODEL=burger
$ roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

 ![](https://emanual.robotis.com/assets/images/icon_unfold.png) Read more about **How to load TurtleBot3 House**

```
$ export TURTLEBOT3_MODEL=burger
$ roslaunch turtlebot3_gazebo turtlebot3_house.launch
``` 

### Run Navigation Node[](https://emanual.robotis.com/docs/en/platform/turtlebot3/nav_simulation/#run-navigation-node)

Open a new terminal from Remote PC with `Ctrl` + `Alt` + `T` and run the Navigation node.

```
$ export TURTLEBOT3_MODEL=burger
$ roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
```

### [Estimate Initial Pose](https://emanual.robotis.com/docs/en/platform/turtlebot3/nav_simulation/#estimate-initial-pose)[](https://emanual.robotis.com/docs/en/platform/turtlebot3/nav_simulation/#estimate-initial-pose)

### [Set Navigation Goal](https://emanual.robotis.com/docs/en/platform/turtlebot3/nav_simulation/#set-navigation-goal)[](https://emanual.robotis.com/docs/en/platform/turtlebot3/nav_simulation/#set-navigation-goal)

1.  Click the `2D Nav Goal` button in the RViz menu.  
    ![](https://emanual.robotis.com/assets/images/platform/turtlebot3/navigation/2d_nav_goal_button.png)
2.  Click on the map to set the destination of the robot and drag the green arrow toward the direction where the robot will be facing.
    -   This green arrow is a marker that can specify the destination of the robot.
    -   The root of the arrow is `x`, `y` coordinate of the destination, and the angle `θ` is determined by the orientation of the arrow.
    -   As soon as x, y, θ are set, TurtleBot3 will start moving to the destination immediately. ![](https://emanual.robotis.com/assets/images/platform/turtlebot3/navigation/2d_nav_goal.png)

<iframe src="https://www.youtube.com/embed/VYlMywwYALU" allowfullscreen="" width="640" height="358" frameborder="0"></iframe>

Just like the SLAM in Gazebo simulator, you can select or create various environments and robot models in virtual Navigation world. However, proper map has to be prepared before running the Navigation. Other than preparing simulation environment instead of bringing up the robot, Navigation Simulation is pretty similar to that of [Navigation](https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/#navigation).

### Launch Simulation World[](https://emanual.robotis.com/docs/en/platform/turtlebot3/nav_simulation/#launch-simulation-world-1)

Terminate all applications with `Ctrl` + `C` that were launced in the previous sections.

In the previous [SLAM](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/#slam) section, TurtleBot3 World is used to creat a map. The same Gazebo environment will be used for Navigation.

Please use the proper keyword among `burger`, `waffle`, `waffle_pi` for the `TURTLEBOT3_MODEL` parameter.

```
$ export TURTLEBOT3_MODEL=burger
$ roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

 ![](https://emanual.robotis.com/assets/images/icon_unfold.png) Read more about **How to load TurtleBot3 House**

```
$ export TURTLEBOT3_MODEL=burger
$ roslaunch turtlebot3_gazebo turtlebot3_house.launch
``` 

### Run Navigation Node[](https://emanual.robotis.com/docs/en/platform/turtlebot3/nav_simulation/#run-navigation-node-1)

Open a new terminal from Remote PC with `Ctrl` + `Alt` + `T` and run the Navigation node.

```
$ export TURTLEBOT3_MODEL=burger
$ roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
```

### [Estimate Initial Pose](https://emanual.robotis.com/docs/en/platform/turtlebot3/nav_simulation/#estimate-initial-pose)[](https://emanual.robotis.com/docs/en/platform/turtlebot3/nav_simulation/#estimate-initial-pose-1)

### [Set Navigation Goal](https://emanual.robotis.com/docs/en/platform/turtlebot3/nav_simulation/#set-navigation-goal)[](https://emanual.robotis.com/docs/en/platform/turtlebot3/nav_simulation/#set-navigation-goal-1)

1.  Click the `2D Nav Goal` button in the RViz menu.  
    ![](https://emanual.robotis.com/assets/images/platform/turtlebot3/navigation/2d_nav_goal_button.png)
2.  Click on the map to set the destination of the robot and drag the green arrow toward the direction where the robot will be facing.
    -   This green arrow is a marker that can specify the destination of the robot.
    -   The root of the arrow is `x`, `y` coordinate of the destination, and the angle `θ` is determined by the orientation of the arrow.
    -   As soon as x, y, θ are set, TurtleBot3 will start moving to the destination immediately. ![](https://emanual.robotis.com/assets/images/platform/turtlebot3/navigation/2d_nav_goal.png)

<iframe src="https://www.youtube.com/embed/VYlMywwYALU" allowfullscreen="" width="640" height="358" frameborder="0"></iframe>

Just like the SLAM in Gazebo simulator, you can select or create various environments and robot models in virtual Navigation world. However, proper map has to be prepared before running the Navigation. Other than preparing simulation environment instead of bringing up the robot, Navigation Simulation is pretty similar to that of [Navigation](https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/#navigation).

### Launch Simulation World[](https://emanual.robotis.com/docs/en/platform/turtlebot3/nav_simulation/#launch-simulation-world-2)

Terminate all applications with `Ctrl` + `C` that were launced in the previous sections.

In the previous [SLAM](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/#slam) section, TurtleBot3 World is used to creat a map. The same Gazebo environment will be used for Navigation.

Please use the proper keyword among `burger`, `waffle`, `waffle_pi` for the `TURTLEBOT3_MODEL` parameter.

```
$ export TURTLEBOT3_MODEL=burger
$ roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

 ![](https://emanual.robotis.com/assets/images/icon_unfold.png) Read more about **How to load TurtleBot3 House**

```
$ export TURTLEBOT3_MODEL=burger
$ roslaunch turtlebot3_gazebo turtlebot3_house.launch
``` 

### Run Navigation Node[](https://emanual.robotis.com/docs/en/platform/turtlebot3/nav_simulation/#run-navigation-node-2)

Open a new terminal from Remote PC with `Ctrl` + `Alt` + `T` and run the Navigation node.

```
$ export TURTLEBOT3_MODEL=burger
$ roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
```

### [Estimate Initial Pose](https://emanual.robotis.com/docs/en/platform/turtlebot3/nav_simulation/#estimate-initial-pose)[](https://emanual.robotis.com/docs/en/platform/turtlebot3/nav_simulation/#estimate-initial-pose-2)

### [Set Navigation Goal](https://emanual.robotis.com/docs/en/platform/turtlebot3/nav_simulation/#set-navigation-goal)[](https://emanual.robotis.com/docs/en/platform/turtlebot3/nav_simulation/#set-navigation-goal-2)

1.  Click the `2D Nav Goal` button in the RViz menu.  
    ![](https://emanual.robotis.com/assets/images/platform/turtlebot3/navigation/2d_nav_goal_button.png)
2.  Click on the map to set the destination of the robot and drag the green arrow toward the direction where the robot will be facing.
    -   This green arrow is a marker that can specify the destination of the robot.
    -   The root of the arrow is `x`, `y` coordinate of the destination, and the angle `θ` is determined by the orientation of the arrow.
    -   As soon as x, y, θ are set, TurtleBot3 will start moving to the destination immediately. ![](https://emanual.robotis.com/assets/images/platform/turtlebot3/navigation/2d_nav_goal.png)

<iframe src="https://www.youtube.com/embed/VYlMywwYALU" allowfullscreen="" width="640" height="358" frameborder="0"></iframe>


<iframe src="https://www.youtube.com/embed/iHXZSLBJHMg" allowfullscreen="" width="640" height="358" frameborder="0"></iframe>

The contents in e-Manual are subject to be updated without a prior notice. Therefore, some video may differ from the contents in e-Manual.

To use `turtlebot3_fake_node`, you need the `turtlebot3_simulation` metapackage. Install the package as shown in the following command.

**TIP**: The terminal application can be found with the Ubuntu search icon on the top left corner of the screen. The shortcut key for running the terminal is `Ctrl`\-`Alt`\-`T`.

**NOTE**: The `turtlebot3_simulation` metapackage requires `turtlebot3` metapackage and `turtlebot3_msgs` package as a prerequisite. If you didn’t install it in the [Install Dependent ROS Packages of PC Setup](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#install-dependent-ros-1-packages-1) section, install it first.

```
$ cd ~/catkin_ws/src/
$ git clone -b kinetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
$ cd ~/catkin_ws && catkin_make
```

To launch the virtual robot, execute the `turtlebot3_fake.launch` file in the `turtlebot3_fake` package as shown below. The `turtlebot3_fake` is a very simple simulation node that can be run without having an actual robot. You can even control the virtual TurtleBot3 in RViz with a teleoperation node.

Please use the proper keyword among `burger`, `waffle`, `waffle_pi` for the `TURTLEBOT3_MODEL` parameter.

```
$ export TURTLEBOT3_MODEL=burger
$ roslaunch turtlebot3_fake turtlebot3_fake.launch
```

```
$ export TURTLEBOT3_MODEL=burger
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

<iframe src="https://www.youtube.com/embed/iHXZSLBJHMg" allowfullscreen="" width="640" height="358" frameborder="0"></iframe>

The contents in e-Manual are subject to be updated without a prior notice. Therefore, some video may differ from the contents in e-Manual.

To use `turtlebot3_fake_node`, you need the `turtlebot3_simulation` metapackage. Install the package as shown in the following command.

**TIP**: The terminal application can be found with the Ubuntu search icon on the top left corner of the screen. The shortcut key for running the terminal is `Ctrl`\-`Alt`\-`T`.

**NOTE**: The `turtlebot3_simulation` metapackage requires `turtlebot3` metapackage and `turtlebot3_msgs` package as a prerequisite. If you didn’t install it in the [Install Dependent ROS Packages of PC Setup](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#install-dependent-ros-1-packages-1) section, install it first.

```
$ cd ~/catkin_ws/src/
$ git clone -b melodic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
$ cd ~/catkin_ws && catkin_make
```

To launch the virtual robot, execute the `turtlebot3_fake.launch` file in the `turtlebot3_fake` package as shown below. The `turtlebot3_fake` is a very simple simulation node that can be run without having an actual robot. You can even control the virtual TurtleBot3 in RViz with a teleoperation node.

Please use the proper keyword among `burger`, `waffle`, `waffle_pi` for the `TURTLEBOT3_MODEL` parameter.

```
$ export TURTLEBOT3_MODEL=burger
$ roslaunch turtlebot3_fake turtlebot3_fake.launch
```

```
$ export TURTLEBOT3_MODEL=burger
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

<iframe src="https://www.youtube.com/embed/iHXZSLBJHMg" allowfullscreen="" width="640" height="358" frameborder="0"></iframe>

The contents in e-Manual are subject to be updated without a prior notice. Therefore, some video may differ from the contents in e-Manual.

To use `turtlebot3_fake_node`, you need the `turtlebot3_simulation` metapackage. Install the package as shown in the following command.

**TIP**: The terminal application can be found with the Ubuntu search icon on the top left corner of the screen. The shortcut key for running the terminal is `Ctrl`\-`Alt`\-`T`.

**NOTE**: The `turtlebot3_simulation` metapackage requires `turtlebot3` metapackage and `turtlebot3_msgs` package as a prerequisite. If you didn’t install it in the [Install Dependent ROS Packages of PC Setup](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#install-dependent-ros-1-packages-1) section, install it first.

```
$ cd ~/catkin_ws/src/
$ git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
$ cd ~/catkin_ws && catkin_make
```

To launch the virtual robot, execute the `turtlebot3_fake.launch` file in the `turtlebot3_fake` package as shown below. The `turtlebot3_fake` is a very simple simulation node that can be run without having an actual robot. You can even control the virtual TurtleBot3 in RViz with a teleoperation node.

Please use the proper keyword among `burger`, `waffle`, `waffle_pi` for the `TURTLEBOT3_MODEL` parameter.

```
$ export TURTLEBOT3_MODEL=burger
$ roslaunch turtlebot3_fake turtlebot3_fake.launch
```

```
$ export TURTLEBOT3_MODEL=burger
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

<iframe src="https://www.youtube.com/embed/iHXZSLBJHMg" allowfullscreen="" width="640" height="358" frameborder="0"></iframe>



