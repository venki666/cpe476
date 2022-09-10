## Simple ROS Examples

### Hello World Package in C++

Open a terminal (ctrl+alt+t). Copy and paste the following codes into your terminal. Please modify the maintainer’s name and email, and also the author’s name and email.

```
# create catkin workspace
cd ~
mkdir -p catkin_ws/src

# initialize catkin workspace
cd ~/catkin_ws
catkin init --workspace .

# create ros package (hello_world)
cd src
catkin create pkg -c roscpp rospy hello_world
＃ change working directory to source folder under package folder
cd hello_world/src
```

Open your favorite editor (e.g., Emacs, Sublime Text 3) to edit the package.xml and CMakeLists.txt files if you did something wrong when you created the package, or you want to add more information.

Create hello_world_node.cpp file with following contents.

// Include the ROS C++ APIs
#include <ros/ros.h>

// Standard C++ entry point
int main(int argc, char** argv) {
  // Announce this program to the ROS master as a "node" called "hello_world_node"
  ros::init(argc, argv, "hello_world_node");
  // Start the node resource managers (communication, time, etc)
  ros::start();
  // Broadcast a simple log message
  ROS_INFO_STREAM("Hello, world!");
  // Process ROS callbacks until receiving a SIGINT (ctrl-c)
  ros::spin();
  // Stop the node's resources
  ros::shutdown();
  // Exit tranquilly
  return 0;
}

Modify corresponding parts in the package.xml and CMakeLists.txt so that the catkin tools can help you to compile the executables correctly.

You can specify additional locations of header files, declare C++ executables and anything else you want to control in the Build part of the CMakeLists.txt file. For example, in this example, we only need to declare one C++ executable named hello_world_node and link the catkin library to the executable.

```
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(hello_world
#   src/${PROJECT_NAME}/hello_world.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(hello_world ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(hello_world_node src/hello_world_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(hello_world_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(hello_world_node
  ${catkin_LIBRARIES}
)
```

It is not necessary to modify anything in the package.xml in this example. However, if you have multiple packages and some of the packages have sort of dependents on the other packages, you can modify the package.xml file to tell the catkin tools the dependent relationship of all the packages.

You should have a similar architecture tree as follows.

my_catkin_ws/
└── src
    └── hello_world
        ├── CMakeLists.txt [modified]
        ├── include
        │   └── hello_world
        ├── package.xml [modified]
        └── src
            └── hello_world_node.cpp [created]

Now you are ready to compile the ROS package by catkin tools using the following command.

```
# add ROS environment variables to your current bash session (modify kinetic to lunar if you are using lunar)

source /opt/ros/kinetic/setup.bash

# since the above command will be used quite frequently, we can add an alias into ~/.bashrc as follows

# echo "'source /opt/ros/kinetic/setup.bash'" >> ~/.bashrc

#### Build the package
catkin build hello_world
```

After compiling the package, you can launch the ROS node. Please do remember to start a the roscore before executing the node.

```
# add catkin workspace environment variables to your current bash session
source ~/catkin_ws/devel/setup.bash

# run the node (executable)
rosrun hello_world hello_world_node
```

By running the node, you should be able to observe the following results in your terminal.

[ INFO] [1475573239.386991148]: Hello, world!



### Play with Turtlesim

Install some dependences and turtlesim.

```
sudo apt-get install qt5-default
sudo apt-get install ros-noetic-ros-tutorials
```

Open three terminals, and add catkin workspace environment variables to each of them.
Terminal 1:
```
# add catkin workspace environment variables to your current bash session
source ~/catkin_ws/devel/setup.bash

# launch roscore
roscore
```

Terminal 2:
```
# add catkin workspace environment variables to your current bash session
source ~/catkin_ws/devel/setup.bash

# run graphic turtle simulator node
rosrun turtlesim turtlesim_node
```

Terminal 3:
```
# add catkin workspace environment variables to your current bash session
source ~/catkin_ws/devel/setup.bash

# run teleoperation keyboard node
rosrun turtlesim turtle_teleop_key
```

Enjoy playing with the lovely little turtle.
Using the following tools can help you to have a better understanding of the topology of the running ROS nodes.

```
rosnode [ping|list|info]
rostopic [echo|pub|hz|info|list|type]
rosrun rqt_graph rqt_graph
```



### Test Webcam 

```
$ lsusb
$ ls /dev | grep video*
```

Install the node ROS usb_cam with the necessary dependencies:

```
$ sudo apt install ros-kinetic-usb-cam
```


 The usb_cam node already has a test startup file:

```
$ cat /opt/ros/kinetic/share/usb_cam/launch/usb_cam-test.launch
```


 Before launching this file, let's run the ROS kernel on master:

```
$ roscore
```


And now run the usb_cam node on the slave:

```
$ roslaunch usb_cam usb_cam-test.launch
```


Now we can see the topics created. We can check them either on the master or on the slave.

Put the current process in the background using CTRL + Z and execute the bg command to continue execution in the background. (on the Ubuntu version not full desktop and no screen, just run another terminal)

To see the themes in the terminal:

```
$ rostopic list
```


 ... or in the graphical interface:

```
$ rqt_graph
```

Reading camera data using image_view:

```
$ rosrun image_view image_view image:=/usb_cam/image_raw
```



### Minimal Nodes Tutorials

#### minimal_publisher Example

in any terminal (e.g. ctl-shift-t or file-> open tab)

```
roscd
catkin_make
```

in a terminal, start a roscore:

```
roscore
```

in another terminal, run the new pgm:

```
rosrun minimal_nodes minimal_publisher
```

in another terminal:

```
rostopic list
rostopic info topic1
rosmsg show std_msgs/Float64
rostopic echo topic1
rostopic hz topic1
```

see performance meter

see performance meter

#### sleepy_minimal_publisher

```
rosrun minimal_nodes sleepy_minimal_publisher
rostopic echo topic1
rostopic hz topic1
see performance meter

rosrun minimal_nodes minimal_subscriber
(see behavior w/ and w/o minimal publisher running)

rostopic pub -r 1 topic1 std_msgs/Float64 1.23

rosnode list
rosnode info minimal_subscriber
rqt_graph

kill running nodes (cltrC)
roslaunch minimal_nodes minimal_nodes.launch
rqt_console
```

----rosbag----

```
rosbag record topic1
restart minimal subscriber
rosbag play fname.bag
rqt_console
```



#### minimal simulator

```
rosrun minimal_nodes minimal_simulator
rostopic pub -r 10 force_cmd std_msgs/Float64 0.1
rqt_plot
kill publisher
rosrun minimal_nodes minimal_controller
rostopic pub -r 10 vel_cmd std_msgs/Float64 1.0
```

--- Let's make a new class and node ---

```
Create a folder: ~/Catkin_ws/src/rosclass_minimal_nodes/src
edit minimal_simulator.cpp and minimal_controller.cpp; save in rosclass_minimal_nodes/src
edit CMakeLists.txt:
cs_add_executable(rc_minimal_simulator src/minimal_simulator.cpp)
cs_add_executable(rc_minimal_controller src/minimal_controller.cpp)
catkin_make
rosrun rosclass_minimal_nodes rc_minimal_simulator
rostopic pub -r 10 force_cmd std_msgs/Float64 0.1
rqt_plot
kill publisher
rosrun rosclass_minimal_nodes rc_minimal_controller
rostopic pub -r 10 vel_cmd std_msgs/Float64 1.0
```

--- alternately you could have also done ---

```
roscd
cd src
catkin_create_pkg rosclass_minimal_nodes roscpp std_msgs
cd rosclass_minimal_nodes/src
Copy minimal_simulator.cpp and minimal_controller.cpp; save in rosclass_minimal_nodes/src
edit CMakeLists.txt:
cs_add_executable(rc_minimal_simulator src/minimal_simulator.cpp)
cs_add_executable(rc_minimal_controller src/minimal_controller.cpp)
catkin_make
rosrun rosclass_minimal_nodes rc_minimal_simulator
rostopic pub -r 10 force_cmd std_msgs/Float64 0.1
rqt_plot
kill publisher
rosrun rosclass_minimal_nodes rc_minimal_controller
rostopic pub -r 10 vel_cmd std_msgs/Float64 1.0
```


