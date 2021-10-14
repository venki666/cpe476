## ROS step by step
(from https://roboticsbackend.com/ros-include-cpp-header-from-another-package/)

Create your library package
First, create a package for your library. We’ll only have one dependency to roscpp here.

$ cd ~/catkin_ws/src
$ catkin_create_pkg my_roscpp_library roscpp
$ cd my_roscpp_library
$ touch src/my_super_roscpp_library.cpp
$ touch include/my_roscpp_library/my_super_roscpp_library.h
After creating the package, add a header and a cpp file. Usually when you create such a library, the name of the package and the name of the file is the same. Here we would’ve had “my_roscpp_library.cpp” and “my_roscpp_library.h”.

For the sake of this tutorial I chose to use a different name for package and files – “my_super_roscpp_library” – so you can clearly differentiate the library name from header name during the post.

It’s also a good practice to put your header files into the include/package_name/ folder, and not just inside include/.

Here’s what your package should look like:

my_roscpp_library/
├── CMakeLists.txt
├── package.xml
├── include
│   └── my_roscpp_library
│       └── my_super_roscpp_library.h
└── src
    └── my_super_roscpp_library.cpp
Write your Cpp library
Here it’s your job to write whatever code you want in your library. The code you write can depend on ROS – as we did here with a dependency to roscpp, but it can also be ROS-independent. You could just make a simple Cpp library available for your other ROS packages.

For simplicity I’ll make the 2 files very short.

my_super_roscpp_library.h

#ifndef MY_SUPER_ROSCPP_LIBRARY_H
#define MY_SUPER_ROSCPP_LIBRARY_H
#include <ros/ros.h>
void sayHello();
#endif
my_super_roscpp_library.cpp

#include "my_roscpp_library/my_super_roscpp_library.h"
void sayHello()
{
    ROS_INFO("Hello!");
}
The header file is not directly in the source/ folder, but instead in the source/my_roscpp_library/ folder. So, you have to take that into account for your include lines.

Note: here I use ROS logging directly into the library. If you don’t use any ROS functionality, it’s better not to use ROS logging either, so your library can be completely ROS-independent. For this example I’ll assume that we’re using ROS features inside the library.

Install the library
CMakeLists.txt
cmake_minimum_required(VERSION 2.8.3)
project(my_roscpp_library)
find_package(catkin REQUIRED COMPONENTS
  roscpp
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
#  DEPENDS other non-ROS libs
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_library(${PROJECT_NAME}
  src/my_super_roscpp_library.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
Breaking down the code
Let’s break this CMakeLists.txt line by line.

cmake_minimum_required(VERSION 2.8.3)
project(my_roscpp_library)
Nothing unusual here, that’s the same for every ROS package you create.

find_package(catkin REQUIRED COMPONENTS
  roscpp
)
Add all the other packages that you use from this package.

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
#  DEPENDS other non-ROS libs
)
Withing catkin_package(), we give some useful info for the build system. Don’t forget INCLUDE_DIRS and LIBRARIES so you can install the library later. Also you’ll put all your ROS dependencies with CATKIN_DEPENDS, and all your non-ROS dependencies with DEPENDS.

The catkin_package() needs to be called before add_library() and install().

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
Add the “include” directory here or uncomment it.

add_library(${PROJECT_NAME}
  src/my_super_roscpp_library.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
Just as you would do for a node executable – with add_executable() – use add_library() to build your library. You’ll also need to call target_link_libraries() with any ROS/non-ROS library you’re using in your code.

Here you can see that your library name doesn’t have to have the same name as your cpp files.

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
And finally, after the build, you can install your library with those lines.

Pay attention to the “include/${PROJECT_NAME}/”. Previously I said it’s a best practice to put your header files into the include/package_name/ folder. Here you can see why it’s important.

package.xml
OK, you’ve now correctly written your CMakeLists.txt. One more thing before you compile: edit your package.xml. This will be much quicker.

In the package.xml you’ll have to add all dependencies you need for your library. For this example we’re using roscpp, so we add <depend>roscpp</depend> after the <buildtool_depend>catkin</buildtool_depend> tag.

Compile your library
Now, compile with catkin_make and you should see those logs.

$ catkin_make
...
Scanning dependencies of target my_roscpp_library
[ 85%] Building CXX object my_roscpp_library/CMakeFiles/my_roscpp_library.dir/src/my_super_roscpp_library.cpp.o
[ 90%] Linking CXX shared library /home/user/catkin_ws/devel/lib/libmy_roscpp_library.so
[ 90%] Built target my_roscpp_library
...
Include your header in another ROS package
The library is now installed and available to your other ROS packages.

To test it, simply create a node in another package of your choice.

#include <ros/ros.h>
#include "my_roscpp_library/my_super_roscpp_library.h"
int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_include_library");
    ros::NodeHandle nh;
    sayHello();
}
You can see that to include the header, first, we use the name of the library (“my_roscpp_library”) followed by the name of the header we want to include (“my_super_roscpp_library.h”).

Then we just call the function declared in the library header, sayHello().

Inside the CMakeLists.txt (of the other package, not the library package), add this:

...
find_package(catkin REQUIRED COMPONENTS
  roscpp
  my_roscpp_library
)
...
add_executable(test_include_library src/test_include_library.cpp)
target_link_libraries(test_include_library ${catkin_LIBRARIES})
...
And add a <depend>my_roscpp_library</depend> tag inside the package.xml.

This way you have properly set your dependencies.

Now you can compile with catkin_make.

Finally, make sure you source ~/.bashrc, run roscore in another terminal, and start the node:

$ rosrun my_robot_tutorials test_include_library 
[ INFO] [1572680138.467959220]: Hello!
Success! Now you know how to include a header file from another package in ROS.
