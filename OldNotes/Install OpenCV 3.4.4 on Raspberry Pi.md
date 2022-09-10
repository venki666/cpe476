## Install OpenCV 3.4.4 on Raspberry Pi

In this post, we will provide a **bash script** for installing **OpenCV-3.4.4** (C++, Python 2.7 and Python 3.5) on Raspbian Operating System on Raspberry Pi. We will also briefly study the script to understand what’s going in it.

**Note that this script takes around 3 times more on Raspberry Pi 2 as compared to Raspberry Pi 3**.

If you are still not able to install OpenCV on your system, but want to get started with it, we suggest using our docker images with pre-installed OpenCV, Dlib, miniconda and jupyter notebooks along with other dependencies as described in **[this post](https://learnopencv.com/install-opencv-docker-image-ubuntu-macos-windows)**.

#### Step 0: Select OpenCV version to install

First let’s prepare the system for the installation. Remove unwanted and create space.
```
sudo apt-get -y purge wolfram-engine
sudo apt-get -y purge libreoffice*
sudo apt-get -y autoremove
```  
We are also going to clean `build` directories and create `installation` directory.

#### Clean build directories
```
mkdir installation/OpenCV-"$cvVersion"
```
Finally, we will be storing the current working directory in `cwd` variable. We are also going to refer to this directory as **OpenCV\_Home\_Dir** throughout this article.


#### Step 1: Update Packages

#### Step 2: Install OS Libraries

**Download Code** To easily follow along this tutorial, please download code by clicking on the button below. It's FREE!

####Install Dependencies
```
sudo apt-get -y remove x264 libx264-dev
sudo apt-get -y install build-essential checkinstall cmake pkg-config yasm
sudo apt-get -y install git gfortran
sudo apt-get -y install libjpeg8-dev libjasper-dev libpng12-dev
sudo apt-get -y install libtiff5-dev
sudo apt-get -y install libtiff-dev
sudo apt-get -y install libavcodec-dev libavformat-dev libswscale-dev libdc1394-22-dev
sudo apt-get -y install libxine2-dev libv4l-dev
sudo ln -s -f ../libv4l1-videodev.h videodev.h
sudo apt-get -y install libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev
sudo apt-get -y install libgtk2.0-dev libtbb-dev qt5-default
sudo apt-get -y install libatlas-base-dev
sudo apt-get -y install libmp3lame-dev libtheora-dev
sudo apt-get -y install libvorbis-dev libxvidcore-dev libx264-dev
sudo apt-get -y install libopencore-amrnb-dev libopencore-amrwb-dev
sudo apt-get -y install libavresample-dev
sudo apt-get -y install x264 v4l-utils
```
#### Optional dependencies
```
sudo apt-get -y install libprotobuf-dev protobuf-compiler
sudo apt-get -y install libgoogle-glog-dev libgflags-dev
sudo apt-get -y install libgphoto2-dev libeigen3-dev libhdf5-dev doxygen
```
#### Step 3: Install Python Libraries
```
sudo apt-get -y install python3-dev python3-pip
sudo -H pip3 install -U pip numpy
sudo apt-get -y install python3-testresources
```
We are also going to install `virtualenv` and `virtualenvwrapper` modules to create Python virtual environments.

#### Install virtual environment
```
python3 -m venv OpenCV-``"$cvVersion"``-py3
echo "# Virtual Environment Wrapper" >> ~/.bashrc
echo "alias workoncv-$cvVersion=\"source $cwd/OpenCV-$cvVersion-py3/bin/activate\"" >> ~/.bashrc
source "$cwd"``/OpenCV-``"$cvVersion"``-py3/bin/activate
```
**If you are solely a Python user, it is easier to use pip install opencv-contrib-python==3.4.4.19**.


############ For Python 3 ############
now install python libraries within this virtual environment
```
sudo sed -i 's/CONF_SWAPSIZE=100/CONF_SWAPSIZE=1024/g' /etc/dphys-swapfile
sudo /etc/init.d/dphys-swapfile stop
sudo /etc/init.d/dphys-swapfile start
pip install numpy dlib
```
quit virtual environment
```
deactivate
```


#### Step 4: Download opencv and opencv\_contrib

#### Step 5: Compile and install OpenCV with contrib modules

First we navigate to the build directory.

Next, we start the compilation and installation process.
```
cmake -D CMAKE_BUILD_TYPE=RELEASE \
 -D CMAKE_INSTALL_PREFIX=$cwd/installation/OpenCV-``"$cvVersion" \
 -D INSTALL_C_EXAMPLES=ON \
 -D INSTALL_PYTHON_EXAMPLES=ON \
 -D OPENCV_PYTHON3_INSTALL_PATH=$cwd/OpenCV-$cvVersion-py3/lib/python3.5/site-packages \
 -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
 -D BUILD_EXAMPLES=ON ..
```
For system wide installation of OpenCV, change **CMAKE\_INSTALL\_PREFIX** to **CMAKE\_INSTALL\_PREFIX=/usr/local \\**.

#### Step 6: Reset swap file

Once we are done with installing heavy Python modules like Numpy, it’s time to reset the swap file.
```
sudo sed -i 's/CONF_SWAPSIZE=1024/CONF_SWAPSIZE=100/g' /etc/dphys-swapfile
sudo /etc/init.d/dphys-swapfile stop
sudo /etc/init.d/dphys-swapfile start
```
Finally, we also need to add a simple statement to make sure that **VideoCapture(0)** works on our Raspberry Pi.
```
echo "sudo modprobe bcm2835-v4l2" >> ~/.profile
```
#### How to use OpenCV in C++
There are two ways to use OpenCV in C++, the preferred way is to use **CMake**, the other one being command line compilation using **g++**. We will have a look at both ways.

### Using CMakeLists.txt

The basic structure of your **CMakeLists.txt** will stay the same. Only difference being, that you will have to set **OpenCV\_DIR** as shown below.
```
SET(OpenCV_DIR <OpenCV_Home_Dir>/installation/OpenCV-3.4.4/share/OpenCV/)
```
Make sure that you replace **OpenCV\_Home\_Dir** with correct path. For example, in my case:
```
SET(OpenCV_DIR /home/hp/OpenCV_installation/installation/OpenCV-3.4.4/share/OpenCV/)
```
Once you have made your CMakeLists.txt, follow the steps given below.
```
cmake --build . --config Release
```
This will generate your executable file in **build** directory.

#### Using g++

To compile a sample file (let’s say `my_sample_file.cpp`), use the following command.
```
g++ `pkg-config --cflags --libs <OpenCV_Home_Dir>/installation/OpenCV-3.4.4/lib/pkgconfig/opencv.pc` my_sample_file.cpp -o my_sample_file
```

#### How to use OpenCV in Python

To use the OpenCV version installed using Python script, first we activate the correct Python Virtual Environment.

#### For OpenCV-3.4.4 : Python 3

Once you have activated the virtual environment, you can enter Python shell and test OpenCV version.

Hope this script proves to be useful for you :). Stay tuned for more interesting stuff. In case of any queries, feel free to comment below and we will get back to you as soon as possible.

