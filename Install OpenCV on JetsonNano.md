##Installing OpenCV 3.4.6 on Jetson Nano

Based on the notes from https://jkjung-avt.github.io/opencv-on-nano/

Aggregate all steps of building/installing OpenCV into a shell scripts. The pre-installed opencv-3.3.1 on Jetson Nano does not support gstreamer functionalities (cannot utilize hardware H.264/H.265 codec on Jetson Nano). That’s the reason why I need to build and install OpenCV by myself. I actually purge the pre-installed opencv-3.3.1 packages at the beginning of my istall_opencv-3.4.6.sh script.

Although NVIDIA provides a script for installing opencv-4.0.0 on Jetson Nano, I’m sticking with 3.4.x because Caffe does not build with opencv-4.x. In my script I use opencv-3.4.6 since that’s the latest 3.4.x release as of the time of this writing.

configured (cmake) my opencv-3.4.6 with -D ENABLE_FAST_MATH=ON -D CUDA_FAST_MATH=ON in the script.

Script are found @ https://github.com/jkjung-avt/jetson_nano 
###Prerequisite

Please go through the steps I described in [Setting up Jetson Nano: The Basics](https://jkjung-avt.github.io/setting-up-nano/). I’d strongly suggest you to **set up a swap file** on the Jetson Nano DevKit since its memory is quite limited.

Installing opencv-3.4.6 on Jetson Nano using my script is straightforward. But I’d like to highlight a few things first:

-   If you’d like to remove previously built and installed OpenCV libraries, just search all files with names containing ‘opencv’ or ‘cv2’ and delete all of them. You can do such a search by using the `find` command.
    
    ```
    $ find /usr/local -name "*opencv*" -o -name "*cv2*"
    ```
    
    This is usually **not** necessary though. When you re-build opencv and do `sudo make install`, the old files would just be replaced by the newly installed files and things would just work as expected.
    
-   If you’d like to modify some configurations and rebuild opencv-3.4.6, it’s recommended to remove the whole `build` directory and then redo the `cmake`. For example,
    
    ```
    $ cd ${HOME}/src/opencv-3.4.6
    $ rm -rf build/
    $ mkdir build
    $ cd build
    $ cmake XXXXXX ..  # details omitted
    $ make -j3
    $ sudo make install
    ```
    
-   The `install_opencv-3.4.6.sh` script should work for **Jetson TX2** or **Jetson AGX Xavier** (with JetPack-4.2) too. Just remember to **set ‘CUDA\_ARCH\_BIN’** (in the `cmake` command in the script) to the right value for the platform.
    
    -   Jetson TX2: `CUDA_ARCH_BIN="6.2"`
    -   Jetson AGX Xavier: `CUDA_ARCH_BIN="7.2"`
-   During execution of the `install_opencv-3.4.6.sh` script, the sudo username/password would likely time out a few times. You’ll have to re-enter your password every time when prompted. If this is an issue for you and you don’t have security related concerns, you could [use visudo to set a longer `timestamp_timeout` value (or set the value to `-1` for no timeout at all) for sudo sessions](https://lifehacker.com/make-sudo-sessions-last-longer-in-linux-1221545774).
    

OK. Here is the part about executing the script and building/installing opencv-3.4.6.

If you haven’t cloned my GitHub ‘jetson\_nano’ repository. Do so now.

Then make sure Jetson Nano is in 10W (maximum) performance mode so the building process could finish as soon as possible.

```
$ sudo nvpmodel -m 0
$ sudo jetson_clocks
```

Then just execute the `install_opencv-3.4.6.sh` script. Note the script would download and unzip opencv-3.4.6 source files into `${HOME}/src/opencv-3.4.6`, and build the code from there.

```
$ cd ${HOME}/project/jetson_nano
$ ./install_opencv-3.4.6.sh
```

The building and installing process would take a couple of hours. When done, you should see both python3 and python2 reporting the correct version number of ‘cv2’ module: `3.4.6`.

Thanks to [mdegans](https://devtalk.nvidia.com/default/topic/1049972/jetson-nano/opencv-cuda-python-with-jetson-nano/post/5342206/#5342206), who pointed out there are built-in tests in opencv which we could use to verify the library we built on Jetson Nano. I ran the test and [reviewed all failure cases](https://devtalk.nvidia.com/default/topic/1049972/jetson-nano/opencv-cuda-python-with-jetson-nano/post/5342609/#5342609). Overall I think the opencv-3.4.6 library I built on Jetson Nano with the script is good.

[tegra-cam.py](https://jkjung-avt.github.io/tx2-camera-with-python/) script to test my opencv-3.4.6 build. Note that I use a USB webcam for the testing. Just plug the USB webcam into one of the USB ports on Jetson Nano and: (Adjust image width/height for the camera you’re using if necessary)

And voila, the Jason Nano DevKit in action, running with the freshly built and installed opencv-3.4.6, and shot from the webcam.

