# Installation Dependency Issues 
You may or may not encounter these, but I documented solutions as I encountered problems. If you were able to successfully compile the build, you can [skip ahead to the next section](https://github.com/jkeller52/astrobee/blob/master/README.md#simulation)

### Luajit20
Error: Luajit20 not found
`sudo apt install libluajit-5.1-dev`
This resolved the Ubuntu 16.04 WSL2 VM's issue finding Luajit, even though the package was installed and build instructions were followed. 
[Luajit Documentation](http://luajit.org/download.html)

### OpenCV
[Installing OpenCV on Ubuntu 16.04](http://www.codebind.com/cpp-tutorial/install-opencv-ubuntu-cpp/)

I encountered a problem after following this tutorial when trying to verify that opencv was installed correctly:
```
jkell@DESKTOP-CSDA0LG:~$ pkg-config --modversion opencv
Package opencv was not found in the pkg-config search path.
Perhaps you should add the directory containing `opencv.pc`
to the PKG_CONFIG_PATH environment variable
No package 'opencv' found
```
Running `sudo apt install libopencv-dev` fixed this issue for me. When I checked the version of opencv I had installed it worked correctly:
```
jkell@DESKTOP-CSDA0LG:~$ pkg-config --modversion opencv
2.4.9.1
```


To install a suitable version of OpenCV (anything above 3 won't work):
```
wget https://github.com/opencv/opencv/archive/3.4.12.zip
unzip 3.4.12.zip
cd opencv-3.4.12
mkdir build
cd build
cmake ../
sudo make install
```

After, test that it is recognized by the astrobee build:
```
cd ~/astrobee
./scripts/configure.sh -l -F -D
```

### LibUSB
After the above step, my new output read:
```
jkell@DESKTOP-CSDA0LG:~/astrobee$ ./scripts/configure.sh -l -F -D
configuring for native linux...
Remove the CMake Cache for /home/jkell/astrobee_build/native
cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_INSTALL_PREFIX=/home/jkell/astrobee_install/native -DUSE_DDS=off -DENABLE_PICOFLEXX=off /home/jkell/astrobee
ROS distro variable not set. Trying to figure it out...
-- Performing Test COMPILER_SUPPORTS_CXX11
-- Performing Test COMPILER_SUPPORTS_CXX11 - Success
-- Performing Test COMPILER_SUPPORTS_CXX0X
-- Performing Test COMPILER_SUPPORTS_CXX0X - Success
-- Try OpenMP C flag = [-fopenmp]
-- Performing Test OpenMP_FLAG_DETECTED
-- Performing Test OpenMP_FLAG_DETECTED - Success
-- Try OpenMP CXX flag = [-fopenmp]
-- Performing Test OpenMP_FLAG_DETECTED
-- Performing Test OpenMP_FLAG_DETECTED - Success
-- Found OpenMP: -fopenmp
-- Found Glog: /usr/include
-- Performing Test GFLAGS_IN_GOOGLE_NAMESPACE
-- Performing Test GFLAGS_IN_GOOGLE_NAMESPACE - Success
-- Found Gflags: /usr/include
-- Found Threads: TRUE
-- Found Protobuf: /usr/lib/x86_64-linux-gnu/libprotobuf.so
-- Found ceres: /usr/lib/libceres.so
-- Found ZLIB: /usr/lib/x86_64-linux-gnu/libz.so (found version "1.2.8")
-- Found Luajit20: /usr/local/lib/libluajit-5.1.so;/usr/lib/x86_64-linux-gnu/libm.so (found version "2.0.4")
-- Found OpenCV: /usr/local (found suitable version "3.4.10", minimum required is "3")
-- Boost version: 1.58.0
-- Found the following Boost libraries:
--   serialization
--   system
--   filesystem
--   thread
--   program_options
--   date_time
--   timer
--   chrono
--   regex
--   atomic
-- GTSAM include directory:  /usr/share/gtsam/cmake/../../../include
-- Found jsoncpp: /usr/include/jsoncpp
CMake Error at /usr/share/cmake-3.5/Modules/FindPackageHandleStandardArgs.cmake:148 (message):
  Could NOT find USB (missing: USB_LIBRARY USB_INCLUDE_DIR)
Call Stack (most recent call first):
  /usr/share/cmake-3.5/Modules/FindPackageHandleStandardArgs.cmake:388 (_FPHSA_FAILURE_MESSAGE)
  cmake/FindUSB.cmake:39 (find_package_handle_standard_args)
  CMakeLists.txt:255 (find_package)


-- Configuring incomplete, errors occurred!
See also "/home/jkell/astrobee_build/native/CMakeFiles/CMakeOutput.log".
```
Better... Now to troubleshoot my new errors. Seems that it can't find libusb

I attempted to fix this by running `sudo apt-get install libpng-dev libimlib2-dev`. No luck.


Then tried installing these dependencies 

```
sudo apt-get install libxext-dev
sudo apt-get install libpng-dev
sudo apt-get install libimlib2-dev
sudo apt-get install libglew-dev
sudo apt-get install libxrender-dev
sudo apt-get install libxrandr-dev
sudo apt-get install libglm-dev
```
Also no luck. 


Then tried `sudo apt install libusb-dev`. No luck there.


This worked: (!!)
```
sudo apt-get install libusb-1.0-0-dev
```
 New Output:
 ```
 jkell@DESKTOP-CSDA0LG:~/astrobee$ ./scripts/configure.sh -l -F -D
configuring for native linux...
Remove the CMake Cache for /home/jkell/astrobee_build/native
cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_INSTALL_PREFIX=/home/jkell/astrobee_install/native -DUSE_DDS=off -DENABLE_PICOFLEXX=off /home/jkell/astrobee
ROS distro variable not set. Trying to figure it out...
-- Performing Test COMPILER_SUPPORTS_CXX11
-- Performing Test COMPILER_SUPPORTS_CXX11 - Success
-- Performing Test COMPILER_SUPPORTS_CXX0X
-- Performing Test COMPILER_SUPPORTS_CXX0X - Success
-- Try OpenMP C flag = [-fopenmp]
-- Performing Test OpenMP_FLAG_DETECTED
-- Performing Test OpenMP_FLAG_DETECTED - Success
-- Try OpenMP CXX flag = [-fopenmp]
-- Performing Test OpenMP_FLAG_DETECTED
-- Performing Test OpenMP_FLAG_DETECTED - Success
-- Found OpenMP: -fopenmp
-- Found Glog: /usr/include
-- Performing Test GFLAGS_IN_GOOGLE_NAMESPACE
-- Performing Test GFLAGS_IN_GOOGLE_NAMESPACE - Success
-- Found Gflags: /usr/include
-- Found Threads: TRUE
-- Found Protobuf: /usr/lib/x86_64-linux-gnu/libprotobuf.so
-- Found ceres: /usr/lib/libceres.so
-- Found ZLIB: /usr/lib/x86_64-linux-gnu/libz.so (found version "1.2.8")
-- Found Luajit20: /usr/local/lib/libluajit-5.1.so;/usr/lib/x86_64-linux-gnu/libm.so (found version "2.0.4")
-- Found OpenCV: /usr/local (found suitable version "3.4.10", minimum required is "3")
-- Boost version: 1.58.0
-- Found the following Boost libraries:
--   serialization
--   system
--   filesystem
--   thread
--   program_options
--   date_time
--   timer
--   chrono
--   regex
--   atomic
-- GTSAM include directory:  /usr/share/gtsam/cmake/../../../include
-- Found jsoncpp: /usr/include/jsoncpp
-- Found USB: /usr/lib/x86_64-linux-gnu/libusb-1.0.so
CMake Error at /usr/share/cmake-3.5/Modules/FindPackageHandleStandardArgs.cmake:148 (message):
  Could NOT find JSONC (missing: JSONC_LIBRARY JSONC_INCLUDE_DIR)
Call Stack (most recent call first):
  /usr/share/cmake-3.5/Modules/FindPackageHandleStandardArgs.cmake:388 (_FPHSA_FAILURE_MESSAGE)
  cmake/FindJSONC.cmake:39 (find_package_handle_standard_args)
  CMakeLists.txt:256 (find_package)


-- Configuring incomplete, errors occurred!
See also "/home/jkell/astrobee_build/native/CMakeFiles/CMakeOutput.log".
```
It seems unable to find JSONC now. Ugh.

### JSONC
Tried `sudo apt-get install libjson0 libjson0-dev` This worked. 



### ARGTABLE2
New Error:

```
CMake Error at /usr/share/cmake-3.5/Modules/FindPackageHandleStandardArgs.cmake:148 (message):
  Could NOT find ARGTABLE2 (missing: ARGTABLE2_LIBRARY ARGTABLE2_INCLUDE_DIR)
Call Stack (most recent call first):
  /usr/share/cmake-3.5/Modules/FindPackageHandleStandardArgs.cmake:388 (_FPHSA_FAILURE_MESSAGE)
  cmake/FindARGTABLE2.cmake:39 (find_package_handle_standard_args)
  CMakeLists.txt:257 (find_package)
```

`sudo apt-get install libargtable2-dev` fixed this error. 

At this point, it seems that the issue is the dev packages for the dependencies not being installed correctly. I can see the ones I still need to install at ~/astrobee/cmake/. Trying to reinstall build-essential didn't fix it. 



### catkin.environment_cache
Error Message:
```
Traceback (most recent call last):
  File "/home/jkell/astrobee_build/native/catkin_generated/generate_cached_setup.py", line 19, in <module>
    from catkin.environment_cache import generate_environment_script
ImportError: No module named catkin.environment_cache
CMake Error at cmake/catkin/safe_execute_process.cmake:11 (message):
  execute_process(/usr/bin/python
  "/home/jkell/astrobee_build/native/catkin_generated/generate_cached_setup.py")
  returned error code 1
Call Stack (most recent call first):
  cmake/catkin/all.cmake:185 (safe_execute_process)
  cmake/catkin2Config.cmake:37 (include)
  CMakeLists.txt:311 (find_package)


-- Configuring incomplete, errors occurred!
See also "/home/jkell/astrobee_build/native/CMakeFiles/CMakeOutput.log".
```


`ImportError: No module named catkin.environment_cache` is the most important part of this output, so we'll start there.
Seems to be an issue with PYTHONPATH for catkin... I need to install pip and then 'catkin_pkg'
```
sudo apt-get install pip
pip install catkin_pkg
```

Moment of truth: `jkell@DESKTOP-CSDA0LG:~/astrobee$ ./scripts/configure.sh -l -F -D`
Output:
```
Traceback (most recent call last):
  File "/home/jkell/astrobee_build/native/catkin_generated/generate_cached_setup.py", line 19, in <module>
    from catkin.environment_cache import generate_environment_script
ImportError: No module named catkin.environment_cache
CMake Error at cmake/catkin/safe_execute_process.cmake:11 (message):
  execute_process(/usr/bin/python
  "/home/jkell/astrobee_build/native/catkin_generated/generate_cached_setup.py")
  returned error code 1
Call Stack (most recent call first):
  cmake/catkin/all.cmake:185 (safe_execute_process)
  cmake/catkin2Config.cmake:37 (include)
  CMakeLists.txt:311 (find_package)
```
This seems to be a common hair-pulling issue for installing ROS Melodic. 

# Installing catkin_tools to fix the above error

First you must have the ROS repositories which contain the .deb for catkin_tools:
```
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
$ wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
```
Once you have added that repository, run these commands to install catkin_tools:
```
$ sudo apt-get update
$ sudo apt-get install python-catkin-tools
```

In one instance, I had to simply restart with a fresh version of Ubuntu and retry the NASA installation. 


-----
