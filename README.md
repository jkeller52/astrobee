
# Astrobee Software Installation and Configuration
This repository documents installation and configuration of NASA's Astrobee software for use in ROS and Gazebo simulation. The Cognitive Systems Engineering Laboratory at Ohio State University is currently exploring the development of an experimental human-machine teaming testbed using ROS and Gazebo, prompting exploration into NASA's open source astrobee software.

If you are unfamiliar with astrobee, please refer to [NASA's official documentation](https://github.com/nasa/astrobee). This repository addresses and troubleshoots problems encountered when following NASA's official [installation documentation](https://nasa.github.io/astrobee/html/md_INSTALL.html), supporting the install of astrobee simulation capabilities on both native Ubuntu 16.04 and Ubuntu 16.04 ran via Windows Subsystem for Linux 2 (WSL2). It is our hope that other researchers

Contents
- [Operating System](https://github.com/jkeller52/astrobee/blob/master/README.md#operating-system)
- [Installation Instructions](https://github.com/jkeller52/astrobee/blob/master/README.md#operating-system)
- [Missing Dependencies](https://github.com/jkeller52/astrobee/blob/master/README.md#installing-dependencies)
- [Use Case](https://github.com/jkeller52/astrobee/blob/master/README.md#use-case)
- [Original NASA Documentation]()


## Operating System

### Ubuntu 16.04 (recommended)
A native Ubuntu 16.04 Operating System is ideal for this installation and recommended by NASA.



#### Windows Subsystem for Linux Configuration (Optional)
If your native machine is using Windows 10, you may want to configure Windows Subsystem for Linux (WSL). https://docs.microsoft.com/en-us/windows/wsl/install-win10
I used Windows Subsystem for Linux 2 (WSL2) to act as an Ubuntu 16.04 VM. This required installation of the [NVIDIA CUDA Graphics Driver](https://developer.nvidia.com/cuda/wsl/download). Your GPU may have different driver requirements.

In your Ubuntu terminal: run the following commands:
```
$wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/x86_64/cuda-ubuntu1604.pin
$sudo mv cuda-ubuntu1604.pin /etc/apt/preferences.d/cuda-repository-pin-600
$wget https://developer.download.nvidia.com/compute/cuda/11.3.1/local_installers/cuda-repo-ubuntu1604-11-3-local_11.3.1-465.19.01-1_amd64.deb
$sudo dpkg -i cuda-repo-ubuntu1604-11-3-local_11.3.1-465.19.01-1_amd64.deb
$sudo apt-key add /var/cuda-repo-ubuntu1604-11-3-local/7fa2af80.pub
$sudo apt-get update
$sudo apt-get -y install cuda
```
-----

# Installation Instructions

So far, I followed official nasa install docs until hitting an error.

An error message appeared after `./build_install_debians.sh` completes:
`/sbin/ldconfig.real: /usr/lib/wsl/lib/libcuda.so.1 is not a symbolic link`


Solution Steps:

First, we'll need to disable automount in /etc/wsl.conf. My WSL2 build did not have this file, but yours might in native Ubuntu 16.04.
  [automount]
  ldconfig = false

The, copy /usr/lib/wsl/lib to /usr/lib/wsl2/ (in wsl, writable)
`sudo cp -r /usr/lib/wsl/lib /usr/lib/wsl2/`

Next, we'll edit /etc/ld.so.conf.d/ld.wsl.conf and change change "/usr/lib/wsl/lib" --> "/usr/lib/wsl2/lib" (new location)
`sudo nano /etc/ld.so.conf.d/ld.wsl.conf`






Then run:
`sudo rm /usr/lib/wsl2/lib/libcuda.so.1` and `sudo ldconfig`


This should fix the issue. 
(note: works for CUDA in WSL, but "Segmentation fault" in DirectML)

[source](https://github.com/microsoft/WSL/issues/5548)


# Build Issues

Errors:
```
/usr/include/gazebo-7/gazebo/msgs/altimeter.pb.h:19:2: error: #error regenerate this file with a newer version of protoc.
 #error regenerate this file with a newer version of protoc.
 ```
 I verified that I had Protoc 3.17.2 (the newest version at time of writing) installed. I am suprised to see it says I need a newer version of protoc. Maybe I really need an older version since this is running Ubuntu 16.04? Will test with Protoc 3.16.0. 
 






# Dependency Issues

### Protobuf
To fix this we must install some prerequisites:
`sudo apt-get install autoconf automake libtool curl make g++ unzip`
Then: 
```
cd ~
wget https://github.com/protocolbuffers/protobuf/releases/download/v3.17.2/protobuf-all-3.17.2.zip
unzip protobuf-all-3.17.2.zip && cd protobuf-3.17.2
./configure
make
make check
sudo make install
sudo ldconfig # refresh shared library cache.
```
You can check that this worked by running the following command: `$ protoc --version`
`libprotoc 3.17.2`
[source](https://askubuntu.com/questions/1072683/how-can-i-install-protoc-on-ubuntu-16-04
### Protoc
Error: Protoc not found
[Installing Protoc on Ubuntu 16.04](https://askubuntu.com/questions/1072683/how-can-i-install-protoc-on-ubuntu-16-04)

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

still haven't gotten past this hurdle


























-----
# NASA Original Documentation Begins Here
-----
## Astrobee Robot Software

### About

<p>
<img src="doc/images/astrobee.png" srcset="../images/astrobee.png 1x" 
  title="Astrobee" align="right" style="display: inline"/>
Astrobee is a free-flying robot designed to operate as a payload inside
the International Space Station (ISS). The Astrobee Robot Software consists of
embedded (on-board) software, supporting tools and a simulator. The Astrobee
Robot Software operates on Astrobee's three internal single board computers and
uses the open-source Robot Operating System (ROS) framework as message-passing
middleware. The Astrobee Robot Software performs vision-based localization,
provides autonomous navigation, docking and perching, manages various sensors
and actuators, and supports user interaction via screen-based displays, light
signaling, and sound. The Astrobee Robot Software enables Astrobee to be
operated in multiple modes: plan-based task execution (command sequencing),
teleoperation, or autonomously through execution of hosted code uploaded by
project partners (guest science). The software simulator enables Astrobee Robot
Software to be evaluated without the need for robot hardware.
</p>

This repository provides flight software and a simulator, both primarily written
in C++. The repository also provides several other utilities, including a tool
for creating maps for localization. A separate repository,
[`astrobee_android`](https://github.com/nasa/astrobee_android), contains the
Java API, which uses the ROS messaging system to communicate with flight
software.

The Astrobee Robot Software is in a beta stage. This means that some
features are incomplete, and extensive changes can be expected. Please consult the
[release](https://nasa.github.io/astrobee/html/md_RELEASE.html) for the current list of features and limitations.

### Usage instructions

To install and use astrobee, please see the
[usage instructions](https://nasa.github.io/astrobee/html/md_INSTALL.html).

### Contributors

The Astrobee Robot Software is open source, and we welcome contributions
from the public. Please submit pull requests to the develop branch.
For us to merge any pull requests, we must request that contributors sign and submit a
[Contributor License Agreement](https://www.nasa.gov/sites/default/files/atoms/files/astrobee_individual_contributor_license_agreement.pdf)
due to NASA legal requirements. Thank you for your understanding.

### Documentation

To view all the Astrobee documentation, please visit [documentation](https://nasa.github.io/astrobee/documentation.html).

If you want to perform research using the astrobee platform, a good tutorial guide is ["A Brief Guide to Astrobee’s Flight Software"](https://github.com/albee/a-brief-guide-to-astrobee/raw/master/a_brief_guide_to_astrobee_v1.0.pdf). This will teach you what Astrobee is, how the robot works, how to make your own package, and much more!

For more information, read the Astrobee [publications](https://www.nasa.gov/content/research-publications-0).
Learning about the Astrobee [platform](https://www.nasa.gov/sites/default/files/atoms/files/bualat_spaceops_2018_paper.pdf),
[software](https://www.nasa.gov/sites/default/files/atoms/files/fluckiger2018astrobee.pdf),
and [localization](https://www.nasa.gov/sites/default/files/atoms/files/coltin2016localization.pdf)
are good starting points.

### Guest Science

If you are interested in guest science, please checkout the astrobee_android nasa github
project (if you followed the usage instructions, you should have checked this
out already). Once that is checked out, please see
[`astrobee_android/README.md`](https://github.com/nasa/astrobee_android/blob/master/README.md)
located in the `astrobee_android/` folder.

### License

Copyright (c) 2017, United States Government, as represented by the
Administrator of the National Aeronautics and Space Administration.
All rights reserved.

The Astrobee platform is licensed under the Apache License, Version 2.0 (the
"License"); you may not use this file except in compliance with the License. You
may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0.

Unless required by applicable law or agreed to in writing, software distributed
under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
CONDITIONS OF ANY KIND, either express or implied. See the License for the
specific language governing permissions and limitations under the License.
