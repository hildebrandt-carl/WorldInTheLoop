
# Mixed Reality Testing

This is software designed to showcase a new type of testing called mixed reality testing. This testing involves adding parts of the simulated test environment into the real world.

# Installation

To install all the required software you need to do the following:
**Note:** this was tested on a clean install of Ubuntu 16.04.

## Installing ROS

Ubuntu 16.04 supports [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu). To install this you do the following:

Add the software packages:
```
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

Add ROS's keys:
```
$ sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

Update your debian package and install ROS:
```
$ sudo apt-get update
$ sudo apt-get install ros-kinetic-desktop-full
```

Update Rosdep
```
$ sudo rosdep init
$ rosdep update
```

Add the commands to your shell. I personally use ZSH, thus to add ROS to your shell you can run the following command:
```
$ echo "source /opt/ros/kinetic/setup.zsh" >> ~/.zshrc
$ source ~/.zshrc
```

## Installing Sphix

To install sphinx you need to do the following. First setup your computer to accept packages from Parrots public server:
```
$ echo "deb http://plf.parrot.com/sphinx/binary `lsb_release -cs`/" | sudo tee /etc/apt/sources.list.d/sphinx.list > /dev/null
$ sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 508B1AE5
```

Next install the packages using:
```
$ sudo apt update
$ sudo apt install parrot-sphinx
```

During the installation it will ask you which user groups you want to install the firmware to. To figure out what user you are you can run the command:
```
$ whoami
```

Test that sphinx was install correctly by starting the Firmware you installed using:
```
$ sudo systemctl start firmwared.service
$ fdc ping
>>> PONG
$ sphinx /opt/parrot-sphinx/usr/share/sphinx/drones/anafi4k.drone
```

## Installing Olympe

To install you need to have to `repo` tool installed. To do that run the command:
```
$ sudo apt-get install repo -y
```

Next we need to install the parrot-groundsdk using:
```
$ cd $HOME
$ mkdir -p code/parrot-groundsdk
$ cd ~/code/parrot-groundsdk
$ repo init -u https://github.com/Parrot-Developers/groundsdk-manifest.git
$ repo sync

```

Next you need to install all the dependencies which can be unstalled using:
```
$ cd ~/code/parrot-groundsdk
$ ./products/olympe/linux/env/postinst
```

Next we need to build olympe. You can do that by running:
```
$ cd ~/code/parrot-groundsdk
$ ./build.sh -p olympe-linux -A all final -j
```

## Installing OPENCV for Python3

You can install opencv on ubuntu 18.04 using:
```
$ sudo apt-get install python3-opencv
```

You can install opencv on ubuntu 16.04 using:
```
$ pip3 install opencv-python
```

For this project we will only be using Python3. However ROS imports a python2 version of CV2. We thus are going to remove this from ROS, so that we can use our own install of OpenCV in Python3. To do this run the following command:
```
rm /opt/ros/kinetic/lib/python2.7/dist-packages/cv2.so
```

## Getting ROS to work with Olympe

The main problem is that ROS runs in Python2. Olympe works in Python3. Additionally Olympe was designed to work in a virtual environment. Thus we need to make Olympe accessable to the main environment as well as setup ROS to work with Python3.

First we are going to need to install a bunch of Python packages so that Olympe can work in the main path. To do that you can run the following:
```
$ python3 -m pip install --upgrade pip
$ python3 -m pip install scipy rospkg colorlog future aenum boltons yapf tzlocal numpy websocket websocket-client pygame EmPy
```

Next we need to create the following script inside (`~/code/parrot-groundsdk`) our Olympe package. Call the file `olympe_custom_env.sh`. You can do this using:
```
$ cd ~/code/parrot-groundsdk
$ touch olympe_custom_env.sh
```

Next add the following code to the `olympe_custom_env.sh` file.
```bash
#!/bin/bash


[[ $0 != $BASH_SOURCE ]] && \
	SCRIPT_PATH=$(realpath $BASH_SOURCE) || \
	SCRIPT_PATH="`readlink -f "$0"`"
GSDK_DIR="`dirname "$SCRIPT_PATH"`"

# TODO: activate your python environment here. For example:
#     source ~/code/ros_project/env/bin/activate

# The following line installs Olympe Python dependencies into your Python
# virtual environement. It should only be necessary the first time and can
# be commented out after that.
pip install -r "$GSDK_DIR/packages/olympe/requirements.txt"
# Add Olympe and GSDK Python dependencies to your PYTHONPATH
export PYTHONPATH="${PYTHONPATH}:/$GSDK_DIR/out/olympe-linux/final/usr/lib/python/site-packages"

# Add Olympe GSDK C dependencies to LD_LIBRARY_PATH
source "$GSDK_DIR/out/olympe-linux/final/native-wrapper.sh"
```

Then we need to source it each time we want to run olympe, by running:
```
source ~/code/parrot-groundsdk/olympe_custom_env.sh
```

## Installing CV_Bridge

We need to install a bunch of dependencies for CV_Bridge. You can install then using:
```
$ sudo apt-get install python-catkin-tools python3-dev python3-catkin-pkg-modules python3-numpy python3-yaml ros-kinetic-cv-bridge python3-catkin-pkg-modules python3-rospkg-modules python3-empy
```

**Note:** For Ubunut 16.04 I had to make changes to the file `src/vision_opencv/cv_bridge/CMakeLists.txt`. I did the following:
```
...
if(NOT ANDROID)
  find_package(PythonLibs)
  if(PYTHONLIBS_VERSION_STRING VERSION_LESS 3)
    find_package(Boost REQUIRED python)
  else()
    # SWITCH THESE LINES FOR UBUNTU 18.04
    # find_package(Boost REQUIRED python3) 
    find_package(Boost REQUIRED python-py35)
  endif()
else()
...
```

# Executing

## Testing everything works

To start lets make sure we are able to access everything we need in Python. To do that run the following command:
```
$ python3
>>> import rospy
>>> import cv2
>>> import olympe
```

If you get no errors, the next thing to check is Sphinx. To test sphinx run the following commands:
```
$ sudo systemctl start firmwared.service
$ fdc ping
>>> PONG
$ cd ~/MixedRealityTesting/monocular_avoidance_ws/simulator_launch_scripts
$ ./simulation.sh
```

You will be presented with two drones at this point.

## Building the project

Finally we are ready to build the entire project. To do that lets config catkin build to use python3. We can do that using:
```
$ cd ~/MixedRealityTesting/monocular_avoidance_ws
$ catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.5m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.5m.so -DPYTHON_VERSION=3.5
$ catkin config --no-install
```

Make sure we have all the correct dependencies:
```
$ cd ~/MixedRealityTesting/monocular_avoidance_ws
$ rosdep install --from-paths src --ignore-src -r -y
```

We are now ready to build. We can do that using
```
$ catkin build -DCMAKE_BUILD_TYPE=Release
```

## Putting everything together

Finally lets test the avoidance demo. To do that you will need to open two terminals and run the following commands in each:

Terminal 1:
```
$ sudo systemctl start firmwared.service
$ cd ~/MixedRealityTesting/monocular_avoidance_ws/simulator_launch_scripts
$ ./simulation.sh
```

Terminal 2:
```
$ source ~/code/parrot-groundsdk/olympe_custom_env.sh
$ cd ~/Desktop/MixedRealityTesting/monocular_avoidance_ws
$ source devel/setup.zsh
```













# Ignore from this point















# Installing CUDA

**Note:** These instructions are tested on Ubuntu 18.04 and not Ubuntu 16.04.

You can install Cuda using the following commands:
Add NVIDIA package repositories
```
$ wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/cuda-repo-ubuntu1804_10.1.243-1_amd64.deb
$ sudo apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/7fa2af80.pub
$ sudo dpkg -i cuda-repo-ubuntu1804_10.1.243-1_amd64.deb
$ sudo apt-get update
$ wget http://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1804/x86_64/nvidia-machine-learning-repo-ubuntu1804_1.0.0-1_amd64.deb
$ sudo apt install ./nvidia-machine-learning-repo-ubuntu1804_1.0.0-1_amd64.deb
$ sudo apt-get update
```

Install NVIDIA driver:
```
$ sudo apt-get install --no-install-recommends nvidia-driver-430
```

Reboot. Check that GPUs are visible using the command:
```
nvidia-smi
```
Install development and runtime libraries (~4GB)
```
sudo apt-get install --no-install-recommends \
    cuda-10-1 \
    libcudnn7=7.6.4.38-1+cuda10.1  \
    libcudnn7-dev=7.6.4.38-1+cuda10.1
```

Install TensorRT. Requires that libcudnn7 is installed above.
```
sudo apt-get install -y --no-install-recommends libnvinfer6=6.0.1-1+cuda10.1 \
    libnvinfer-dev=6.0.1-1+cuda10.1 \
    libnvinfer-plugin6=6.0.1-1+cuda10.1

```











# Installing gazebo_plugins
```
To install gazebo plugins you have to do the following:
cd ~/catkin_ws/src
git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git -b melodic-devel
```

Next check for any missing dependencies
```
rosdep update
rosdep check --from-paths . --ignore-src --rosdistro melodic
```

Then build the project
```
catkin build
```




# installing requirements to create your own pluging for sphinx

You need to have libgazebo7-dev installed. To install that on Ubuntu18.04 you need to run the following commands:
```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
```

Setup the key for the packages site
```
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
```

Install gazebo
```
sudo apt-get update
sudo apt-get install gazebo7
sudo apt-get install libgazebo7-dev
```



# Creating a plugin





# Running the software

To run the software you just installed you can do the following:

## Runnning Sphix

You can launch two drones by running:
```
$ sudo systemctl start firmwared.service
$ sphinx /opt/parrot-sphinx/usr/share/sphinx/drones/anafi4k.drone::stolen_interface=::pose="2 0 0.2 0 0 0"::with_front_cam=true /opt/parrot-sphinx/usr/share/sphinx/drones/anafi4k.drone::name=other::stolen_interface=::pose="-2 0 0.2 0 0 0"::with_front_cam=false
```

We then have a bunch of different python scripts to control the drone using:
```
$ source ~/code/parrot-groundsdk/./products/olympe/linux/env/shell
$ python <PYTHONSCRIPT>.py
```

You might find that the ethernet does not reset correctly. To reset the ethernet you can run the following:
```
$ sudo ip link set eth0 down
$ sudo ip link set eth0 name enp0s5
$ sudo ip link set enp0s5 up
```

## Flying the second drone

Create a virtual ethernet connection using:
```
$ sudo ip link add eth10 type dummy
```

Now you can launch your drone using:
```
$ sudo systemctl start firmwared.service
$ sphinx /opt/parrot-sphinx/usr/share/sphinx/drones/anafi4k.drone::stolen_interface=enp0s5:eth0:192.168.42.1/24 /opt/parrot-sphinx/usr/share/sphinx/drones/anafi4k.drone::name=other::stolen_interface=eth10:eth0:192.168.42.1/24::pose="5 0 0.2 0 0 0"::with_front_cam=false
```