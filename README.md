
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
$ python3 -m pip install rospkg colorlog future aenum boltons yapf tzlocal numpy websocket websocket-client pygame EmPy
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

At this point you now should be able to use ROS, OpenCV and Olympe all in python3. Run the following test:
```
$ python3
>>> import rospy
>>> import cv2
>>> import olympe
```











# Installing OPENCV

To install open CV you can run:
```
$ sudo apt update
$ sudo apt install python-opencv
```

# Installing SCIPY

To install scipt run:
```
$ sudo apt-get install python-pip
$ python -m pip install scipy
$ 
```

# gt-cp-2017-project

To run this you can use the following:
```
$ python main.py --debug True --skip 1 --video ../../videos/CameraFeed.mp4
```

# YOLO for ROS

Installation
```
$ mkdir -p catkin_workspace/src
$ cd catkin_workspace/src
$ git clone --recursive git@github.com:leggedrobotics/darknet_ros.git
$ cd ../
$ catkin build darknet_ros -DCMAKE_BUILD_TYPE=Release
$ catkin build darknet_ros --no-deps --verbose --catkin-make-args run_tests
$ catkin build
```

To run it use the following:
```
$ source ./devel/setup.zsh
$ roslaunch controller main.launch
```


# Making CV_Bridge work with Python3

You have to do the following. Note you need to change the catkin config command to include the python version you are using. so for example I am using python3.6.9 so I ran the command python3.6m
You also need to change the ros version to melodic in the grep command

```
# `python-catkin-tools` is needed for catkin tool
# `python3-dev` and `python3-catkin-pkg-modules` is needed to build cv_bridge
# `python3-numpy` and `python3-yaml` is cv_bridge dependencies
# `ros-kinetic-cv-bridge` is needed to install a lot of cv_bridge deps. Probaply you already have it installed.
sudo apt-get install python-catkin-tools python3-dev python3-catkin-pkg-modules python3-numpy python3-yaml ros-kinetic-cv-bridge
# Create catkin workspace
mkdir catkin_workspace
cd catkin_workspace
catkin init
# Instruct catkin to set cmake variables
catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.5m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.5m.so
# Instruct catkin to install built packages into install place. It is $CATKIN_WORKSPACE/install folder
catkin config --install
# Clone cv_bridge src
git clone https://github.com/ros-perception/vision_opencv.git src/vision_opencv
# Find version of cv_bridge in your repository
apt-cache show ros-kinetic-cv-bridge | grep Version
    Version: 1.12.8-0xenial-20180416-143935-0800
# Checkout right version in git repo. In our case it is 1.12.8
cd src/vision_opencv/
git checkout 1.12.8
cd ../../
# Build
catkin build cv_bridge
# Extend environment with new package
source install/setup.bash --extend
```

I also needed to use catkin config --no-install



# Installing CUDA

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


# Installing TF2 packages for Python3

Start by installing the dependencies
```
sudo apt update
sudo apt install python3-catkin-pkg-modules python3-rospkg-modules python3-empy
```

download the source code
```
mkdir -p ~/catkin_ws/src; cd ~/catkin_ws
catkin_make
source devel/setup.bash
wstool init
wstool set -y src/geometry2 --git https://github.com/ros/geometry2 -v 0.6.5
wstool up
rosdep install --from-paths src --ignore-src -y -r
```

Build the file
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