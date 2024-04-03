# TELEGRAM FLEXBE APP: Real-Time control my robot with Telegram

## Overview
This is a flexbe application that executes commands to the Kobuki robot via messages received from Telegram. The FlexBe application can perform four commands: Call, Charge, and Patrol. Follow this [link](https://t.me/kobuki_test) and give Kobuki a command.

### 1. FlexBE
Below is a state diagram of this application:

![Diagram](./images/flexbe.png)

- First, Kobuki gets ready to go on a mission.
- Second, check to see if there are any messages from Telegram.
- Third, it analyzes the commands in the message and instructs the Kobuki.
- Finally, when the mission is complete, it clears all files created during the mission and prepares to receive new commands.

Kobuki repeats the above process until the program ends. 

## Requirements
For building and running the application you need:
- [ros](http://wiki.ros.org/noetic/Installation/Ubuntu) (version: neotic)
- [python](https://www.python.org/downloads/release/python-3810/) (version: 3.8.10)

## Installation
```bash
# telegram-python-bot
$ pip install python-telegram-bot --upgrade

# numpy-python
$ pip install numpy

# yolo-python
$ pip install ultralytics

# ovencv-python
$ pip install opencv-python

# pcl-python
$ sudo apt-get update -y
$ sudo apt-get install libpcl-dev -y
$ sudo apt install python3-pcl
```

## Building
```bash
$ cd catkin_workspace/src
```
telegram_ros
```
$ git clone git@github.com:wntdev99/telegram_ros.git 
```
flexbe
```
$ git clone -b noetic https://github.com/FlexBE/flexbe_behavior_engine.git
$ git clone -b noetic https://github.com/FlexBE/flexbe_app.git
```
usb-cam
```
$ git clone -b develop https://github.com/ros-drivers/usb_cam.git
```
rplidar
```
$ git clone https://github.com/Slamtec/rplidar_ros.git
```
realsense2_camera
```
$ git clone -b development https://github.com/IntelRealSense/realsense-ros.git
```
Kobuki
```
$ git clone -b release/0.61-noetic https://github.com/stonier/ecl_tools.git
$ git clone -b release/0.62-noetic https://github.com/stonier/ecl_core.git
$ git clone -b release/0.61-noetic https://github.com/stonier/ecl_lite.git
$ sudo apt install ros-noetic-sophus # (https://github.com/stonier/ecl_core/issues/57)
$ git clone -b release/0.60-noetic https://github.com/stonier/ecl_navigation.git
$ git clone https://bitbucket.org/DataspeedInc/lusb.git # (https://index.ros.org/p/lusb/#noetic-overview)
$ sudo apt-get install libusb-dev
$ sudo apt-get install libftdi-dev
$ sudo apt-get install libsdl-image1.2-dev
$ git clone -b melodic-devel https://github.com/ros-perception/openslam_gmapping.git
$ git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
$ git clone -b main https://github.com/ros-drivers/joystick_drivers.git
$ git clone -b noetic-devel https://github.com/ros/geometry2.git
$ git clone https://github.com/ros-planning/navigation_msgs.git
$ git clone -b noetic-devel https://github.com/ros-perception/ar_track_alvar.git
$ sudo apt-get install libspnav-dev
```
Application
```
$ git clone https://github.com/wntdev99/telegram_flexbe_app.git
```
Build
```
$ cd ..;catkin_make
```
## How to run the application
Run FlexBE and import the flexbe_telegram_app Behavior. Next, press the Run button to run the app. 

```bash
# Execute Kobuki 
$ roslaunch kobuki_metapackage kobuki_navigation_flex.launch
$ roslaunch realsense2_camera rs_rgdb.launch
$ roslaunch usb_cam usb_cam-test.launch
```
```bash
# Telegram_Ros
$ roslaunch telegram_ros telegram_ros.launch
```
```bash
# FlexBE Application
$ roslaunch flexbe_app flexbe_full.launch
```

## How to use the application
This application extracts certain keywords from the commands. These keywords are `호출`, `배송`, `순찰`, and `충전` as described earlier. The `호출` and `배송` commands must be written with a destination, currently available destinations are 101, 102, 103, and 104. 

Example commands:
>꼬부기야 순찰해줘.

>102호로 배송해줘.

>꼬부기 104호로 호출해줘.

>충전시켜줘.

If two commands overlap, the prioritized command will be executed based on the order written above. Also, commands requested while Kobuki is on a mission can be rejected using telegram_node. Additionally, if the command is invalid, Kobuki will not perform the command and will wait for you to give it again. 


## Real environment
