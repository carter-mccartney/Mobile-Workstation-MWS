# MWS-Repo
This is for the Mobile Workstation (MWS) used for Senor Design II in the Fall of 2023.

## Overview
This project is to create a moving desk that can follow its user around. It runs off of a Raspberry Pi 4 using Ubuntu and [ROS2 Iron](https://docs.ros.org/en/iron/index.html) and communicates with its user using a smartphone app. It locates the user's position using a technique known as trilateration to use the Bluetooth Received Signal Strength Indicator (RSSI) measured from the three nearest of four ESP32 microcontrollers on the device from the advertisement packet sent from the user's smarthphone. It's motors are controlled with an Arduino Mega board that controls a [Vedder Electronic Speed Controller, or (VESC)](https://vesc-project.com/) board. It is set up as a differential drive robot. It uses LiDAR to see obstacles to avoid.

## Project Structure
The project is broken into several modules:
### Bluetooth Communication Module
This module contains the applications necessary to control the MWS via a smartphone app. 

Inside is an app made with .NET Maui for deployment on smarthphones to initiate communications and provide input for the distance-finding module, which is currently functional for Android.

Additionally, there is a Bluetooth Low Energy (BLE) Generic Attribute (GATT) server implementation based off of the repo [gobbledegook by nettlep](https://github.com/nettlep/gobbledegook) and extended for the needs of this project.
This takes the form of a library (gobbledegook) and a standalone application (MwsGatt).

### Distance-Finding Module
This module contains the ROS node for combining the RSSI values received at the three nearest ESP32 microcontrollers in order to approximate the position of the user and provide a goal to the navigation system when commanded to do so by the user.

There is also the microcontroller source code for the ESP32 controllers themselves for reading BLE advertisements and filtering them.

In addition, several debugging tools and prototype programs are provided for testing this functionality early on.

### Localization Module
This module contains a ROS node for an mpu9250 IMU to be used with the robot_localization package. Ideally, it would be used in the final product, but time constraints led to this being scrapped since it was not entirely necessary as the MWS has odometry and the postion of the user to help it localize to the extent it needs.

### Movement Module
This module contains the differential drive hardware interface for ROS that conforms to the protocol defined in the Arduino for sending motor velocities and receiving tachometer and velocity values.

This also contains the code for the Arduino, which takes advantage of the library from the repo [VESCUART by SolidGeek](https://github.com/SolidGeek/VescUart).

### Navigation Module
This module contains the relevant programs and packages for navigation.

There is a set of programs used for testing navigation functionality on a ROS-enabled desktop based off of the [nav2_bringup package](https://github.com/ros-planning/navigation2/blob/main/nav2_bringup/README.md).

The MwsNavigationController program is used to receive requests from the GATT server and activate or cancel navigation from the [Nav2](https://navigation.ros.org/) stack's action server interface.

The package mws_behavior_tree_plugins contains a behavior tree node for setting custom truncate ranges and an adapted truncate path node pulled from the [nav2 repo](https://github.com/ros-planning/navigation2/tree/main).

The package mws_nav_plugins contains plugins for the Nav2 controller server to override the default goal checker and progress checker behavior to account for the unreachable goal of the MWS.

The BehaviorTrees folder contains the behavior tree used to follow a point that is passed to the navigation stack.

The rplidar_ros package is an adaptation of the [rplidar_ros repo](https://github.com/Slamtec/rplidar_ros) for pulling data from the rplidar A1 used for collision detection with new default values.

### ROS Makefile
This contains a makefile that can be used for simple programs that have only one ROS dependency for those who prefer make over CMake.

### Robot Bringup Module
This contains the package with the various launch files and parameters used for starting up the robot and its ROS nodes.

### Startup
This contains several shell scripts for starting up the various programs on the MWS that are not defined in a launch file, as well as starting the launch files.

## Deployment
Any program that is not part of a traditional ROS package should be placed at ~/Startup/Programs/

ROS packages are expected at ~/src/

The behavior tree is expected to be located at ~/

The name of the robot to display in the app is located at ~/.MwsName

The motor-system-code arduino file should be deployed on an Arduino Mega board attached to the Raspberry Pi 4 through the programming serial port.

The Arduino connects to the VESC through the TX1 RX1 and TX2 RX2 lines.

The ESP32s should all be loaded with firmware from the ESPBluetooth arduino file with their given name. They are placed in a square on top of the desk 0.30 meters from center along one axis and 0.29 meters along the other. The one named One is located at the front on the left from the persepctive of the user. Two is also on the front in the other corner. Three is in the other adjacent corner to One. Four is in the remaining corner.

The script startup.sh should be registered with the startup programs of the user who is automatically logged in.

The automatically logged in user must be a member of the dialout and bluetooth groups.

PIGPIO and ROS Iron must be installed.

The gobbledegook library must be located in /opt/ and built with ./configure && make from within.

All programs referenced in the startup scripts must be present.
