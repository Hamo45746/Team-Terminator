# METR4202 Team Project - Group 12
  
[![forthebadge](https://forthebadge.com/images/badges/powered-by-electricity.svg)](https://forthebadge.com)
[![forthebadge](https://forthebadge.com/images/badges/for-robots.svg)](https://forthebadge.com)
[![forthebadge](https://forthebadge.com/images/badges/built-with-love.svg)](https://forthebadge.com)

  

<h3 align="center">A robot controller built for the METR4202 Robotics & Automation 2022 Semester 2 Team Project 🤖</a></h3>

<p align="center">
  <a href="#Overview">Overview</a> •
  <a href="#download">Download</a> •
  <a href="#how-to-use">How To Use</a> •
  <a href="#credits">Credits</a> •
  <a href="#version-control">Version Control</a>
</p>

## Overview

A controller designed for the 4R robot shown. 

<p align="center">
  <img src="20221023_151510_1.gif" alt="animated" />
</p>

The robot has the task of finding, grabbing and shifting aruco tagged cubes using a camera, from a rotating plate to colour designated drop off points.  


## Download
To download the repo use the following:

```bash
git clone https://github.com/Hamo45746/Team-Terminator
```

## How To Use

To clone and run this project, you will need [Ubuntu MATE v20.04](https://ubuntu-mate.org/blog/ubuntu-mate-focal-fossa-release-notes/) and [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu).

To install and setup the project dependancies follow the directions in [this setup file](https://github.com/UQ-METR4202/METR4202_S2-2022_Resources/blob/main/RPi4_Setup.md) 

Once ROS is installed, in the Ubuntu terminal the controller can be launched by running the following commands (individually):

```bash
source /opt/ros/noetic/setup.bash # once per terminal
source ./devel/setup.bash
roslaunch team12 robot_arm.launch
```
Then, in the workspace run the terminal command:

```bash
catkin build
```

Once the first camera image window appears, press spacebar to make it colour, then the controller will run.

## Credits

Team members:
[Hamish Macintosh](https://github.com/Hamo45746)
[Samual Noffke](https://github.com/noff04)
[Rhys Mead](https://github.com/RhysM23)
[Sanjeet Bharaj](https://github.com/sanjeetsb)

Acknowledgements:

1. 

   * Author: [UQ METR4202 Staff](https://github.com/UQ-METR4202)
   * Title: Dynamixel Interface
   * Type: source code
   * [Web Address](https://github.com/UQ-METR4202/dynamixel_interface)

2.  

   * Author: [UQ METR4202 Staff](https://github.com/UQ-METR4202)
   * Title: METR4202 Ximea Camera Setup
   * Type: tutorial/source code
   * [Web Address](https://github.com/UQ-METR4202/metr4202_ximea_ros)

3. 

   * Author: [Miguel Valencia](https://github.com/miggyval)
   * Title: METR4202 Ximea Camera Setup
   * Type: tutorial/source code
   * [Web Address](https://github.com/miggyval/metr4202_ximea_tutorial)

4.  

   * Author: [WaveLab](https://github.com/wavelab)
   * Title: Ximea_ros_cam
   * Type: source code
   * [Web Address](https://github.com/wavelab/ximea_ros_cam.git)

5. 

   * Author: [UQ METR4202 Staff](https://github.com/UQ-METR4202)
   * Title: METR4202_S2-2022_Resources
   * Type: tutorial
   *[web address](https://github.com/UQ-METR4202/METR4202_S2-2022_Resources/blob/main/RPi4_Setup.md)

## Version Control

Though we later realised it is probably bad practice, different [Github](https://github.com/Hamo45746/Team-Terminator) branches were made and dated with the progress at that time, to demonstrate the development of the project.
