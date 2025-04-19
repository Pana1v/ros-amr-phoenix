# Packages Directory

## Overview
This directory contains various ROS packages developed for the ROS-AMR project. Each package serves a specific purpose, such as controlling the robot, simulating its behavior, or integrating sensors.

## Subdirectories
- **fuji_mecanum/**: Contains files for controlling a mecanum-wheeled robot.
- **joystick_to_cmdvel/**: Converts joystick inputs to velocity commands.
- **ros_agv_arduino/**: Arduino-based control for the AGV.
- **ros_agv_base/**: Base functionalities for the AGV.
- **ros_agv_bringup/**: Launch files and configurations for launching everything.
- **navigation/**: Contains launch files and configurations for autonomous navigation.
- **ros_agv_description/**: URDF and description files for the AGV.

## Usage
Refer to the README files within each subdirectory for detailed usage instructions and examples.

## Launch Files
- **amcl.launch**: Launch file for localization using AMCL.
- **bringup.launch**: Launches the entire system.
- **lidar.launch**: Sets up the LIDAR sensor.
- **mapping.launch**: Launch file for making a map. Initially, AMCL was tried but later replaced with gmapping.
- **navigation.launch**: Launch file for autonomous navigation.
- **teleop.launch**: Launch file for manual teleoperation.

## PS4 Controller Setup
To use a PS4 controller:
1. Ensure Blueman is installed and configured:
   sudo apt-get install blueman
2. Pair your controller:
   - Open Blueman.
   - Hold the PS and Share buttons on the controller until the light blinks.
   - Select the controller in Blueman and pair it.
3. Launch PS4 teleoperation:
   - Use the teleop.launch file by running:
     roslaunch ros_agv_base teleop.launch
4. Verify the connection with jstest or the appropriate ROS joystick package.