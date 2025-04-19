# ROS AGV Description Package

## Overview
This package contains the URDF and configuration files for describing the AGV's physical structure and properties.

## Contents
- **config/**: Configuration files for the robot's description.
- **description/**: URDF files for the AGV.
- **launch/**: Launch files for loading the robot description.

## How to Run
### Load the Robot Description
```bash
roslaunch ros_agv_description load_description.launch
```

### Visualize the Robot in RViz
```bash
roslaunch ros_agv_description view_description.launch
```