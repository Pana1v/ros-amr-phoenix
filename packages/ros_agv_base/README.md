# ROS AGV Base Package

## Overview
This package provides the base functionalities for the AGV, including hardware configurations, controllers, and launch files.

## Contents
- **config/**: Configuration files for the robot's hardware and controllers.
- **controllers/**: Controller configurations for the AGV.
- **launch/**: Launch files for starting the base functionalities.
- **map/**: Map files for navigation.
- **rviz/**: RViz configuration files for visualization.
- **src/**: Source code for the package.

## How to Run
### Start the Base Node
```bash
roslaunch ros_agv_base base_bringup.launch
```

### Visualize in RViz
```bash
roslaunch ros_agv_base view_base.launch
```