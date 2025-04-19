# ROS AGV Arduino Package

## Overview
This package provides Arduino-based control for the AGV. It includes source code and configuration files for interfacing with the robot's hardware.

## Contents
- **include/**: Header files for the package.
- **src/**: Source code for Arduino integration.
- **test/**: Test files for validating Arduino functionality.
- **platformio.ini**: Configuration file for PlatformIO.

## How to Run
### Upload Code to Arduino
1. Connect the Arduino to your computer.
2. Use PlatformIO to upload the code:
   ```bash
   platformio run --target upload
   ```

### Start the ROS Node
```bash
rosrun ros_agv_arduino arduino_node
```