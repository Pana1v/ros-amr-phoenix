# Joystick to CmdVel Package

## Overview
This package converts joystick inputs into velocity commands for controlling the robot. It includes launch files and scripts for easy integration.

## Contents
- **launch/**: Launch files for starting the joystick node.
- **scripts/**: Python scripts for processing joystick inputs.
- **src/**: Source code for the package.

## How to Run
### Start the Joystick Node
```bash
roslaunch joystick_to_cmdvel joystick_control.launch
```

### Test the Node
Run the script directly:
```bash
rosrun joystick_to_cmdvel joystick_to_cmdvel.py
```