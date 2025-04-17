# nauti_teleop

A ROS package for teleoperation of an Autonomous Underwater Vehicle (AUV) in ROV mode using a joystick controller. This package provides a complete solution for controlling the AUV through joystick inputs, converting them to command velocity messages, and ultimately translating these into MAVLink messages for Pixhawk interpretation.

## Control Mapping

The joystick controls are mapped as follows:

- **Left Joystick**:
  - Up/Down: Surge (forward/backward motion)
  - Left/Right: Sway (lateral motion)

- **Right Joystick**:
  - Left/Right: Yaw (rotational motion)

- **Trigger Buttons**:
  - Left/Right Triggers: Heave (vertical motion)


## Overview

This package serves as an alternative to QGroundControl (QGC) for ROV control, providing direct joystick-based teleoperation capabilities. It includes:
- Joystick input interpretation
- Command velocity message conversion
- MAVLink message generation for Pixhawk communication

## Dependencies

- ROS (Robot Operating System)
- joy ROS package
- python3
- MAVLink
- Additional ROS dependencies (specified in package.xml)

## Installation

1. Create a ROS workspace (if not already created):
```bash
mkdir -p ~/rov_ws/src
cd ~/rov_ws/src
```

2. Clone this package into your workspace's src directory

3. Build the workspace:
```bash
cd ~/rov_ws
catkin build
```

4. Source the workspace:
```bash
source devel/setup.bash
```

## Usage

To run the teleoperation system:

1. Start the joy node:
```bash
rosrun joy joy_node
```

2. Launch the Xbox controller interpreter:
```bash
python3 src/xbox_controller.py
```

3. Start the Pixhawk interface:
```bash
python3 src/px4_interface.py
```

## Support
For issues and questions, please open an issue in this repository.

