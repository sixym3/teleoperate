# Teleoperate Workspace

Multi-package Python workspace for LeRobot ROS 2 integration.

## Packages

- **lerobot_robot_ros** - ROS 2 wrapper for LeRobot robots (UR5e velocity control)
- **lerobot_teleoperator_devices** - Teleoperation input devices (gamepad, keyboard)

## Installation

```bash
pip install -e src/lerobot_robot_ros
pip install -e src/lerobot_teleoperator_devices
```

## Quick Start

### Velocity Control Example

```bash
python src/lerobot_robot_ros/examples/velocity_control_example.py
```

### SO-100 to UR5e Teleoperation

```bash
# Basic usage
python src/lerobot_robot_ros/examples/so100_ur5e_velocity_teleop.py

# With options
python src/lerobot_robot_ros/examples/so100_ur5e_velocity_teleop.py \
    --port /dev/ttyUSB0 --rate 100 --gains 2.0 2.0 2.5 2.5 0.0 2.5
```

See [Teleoperation Guide](src/lerobot_robot_ros/examples/TELEOP_GUIDE.md) for detailed instructions.

## Structure

```
teleoperate/
├── src/
│   ├── lerobot_robot_ros/          # ROS 2 robot interfaces
│   └── lerobot_teleoperator_devices/  # Input device handlers
└── references/                      # Reference implementations (not installed)
```

## Requirements

- Python 3.10+
- ROS 2 (Humble or later)
- LeRobot library
