# SO-100 to UR5e Velocity Teleoperation Guide

Complete guide for running leader-follower teleoperation between SO-100 arm and UR5e robot using velocity control.

## Setup

### 1. Start UR5e ROS 2 Driver

```bash
# Terminal 1: Launch UR driver
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5e \
    robot_ip:=<ROBOT_IP> \
    launch_rviz:=true \
    controller_spawner_timeout:=60 \
    initial_joint_controller:=forward_velocity_controller
```

### 2. Load Velocity Controller

```bash
# Terminal 2: Load velocity controller configuration
ros2 control load_controller --set-state active joint_group_vel_controller

# Verify controller is active
ros2 control list_controllers
```

## Running Teleoperation

### Basic Usage

```bash
# Navigate to examples directory
cd /home/sixym3/teleoperate

# Run with defaults (port: /dev/ttyUSB0, rate: 100 Hz)
python3 src/lerobot_robot_ros/examples/so100_ur5e_velocity_teleop.py
```

### Advanced Options

```bash
# Combine options
python3 src/lerobot_robot_ros/examples/so100_ur5e_velocity_teleop.py \
    --port /dev/ttyACM0 \
    --rate 100 \
    --gains 2.5 2.5 3.0 3.0 0.0 3.0
```

## Joint Mapping

### SO-100 (5-DOF) → UR5e (6-DOF)

| SO-100 Joint | Index | → | UR5e Joint | Index |
|--------------|-------|---|------------|-------|
| shoulder_pan | 0 | → | shoulder_pan_joint | 0 |
| shoulder_lift | 1 | → | shoulder_lift_joint | 1 |
| elbow_flex | 2 | → | elbow_joint | 2 |
| wrist_flex | 3 | → | wrist_1_joint | 3 |
| (none) | - | → | wrist_2_joint | 4 | **FIXED at -π/2** |
| wrist_roll | 4 | → | wrist_3_joint | 5 |
| gripper | - | → | gripper | - |

**Note**: wrist_2 is fixed at -90° (-π/2 radians) because SO-100 has only 5 DOF.

### Understanding Gains

```
velocity = K * error
```

Where `error = leader_position - follower_position`

### Default Gains

```python
K = [2.0, 2.0, 2.5, 2.5, 0.0, 2.5]
#    ^    ^    ^    ^    ^    ^
#    |    |    |    |    |    └─ wrist_3
#    |    |    |    |    └────── wrist_2 (fixed)
#    |    |    |    └─────────── wrist_1
#    |    |    └──────────────── elbow
#    |    └───────────────────── shoulder_lift
#    └────────────────────────── shoulder_pan
```


**Activate Velocity Controller**:
```bash
# Load and activate controller
ros2 control load_controller --set-state active joint_group_vel_controller

# Verify
ros2 control list_controllers
```

### Robot PID

**PID**: K gain for each joint
```bash
python3 src/lerobot_robot_ros/examples/so100_ur5e_velocity_teleop.py \
    --gains 1.5 1.5 2.0 2.0 0.0 2.0
```

### Control Rate 

Specify control rate
```bash
python3 src/lerobot_robot_ros/examples/so100_ur5e_velocity_teleop.py --rate 50
```

### Modifying Configuration File

Edit `/home/sixym3/teleoperate/src/lerobot_robot_ros/config/so100_ur5e_velocity_config.yaml`:

```yaml
control:
  rate_hz: 100.0
  joint_gains: [2.0, 2.0, 2.5, 2.5, 0.0, 2.5]
  velocity_limits: [1.5, 1.5, 2.0, 2.0, 2.0, 2.0]

safety:
  check_joint_limits: true
  limit_warning_threshold: 0.9
  watchdog_timeout: 1.0
```

### Monitoring Performance

Status is printed every 100 cycles (1 second at 100 Hz):

```
Rate: 99.8 Hz | Max vel: 0.342 rad/s
```

- **Rate**: Actual control loop frequency
- **Max vel**: Highest velocity command across all joints

## Additional Resources

- **UR5e Documentation**: https://www.universal-robots.com/
- **LeRobot Documentation**: https://huggingface.co/docs/lerobot
- **ROS 2 Control**: https://control.ros.org/
- **SO-100 Hardware**: https://github.com/TheRobotStudio/SO-ARM100