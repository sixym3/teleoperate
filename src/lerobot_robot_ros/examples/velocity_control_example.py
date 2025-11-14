#!/usr/bin/env python3
"""
Example script demonstrating joint velocity control with LeRobot ROS.

This script shows how to:
1. Connect to a UR5e robot with velocity control
2. Send simple velocity commands
3. Read joint states
4. Implement basic filtering

Usage:
    python3 velocity_control_example.py
"""

import time
import numpy as np
from lerobot_robot_ros.robot import UR5eROSVelocity
from lerobot_robot_ros.config import UR5eVelocityConfig


def simple_velocity_control_example():
    """Basic example: Send velocity commands to the robot."""
    print("=== Simple Velocity Control Example ===\n")

    # Create configuration
    config = UR5eVelocityConfig()

    # Optional: Adjust velocity limits for safety
    config.ros2_interface.max_joint_velocities = [0.5, 0.5, 0.8, 0.8, 0.8, 0.8]

    # Create robot instance
    robot = UR5eROSVelocity(config)

    print("Connecting to robot...")
    robot.connect()
    time.sleep(1)  # Wait for connection to stabilize

    print("Robot connected!")
    print(f"Joint names: {config.ros2_interface.arm_joint_names}\n")

    # Get initial joint state
    obs = robot.get_observation()
    print("Initial joint positions (rad):")
    for joint in config.ros2_interface.arm_joint_names:
        print(f"  {joint}: {obs[f'{joint}.pos']:.3f}")
    print()

    # Example 1: Move first joint slowly
    print("Example 1: Moving shoulder_pan_joint slowly...")
    for i in range(50):  # Run for 1 second at 50Hz
        action = {
            "shoulder_pan_joint.vel": 0.3,  # Normalized velocity [-1, 1]
            "shoulder_lift_joint.vel": 0.0,
            "elbow_joint.vel": 0.0,
            "wrist_1_joint.vel": 0.0,
            "wrist_2_joint.vel": 0.0,
            "wrist_3_joint.vel": 0.0,
            "gripper.pos": 0.0,  # Gripper open
        }
        robot.send_action(action)
        time.sleep(0.02)  # 50Hz control loop

    # Example 2: Stop the robot
    print("Stopping robot...")
    zero_action = {
        f"{joint}.vel": 0.0 for joint in config.ros2_interface.arm_joint_names
    }
    zero_action["gripper.pos"] = 0.0

    for i in range(10):  # Send zeros for 0.2 seconds
        robot.send_action(zero_action)
        time.sleep(0.02)

    # Get final joint state
    obs = robot.get_observation()
    print("\nFinal joint positions (rad):")
    for joint in config.ros2_interface.arm_joint_names:
        print(f"  {joint}: {obs[f'{joint}.pos']:.3f}")

    print("\nDisconnecting...")
    robot.disconnect()
    print("Done!")


def filtered_velocity_control_example():
    """Advanced example: Velocity control with filtering."""
    print("\n=== Filtered Velocity Control Example ===\n")

    config = UR5eVelocityConfig()
    robot = UR5eROSVelocity(config)

    print("Connecting to robot...")
    robot.connect()
    time.sleep(1)

    # Simple velocity filter
    class VelocityFilter:
        def __init__(self, deadband=0.02, alpha=0.3):
            self.deadband = deadband
            self.alpha = alpha  # Low-pass filter coefficient
            self.prev_velocities = None

        def filter(self, velocities):
            # Apply deadband
            filtered = [v if abs(v) > self.deadband else 0.0 for v in velocities]

            # Apply low-pass filter
            if self.prev_velocities is not None:
                filtered = [
                    self.alpha * v + (1 - self.alpha) * pv
                    for v, pv in zip(filtered, self.prev_velocities)
                ]

            self.prev_velocities = filtered
            return filtered

    velocity_filter = VelocityFilter()

    print("Example 2: Filtered sinusoidal motion on shoulder_pan_joint...")
    duration = 5.0  # seconds
    frequency = 0.5  # Hz
    amplitude = 0.4  # normalized velocity

    start_time = time.time()
    while time.time() - start_time < duration:
        # Generate sinusoidal velocity command
        t = time.time() - start_time
        raw_velocity = amplitude * np.sin(2 * np.pi * frequency * t)

        # Create raw velocity array
        raw_velocities = [raw_velocity, 0.0, 0.0, 0.0, 0.0, 0.0]

        # Apply filtering
        filtered_velocities = velocity_filter.filter(raw_velocities)

        # Create action
        action = {
            joint + ".vel": vel
            for joint, vel in zip(
                config.ros2_interface.arm_joint_names, filtered_velocities
            )
        }
        action["gripper.pos"] = 0.0

        robot.send_action(action)
        time.sleep(0.02)  # 50Hz

    # Stop the robot
    print("Stopping robot...")
    zero_action = {
        f"{joint}.vel": 0.0 for joint in config.ros2_interface.arm_joint_names
    }
    zero_action["gripper.pos"] = 0.0

    for i in range(20):
        robot.send_action(zero_action)
        time.sleep(0.02)

    print("Disconnecting...")
    robot.disconnect()
    print("Done!")


def safe_velocity_control_with_limits():
    """Example with safety checks and velocity limiting."""
    print("\n=== Safe Velocity Control Example ===\n")

    config = UR5eVelocityConfig()

    # Set conservative velocity limits
    config.ros2_interface.max_joint_velocities = [0.3, 0.3, 0.5, 0.5, 0.5, 0.5]

    # Enable velocity clipping
    config.max_relative_target = 0.5  # Clip velocities to 50% of max

    robot = UR5eROSVelocity(config)

    print("Connecting to robot...")
    robot.connect()
    time.sleep(1)

    print("Example 3: Safe velocity control with clipping...")

    # Try to send large velocity (will be clipped)
    for i in range(100):  # 2 seconds at 50Hz
        action = {
            "shoulder_pan_joint.vel": 1.5,  # Will be clipped to 0.5
            "shoulder_lift_joint.vel": 0.0,
            "elbow_joint.vel": 0.0,
            "wrist_1_joint.vel": 0.0,
            "wrist_2_joint.vel": 0.0,
            "wrist_3_joint.vel": 0.0,
            "gripper.pos": 0.0,
        }

        # send_action will clip this to safe values
        actual_action = robot.send_action(action)

        if i == 0:
            print(f"Requested velocity: {action['shoulder_pan_joint.vel']}")
            print(f"Actual velocity: {actual_action['shoulder_pan_joint.vel']}")

        time.sleep(0.02)

    # Stop
    print("Stopping robot...")
    zero_action = {
        f"{joint}.vel": 0.0 for joint in config.ros2_interface.arm_joint_names
    }
    zero_action["gripper.pos"] = 0.0

    for i in range(10):
        robot.send_action(zero_action)
        time.sleep(0.02)

    robot.disconnect()
    print("Done!")


if __name__ == "__main__":
    print("UR5e Joint Velocity Control Examples")
    print("=" * 50)
    print("\nWARNING: These examples will move the robot!")
    print("Make sure:")
    print("  1. Robot is powered on and in remote control mode")
    print("  2. Emergency stop is within reach")
    print("  3. Workspace is clear of obstacles")
    print("  4. joint_group_vel_controller is active")
    print("\nPress Ctrl+C at any time to stop.")
    print("=" * 50)

    try:
        # Run examples
        simple_velocity_control_example()
        time.sleep(2)

        filtered_velocity_control_example()
        time.sleep(2)

        safe_velocity_control_with_limits()

    except KeyboardInterrupt:
        print("\n\nInterrupted by user. Stopping...")
    except Exception as e:
        print(f"\n\nError: {e}")
        import traceback

        traceback.print_exc()
    finally:
        print("\nExiting.")
