#!/usr/bin/env python3

"""
SO-100 to UR5e Velocity Teleoperation Script

This script implements leader-follower teleoperation using:
- Leader: SO-100 arm (5-DOF + gripper) via LeRobot API
- Follower: UR5e robot (6-DOF + gripper) via ROS 2 velocity control
- Control: 100 Hz loop with per-joint PID gains (K matrix)

Joint Mapping:
    SO-100 (5 DOF)          -> UR5e (6 DOF)
    shoulder_pan            -> shoulder_pan_joint
    shoulder_lift           -> shoulder_lift_joint
    elbow_flex              -> elbow_joint
    wrist_flex              -> wrist_1_joint
    (none)                  -> wrist_2_joint (FIXED at current + π at launch)
    wrist_roll              -> wrist_3_joint
    gripper                 -> gripper

Control Law:
    velocity = K * (leader_position - follower_position)

Where K is a diagonal matrix of per-joint proportional gains.

Usage:
    python3 so100_ur5e_velocity_teleop.py [--port /dev/ttyUSB0] [--rate 100]

Requirements:
    - SO-100 leader arm connected via USB
    - UR5e running with ROS 2 driver
    - joint_group_vel_controller active on UR5e
"""

import argparse
import time
import numpy as np
import sys
from pathlib import Path

try:
    from lerobot.teleoperators.so100_leader import SO100Leader
    from lerobot.teleoperators.so100_leader.config_so100_leader import SO100LeaderConfig
    from lerobot.utils.robot_utils import busy_wait
except ImportError as e:
    print(f"Error importing lerobot: {e}")
    print("Please install lerobot: pip install lerobot")
    sys.exit(1)

try:
    from lerobot_robot_ros.robot import UR5eROSVelocity
    from lerobot_robot_ros.config import UR5eVelocityConfig
except ImportError as e:
    print(f"Error importing lerobot_robot_ros: {e}")
    print("Please install: pip install -e src/lerobot_robot_ros")
    sys.exit(1)


class SO100UR5eVelocityTeleop:
    """Velocity-based teleoperation from SO-100 leader to UR5e follower."""

    def __init__(
        self,
        so100_port: str = "/dev/ttyUSB0",
        control_rate: float = 100.0,
        k_gains: np.ndarray | None = None,
    ):
        """
        Initialize teleoperation system.

        Args:
            so100_port: USB port for SO-100 leader arm
            control_rate: Control loop frequency in Hz (default: 100)
            k_gains: Per-joint proportional gains [k1, k2, k3, k4, k5, k6]
                     If None, uses default gains
        """
        self.control_rate = control_rate
        self.dt = 1.0 / control_rate

        # SO-100 joint mapping (5 DOF)
        self.so100_joint_names = [
            "shoulder_pan",
            "shoulder_lift",
            "elbow_flex",
            "wrist_flex",
            "wrist_roll",
        ]

        # UR5e joint mapping (6 DOF)
        self.ur5e_joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

        # SO-100 normalization ranges (from LeRobot config)
        self.so100_min = -100.0
        self.so100_max = 100.0
        self.gripper_min = 0.0
        self.gripper_max = 100.0

        # UR5e joint limits (radians)
        self.ur5e_min = np.array([3.14, -3.49, -0.5, -4.37, 1.57, -3.14])
        self.ur5e_max = np.array([-3.14, 0.35, 2.8, -0.785, -4.71, 3.14])

        # Gripper mapping
        self.gripper_open = 0.0
        self.gripper_close = 0.8

        # wrist_2 fixed position (SO-100 has only 5 DOF)
        # Will be set to current position + 180° after connecting
        self.wrist_2_fixed = None

        # PID gains (K matrix) - Per-joint proportional gains
        if k_gains is None:
            # Default conservative gains
            # [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
            self.k_gains = np.array([2.0, 2.0, 2.5, 2.5, 0.0, 2.5])
        else:
            self.k_gains = np.array(k_gains)

        print("=" * 60)
        print("SO-100 to UR5e Velocity Teleoperation")
        print("=" * 60)

        # Initialize SO-100 leader
        print(f"\n[1/3] Connecting to SO-100 on {so100_port}...")
        leader_config = SO100LeaderConfig(port=so100_port)
        self.leader = SO100Leader(leader_config)
        self.leader.connect(calibrate=True)
        print("✓ SO-100 connected and calibrated")

        # Initialize UR5e follower
        print("\n[2/3] Connecting to UR5e via ROS 2...")
        follower_config = UR5eVelocityConfig()
        follower_config.ros2_interface.max_joint_velocities = [1.5, 1.5, 2.0, 2.0, 2.0, 2.0]
        follower_config.max_relative_target = None  # Disable safety clipping (we handle limits in calculate_velocities)
        self.follower = UR5eROSVelocity(follower_config)
        self.follower.connect()
        print("✓ UR5e connected")

        # Wait for initial joint states
        print("\n[3/3] Waiting for joint states...")
        time.sleep(1.0)

        # Get initial state and set wrist_2 fixed position
        try:
            obs = self.follower.get_observation()
            current_wrist_2 = obs["wrist_2_joint.pos"]
            # Set wrist_2 to be 180 degrees (π radians) from current position
            self.wrist_2_fixed = current_wrist_2 + np.pi
            print(f"✓ Joint states received")
            print(f"  Current wrist_2: {current_wrist_2:.3f} rad ({np.degrees(current_wrist_2):.1f}°)")
            print(f"  Fixed wrist_2 set to: {self.wrist_2_fixed:.3f} rad ({np.degrees(self.wrist_2_fixed):.1f}°)")
        except Exception as e:
            print(f"⚠ Warning: Could not get initial joint states: {e}")
            print("  Using default wrist_2 position: -π/2")
            self.wrist_2_fixed = -np.pi / 2

        print("\n" + "=" * 60)
        print("Teleoperation Configuration:")
        print("=" * 60)
        print(f"  Control Rate: {self.control_rate} Hz")
        print(f"  K Gains: {self.k_gains}")
        print(f"  wrist_2 Fixed: {self.wrist_2_fixed:.3f} rad ({np.degrees(self.wrist_2_fixed):.1f}°)")
        print("=" * 60)
        print("\nSAFETY REMINDERS:")
        print("  • Emergency stop must be within reach")
        print("  • Press Ctrl+C to stop safely")
        print("  • Workspace must be clear of obstacles")
        print("=" * 60)

        self.running = False

    def normalize_so100_to_ur5e(self, leader_action: dict) -> tuple[np.ndarray, float]:
        """
        Convert SO-100 joint positions to UR5e joint space.

        Args:
            leader_action: Dict from SO-100 with keys like 'shoulder_pan.pos'
                          Values in range [-100, 100] for arm, [0, 100] for gripper

        Returns:
            ur5e_positions: Array of 6 joint positions in radians
            gripper_position: Gripper position in range [0.0, 0.8]
        """
        # Extract SO-100 positions
        so100_positions = []
        for joint_name in self.so100_joint_names:
            key = f"{joint_name}.pos"
            if key in leader_action:
                so100_positions.append(leader_action[key])
            else:
                print(f"⚠ Warning: Missing SO-100 joint: {key}")
                so100_positions.append(0.0)
        so100_positions = np.array(so100_positions)

        # Normalize from [-100, 100] to [0, 1]
        normalized = (so100_positions - self.so100_min) / (self.so100_max - self.so100_min)

        # Map to UR5e joint space
        ur5e_positions = np.zeros(6)

        # Map first 4 SO-100 joints to first 4 UR5e joints
        for i in range(4):
            ur5e_positions[i] = (
                normalized[i] * (self.ur5e_max[i] - self.ur5e_min[i]) + self.ur5e_min[i]
            )

        # Keep wrist_2 fixed (SO-100 has only 5 DOF)
        ur5e_positions[4] = self.wrist_2_fixed

        # Map SO-100 wrist_roll (index 4) to UR5e wrist_3 (index 5)
        ur5e_positions[5] = (
            normalized[4] * (self.ur5e_max[5] - self.ur5e_min[5]) + self.ur5e_min[5]
        )

        # Convert gripper
        gripper_value = leader_action.get("gripper.pos", 0.0)
        gripper_normalized = (gripper_value - self.gripper_min) / (self.gripper_max - self.gripper_min)
        gripper_position = gripper_normalized * (self.gripper_close - self.gripper_open) + self.gripper_open

        return ur5e_positions, gripper_position

    def calculate_velocities(self, target_positions: np.ndarray, current_positions: np.ndarray) -> np.ndarray:
        """
        Calculate velocity commands using proportional control.

        Control law: v = K * (target - current)

        Args:
            target_positions: Desired positions (6 joints)
            current_positions: Current positions (6 joints)

        Returns:
            velocities: Velocity commands (6 joints) in rad/s
        """
        # Compute position errors
        errors = target_positions - current_positions

        # Apply proportional gains (K matrix)
        velocities = self.k_gains * errors

        # Clamp to safe maximum velocities (rad/s)
        max_vels = np.array([1.5, 1.5, 2.0, 2.0, 2.0, 2.0])
        velocities = np.clip(velocities, -max_vels, max_vels)

        return velocities

    def run(self):
        """Run the teleoperation control loop."""
        print("\n🚀 Starting teleoperation at {:.0f} Hz...".format(self.control_rate))
        print("   (Press Ctrl+C to stop)\n")

        self.running = True
        iteration = 0

        try:
            while self.running:
                t_start = time.perf_counter()

                # 1. Read leader positions (SO-100)
                leader_action = self.leader.get_action()

                # 2. Read follower positions (UR5e)
                follower_obs = self.follower.get_observation()
                current_positions = np.array([
                    follower_obs[f"{joint}.pos"]
                    for joint in self.ur5e_joint_names
                ])

                # 3. Map SO-100 → UR5e joint space
                target_positions, gripper_pos = self.normalize_so100_to_ur5e(leader_action)

                # 4. Calculate velocity commands
                velocities = self.calculate_velocities(target_positions, current_positions)

                # 5. Build action dict
                action = {
                    f"{joint}.vel": vel
                    for joint, vel in zip(self.ur5e_joint_names, velocities)
                }
                # Only add gripper if configured (optional)
                # action["gripper.pos"] = gripper_pos  # Uncomment if using gripper

                # 6. Send to UR5e
                self.follower.send_action(action)

                # 7. Maintain control rate
                elapsed = time.perf_counter() - t_start
                sleep_time = max(0.0, self.dt - elapsed)
                busy_wait(sleep_time)

                # Print status every 100 iterations (1 second at 100 Hz)
                iteration += 1
                if iteration % 100 == 0:
                    actual_rate = 1.0 / (time.perf_counter() - t_start + sleep_time)
                    print(f"  Rate: {actual_rate:.1f} Hz | Max vel: {np.max(np.abs(velocities)):.3f} rad/s")

        except KeyboardInterrupt:
            print("\n\n⏹ Stopping teleoperation...")
        finally:
            self.stop()

    def stop(self):
        """Stop teleoperation and disconnect devices."""
        self.running = False

        print("  Sending zero velocities...")
        zero_action = {f"{joint}.vel": 0.0 for joint in self.ur5e_joint_names}
        # zero_action["gripper.pos"] = 0.0  # Uncomment if using gripper

        # Send zeros for 10 cycles to ensure stop
        for _ in range(10):
            try:
                self.follower.send_action(zero_action)
                time.sleep(0.01)
            except Exception:
                pass

        print("  Disconnecting devices...")
        try:
            self.follower.disconnect()
        except Exception:
            pass

        try:
            self.leader.disconnect()
        except Exception:
            pass

        print("✓ Teleoperation stopped safely\n")


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description="SO-100 to UR5e velocity teleoperation")
    parser.add_argument(
        "--port",
        type=str,
        default="/dev/ttyUSB0",
        help="USB port for SO-100 leader arm (default: /dev/ttyUSB0)",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=100.0,
        help="Control loop rate in Hz (default: 100)",
    )
    parser.add_argument(
        "--gains",
        type=float,
        nargs=6,
        default=None,
        help="Per-joint K gains [k1 k2 k3 k4 k5 k6] (default: 2.0 2.0 2.5 2.5 0.0 2.5)",
    )

    args = parser.parse_args()

    # Create and run teleoperation system
    teleop = SO100UR5eVelocityTeleop(
        so100_port=args.port,
        control_rate=args.rate,
        k_gains=args.gains,
    )

    teleop.run()


if __name__ == "__main__":
    main()
