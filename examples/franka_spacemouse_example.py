#!/usr/bin/env python

# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Example script showing how to use SpaceMouse with Franka robot.

This example demonstrates the integration between the SpaceMouse teleoperator
and the Franka robot using the LeRobot framework.

Prerequisites:
- Franka robot connected and accessible at the specified IP
- SpaceMouse device connected to the computer
- spnav library installed for SpaceMouse support
- panda_py library installed for Franka control

Usage:
    python franka_spacemouse_example.py --robot_ip 172.16.0.2
"""

import argparse
import time
from typing import Optional

from lerobot.teleoperators import make_teleoperator_from_config
from lerobot.teleoperators.spanav.config_spacemouse import SpacemouseTeleopConfig


def franka_spacemouse_teleoperation(robot_ip: str = "172.16.0.2", duration: Optional[float] = None):
    """
    Run SpaceMouse teleoperation with Franka robot.

    Args:
        robot_ip: IP address of the Franka robot
        duration: Duration to run teleoperation (None for indefinite)
    """

    print("Franka SpaceMouse Teleoperation")
    print("=" * 40)
    print(f"Robot IP: {robot_ip}")
    print(
        f"Duration: {duration}s" if duration else "Duration: Indefinite (Ctrl+C to stop)")
    print()

    teleop = None

    try:
        # Create SpaceMouse teleoperator configuration
        teleop_config = SpacemouseTeleopConfig(
            id="franka_spacemouse",
            deadzone=0.3,
            move_increment=0.01,
            rotation_scale=0.03,
            use_gripper=True,
            robot_ip=robot_ip
        )

        print("Creating SpaceMouse teleoperator...")
        teleop = make_teleoperator_from_config(teleop_config)

        print("Connecting to SpaceMouse...")
        teleop.connect()

        if not teleop.is_connected:
            raise RuntimeError("Failed to connect to SpaceMouse")

        print("✓ SpaceMouse connected successfully!")
        print(f"Action features: {teleop.action_features}")
        print()

        # Note: In a real implementation, you would also create and connect to the robot here
        # For example:
        # from lerobot.robots import make_robot_from_config
        # robot_config = FrankaRobotConfig(robot_ip=robot_ip)
        # robot = make_robot_from_config(robot_config)
        # robot.connect()

        print("Starting teleoperation...")
        print("SpaceMouse controls:")
        print("- Translation: Move the puck in X/Y/Z")
        print("- Rotation: Twist the puck around X/Y/Z axes")
        print("- Gripper: Button 0 (close) / Button 1 (open)")
        print("- Press Ctrl+C to stop")
        print()

        start_time = time.time()
        iteration = 0

        while True:
            loop_start = time.time()

            # Get action from SpaceMouse
            action = teleop.get_action()

            # Display action (in a real implementation, you would send this to the robot)
            if iteration % 10 == 0:  # Print every 10th iteration to avoid spam
                print(f"Iteration {iteration+1}:")
                print(
                    f"  Translation: [{action['delta_x']:.3f}, {action['delta_y']:.3f}, {action['delta_z']:.3f}]")
                print(
                    f"  Rotation: [{action['delta_rx']:.3f}, {action['delta_ry']:.3f}, {action['delta_rz']:.3f}]")
                if 'gripper' in action:
                    print(f"  Gripper: {action['gripper']:.3f}")
                print()

            # In a real implementation, you would send the action to the robot:
            # robot.send_action(action)

            iteration += 1

            # Check duration
            if duration and (time.time() - start_time) >= duration:
                print(f"Reached duration limit of {duration}s")
                break

            # Maintain loop rate (30 Hz)
            elapsed = time.time() - loop_start
            sleep_time = max(0, 1/30 - elapsed)
            time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\nStopped by user")

    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

    finally:
        # Cleanup
        if teleop is not None:
            try:
                teleop.disconnect()
                print("✓ SpaceMouse disconnected")
            except Exception as e:
                print(f"Warning: Error during teleop cleanup: {e}")

        # In a real implementation, you would also disconnect from the robot:
        # if robot is not None:
        #     robot.disconnect()
        #     print("✓ Robot disconnected")


def main():
    """Main function with command line argument parsing."""
    parser = argparse.ArgumentParser(
        description="Franka SpaceMouse Teleoperation Example")
    parser.add_argument("--robot_ip", type=str, default="172.16.0.2",
                        help="IP address of the Franka robot")
    parser.add_argument("--duration", type=float, default=None,
                        help="Duration to run teleoperation in seconds (default: indefinite)")

    args = parser.parse_args()

    franka_spacemouse_teleoperation(
        robot_ip=args.robot_ip,
        duration=args.duration
    )


if __name__ == "__main__":
    main()
