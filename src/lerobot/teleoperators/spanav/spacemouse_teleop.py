# spacemouse_teleop.py

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

import time
from multiprocessing.managers import SharedMemoryManager
from typing import Any

from franka.utils.inputs.spacemouse_shared_memory import Spacemouse

from ..teleoperator import Teleoperator
from .config_spacemouse import SpacemouseTeleopConfig


class SpacemouseTeleop(Teleoperator):
    """
    LeRobot-compatible SpaceMouse teleoperator for Franka robot control.

    This teleoperator reads from a SpaceMouse 3D controller and provides
    6-DOF pose commands plus gripper control for robot teleoperation.
    """

    config_class = SpacemouseTeleopConfig
    name = "spacemouse"

    def __init__(self, config: SpacemouseTeleopConfig):
        super().__init__(config)
        self.config = config
        self._is_connected = False
        self.shm_manager = None
        self.spacemouse_controller = None

    @property
    def action_features(self) -> dict:
        """Define the action space for spacemouse control."""
        if self.config.use_gripper:
            return {
                "dtype": "float32",
                "shape": (7,),  # x, y, z, rx, ry, rz, gripper
                "names": {
                    "delta_x": 0,
                    "delta_y": 1,
                    "delta_z": 2,
                    "delta_rx": 3,
                    "delta_ry": 4,
                    "delta_rz": 5,
                    "gripper": 6
                },
            }
        else:
            return {
                "dtype": "float32",
                "shape": (6,),  # x, y, z, rx, ry, rz
                "names": {
                    "delta_x": 0,
                    "delta_y": 1,
                    "delta_z": 2,
                    "delta_rx": 3,
                    "delta_ry": 4,
                    "delta_rz": 5
                },
            }

    @property
    def feedback_features(self) -> dict:
        """SpaceMouse doesn't support haptic feedback."""
        return {}

    @property
    def is_connected(self) -> bool:
        """Check if SpaceMouse is connected."""
        return self._is_connected

    def connect(self, calibrate: bool = True) -> None:
        """Establish connection with SpaceMouse."""
        try:
            # Initialize shared memory manager
            self.shm_manager = SharedMemoryManager()
            self.shm_manager.start()

            # Create SpaceMouse controller
            self.spacemouse_controller = Spacemouse(
                shm_manager=self.shm_manager,
                deadzone=self.config.deadzone
            )

            # Start the SpaceMouse process
            self.spacemouse_controller.start()

            # Give it a moment to initialize
            time.sleep(0.1)

            self._is_connected = True
            print("SpaceMouse connected successfully")

        except Exception as e:
            print(f"Failed to connect to SpaceMouse: {e}")
            self._is_connected = False
            raise

    def disconnect(self) -> None:
        """Disconnect from SpaceMouse and cleanup resources."""
        if self.spacemouse_controller is not None:
            try:
                self.spacemouse_controller.stop()
                self.spacemouse_controller.join(timeout=1.0)
            except Exception as e:
                print(f"Error stopping SpaceMouse controller: {e}")
            finally:
                self.spacemouse_controller = None

        if self.shm_manager is not None:
            try:
                self.shm_manager.shutdown()
            except Exception as e:
                print(f"Error shutting down shared memory manager: {e}")
            finally:
                self.shm_manager = None

        self._is_connected = False
        print("SpaceMouse disconnected")

    @property
    def is_calibrated(self) -> bool:
        """SpaceMouse doesn't require calibration."""
        return True

    def calibrate(self) -> None:
        """SpaceMouse doesn't require calibration."""
        pass

    def configure(self) -> None:
        """Configure SpaceMouse settings."""
        # No additional configuration needed
        pass

    def get_action(self) -> dict[str, Any]:
        """
        Get current action from SpaceMouse.

        Returns:
            dict: Action dictionary containing pose deltas and gripper command
        """
        if not self.is_connected or self.spacemouse_controller is None:
            raise RuntimeError(
                "SpaceMouse not connected. Call connect() first.")

        try:
            # Get motion state from SpaceMouse
            sm_state = self.spacemouse_controller.get_motion_state_transformed()

            # Extract translation and rotation
            translation = sm_state[:3] * self.config.move_increment
            rotation = sm_state[3:] * self.config.rotation_scale

            # Create action dictionary
            action_dict = {
                "delta_x": float(translation[0]),
                "delta_y": float(translation[1]),
                "delta_z": float(translation[2]),
                "delta_rx": float(rotation[0]),
                "delta_ry": float(rotation[1]),
                "delta_rz": float(rotation[2]),
            }

            # Add gripper control if enabled
            if self.config.use_gripper:
                # Button 0 = close, Button 1 = open
                button_0 = int(self.spacemouse_controller.is_button_pressed(0))
                button_1 = int(self.spacemouse_controller.is_button_pressed(1))
                gripper_delta = (button_0 - button_1) * \
                    self.config.move_increment
                action_dict["gripper"] = float(gripper_delta)

            return action_dict

        except Exception as e:
            print(f"Error getting SpaceMouse action: {e}")
            # Return zero action on error
            action_dict = {
                "delta_x": 0.0,
                "delta_y": 0.0,
                "delta_z": 0.0,
                "delta_rx": 0.0,
                "delta_ry": 0.0,
                "delta_rz": 0.0,
            }
            if self.config.use_gripper:
                action_dict["gripper"] = 0.0
            return action_dict

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        """SpaceMouse doesn't support haptic feedback."""
        pass
