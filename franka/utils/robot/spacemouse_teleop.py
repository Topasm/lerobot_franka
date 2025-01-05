# spacemouse_teleop.py

import time
import numpy as np
from scipy.spatial.transform import Rotation as R

from franka.utils.inputs.spacemouse_shared_memory import Spacemouse

MOVE_INCREMENT = 0.010
ROTATION_SCALE = 0.03
SPEED = 0.1  # [m/s]
FORCE = 20.0  # [N]


class SpacemouseTeleop:
    """
    A simple Teleop class that reads from a Spacemouse and sends incremental
    move commands to a FrankaAPI instance.
    """

    def __init__(self, robot_instance=False, deadzone=0.3, shm_manager=None):
        self.robot_instance = robot_instance
        self.deadzone = deadzone
        self.is_running = False
        self.shm_manager = shm_manager

        self.Spacemouse_controller = Spacemouse(
            shm_manager=self.shm_manager,  # or pass a real SharedMemoryManager if needed
            deadzone=self.deadzone
        )

    def startup(self, robot=None):
        """
        Called once to 'start' the teleoperation session.
        """
        self.robot = robot
        self.Spacemouse_controller.start()
        self.is_running = True

    def shutdown(self):
        """
        If needed, gracefully stop the Spacemouse or cleanup resources.
        """
        self.is_running = False
        self.Spacemouse_controller.stop()

    def get_state(self) -> dict:
        """
        Returns the current Spacemouse motion state in a dictionary.
        Example keys: translation (XYZ), rotation (XYZ).
        """
        sm_state = self.Spacemouse_controller.get_motion_state_transformed()

        return {
            "translation": sm_state[:3]*MOVE_INCREMENT,
            "rotation": sm_state[3:]*ROTATION_SCALE,
            "gripper": (int(self.Spacemouse_controller.is_button_pressed(0))-int(self.Spacemouse_controller.is_button_pressed(1)))*MOVE_INCREMENT,
        }
