# spacemouse_teleop.py

import time
import numpy as np
from scipy.spatial.transform import Rotation as R

from franka.utils.inputs.spacemouse_shared_memory import Spacemouse

MOVE_INCREMENT = 0.005
ROTATION_SCALE = 0.015
SPEED = 0.1  # [m/s]
FORCE = 20.0  # [N]


class SpacemouseTeleop:
    """
    A simple Teleop class that reads from a Spacemouse and sends incremental
    move commands to a FrankaAPI instance.
    """

    def __init__(self, robot_instance=False, deadzone=0.3):
        self.robot_instance = robot_instance
        self.deadzone = deadzone
        self.is_running = False

        self.Spacemouse_controller = Spacemouse(
            shm_manager=None,  # or pass a real SharedMemoryManager if needed
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
            "translation": sm_state[:3],
            "rotation": sm_state[3:]
        }

    def do_motion(self, robot):
        """
        Reads the Spacemouse state and applies position/orientation increments
        to the robot's current target.
        """
        if not self.is_running:
            return

        sm_state = self.Spacemouse_controller.get_motion_state_transformed()
        dpos = sm_state[:3] * MOVE_INCREMENT
        drot_xyz = sm_state[3:] * ROTATION_SCALE

        # Example: ignoring rotation if not needed
        # drot_xyz[:] = 0

        current_pos = robot.panda.get_position()
        current_ori = robot.panda.get_orientation()  # quaternion [x, y, z, w]

        # Apply translation
        new_pos = current_pos + dpos

        # Apply rotation
        delta_rotation = R.from_euler('xyz', drot_xyz)
        new_ori = (delta_rotation * R.from_quat(current_ori)).as_quat()

        # Set the new pose via the robot's controller
        robot.controller.set_control(new_pos, new_ori)

        # Handle gripper state changes
        if self.Spacemouse_controller.is_button_pressed(0):
            success = robot.gripper.grasp(
                0.03,
                speed=SPEED,
                force=FORCE,
                epsilon_inner=0.05,
                epsilon_outer=0.05
            )
            print("Grasp successful" if success else "Grasp failed")

        elif self.Spacemouse_controller.is_button_pressed(1):
            success = robot.gripper.move(0.08, speed=SPEED)
            print("Release successful" if success else "Release failed")
