# franka_api.py

import time
import multiprocessing as mp
import numpy as np
import torch

import panda_py
from panda_py import libfranka
import transforms3d
from scipy.spatial.transform import Rotation as R

# Import your SpacemouseTeleop
from franka.utils.robot.spacemouse_teleop import SpacemouseTeleop


class FrankaAPI(mp.Process):
    def __init__(
        self,
        robot_ip=None,
        obs_fps=15,
        use_robot=True,
        verbose=False,
        speed=50,
        wrist=None,
        init_joints=False,
    ):
        super().__init__()
        self.device = torch.device(
            "cuda" if torch.cuda.is_available() else "cpu")

        self.obs_fps = obs_fps
        self.use_robot = use_robot
        self.init_joints = init_joints

        # Teleop reference
        self.teleop: SpacemouseTeleop | None = None

        # Panda + Gripper
        self.panda = panda_py.Panda(robot_ip)
        self.gripper = libfranka.Gripper(robot_ip)
        self.controller = panda_py.controllers.CartesianImpedance()

        # For demonstration: event to signal readiness
        self.ready_event = mp.Event()

    def run(self):
        """
        Main loop for mp.Process (if you are using multiprocessing).
        """
        try:
            self.init_robot()
            self.ready_event.set()
            self.panda.start_controller(self.controller)
            while True:
                time.sleep(0.001)  # ~1000Hz loop
        except Exception as e:
            print(e)

    def init_robot(self):
        """
        Initialize the robot and gripper.
        """
        joint_pose = [
            -0.01588696, -0.25534376, 0.18628714,
            -2.28398158, 0.0769999, 2.02505396, 0.07858208
        ]
        self.panda.move_to_joint_position(joint_pose)
        self.gripper.move(width=0.9, speed=0.1)

    def get_state(self):
        """
        Return a dictionary of relevant robot state.
        """
        EEF_pos = self.panda.get_position()
        EEF_ori = self.panda.get_orientation()
        return {
            "EEF_position": EEF_pos,
            "EEF_orientation": EEF_ori,
            "time": time.time()
        }

    def push_command(self):
        """
        Finalize or log the command. (Placeholder)
        """
        pass

    @property
    def is_connected(self) -> bool:
        return True
