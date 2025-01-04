import panda_py
from panda_py import libfranka

import time
import threading
import panda_py.controllers
import transforms3d
import roboticstoolbox as rtb
from scipy.spatial.transform import Rotation as R
import multiprocessing as mp
from multiprocessing import Process, Queue, Event
from multiprocessing.managers import SharedMemoryManager
import numpy as np

from typing import Optional
import math
import pathlib
import shutil
from utils.camera.timestamp_accumulator import (
    TimestampObsAccumulator,
    TimestampActionAccumulator,
    align_timestamps,
)
from utils.replay_buffer import ReplayBuffer
from utils.cv2_util import (
    get_image_transform,
    optimal_row_cols,
)
from utils.multi_camera_visualizer import MultiCameraVisualizer
import torch
import os
from utils.inputs.spacemouse_shared_memory import Spacemouse
import pickle
from utils.shared_memory.shared_memory_ring_buffer import (
    SharedMemoryRingBuffer,
)

SPEED = 0.1  # [m/s]
FORCE = 20.0  # [N]
MOVE_INCREMENT = 0.005


class FrankaAPI(mp.Process):
    def __init__(
        self,
        robot_ip=None,
        task_config=None,
        capture_fps=15,
        obs_fps=15,
        n_obs_steps=1,
        enable_color=True,
        enable_depth=True,
        process_depth=False,
        use_robot=True,
        verbose=False,
        gripper_enable=True,
        speed=50,
        wrist=None,
        init_joints=False,
    ):
        super().__init__()
        self.device = torch.device(
            "cuda" if torch.cuda.is_available() else "cpu")

        self.capture_fps = capture_fps
        self.obs_fps = obs_fps
        self.n_obs_steps = n_obs_steps

        # Panda + Gripper
        self.panda = panda_py.Panda(robot_ip)
        self.gripper = libfranka.Gripper(robot_ip)
        self.panda.enable_logging(int(10))

        Robot_state = {
            # [x,y,z, qx,qy,qz,qw, gripper]
            'Robot_state': np.zeros(29, dtype=np.float32)
        }

        # Shared Memory Manager
        self.shm_manager = SharedMemoryManager()
        self.shm_manager.start()

        self.Robot_state_buffer = SharedMemoryRingBuffer.create_from_examples(
            shm_manager=self.shm_manager,
            examples=Robot_state,
            get_max_k=30,  # Store last 30 states
            get_time_budget=0.2,
            put_desired_frequency=self.obs_fps
        )

        # SpaceMouse
        self.space_mouse = Spacemouse(
            shm_manager=self.shm_manager, deadzone=0.3)
        self.space_mouse.start()

        self.ready_event = mp.Event()

        # Flags
        self.enable_color = enable_color
        self.enable_depth = enable_depth
        self.use_robot = use_robot

        self.init_joints = init_joints

        # Episode recording
        self.current_episode = []
        self.episode_counter = 0  # To generate unique filenames
        self.is_recording = False

    # ======== start-stop API =============

    def put_state(self, state_data):
        """Put new state data into ring buffer"""
        self.Robot_state_buffer.put(state_data)

    def get_state(self):
        """
        Placeholder for returning the current robot state.
        You can customize this as needed to fill `EEF_state`.
        """
        EEF_pos = self.panda.get_position()
        EEF_ori = self.panda.get_orientation()  # Quat (x, y, z, w)
        # Custom data shape: 29 floats for EEF state (position, orientation, etc.)
        # Adjust to fit your needs.
        EEF_state = np.zeros(29, dtype=np.float32)

        # Example positions
        EEF_state[:3] = EEF_pos.astype(np.float32)
        EEF_state[3:7] = EEF_ori.astype(np.float32)
        # Gripper
        gripper_state = self.gripper.read_once()
        EEF_state[7] = gripper_state.width if gripper_state is not None else 0

        return {
            "EEF_state": EEF_state,
            "timestamp": np.array([time.time()], dtype=np.float64)
        }

    def run(self):
        try:
            self.init_robot()
            running = True
            self.ready_event.set()
            current_rotation = self.panda.get_orientation()
            current_translation = self.panda.get_position()
            sm_state = self.space_mouse.get_motion_state_transformed()

            ctx = self.panda.create_context(frequency=1000)
            controller = panda_py.controllers.CartesianImpedance()
            self.panda.start_controller(controller)
            time.sleep(1)

            while ctx.ok() and running:
                start_time = time.perf_counter()

                sm_state = self.space_mouse.get_motion_state_transformed()
                dpos = sm_state[:3] * MOVE_INCREMENT
                drot_xyz = sm_state[3:] * MOVE_INCREMENT * 3

                # Remove rotation from Spacemouse if not needed:
                drot_xyz[:] = 0

                state_data = self.get_state()
                current_translation += np.array([dpos[0], dpos[1], dpos[2]])
                if drot_xyz is not None:
                    delta_rotation = R.from_euler('xyz', drot_xyz)
                    current_rotation = (
                        delta_rotation * R.from_quat(current_rotation)
                    ).as_quat()

                controller.set_control(current_translation, current_rotation)

                # Handle gripper state changes
                if self.space_mouse.is_button_pressed(0):
                    success = self.gripper.grasp(
                        0.03,
                        speed=SPEED,
                        force=FORCE,
                        epsilon_inner=0.05,
                        epsilon_outer=0.05
                    )
                    if success:
                        print("Grasp successful")
                    else:
                        print("Grasp failed")

                elif self.space_mouse.is_button_pressed(1):
                    success = self.gripper.move(0.08, speed=SPEED)
                    if success:
                        print("Release successful")
                    else:
                        print("Release failed")

                # Push state data into the ring buffer
                self.put_state(state_data)

                # Sleep to maintain loop frequency of 1000 Hz
                end_time = time.perf_counter()

        except Exception as e:
            print(e)
            running = False

    def init_robot(self):
        joint_pose = [
            -0.01588696, -0.25534376, 0.18628714,
            -2.28398158, 0.0769999, 2.02505396, 0.07858208
        ]

        # Move to home position
        self.panda.move_to_joint_position(joint_pose)

        # Open gripper
        self.gripper.move(width=0.9, speed=0.1)

        action = np.zeros((9,))
        action[:-2] = joint_pose
        self.panda.enable_logging(int(10))

    # ========= context manager ===========
    @property
    def is_ready(self):
        """
        Since there is no camera, you might decide to always return True
        after initialization.
        """
        return True

    def start(self, wait=True, exposure_time=5):
        """
        Start the mp.Process (the FrankaAPI process).
        If you have other services, start them here.
        """
        super().start()
        if wait:
            self.start_wait()

    def stop(self, wait=True):
        """
        Stop the FrankaAPI process.
        If you have other services, stop them here.
        """
        self.join()
        if wait:
            self.stop_wait()

    def start_wait(self):
        """
        Wait for readiness after starting.
        """
        self.ready_event.wait()

    def stop_wait(self):
        pass

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()

    # ========= Public APIs ===============
    def get_robot_state(self):
        """Get latest state from ring buffer."""
        try:
            state = self.Robot_state_buffer.get()
            return state
        except Exception as e:
            print(f"Error getting state from buffer: {e}")
            return None

    def get_status(self):
        obs = dict()
        gripper_state = self.gripper.read_once()
        gripper_qpos = gripper_state.width if gripper_state is not None else 0

        self.panda.enable_logging

        log_EE_pose = self.panda.get_position()
        log_EE_orientation = self.panda.get_orientation()

        log_joint_pose = self.panda.get_log().get("q")
        log_joint_vel = self.panda.get_log().get("dq")
        log_joint_torque = self.panda.get_log().get("tau_J")

        log = {
            "arm": {
                "q": log_joint_pose,
                "dq": log_joint_vel,
                "tau_J": log_joint_torque,
                "EE_position": log_EE_pose,
                "EE_orientation": log_EE_orientation,
                "gripper_state": gripper_qpos
            }
        }

        return log

    def move_to(self, positions, orientations):
        self.panda.move_to_pose(positions, orientations)

    def grasp(self):
        self.gripper.grasp(
            0.03,
            speed=SPEED,
            force=FORCE,
            epsilon_inner=0.3,
            epsilon_outer=0.3
        )

    def release(self):
        self.gripper.move(width=0.08, speed=0.1)
