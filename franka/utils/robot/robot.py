# franka_api.py

import time
import multiprocessing as mp
import numpy as np
import panda_py.controllers
import torch
import panda_py
from panda_py import libfranka
import transforms3d
from scipy.spatial.transform import Rotation as R
from multiprocessing.managers import SharedMemoryManager

# Import your SpacemouseTeleop
from franka.utils.robot.spacemouse_teleop import SpacemouseTeleop
from franka.utils.shared_memory.shared_memory_ring_buffer import SharedMemoryRingBuffer


class FrankaAPI(mp.Process):
    def __init__(
        self,
        robot_ip=None
    ):
        super().__init__()

        self.obs_fps = 15

        # Teleop reference
        self.teleop: SpacemouseTeleop | None = None

        self.panda = panda_py.Panda(hostname=robot_ip)
        self.gripper = libfranka.Gripper(robot_ip)
        robot_state = {
            # [x,y,z, qx,qy,qz,qw, gripper]
            'Robot_state': np.zeros(29, dtype=np.float32)
        }

        # Shared Memory Manager
        self.shm_manager = SharedMemoryManager()
        self.shm_manager.start()

        self.Robot_state_buffer = SharedMemoryRingBuffer.create_from_examples(
            shm_manager=self.shm_manager,
            examples=robot_state,
            get_max_k=30,  # Store last 30 states
            get_time_budget=0.2,
            put_desired_frequency=self.obs_fps
        )

        # For demonstration: event to signal readiness
        self.ready_event = mp.Event()
        self.queue = mp.Queue()  # Queue for communication

    def run(self):
        try:
            running = True
            self.ready_event.set()
            current_rotation = self.panda.get_orientation()
            current_translation = self.panda.get_position()
            ctx = self.panda.create_context(frequency=1000)
            controller = panda_py.controllers.CartesianImpedance()
            self.panda.start_controller(controller)
            # time.sleep(1)

            while ctx.ok() and running:
                # Check for new dpos and drot values
                if not self.queue.empty():
                    dpos, drot = self.queue.get()
                    current_translation += np.array(
                        [dpos[0], dpos[1], dpos[2]])
                    if drot is not None:
                        delta_rotation = R.from_euler('xyz', drot)
                        current_rotation = (
                            delta_rotation * R.from_quat(current_rotation)
                        ).as_quat()
                    controller.set_control(
                        current_translation, current_rotation)

                time.sleep(0.001)  # Small delay to prevent CPU overuse

        except Exception as e:
            print(e)
            running = False

    def send_command(self, dpos, drot):
        """Send dpos and drot to the run process."""
        self.queue.put((dpos, drot))

    def put_state(self, state_data):
        """Put new state data into ring buffer"""
        self.Robot_state_buffer.put(state_data)

    def startup(self):
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

        self.start()

        return True

    def init_robot(self):
        # joint_pose = [
        #     -0.01588696, -0.25534376, 0.18628714,
        #     -2.28398158, 0.0769999, 2.02505396, 0.07858208
        # ]

        # # Move to home position
        # self.panda.move_to_joint_position(joint_pose)

        # # Open gripper
        # self.gripper.move(width=0.9, speed=0.1)

        print("Robot initialized")

    def get_status(self):
        obs = dict()
        gripper_state = self.gripper.read_once()
        gripper_qpos = gripper_state.width if gripper_state is not None else 0

        log_EE_pose = self.panda.get_position()
        log_EE_orientation = self.panda.get_orientation()
        self.robot_state = self.panda.get_state()

        log_joint_pose = self.robot_state.q
        log_joint_vel = self.robot_state.dq
        log_joint_torque = self.robot_state.tau_J

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
