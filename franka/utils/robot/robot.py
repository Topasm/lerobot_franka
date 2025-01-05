# franka_api_threads.py

import time
import threading
import queue
import numpy as np
import panda_py.controllers
import torch
import panda_py
from panda_py import libfranka
from scipy.spatial.transform import Rotation as R
from multiprocessing.managers import SharedMemoryManager

# If you still want to keep your SharedMemoryRingBuffer usage:
from franka.utils.shared_memory.shared_memory_ring_buffer import SharedMemoryRingBuffer
# If you have a SpacemouseTeleop, import it here:
# from franka.utils.robot.spacemouse_teleop import SpacemouseTeleop
MOVE_INCREMENT = 0.0002
GRIPPER_INCREMENT = 0.02


class FrankaAPI:
    def __init__(self, robot_ip=None):
        """
        Thread-based FrankaAPI. No mp.Process subclass.
        """
        self.robot_ip = robot_ip

        # Robot and gripper
        self.panda = panda_py.Panda(hostname=self.robot_ip)
        self.gripper = libfranka.Gripper(self.robot_ip)
        # Example ring buffer usage
        self.obs_fps = 60
        robot_state = {
            # [x, y, z, qx, qy, qz, qw, gripper] => total 29 floats for example
            "Robot_state": np.zeros(29, dtype=np.float32)
        }
        self.shm_manager = SharedMemoryManager()
        self.shm_manager.start()

        # self.Robot_state_buffer = SharedMemoryRingBuffer.create_from_examples(
        #     shm_manager=self.shm_manager,
        #     examples=robot_state,
        #     get_max_k=30,
        #     get_time_budget=0.2,
        #     put_desired_frequency=self.obs_fps,
        # )

        # Queues & events
        self._stop_event = threading.Event()
        self._robot_thread = None
        self._gripper_thread = None
        self._ready_event = threading.Event()

    # ------------------ Thread Loops ------------------ #
    def _robot_loop(self):
        """
        Main control loop (simulating ~1kHz).
        """
        try:
            # Mark ready
            self._ready_event.set()

            current_rotation = self.panda.get_orientation()
            current_translation = self.panda.get_position()

            # Create a high-freq context
            ctx = self.panda.create_context(frequency=1000)
            controller = panda_py.controllers.CartesianImpedance()
            self.panda.start_controller(controller)
            time.sleep(1)

            while ctx.ok() and not self._stop_event.is_set():
                # Check if we have a new command (dpos, drot)
                try:
                    smstate = self.spanvstate.get_motion_state_transformed()
                    dpos = smstate[:3]*MOVE_INCREMENT
                    drot = smstate[3:]*MOVE_INCREMENT*3
                    current_translation += np.array(
                        [dpos[0], dpos[1], dpos[2]])
                    if drot is not None:
                        delta_rotation = R.from_euler(
                            "xyz", drot, degrees=False)
                        curr_q = R.from_quat(current_rotation)
                        new_q = (delta_rotation * curr_q).as_quat()
                        current_rotation = new_q

                    # Update controller
                    controller.set_control(
                        current_translation, current_rotation)

                except queue.Empty:
                    pass

                # Sleep a bit to avoid busy-wait
                # time.sleep(0.001)

        except Exception as e:
            print(f"[FrankaAPI] Robot loop exception: {e}")
        finally:
            print("[FrankaAPI] Robot loop exiting.")

    def _gripper_loop(self):
        """
        Asynchronous gripper control loop.
        """
        # curwidth = self.gripper.read_once().width
        print("[FrankaAPI] Gripper thread started.")

        while not self._stop_event.is_set():
            try:

                if self.spanvstate.is_button_pressed(1):
                    self.gripper.grasp(width=0.01, speed=0.1, force=10)
                elif self.spanvstate.is_button_pressed(0):
                    self.gripper.move(width=0.09, speed=0.1)

            except queue.Empty:
                pass

            time.sleep(0.001)
        print("[FrankaAPI] Gripper thread stopping.")

    # ------------------ Public Methods ------------------ #
    def startup(self, spnavstate):
        """
        Initialize the robot, move to a starting pose, etc.
        Then start the robot & gripper threads.
        """

        self.spanvstate = spnavstate
        # Example: move to a home position
        home_pose = [
            -0.01588696,
            -0.25534376,
            0.18628714,
            -2.28398158,
            0.0769999,
            2.02505396,
            0.07858208,
        ]
        self.panda.move_to_joint_position(home_pose)

        # (Optional) open gripper here synchronously
        self.gripper.move(width=0.09, speed=0.1)

        # Create and start threads
        self._stop_event.clear()

        self._robot_thread = threading.Thread(
            target=self._robot_loop, daemon=True)
        self._gripper_thread = threading.Thread(
            target=self._gripper_loop, daemon=True)

        self._robot_thread.start()
        self._gripper_thread.start()

        # Wait until the robot thread signals it's ready
        self._ready_event.wait()

        return True

    def stop(self):
        """
        Signal threads to stop and join them.
        """
        self._stop_event.set()
        if self._robot_thread is not None:
            self._robot_thread.join()
            self._robot_thread = None
        if self._gripper_thread is not None:
            self._gripper_thread.join()
            self._gripper_thread = None

    def on_teleop(self, spnavstate):
        """
        Asynchronously send new (dpos, drot) to the robot loop.
        dpos in meters, drot in euler angles (xyz) in radians.
        """
        self.spanvstate = spnavstate

    # def put_state(self, state_data):
    #     """
    #     If you want to store state data in the ring buffer.
    #     """
    #     self.Robot_state_buffer.put(state_data)

    def get_status(self):
        """
        Return a dict describing the current robot + gripper status.
        E.g., as expected by your FrankaControl code.
        """
        gripper_state = self.gripper.read_once()
        gripper_qpos = gripper_state.width
        ee_pos = self.panda.get_position()
        ee_ori = self.panda.get_orientation()
        robot_state = self.panda.get_state()

        return {
            "arm": {
                "q": robot_state.q,
                "dq": robot_state.dq,
                "tau_J": robot_state.tau_J,
                "EE_position": ee_pos,
                "EE_orientation": ee_ori,
                "gripper_state": gripper_qpos,
            }
        }

    def init_robot(self):
        print("[FrankaAPI] Robot initialization steps here.")

    # ------------- Context manager (optional) -------------
    def __enter__(self):
        self.startup()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()

    def __del__(self):
        # Ensure we stop threads and release shared memory
        self.stop()
        if self.shm_manager is not None:
            self.shm_manager.shutdown()
        print("[FrankaAPI] Cleaned up.")
