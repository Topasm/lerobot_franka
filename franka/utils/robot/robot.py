# franka_api_process.py

import time
import multiprocessing as mp
import threading
import numpy as np
import panda_py.controllers
import panda_py
from panda_py import libfranka
from scipy.spatial.transform import Rotation as R
from multiprocessing.managers import SharedMemoryManager
from franka.utils.shared_memory.shared_memory_ring_buffer import SharedMemoryRingBuffer
import ctypes
import os

MOVE_INCREMENT = 0.006
GRIPPER_INCREMENT = 0.02

# Constants from <linux/sched.h>
SCHED_FIFO = 1
# Create a struct for sched_param


class sched_param(ctypes.Structure):
    _fields_ = [("sched_priority", ctypes.c_int)]


libc = ctypes.CDLL("libc.so.6", use_errno=True)


def set_realtime_priority(priority: int = 50) -> None:
    """
    Set the *current process* (i.e. child process) to SCHED_FIFO with given priority [1..99].
    Requires running as root or having CAP_SYS_NICE capability.
    """
    param = sched_param(priority)
    # 0 => self (the calling process) on Linux for sched_setscheduler
    result = libc.sched_setscheduler(0, SCHED_FIFO, ctypes.byref(param))
    if result != 0:
        err = ctypes.get_errno()
        raise OSError(
            err, f"sched_setscheduler(SCHED_FIFO, priority={priority}) failed")


class FrankaAPI(mp.Process):
    def __init__(self, robot_ip=None):
        """
        mp.Process-based FrankaAPI.
        After you call .start(), the child process runs 'run()'.
        """
        super().__init__()
        self.robot_ip = robot_ip

        # Robot and gripper are constructed here in the parent,
        # but also accessible in the child after fork/spawn.
        self.panda = panda_py.Panda(
            hostname=self.robot_ip)
        self.gripper = libfranka.Gripper(self.robot_ip)
        robot_state = {
            "q": np.zeros(7, dtype=np.float32),
            "dq": np.zeros(7, dtype=np.float32),
            "tau_J": np.zeros(7, dtype=np.float32),
            "EE_position": np.zeros(3, dtype=np.float32),
            "EE_orientation": np.zeros(4, dtype=np.float32),
            "gripper_state": 0.0
        }
        self.shm_manager = SharedMemoryManager()
        self.shm_manager.start()

        # We use Events for start/stop signals:
        self._stop_event = mp.Event()    # signals the child to exit
        self._ready_event = mp.Event()   # signals the parent that child has started

        self.Robot_state_buffer = SharedMemoryRingBuffer.create_from_examples(
            shm_manager=self.shm_manager,
            examples=robot_state,
            get_max_k=32,
            get_time_budget=0.1,
            put_desired_frequency=15
        )

        # We'll store the 'spnavstate' reference here.
        # By default, it's None until .startup(spnavstate) is called.
        self.spnavstate = None

        # For the gripper loop inside the child process
        self._gripper_stop_event = None
        self._gripper_thread = None

    # ----------------------- Child Process Entry ----------------------- #
    def run(self):
        try:
            # 1. Attempt to elevate scheduling priority *before* starting the main loop
            #    (Requires root or cap_sys_nice capability)
            try:
                os.sched_setaffinity(0, {24})  # pin to CPU 3
                set_realtime_priority(priority=90)  # adjust priority as needed
                print(
                    "[FrankaAPI] Child process set to real-time scheduling (SCHED_FIFO).")
            except OSError as e:
                print(f"[FrankaAPI] Could NOT set real-time priority: {e}")

            # 2. Signal the parent that the child process started
            self._ready_event.set()

            # 3. Create a ~200 Hz context in the child
            ctx = self.panda.create_context(frequency=1000)
            controller = panda_py.controllers.CartesianImpedance()
            self.panda.start_controller(controller)
            time.sleep(1)  # let the controller settle

            current_rotation = self.panda.get_orientation()
            current_translation = self.panda.get_position()

            # 4. Start a thread for the gripper in this child process
            self._gripper_stop_event = threading.Event()
            self._gripper_thread = threading.Thread(
                target=self._gripper_loop,
                daemon=True
            )
            self._gripper_thread.start()

            # 5. Main robot loop
            while ctx.ok() and not self._stop_event.is_set():
                # read the spnav state if available

                if self.spnavstate is not None:
                    smstate = self.spnavstate.get_motion_state_transformed()
                    dpos = smstate[:3] * MOVE_INCREMENT
                    drot = smstate[3:] * MOVE_INCREMENT*2

                    # Update the translation
                    current_translation += np.array(dpos, dtype=np.float32)

                    # Update rotation
                    delta_rotation = R.from_euler("xyz", drot, degrees=False)
                    curr_q = R.from_quat(current_rotation)
                    new_q = (delta_rotation * curr_q).as_quat()
                    current_rotation = new_q

                    # Send to controller
                    controller.set_control(
                        current_translation, current_rotation)

                # time.sleep(0.001)  # avoid 100% CPU usage
                state_data = self.get_robot_state()
                self.put_state(state_data)

            # 6. If we exit the while loop, stop the gripper thread
            if self._gripper_stop_event is not None:
                self._gripper_stop_event.set()
                self._gripper_thread.join()

        except Exception as e:
            print(f"[FrankaAPI] run() exception in child process: {e}")
        finally:
            print("[FrankaAPI] run() ended in the child process.")

    def _gripper_loop(self):
        """
        A separate thread (inside the child process) that checks spnavstate
        to open/close the gripper asynchronously.
        """
        print("[FrankaAPI] Gripper thread started.")
        while not self._gripper_stop_event.is_set():
            try:
                if self.spnavstate is not None:
                    # if button 1 is pressed => do a grasp
                    if self.spnavstate.is_button_pressed(1):
                        self.gripper.grasp(width=0.01, speed=0.1, force=20)
                    # if button 0 is pressed => open
                    elif self.spnavstate.is_button_pressed(0):
                        self.gripper.move(width=0.09, speed=0.1)

            except Exception as e:
                print(f"[FrankaAPI] Gripper loop exception: {e}")
            # You can optionally add a small sleep to reduce CPU usage:
            time.sleep(0.001)

        print("[FrankaAPI] Gripper thread stopping.")

    # ----------------------- Parent-Side Methods ----------------------- #
    def startup(self, spnavstate):
        """
        Called in the parent process before .start().
        1) Move the robot to home.
        2) Set spnavstate object.
        3) Actually start() the child process.
        4) Wait until the child signals it's ready.
        """
        # Store the spnavstate for the child
        self.spnavstate = spnavstate

        # Move to a home position in the parent
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

        # Open the gripper once in the parent
        self.gripper.move(width=0.09, speed=0.1)

        # Now start the child process (calls run())
        self.start()
        # Wait for child to say it's ready
        self._ready_event.wait()
        return True

    def put_state(self, state_data):
        """
        Put the robot state into the shared memory ring buffer.
        """
        self.Robot_state_buffer.put(state_data)

    def stop(self):
        """
        Called in the parent process to gracefully stop the child process.
        """
        self._stop_event.set()
        if self.is_alive():
            self.join()

    def on_teleop(self, spnavstate):
        """
        If you want to update the spnavstate object in the parent process
        after the child is already running, do so here.
        But note: the child only sees the *current* reference,
        so if spnavstate is re-created, you'd have to pass it again.
        """
        self.spnavstate = spnavstate

    def get_status(self):
        """
        Get the current robot status.
        """
        return self.Robot_state_buffer.get()

    def get_robot_state(self):

        gripper_state = self.gripper.read_once()
        gripper_qpos = gripper_state.is_grasped
        ee_pos = self.panda.get_position()
        ee_ori = self.panda.get_orientation()
        robot_state = self.panda.get_state()
        return {
            "q": robot_state.q,
            "dq": robot_state.dq,
            "tau_J": robot_state.tau_J,
            "EE_position": ee_pos,
            "EE_orientation": ee_ori,
            "gripper_state": gripper_qpos,
        }

    def __enter__(self):
        # Optional context manager usage
        self.startup(None)  # or pass a spnavstate
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        # Stop child on exit
        self.stop()

    def __del__(self):
        # Final cleanup if object is destroyed
        if self.is_alive():
            try:
                self.terminate()
                self.join()
            except Exception:
                pass
        if self.shm_manager is not None:
            self.shm_manager.shutdown()
        print("[FrankaAPI] Cleaned up parent resources.")
