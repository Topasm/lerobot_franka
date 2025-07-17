import multiprocessing as mp
import time

import numpy as np
import pyspacemouse

from franka.utils.shared_memory.shared_memory_ring_buffer import SharedMemoryRingBuffer


class Spacemouse(mp.Process):
    def __init__(self,
                 shm_manager,
                 get_max_k=30,
                 frequency=200,
                 max_value=500,
                 deadzone=(0, 0, 0, 0, 0, 0),
                 dtype=np.float32,
                 n_buttons=2,
                 device_path=None,
                 ):
        """
        Continuously listen to 3D connection space navigator events
        and update the latest state using pyspacemouse.

        max_value: {300, 500} 300 for wired version and 500 for wireless
        deadzone: [0,1], number or tuple, axis with value lower than this value will stay at 0
        device_path: optional device path for the spacemouse (e.g., "/dev/hidraw4")

        front
        z
        ^   _
        |  (O) space mouse
        |
        *----->x right
        y
        """
        super().__init__()
        if np.issubdtype(type(deadzone), np.number):
            deadzone = np.full(6, fill_value=deadzone, dtype=dtype)
        else:
            deadzone = np.array(deadzone, dtype=dtype)
        assert (deadzone >= 0).all()

        # copied variables
        self.frequency = frequency
        self.max_value = max_value
        self.dtype = dtype
        self.deadzone = deadzone
        self.n_buttons = n_buttons
        self.device_path = device_path
        self.mouse = None
        self.latest_state = None

        # transformation matrix from spacemouse to desired coordinate system
        self.tx_zup_spnav = np.array([
            [0, 0, -1],
            [1, 0, 0],
            [0, 1, 0]
        ], dtype=dtype)

        example = {
            # 3 translation, 3 rotation, 1 period
            'motion_event': np.zeros((7,), dtype=np.int64),
            # left and right button
            'button_state': np.zeros((n_buttons,), dtype=bool),
            'receive_timestamp': time.time()
        }
        ring_buffer = SharedMemoryRingBuffer.create_from_examples(
            shm_manager=shm_manager,
            examples=example,
            get_max_k=get_max_k,
            get_time_budget=0.2,
            put_desired_frequency=frequency
        )

        # shared variables
        self.ready_event = mp.Event()
        self.stop_event = mp.Event()
        self.ring_buffer = ring_buffer

    # ======= get state APIs ==========

    def get_motion_state(self):
        state = self.ring_buffer.get()
        state = np.array(state['motion_event'][:6],
                         dtype=self.dtype) / self.max_value
        is_dead = (-self.deadzone < state) & (state < self.deadzone)
        state[is_dead] = 0
        return state

    def get_motion_state_transformed(self):
        """
        Return in right-handed coordinate
        z
        *------>y right
        |   _
        |  (O) space mouse
        v
        x
        back

        """
        state = self.get_motion_state()
        tf_state = np.zeros_like(state)
        tf_state[:3] = self.tx_zup_spnav @ state[:3]
        tf_state[3:] = self.tx_zup_spnav @ state[3:]
        return tf_state

    def get_button_state(self):
        state = self.ring_buffer.get()
        return state['button_state']

    def is_button_pressed(self, button_id):
        return self.get_button_state()[button_id]

    # ========== start stop API ===========

    def start(self, wait=True):
        super().start()
        if wait:
            self.ready_event.wait()

    def stop(self, wait=True):
        self.stop_event.set()
        if wait:
            self.join()

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()

    def run(self):
        """Main loop using pyspacemouse instead of spnav."""
        # Open spacemouse connection
        if self.device_path is None:
            self.mouse = pyspacemouse.open()
        else:
            self.mouse = pyspacemouse.open(path=self.device_path)

        if not self.mouse:
            print("Failed to open spacemouse device")
            return

        try:
            motion_event = np.zeros((7,), dtype=np.int64)
            button_state = np.zeros((self.n_buttons,), dtype=bool)

            # send one message immediately so client can start reading
            self.ring_buffer.put({
                'motion_event': motion_event,
                'button_state': button_state,
                'receive_timestamp': time.time()
            })
            self.ready_event.set()

            while not self.stop_event.is_set():
                # Read spacemouse state
                state = self.mouse.read()
                receive_timestamp = time.time()

                if state is not None:
                    # Update motion event from pyspacemouse state
                    # Scale values to match expected range
                    motion_event[0] = int(
                        state.x * self.max_value)  # translation x
                    motion_event[1] = int(
                        state.y * self.max_value)  # translation y
                    motion_event[2] = int(
                        state.z * self.max_value)  # translation z
                    motion_event[3] = int(
                        state.roll * self.max_value)   # rotation x
                    motion_event[4] = int(
                        state.pitch * self.max_value)  # rotation y
                    motion_event[5] = int(
                        state.yaw * self.max_value)    # rotation z
                    motion_event[6] = int(state.t * 1000)  # timestamp in ms

                    # Update button state
                    if hasattr(state, 'buttons') and len(state.buttons) >= self.n_buttons:
                        button_state[:len(state.buttons)
                                     ] = state.buttons[:self.n_buttons]

                # Send data to ring buffer
                self.ring_buffer.put({
                    'motion_event': motion_event,
                    'button_state': button_state,
                    'receive_timestamp': receive_timestamp
                })

                # Sleep to maintain frequency
                time.sleep(1.0 / self.frequency)

        except Exception as e:
            print(f"Error in spacemouse run loop: {e}")
        finally:
            if self.mouse:
                self.mouse.close()
