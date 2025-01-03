import time
import torch
import numpy as np
import cv2
from dataclasses import dataclass, field, replace
from pathlib import Path
from multiprocessing.managers import SharedMemoryManager
from franka.utils.robot.robot import FrankaAPI
from lerobot.common.robot_devices.cameras.utils import Camera
from franka.utils.robot.spacemouse_teleop import SpacemouseTeleop


@dataclass
class FrankaRobotConfig:
    robot_type: str | None = "Franka"
    # Cameras or other additional configs
    cameras: dict = field(default_factory=lambda: {})
    max_relative_target: list[float] | float | None = None
    robot_ip: str | None = None


class FrankaControl(FrankaAPI):
    def __init__(self, config: FrankaRobotConfig | None = None, **kwargs):
        super().__init__()
        self.config = replace(config, **kwargs)

        self.robot_type = self.config.robot_type
        self.cameras = self.config.cameras
        self.teleop = None
        self.is_connected = False
        self.logs = {}

    def connect(self):
        self.is_connected = self.startup()
        if not self.is_connected:
            print(
                "Another process is already using Stretch. Try running 'stretch_free_robot_process.py'")
            raise ConnectionError()

        for name in self.cameras:
            self.cameras[name].connect()
            self.is_connected = self.is_connected and self.cameras[name].is_connected

        if not self.is_connected:
            print(
                "Could not connect to the cameras, check that all cameras are plugged-in.")
            raise ConnectionError()

        self.run_calibration()

    def run_calibration(self):
        self.init_robot()

    def teleop_step(
        self, record_data=False
    ) -> None | tuple[dict[str, torch.Tensor], dict[str, torch.Tensor]]:
        # TODO(aliberts): return ndarrays instead of torch.Tensors
        if not self.is_connected:
            raise ConnectionError()

        if self.teleop is None:
            self.teleop = SpacemouseTeleop(robot_instance=False)
            self.teleop.startup(robot=self)

        before_read_t = time.perf_counter()
        state = self.get_state()
        action = self.teleop.get_state()
        self.logs["read_pos_dt_s"] = time.perf_counter() - before_read_t

        before_write_t = time.perf_counter()
        self.teleop.do_motion(robot=self)
        self.push_command()
        self.logs["write_pos_dt_s"] = time.perf_counter() - before_write_t

        if self.state_keys is None:
            self.state_keys = list(state)

        if not record_data:
            return

        state = torch.as_tensor(list(state.values()))
        action = torch.as_tensor(list(action.values()))

        # Capture images from cameras
        images = {}
        for name in self.cameras:
            before_camread_t = time.perf_counter()
            images[name] = self.cameras[name].async_read()
            images[name] = torch.from_numpy(images[name])
            self.logs[f"read_camera_{name}_dt_s"] = self.cameras[name].logs["delta_timestamp_s"]
            self.logs[f"async_read_camera_{name}_dt_s"] = time.perf_counter(
            ) - before_camread_t

        # Populate output dictionnaries
        obs_dict, action_dict = {}, {}
        obs_dict["observation.state"] = state
        action_dict["action"] = action
        for name in self.cameras:
            obs_dict[f"observation.images.{name}"] = images[name]

        return obs_dict, action_dict

    def get_state(self):

        status = self.get_status()

        return {
            "arm.joint_positions": status["arm"]["q"],
            "arm.joint_velocities": status["arm"]["dq"],
            "arm.joint_torques": status["arm"]["tau_J"],
            "arm.EE_position": status["arm"]["EE_position"],
            "arm.EE_orientation": status["arm"]["EE_orientation"],
            "gripper.width": status["arm"]["gripper_state"]}

    def capture_observation(self) -> dict:
        # TODO(aliberts): return ndarrays instead of torch.Tensors
        before_read_t = time.perf_counter()
        state = self.get_state()
        self.logs["read_pos_dt_s"] = time.perf_counter() - before_read_t

        if self.state_keys is None:
            self.state_keys = list(state)

        state = torch.as_tensor(list(state.values()))

        # Capture images from cameras
        images = {}
        for name in self.cameras:
            before_camread_t = time.perf_counter()
            images[name] = self.cameras[name].async_read()
            images[name] = torch.from_numpy(images[name])
            self.logs[f"read_camera_{name}_dt_s"] = self.cameras[name].logs["delta_timestamp_s"]
            self.logs[f"async_read_camera_{name}_dt_s"] = time.perf_counter(
            ) - before_camread_t

        # Populate output dictionnaries
        obs_dict = {}
        obs_dict["observation.state"] = state
        for name in self.cameras:
            obs_dict[f"observation.images.{name}"] = images[name]

        return obs_dict

    def send_action(self, action):

        self.move_to(positions=action)

    def disconnect(self):
        """
        Disconnect from the robot and clean up resources.
        # """
        # if self.is_connected:
        #     print("Disconnecting from Franka.")
        #     self.is_connected = False

    def __del__(self):
        # self.disconnect()
        print("FrankaControl object deleted.")
