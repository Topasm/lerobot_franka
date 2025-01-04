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
    cameras: dict[str, Camera] = field(
        default_factory=lambda: {})
    robot_ip: str | None = None


class FrankaControl(FrankaAPI):
    # Exclude from dataclass fields
    is_connected: bool = field(init=False, default=False)
    state_keys: list | None = field(init=False, default=None)  # Add this line

    def __init__(self, config: FrankaRobotConfig | None = None, **kwargs):

        if config is None:
            config = FrankaRobotConfig()
        self.config = replace(config, **kwargs)

        super().__init__(robot_ip=self.config.robot_ip)
        self.robot_type = self.config.robot_type
        self.cameras = self.config.cameras
        self.is_connected = False
        self.teleop = None
        self.logs = {}

    @property
    def camera_features(self) -> dict:
        cam_ft = {}
        for cam_key, cam in self.cameras.items():
            key = f"observation.images.{cam_key}"
            cam_ft[key] = {
                "shape": (cam.height, cam.width, cam.channels),
                "names": ["height", "width", "channels"],
                "info": None,
            }
        return cam_ft

    @property
    def motor_features(self) -> dict:
        action_names = ["arm.joint_positions",
                        "arm.joint_velocities", "gripper.width"]
        state_names = ["arm.joint_positions", "arm.joint_velocities",
                       "arm.joint_torques", "arm.EE_position", "arm.EE_orientation", "gripper.width"]
        return {
            "action": {
                "dtype": "float32",
                "shape": (len(action_names),),
                "names": action_names,
            },
            "observation.state": {
                "dtype": "float32",
                "shape": (len(state_names),),
                "names": state_names,
            },
        }

    @property
    def features(self):
        return {**self.motor_features, **self.camera_features}

    @property
    def has_camera(self):
        return len(self.cameras) > 0

    @property
    def num_cameras(self):
        return len(self.cameras)

    def connect(self):
        self.is_connected = self.startup()
        self.control_out = SpacemouseTeleop(
            shm_manager=self.shm_manager)
        self.control_out.startup(robot=self)
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

        before_read_t = time.perf_counter()
        state = self.get_state()
        spa_out = self.control_out.get_state()
        self.logs["read_pos_dt_s"] = time.perf_counter() - before_read_t

        self.send_command(dpos=spa_out["translation"],
                          drot=spa_out["rotation"])
        before_write_t = time.perf_counter()

        self.logs["write_pos_dt_s"] = time.perf_counter() - before_write_t

        if self.state_keys is None:
            self.state_keys = list(state)

        if not record_data:
            return
        action = spa_out
        # action = torch.as_tensor(list(action.values()))

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

    def send_action(self, action: torch.Tensor) -> torch.Tensor:
        # TODO(aliberts): return ndarrays instead of torch.Tensors
        if not self.is_connected:
            raise ConnectionError()

        # if self.teleop is None:
        #     self.teleop = SpacemouseTeleop(robot_instance=False)
        #     self.teleop.startup(robot=self)

        # before_write_t = time.perf_counter()
        # self.teleop.do_motion(state=action_dict, robot=self)
        # self.logs["write_pos_dt_s"] = time.perf_counter() - before_write_t
        print("Sending action to robot.")
        # TODO(aliberts): return action_sent when motion is limited
        return action

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
