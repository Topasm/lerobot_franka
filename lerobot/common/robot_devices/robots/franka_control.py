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
        action_names = ["motor1", "motor2", "motor3",
                        "motor4", "motor5", "motor6", "motor7"]
        state_names = ["motor1", "motor2", "motor3",
                       "motor4", "motor5", "motor6", "motor7"]
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
            "observation.velocity": {
                "dtype": "float32",
                "shape": (len(state_names),),
                "names": state_names,
            },
            "observation.torque": {
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

        self.is_connected = True
        self.teleop_start = False
        # self.on_teleop()
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
        print("Running calibration.")

    def teleop_step(
        self, record_data=False
    ) -> None | tuple[dict[str, torch.Tensor], dict[str, torch.Tensor]]:
        # TODO(aliberts): return ndarrays instead of torch.Tensors
        if not self.is_connected:
            raise ConnectionError()
        # run only once to start the teleop just startup countrol_out is not none this is a one time
        if self.teleop_start is False:
            self.control_out = SpacemouseTeleop(
                shm_manager=self.shm_manager)
            self.is_connected = self.startup(
                self.control_out.Spacemouse_controller)
            self.control_out.startup(robot=self)
            self.teleop_start = True

        before_read_t = time.perf_counter()
        state = self.get_state()

        self.logs["read_pos_dt_s"] = time.perf_counter() - before_read_t

        before_write_t = time.perf_counter()

        self.logs["write_pos_dt_s"] = time.perf_counter() - before_write_t

        if self.state_keys is None:
            self.state_keys = list(state)

        if not record_data:
            return
        action = 0
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
        obs_dict["observation.state"] = state["arm.joint_positions"]
        obs_dict["observation.velocity"] = state["arm.joint_velocities"]
        obs_dict["observation.torque"] = state["arm.joint_torques"]
        # action_dict["action"] = action
        action_dict["action"] = state["arm.joint_positions"]
        for name in self.cameras:
            obs_dict[f"observation.images.{name}"] = images[name]

        return obs_dict, action_dict

    def get_state(self):

        status = self.get_status()

        return {
            "arm.joint_positions": status["q"],
            "arm.joint_velocities": status["dq"],
            "arm.joint_torques": status["tau_J"],
            "arm.EE_position": status["EE_position"],
            "arm.EE_orientation": status["EE_orientation"],
            "gripper.width": status["gripper_state"]}

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
        obs_dict["observation.state"] = state["arm.joint_positions"]
        obs_dict["observation.velocity"] = state["arm.joint_velocities"]
        obs_dict["observation.torque"] = state["arm.joint_torques"]
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
