"""
AgileX Piper robot implementation for the LeRobot framework.

This module provides a Robot implementation for the AgileX Piper robot arm
that integrates with the LeRobot framework's standardized interfaces.
"""

from dataclasses import dataclass, field
from typing import Any

from lerobot.cameras import make_cameras_from_configs
from ..robot import Robot

from .config_piper_dual import PiperConfigDual
from .piper_sdk_interface_dual import PiperSDKInterfaceDual
import torch
import numpy as np
import time

class PiperDual(Robot):
    config_class = PiperConfigDual
    name = "piper_dual"

    def __init__(self, config: PiperConfigDual, **kwargs):
        """
        Initialize the Piper robot.
        
        Args:
            config: Configuration for the Piper robot
            **kwargs: Additional configuration overrides
        """
        super().__init__(config, **kwargs)
        self.sdk = PiperSDKInterfaceDual(port_r=config.can_interface_r, port_l=config.can_interface_l, gripper_r_serial = config.gripper_serial_r, gripper_l_serial = config.gripper_serial_l)
        self.cameras = make_cameras_from_configs(config.cameras)
        # Connect cameras
        for name, camera in self.cameras.items():
            camera.connect()
            if not camera.is_connected:
                raise ConnectionError(f"Failed to connect camera {name}")
            print(f"Camera {name} connected")
        self._is_connected = True
    
    @property
    def _motors_ft(self) -> dict[str, type]:
        EEF_KEYS = [
        	"X_r","Y_r","Z_r","RX_r","RY_r","RZ_r","Gripper_dis_r",
    		"X_l","Y_l","Z_l","RX_l","RY_l","RZ_l","Gripper_dis_l"
		]
        return {k: float for k in EEF_KEYS}

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {cam: (self.cameras[cam].height, self.cameras[cam].width, 3) for cam in self.cameras}

    @property
    def observation_features(self) -> dict:
        return {**self._motors_ft, **self._cameras_ft}

    @property
    def action_features(self) -> dict:
        return self._motors_ft
    
    @property
    def is_connected(self) -> bool:
        """Check if the robot is connected."""
        return self._is_connected
    
    @property
    def is_calibrated(self) -> bool:
        """Check if the robot is calibrated."""
        return self._is_calibrated

    def connect(self) -> None:
        # Connect piper motors
        self.arm.connect(enable=True)
        logger.info("Piper arm connected")

        # Connect cameras
        for name, camera in self.cameras.items():
            camera.connect()
            if not camera.is_connected:
                raise ConnectionError(f"Failed to connect camera {name}")
            logger.info(f"Camera {name} connected")
        
        self._is_connected = True
        logger.info("All devices connected")

    def calibrate(self) -> None:
        pass
        # """Calibrate the robot by moving to home position."""
        # if not self._is_connected:
        #     raise RobotDeviceNotConnectedError(
        #         "Piper is not connected. You need to run `robot.connect()`."
        #     )
        
        # logger.info("Starting Piper calibration")
        # self.arm.apply_calibration()
        # self._is_calibrated = True
        # logger.info("Piper calibration completed")

    def configure(self) -> None:
        pass
        # """Configure the robot with recommended settings."""
        # if not self._is_connected:
        #     raise RobotDeviceNotConnectedError(
        #         "Piper is not connected. You need to run `robot.connect()`."
        #     )
        
        # # Configure motors if needed
        # if hasattr(self.arm, 'configure_motors'):
        #     self.arm.configure_motors()
        
        # logger.info("Piper configuration completed")

    def disconnect(self) -> None:
        """Disconnect piper and cameras."""
        if not self._is_connected:
            return
        
        # Disconnect piper motors
        print("Piper arm disconnected after 3 seconds")
        self.sdk.disconnect()

        # Disconnect cameras
        for name, camera in self.cameras.items():
            camera.disconnect()
            print(f"Camera {name} disconnected")

        self._is_connected = False
        self._is_calibrated = False
        print("All devices disconnected")

    def get_observation(self) -> dict[str, torch.Tensor]:
        obs_dict = self.sdk.get_pos()

        for cam_key, cam in self.cameras.items():
            obs_dict[cam_key] = cam.async_read()
        return obs_dict

    ## Remain Fixed ##
    def send_action_joint(self, action: torch.Tensor) -> torch.Tensor:
        # Apply safety limits if configured
        a = action.detach().float().cpu().view(-1)
        a_np = a.numpy()
        positions = [
            a_np[0],
            a_np[1],
            a_np[2],
            a_np[3],
            a_np[4],
            a_np[5],
            a_np[6],
            a_np[7],
            a_np[8],
            a_np[9],
            a_np[10],
            a_np[11],
            a_np[12],
            a_np[13]
        ]
        self.sdk.set_joint_positions(positions)
        return action

    def send_action(self, action: torch.Tensor) -> torch.Tensor:
        # Apply safety limits if configured
        a = action.detach().float().cpu().view(-1)
        a_np = a.numpy()
        positions = [
            a_np[0],
            a_np[1],
            a_np[2],
            a_np[3],
            a_np[4],
            a_np[5],
            a_np[6],
            a_np[7],
            a_np[8],
            a_np[9],
            a_np[10],
            a_np[11],
            a_np[12],
            a_np[13]
        ]
        self.sdk.set_end_position(positions)
        return action

    def go_zero(self):
        self.sdk.go_zero()

    def __del__(self):
        """Cleanup when the object is destroyed."""
        if hasattr(self, '_is_connected') and self._is_connected:
            self.disconnect()
