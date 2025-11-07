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
import math

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
    
    @property
    def _motors_ft(self) -> dict[str, type]:
        EEF_KEYS = [
        	"r_joint_0.pos","r_joint_1.pos","r_joint_2.pos","r_joint_3.pos","r_joint_4.pos","r_joint_5.pos","r_gripper.dis",
    		"l_joint_0.pos","l_joint_1.pos","l_joint_2.pos","l_joint_3.pos","l_joint_4.pos","l_joint_5.pos","l_gripper.dis"
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
        self.sdk.connect()
        print("Piper arm connected")

        # Connect cameras
        for name, camera in self.cameras.items():
            camera.connect()
            if not camera.is_connected:
                raise ConnectionError(f"Failed to connect camera {name}")
            print(f"Camera {name} connected")
        
        self._is_connected = True

    def calibrate(self) -> None:
        pass
        # """Calibrate the robot by moving to home position."""

    def configure(self) -> None:
        pass
        # """Configure the robot with recommended settings."""

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
        obs_dict = self.sdk.get_status()

        for cam_key, cam in self.cameras.items():
            obs_dict[cam_key] = cam.async_read()
        return obs_dict

    ## Remain Fixed ##
    def send_action(self, action: torch.Tensor) -> torch.Tensor:
        # Apply safety limits if configured
        a = action.detach().float().cpu().view(-1) # unit in rad and 100 mm
        a_np = a.numpy()
        joint_positions = [
            a_np[0],
            a_np[1],
            a_np[2],
            a_np[3],
            a_np[4],
            a_np[5],
            a_np[6], # Gripper distance 
            a_np[7],
            a_np[8],
            a_np[9],
            a_np[10],
            a_np[11],
            a_np[12],
            a_np[13] # Gripper distance
        ]
        self.sdk.set_joint_positions(joint_positions)
        return action

    def send_action_eef(self, action: torch.Tensor) -> torch.Tensor:
        # Apply safety limits if configured
        obs = self.get_observation()
        rad_2_deg = 180/math.pi
        positions = [
            obs["X_r"] + action["X_r"],
            obs["Y_r"] + action["Y_r"],
            obs["Z_r"] + action["Z_r"],
            (obs["RX_r"] + action["RX_r"])*rad_2_deg,
            (obs["RY_r"] + action["RY_r"])*rad_2_deg,
            (obs["RZ_r"] + action["RZ_r"])*rad_2_deg,
            (obs["Gripper_dis_r"] + action['Gripper_dis_r']) * 95.741,
            obs["X_l"] + action["X_l"],
            obs["Y_l"] + action["Y_l"],
            obs["Z_l"] + action["Z_l"],
            (obs["RX_l"] + action["RX_l"])*rad_2_deg,
            (obs["RY_l"] + action["RY_l"])*rad_2_deg,
            (obs["RZ_l"] + action["RZ_l"])*rad_2_deg,
            (obs["Gripper_dis_l"] + action['Gripper_dis_l']) * 95.741
        ]
        self.sdk.set_end_position(positions)
        return action

    def go_zero(self):
        self.sdk.go_zero()

    def __del__(self):
        """Cleanup when the object is destroyed."""
        if hasattr(self, '_is_connected') and self._is_connected:
            self.disconnect()
