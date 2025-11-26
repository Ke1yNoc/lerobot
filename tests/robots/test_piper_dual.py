#!/usr/bin/env python3
import argparse
import time
import torch

from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.robots.piper_dual import PiperConfigDual, PiperDual
import math

# ---- Fixed hardware config (edit here if your ports change) ----
CAN_R = "can0"
GRIPPER_R = "/dev/ttyUSB81"
CAM_R = 81  # single arm uses this too

CAN_L = "can1"
GRIPPER_L = "/dev/ttyUSB82"
CAM_L = 82

# Sleep interval between actions
DT = 3

def run_dual() -> None:
    camera_cfg_dual = {
        "cam_r": OpenCVCameraConfig(index_or_path=CAM_R, fps=30, width=640, height=480),
        "cam_l": OpenCVCameraConfig(index_or_path=CAM_L, fps=30, width=640, height=480),
    }
    cfg_dual = PiperConfigDual(
        can_interface_r=CAN_R,
        gripper_serial_r=GRIPPER_R,
        can_interface_l=CAN_L,
        gripper_serial_l=GRIPPER_L,
        cameras=camera_cfg_dual,
    )
    robot = PiperDual(cfg_dual)
    robot.connect()
    seq = [
        ## joint angle
        [0.0, 0.2, -0.4, 0.0, 0.6, 0, 0.96, 0.3, 0, -0.1, 0, 0, 0, 0.36],
        # [0, 0, -0.1, 0, 0, 0, 0.66, 0, 0, -0.1, 0, 0, 0, 0.66],
        # [-0.3, 0, -0.1, 0, 0, 0, 0.36, -0.3, 0, -0.1, 0, 0, 0, 0.96],
    ]
    try:
        for v in seq:
            robot.send_action(torch.tensor(v, dtype=torch.float32))
            time.sleep(DT)
        obs = robot.get_observation()
    finally:
        # robot.close()
        pass
    robot.disconnect()


def main():
    run_dual()


if __name__ == "__main__":
    main()
