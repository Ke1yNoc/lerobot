#!/usr/bin/env python3
import argparse
import time
import torch

from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.robots.piper import PiperConfig, Piper
from lerobot.robots.piper_dual import PiperConfigDual, PiperDual

# ---- Fixed hardware config (edit here if your ports change) ----
CAN_R = "can0"
GRIPPER_R = "/dev/ttyUSB81"
CAM_R = 81  # single arm uses this too

CAN_L = "can1"
GRIPPER_L = "/dev/ttyUSB82"
CAM_L = 82

# Sleep interval between actions
DT = 0.1


def run_single() -> None:
    camera_cfg = {
        "cam": OpenCVCameraConfig(index_or_path=CAM_R, fps=30, width=640, height=480),
    }
    cfg = PiperConfig(
        can_interface=CAN_R,
        gripper_serial=GRIPPER_R,
        cameras=camera_cfg,
    )
    robot = Piper(cfg)

    seq = [
        [58, 0, 255, 0, 85, 0, 10],
        [68, 0, 255, 0, 85, 0, 20],
        [78, 0, 255, 0, 85, 0, 30],
        [88, 0, 255, 0, 85, 0, 40],
        [78, 0, 255, 0, 85, 0, 50],
        [68, 0, 255, 0, 85, 0, 60],
        [58, 0, 255, 0, 85, 0, 70],
    ]
    try:
        for v in seq:
            robot.send_action(torch.tensor(v, dtype=torch.float32))
            time.sleep(DT)
        robot.go_zero()
        obs = robot.get_observation()
        print("Single observation keys:", list(obs.keys()))
    finally:
        # robot.close()  # uncomment if your SDK provides a close/shutdown
        pass


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

    seq = [
        # [R7 ..., L7 ...]
        [68, 0, 255, 0, 90, 0, 10,  78, 0, 255, 0, 90, 0, 30],
        [78, 0, 255, 0, 90, 0, 30,  68, 0, 255, 0, 90, 0, 10],
        [88, 0, 255, 0, 90, 0, 50,  58, 0, 255, 0, 90, 0, 50],
        [78, 0, 255, 0, 90, 0, 60,  68, 0, 255, 0, 90, 0, 60],
        [68, 0, 255, 0, 90, 0, 70,  78, 0, 255, 0, 90, 0, 70],
        [58, 0, 255, 0, 90, 0, 80,  88, 0, 255, 0, 90, 0, 80],
    ]
    try:
        for v in seq:
            robot.send_action(torch.tensor(v, dtype=torch.float32))
            time.sleep(DT)
        robot.go_zero()
        obs = robot.get_observation()
        print("Dual observation keys:", list(obs.keys()))
    finally:
        # robot.close()
        pass


def main():
    parser = argparse.ArgumentParser(description="Piper test runner (fixed hardware config)")
    parser.add_argument("--dual", type=lambda s: s.lower() in {"1", "true", "yes"}, default=False,
                        help="Run dual-arm test if True; single-arm otherwise.")
    args = parser.parse_args()

    if args.dual:
        run_dual()
    else:
        run_single()


if __name__ == "__main__":
    main()

