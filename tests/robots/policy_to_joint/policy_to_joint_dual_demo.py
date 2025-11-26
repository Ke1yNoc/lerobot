#!/usr/bin/env python3
import argparse
import math
import numpy as np
import torch
import time

from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.robots.piper_dual import PiperConfigDual, PiperDual

from arm_FIK import Arm_FK, Arm_IK


def matrix_to_xyzrpy(matrix: np.ndarray):
    x = matrix[0, 3]
    y = matrix[1, 3]
    z = matrix[2, 3]
    roll = math.atan2(matrix[2, 1], matrix[2, 2])
    pitch = math.asin(-matrix[2, 0])
    yaw = math.atan2(matrix[1, 0], matrix[0, 0])
    return [x, y, z, roll, pitch, yaw]

def xyzrpy_to_str(xyzrpy):
    try:
        return (
            f"[{float(xyzrpy[0]):.3f}, {float(xyzrpy[1]):.3f}, {float(xyzrpy[2]):.3f}, "
            f"roll={float(xyzrpy[3]):.3f}, pitch={float(xyzrpy[4]):.3f}, yaw={float(xyzrpy[5]):.3f}]"
        )
    except Exception:
        return str(xyzrpy)


def create_transformation_matrix(x: float, y: float, z: float, roll: float, pitch: float, yaw: float):
    T = np.eye(4)
    A = math.cos(yaw)
    B = math.sin(yaw)
    C = math.cos(pitch)
    D = math.sin(pitch)
    E = math.cos(roll)
    F = math.sin(roll)
    DE = D * E
    DF = D * F
    T[0, 0] = A * C
    T[0, 1] = A * DF - B * E
    T[0, 2] = B * F + A * DE
    T[0, 3] = x
    T[1, 0] = B * C
    T[1, 1] = A * E + B * DF
    T[1, 2] = B * DE - A * F
    T[1, 3] = y
    T[2, 0] = -D
    T[2, 1] = C * F
    T[2, 2] = C * E
    T[2, 3] = z
    return T


def clamp_position_xyz(xyz, min_x=0.0, min_z=0.006, min_y=-0.3, max_y=0.3):
    clamped = list(xyz)
    changed = False
    if clamped[0] < min_x:
        clamped[0] = min_x
        changed = True
    if clamped[2] < min_z:
        clamped[2] = min_z
        changed = True
    if clamped[1] < min_y:
        clamped[1] = min_y
        changed = True
    if clamped[1] > max_y:
        clamped[1] = max_y
        changed = True
    return clamped, changed


def incremental_solve(arm_fk, arm_ik, current_joint, current_xyzrpy, delta_xyzrpy, step_trans, step_rot_deg,
                      safety):
    max_steps_trans = int(max(abs(v) for v in delta_xyzrpy[:3]) / step_trans) if step_trans > 0 else 0
    max_steps_rot = int(max(abs(v) for v in delta_xyzrpy[3:]) / math.radians(step_rot_deg)) if step_rot_deg > 0 else 0
    steps = max(1, max(max_steps_trans, max_steps_rot))
    chunks = [[delta_xyzrpy[i] / steps for i in range(6)] for _ in range(steps)]

    joint = np.array(current_joint, dtype=float)
    xyzrpy = list(current_xyzrpy)
    for ch in chunks:
        T_cur = create_transformation_matrix(*xyzrpy)
        T_delta = create_transformation_matrix(*ch)
        T_tar = T_cur.dot(T_delta)
        target = matrix_to_xyzrpy(T_tar)
        # safety clamp
        clamped_pos, changed = clamp_position_xyz(target[:3], safety['min_x'], safety['min_z'], safety['min_y'], safety['max_y'])
        if changed:
            target[:3] = clamped_pos
        # IK
        sol_q, _, ok = arm_ik.ik_fun(create_transformation_matrix(*target), gripper=0.0, motorstate=joint)
        if not ok or sol_q is None:
            return None, False
        joint = sol_q[:6]
        xyzrpy = arm_ik.get_pose(joint)
    return joint, True


def parse_args():
    p = argparse.ArgumentParser(description="Dual-arm delta→IK→send demo")
    p.add_argument('--delta_r', type=float, nargs=6, default=[0.0, 0.02, 0.0, 0.0, 0.0, 0.0], help='Right EE delta (xyzrpy)')
    p.add_argument('--delta_l', type=float, nargs=6, default=[0.0, 0.02, 0.0, 0.0, 0.0, 0.0], help='Left EE delta (xyzrpy)')
    p.add_argument('--gripper_r', type=float, default=0.5, help='Right gripper value')
    p.add_argument('--gripper_l', type=float, default=0.5, help='Left gripper value')
    p.add_argument('--step_trans', type=float, default=0.005, help='Incremental translation step (m)')
    p.add_argument('--step_rot_deg', type=float, default=2.0, help='Incremental rotation step (deg)')
    p.add_argument('--safety_min_x', type=float, default=0.0)
    p.add_argument('--safety_min_z', type=float, default=0.006)
    p.add_argument('--safety_min_y', type=float, default=-0.3)
    p.add_argument('--safety_max_y', type=float, default=0.3)
    # hardware
    p.add_argument('--can_r', type=str, default='can0')
    p.add_argument('--can_l', type=str, default='can1')
    p.add_argument('--gripper_serial_r', type=str, default='/dev/ttyUSB81')
    p.add_argument('--gripper_serial_l', type=str, default='/dev/ttyUSB82')
    p.add_argument('--cam_r', type=int, default=81)
    p.add_argument('--cam_l', type=int, default=82)
    p.add_argument('--use_obs', action='store_true', help='Use robot.get_observation() to fetch current joints', default=True)
    p.add_argument('--dry_run', action='store_true', help='Do not send commands, only print results', default=False)
    p.add_argument('--hold_s', type=float, default=1.0, help='Hold time after sending action (seconds)')
    return p.parse_args()


def main():
    args = parse_args()
    # build IK solvers (same model for both arms)
    class A:
        pass
    a_r = A(); a_r.use_orient = False; a_r.lift = False; a_r.gripper_xyzrpy = [0.19, 0, 0, 0, 0, 0]
    a_l = A(); a_l.use_orient = False; a_l.lift = False; a_l.gripper_xyzrpy = [0.19, 0, 0, 0, 0, 0]
    fk_r, fk_l = Arm_FK(a_r), Arm_FK(a_l)
    ik_r, ik_l = Arm_IK(a_r), Arm_IK(a_l)

    # connect dual robot first when reading observation as current
    camera_cfg_dual = {
        "pikaFisheyeCamera_r": OpenCVCameraConfig(index_or_path=args.cam_r, fps=30, width=640, height=480),
        "pikaFisheyeCamera_l": OpenCVCameraConfig(index_or_path=args.cam_l, fps=30, width=640, height=480),
    }
    cfg_dual = PiperConfigDual(
        can_interface_r=args.can_r,
        gripper_serial_r=args.gripper_serial_r,
        can_interface_l=args.can_l,
        gripper_serial_l=args.gripper_serial_l,
        cameras=camera_cfg_dual,
    )
    robot = PiperDual(cfg_dual)
    robot.connect()
    if args.use_obs:
        try:
            # Try multiple times to get fresh observation (some devices report zeros right after connect)
            for attempt in range(10):
                obs = robot.get_observation()
                curr_r = [float(obs.get(f"r_joint_{i}.pos", 0.0)) for i in range(6)]
                curr_l = [float(obs.get(f"l_joint_{i}.pos", 0.0)) for i in range(6)]
                if any(abs(v) > 1e-6 for v in curr_r + curr_l) or attempt == 9:
                    args.current_r = curr_r
                    args.current_l = curr_l
                    break
                time.sleep(0.2)
            args.gripper_r = float(obs.get("r_gripper.dis", args.gripper_r))
            args.gripper_l = float(obs.get("l_gripper.dis", args.gripper_l))
            print("Fetched current joints from observation (radian):")
            print(" current_r:", [f"{v:.3f}" for v in args.current_r])
            print(" current_l:", [f"{v:.3f}" for v in args.current_l])
        except Exception as e:
            print("Failed to fetch observation, fallback to provided currents:", e)

    # current EE pose
    ee_r = fk_r.get_pose(np.array(args.current_r, dtype=float))
    ee_l = fk_l.get_pose(np.array(args.current_l, dtype=float))
    print("Current EE pose (meters & radians):")
    print(" right:", xyzrpy_to_str(ee_r))
    print(" left:", xyzrpy_to_str(ee_l))
    # Safety clamps configuration
    safety = {
        'min_x': args.safety_min_x,
        'min_z': args.safety_min_z,
        'min_y': args.safety_min_y,
        'max_y': args.safety_max_y,
    }

    # Preset motion playlist (10 pairs of small xyz+rotation deltas, EE frame, meters/radians)
    BASE_DELTAS = [
        # 1: forward 10mm
        ([0.01, 0.00, 0.00, 0.0, 0.0, 0.0], [0.01, 0.00, 0.00, 0.0, 0.0, 0.0]),
        # 2: Y 方向 10mm（两臂相同）
        ([0.00, 0.01, 0.00, 0.0, 0.0, 0.0], [0.00, 0.01, 0.00, 0.0, 0.0, 0.0]),
        # 3: up 10mm
        ([0.00, 0.00, 0.01, 0.0, 0.0, 0.0], [0.00, 0.00, 0.01, 0.0, 0.0, 0.0]),
        # 4: small yaw +2° 与轻微平移（两臂相同）
        ([0.005, 0.005, 0.00, 0.0, 0.0, 0.0349], [0.005, 0.005, 0.00, 0.0, 0.0, 0.0349]),
        # 5: small roll +2° with Y
        ([0.00, 0.01, 0.00, 0.0349, 0.0, 0.0], [0.00, 0.01, 0.00, 0.0349, 0.0, 0.0]),
        # 6: small pitch +3° 与 X
        ([0.01, 0.00, 0.00, 0.0, 0.0524, 0.0], [0.01, 0.00, 0.00, 0.0, 0.0524, 0.0]),
        # 7: up+yaw +3° with slight Y
        ([0.00, 0.005, 0.005, 0.0, 0.0, 0.0524], [0.00, 0.005, 0.005, 0.0, 0.0, 0.0524]),
        # 8: roll +2° with diagonal move
        ([0.005, -0.005, 0.005, 0.0349, 0.0, 0.0], [0.005, -0.005, 0.005, 0.0349, 0.0, 0.0]),
        # 9: pitch +2° with forward+up
        ([0.01, 0.00, 0.005, 0.0, 0.0349, 0.0], [0.01, 0.00, 0.005, 0.0, 0.0349, 0.0]),
        # 10: combined small rotations +2°
        ([0.005, 0.005, 0.005, 0.0349, 0.0349, 0.0349], [0.005, 0.005, 0.005, 0.0349, 0.0349, 0.0349]),
    ]
    # Supersample: expand 10 base pairs into 100 pairs by subdividing each
    def _supersample_pairs(base_pairs, factor=10):
        out = []
        for dr, dl in base_pairs:
            dr_step = [v / float(factor) for v in dr]
            dl_step = [v / float(factor) for v in dl]
            for _ in range(factor):
                out.append((dr_step, dl_step))
        return out
    # Supersample identical left/right motions
    PRESET_DELTAS = _supersample_pairs(BASE_DELTAS, factor=10)
    REPEATS = 4
    print(f"Begin preset playlist: {len(PRESET_DELTAS)} motions, repeats={REPEATS}")
    for rep in range(REPEATS):
        print(f"Playlist repeat {rep+1}/{REPEATS}")
        for idx, (delta_r, delta_l) in enumerate(PRESET_DELTAS, start=1):
            # refresh currents (robust retries)
            if args.use_obs:
                for attempt in range(5):
                    o = robot.get_observation()
                    cr = [float(o.get(f"r_joint_{i}.pos", 0.0)) for i in range(6)]
                    cl = [float(o.get(f"l_joint_{i}.pos", 0.0)) for i in range(6)]
                    if any(abs(v) > 1e-6 for v in cr + cl) or attempt == 4:
                        args.current_r = cr
                        args.current_l = cl
                        break
                    time.sleep(0.2)
            # current EE pose
            ee_r = fk_r.get_pose(np.array(args.current_r, dtype=float))
            ee_l = fk_l.get_pose(np.array(args.current_l, dtype=float))
            print(f"Motion {idx}: current EE right={xyzrpy_to_str(ee_r)} left={xyzrpy_to_str(ee_l)}")
            # solve
            sol_r, ok_r = incremental_solve(fk_r, ik_r, args.current_r, ee_r, delta_r, args.step_trans, args.step_rot_deg, safety)
            sol_l, ok_l = incremental_solve(fk_l, ik_l, args.current_l, ee_l, delta_l, args.step_trans, args.step_rot_deg, safety)
            if not ok_r or not ok_l:
                print("IK failed in motion", idx)
                continue
            cmd = [float(v) for v in sol_r[:6]] + [float(args.gripper_r)] + [float(v) for v in sol_l[:6]] + [float(args.gripper_l)]
            if args.dry_run:
                print(f"Dry run — motion {idx} command:", cmd)
            else:
                robot.send_action(torch.tensor(cmd, dtype=torch.float32))
                print(f"Sent dual-arm command (motion {idx}):", cmd)
            time.sleep(max(0.0, float(args.hold_s)))
    robot.disconnect()
    return

    safety = {
        'min_x': args.safety_min_x,
        'min_z': args.safety_min_z,
        'min_y': args.safety_min_y,
        'max_y': args.safety_max_y,
    }

    # No single-shot: the playlist above covers the test run


if __name__ == "__main__":
    main()