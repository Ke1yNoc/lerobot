#!/usr/bin/env python3
"""
Simplified demo that drives Piper IK with a direct end-effector delta pose.

- Inputs: desired EE displacement (delta x, delta y, delta z, delta rx, delta ry, delta rz)
  in meters and radians. Delta can be applied in EE frame or world frame.
- Target pose is composed by:
    - EE frame: T_target = T_current * T_delta
- Supports both Euler mode and Quaternion mode for target orientation.
- Performs 30° threshold / 1° step linear interpolation printing, and consistency
  check with thresholds (position 0.3m, orientation 1 rad). Also simulates a
  1-second timeout for safety printing.

Notes:
- Gripper is ignored (set to 0).
- Uses Arm_IK from local policy_to_joint/arm_FIK.py.
- Requires piper_description to be available for URDF loading.
"""

import os
import sys
import math
import numpy as np
import argparse

from transformations import quaternion_from_euler
from arm_FIK import Arm_IK, Arm_FK


def matrix_to_xyzrpy(matrix: np.ndarray):
    x = matrix[0, 3]
    y = matrix[1, 3]
    z = matrix[2, 3]
    roll = math.atan2(matrix[2, 1], matrix[2, 2])
    pitch = math.asin(-matrix[2, 0])
    yaw = math.atan2(matrix[1, 0], matrix[0, 0])
    return [x, y, z, roll, pitch, yaw]


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
    T[3, 0] = 0
    T[3, 1] = 0
    T[3, 2] = 0
    T[3, 3] = 1
    return T


class Args:
    """Minimal args stub compatible with Arm_IK initialization."""
    def __init__(self):
        # Match defaults used in pika_remote_piper config
        self.index_name = ""
        self.use_orient = False  # default: Euler mode; can toggle to True for quaternion mode
        self.lift = False        # ignore lift axis for this demo
        # Default gripper xyzrpy as in config/piper_rand_params.yaml
        self.gripper_xyzrpy = [0.19, 0.0, 0.0, 0.0, 0.0, 0.0]
        # IK weights
        self.pos_weight = 1.0
        self.ori_weight = 0.5


def compose_target_by_delta(T_current: np.ndarray,
                            delta_xyzrpy: list,
                            frame: str = 'ee') -> np.ndarray:
    """Compose target pose from current pose and delta transform.

    frame='ee'  -> T_target = T_current * T_delta (delta expressed in EE/local frame)
    frame='world' -> T_target = T_delta * T_current (delta expressed in world/global frame)
    """
    T_delta = create_transformation_matrix(*delta_xyzrpy)
    if frame == 'world':
        return T_delta.dot(T_current)
    else:
        return T_current.dot(T_delta)


def incremental_compose_and_solve(arm_fk, arm_ik, current_joint_angles, current_xyzrpy, delta_xyzrpy, frame, step_trans=0.005, step_rot_deg=2.0):
    """Split delta into small chunks and solve IK step by step for robustness."""
    # Build small step target deltas
    max_steps_trans = int(max(abs(v) for v in delta_xyzrpy[:3]) / step_trans) if step_trans > 0 else 0
    max_steps_rot = int(max(abs(v) for v in delta_xyzrpy[3:]) / math.radians(step_rot_deg)) if step_rot_deg > 0 else 0
    steps = max(1, max(max_steps_trans, max_steps_rot))
    # linear split
    chunks = [
        [delta_xyzrpy[i] / steps for i in range(6)]
        for _ in range(steps)
    ]

    joint = np.array(current_joint_angles, dtype=float)
    xyzrpy = list(current_xyzrpy)
    for i, ch in enumerate(chunks, start=1):
        T_cur = create_transformation_matrix(*xyzrpy)
        T_tar = compose_target_by_delta(T_cur, ch, frame)
        target_xyzrpy = matrix_to_xyzrpy(T_tar)
        qx, qy, qz, qw = quaternion_from_euler(target_xyzrpy[3], target_xyzrpy[4], target_xyzrpy[5])
        import pinocchio as pin
        target_se3 = pin.SE3(pin.Quaternion(qw, qx, qy, qz), np.array(target_xyzrpy[:3]))
        sol_q, tau_ff, ok = arm_ik.ik_fun(target_se3.homogeneous, gripper=0.0, motorstate=joint)
        if not ok or sol_q is None:
            return None, False
        joint = sol_q[:6]
        xyzrpy = arm_ik.get_pose(joint)
    return joint, True


def print_interpolated_sequence(last_angles: np.ndarray, target_angles: np.ndarray):
    """Simulate linear interpolation when max diff > threshold_deg, using step_deg."""
    max_diff = float(np.max(np.abs(last_angles - target_angles)))
    threshold_deg = CONFIG['interp_threshold_deg']
    step_deg = CONFIG['interp_step_deg']
    if max_diff > math.radians(threshold_deg):
        steps = int(max_diff / math.radians(step_deg))
        seq = np.linspace(last_angles, target_angles, steps + 1)
        print(f"Interpolation triggered: max_diff={math.degrees(max_diff):.3f} deg, steps={steps}")
        for i in range(1, len(seq)):
            print("Interp angles (deg)", [f"{math.degrees(v):.3f}" for v in seq[i]])
    else:
        print(f"No interpolation (diff <= {threshold_deg}deg)")


def check_consistency(xyzrpy_fk: np.ndarray, xyzrpy_target: np.ndarray):
    dx = abs(xyzrpy_fk[0] - xyzrpy_target[0])
    dy = abs(xyzrpy_fk[1] - xyzrpy_target[1])
    dz = abs(xyzrpy_fk[2] - xyzrpy_target[2])
    droll = abs(xyzrpy_fk[3] - xyzrpy_target[3])
    dpitch = abs(xyzrpy_fk[4] - xyzrpy_target[4])
    dyaw = abs(xyzrpy_fk[5] - xyzrpy_target[5])
    pos_thresh = CONFIG['pos_thresh_m']
    ori_thresh = CONFIG['ori_thresh_rad']
    over_limit = (dx > pos_thresh) or (dy > pos_thresh) or (dz > pos_thresh) or (droll > ori_thresh) or (dpitch > ori_thresh) or (dyaw > ori_thresh)
    print(f"Consistency diff: pos=({dx:.3f},{dy:.3f},{dz:.3f}), ori=({droll:.3f},{dpitch:.3f},{dyaw:.3f})")
    print("over_limit:", over_limit)
    return not over_limit


def parse_args():
    parser = argparse.ArgumentParser(description="Policy-to-joint teleop IK demo")
    parser.add_argument('--use_orient', action='store_true', help='Use quaternion mode for target orientation')
    parser.add_argument('--current', type=float, nargs=6, metavar=('j1','j2','j3','j4','j5','j6'),
                        help='Current joint angles (rad) for 6 joints', default=[0.0, 0.3, -0.2, 0.1, -0.1, 0.0])
    parser.add_argument('--delta', type=float, nargs=6, metavar=('dx','dy','dz','drx','dry','drz'),
                        help='Desired EE displacement (xyzrpy) in meters/radians',
                        default=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    parser.add_argument('--delta_frame', choices=['ee'], default='ee',
                        help='Frame in which delta is expressed: ee (local)')
    parser.add_argument('--step_trans', type=float, default=0.005,
                        help='Incremental solve translation step size in meters (default 0.005)')
    parser.add_argument('--step_rot_deg', type=float, default=2.0,
                        help='Incremental solve rotation step size in degrees (default 2.0)')
    parser.add_argument('--pos_weight', type=float, default=1.0, help='IK position error weight')
    parser.add_argument('--ori_weight', type=float, default=0.5, help='IK orientation error weight')
    parser.add_argument('--pos_thresh', type=float, default=0.3, help='Position consistency threshold (m)')
    parser.add_argument('--ori_thresh', type=float, default=1.0, help='Orientation consistency threshold (rad)')
    parser.add_argument('--interp_threshold_deg', type=float, default=30.0, help='Interpolation trigger threshold (deg)')
    parser.add_argument('--interp_step_deg', type=float, default=1.0, help='Interpolation step size (deg)')
    return parser.parse_args()


CONFIG = {
    'pos_thresh_m': 0.3,
    'ori_thresh_rad': 1.0,
    'interp_threshold_deg': 30.0,
    'interp_step_deg': 1.0,
}


def apply_config_from_args(args_ns):
    CONFIG['pos_thresh_m'] = args_ns.pos_thresh
    CONFIG['ori_thresh_rad'] = args_ns.ori_thresh
    CONFIG['interp_threshold_deg'] = args_ns.interp_threshold_deg
    CONFIG['interp_step_deg'] = args_ns.interp_step_deg


def main():
    args_cli = parse_args()
    apply_config_from_args(args_cli)
    # Mode selection: Euler (default) or Quaternion
    use_quaternion_mode = bool(args_cli.use_orient)
    # Desired EE displacement (xyzrpy)
    delta_xyzrpy = args_cli.delta
    delta_frame = args_cli.delta_frame

    # Current joint angles feedback (simulate real-time robot state)
    # 6 joints when lift disabled; adjust as needed.
    current_joint_angles = np.array(args_cli.current, dtype=float)

    # Build FK to compute current arm end pose from joint angles
    args = Args()
    args.use_orient = use_quaternion_mode
    args.pos_weight = float(args_cli.pos_weight)
    args.ori_weight = float(args_cli.ori_weight)
    arm_fk = Arm_FK(args)
    arm_end_xyzrpy = arm_fk.get_pose(current_joint_angles)
    print("Current FK EE xyzrpy:", [f"{v:.4f}" for v in arm_end_xyzrpy])

    # Build current EE matrix and compose target using delta
    T_arm_end = create_transformation_matrix(*arm_end_xyzrpy)
    T_target = compose_target_by_delta(T_arm_end, delta_xyzrpy, frame=delta_frame)

    # Prepare Arm_IK (same args as FK)
    arm_ik = Arm_IK(args)

    # Construct target SE3
    import pinocchio as pin
    target_xyzrpy = matrix_to_xyzrpy(T_target)
    print("Delta input (xyzrpy):", [f"{v:.4f}" for v in delta_xyzrpy], "frame=", delta_frame)
    print("Target EE xyzrpy:", [f"{v:.4f}" for v in target_xyzrpy])
    if use_quaternion_mode:
        # Build quaternion directly (here we still derive from Euler for demo)
        qx, qy, qz, qw = quaternion_from_euler(target_xyzrpy[3], target_xyzrpy[4], target_xyzrpy[5])
        target_se3 = pin.SE3(pin.Quaternion(qw, qx, qy, qz), np.array(target_xyzrpy[:3]))
    else:
        # Euler mode path (same as piper_IK.py when use_orient=False)
        qx, qy, qz, qw = quaternion_from_euler(target_xyzrpy[3], target_xyzrpy[4], target_xyzrpy[5])
        target_se3 = pin.SE3(pin.Quaternion(qw, qx, qy, qz), np.array(target_xyzrpy[:3]))

    # Simulate 1-second timeout check (demo prints only)
    print("Safety check: ctrl_end_pose timestamp fresh: True (demo)")

    # Call IK (gripper ignored -> set 0)
    # 先尝试分步求解（更稳健）
    sol_step, ok_step = incremental_compose_and_solve(
        arm_fk, arm_ik, current_joint_angles, arm_end_xyzrpy,
        delta_xyzrpy, delta_frame,
        step_trans=float(args_cli.step_trans), step_rot_deg=float(args_cli.step_rot_deg)
    )
    if ok_step and sol_step is not None:
        joint_angles = sol_step[:6]
        print("Solved joint angles (rad):", [f"{v:.6f}" for v in joint_angles])
        print("Solved joint angles (deg):", [f"{math.degrees(v):.3f}" for v in joint_angles])
        last_angles = current_joint_angles
        print_interpolated_sequence(last_angles, joint_angles)
        xyzrpy_fk = np.array(arm_ik.get_pose(joint_angles))
        is_consistent = check_consistency(xyzrpy_fk, np.array(target_xyzrpy))
        if not is_consistent:
            print("Result rejected by consistency thresholds (over_limit)")
        dist = arm_ik.get_dist(joint_angles, np.array(target_xyzrpy[:3]))
        print(f"FK position error to target: {dist:.4f} m")
        return


    # 再次回退：降低姿态权重、减小步长后重试分步（ee 帧）
    print("IK retry: reduce ori_weight and smaller steps for incremental solve")
    args.ori_weight = max(0.2, args.ori_weight * 0.5)
    arm_ik = Arm_IK(args)
    sol_step_small, ok_step_small = incremental_compose_and_solve(
        arm_fk, arm_ik, current_joint_angles, arm_end_xyzrpy,
        delta_xyzrpy, delta_frame,
        step_trans=float(args_cli.step_trans) / 2.0,
        step_rot_deg=float(args_cli.step_rot_deg) / 2.0
    )
    if ok_step_small and sol_step_small is not None:
        joint_angles = sol_step_small[:6]
        print("Solved joint angles (rad):", [f"{v:.6f}" for v in joint_angles])
        print("Solved joint angles (deg):", [f"{math.degrees(v):.3f}" for v in joint_angles])
        last_angles = current_joint_angles
        print_interpolated_sequence(last_angles, joint_angles)
        xyzrpy_fk = np.array(arm_ik.get_pose(joint_angles))
        is_consistent = check_consistency(xyzrpy_fk, np.array(target_xyzrpy))
        if not is_consistent:
            print("Result rejected by consistency thresholds (over_limit)")
        dist = arm_ik.get_dist(joint_angles, np.array(target_xyzrpy[:3]))
        print(f"FK position error to target: {dist:.4f} m")
        return

    # 若分步仍失败，则用一次性目标求解并回退降低姿态权重
    sol_q, tau_ff, ok = arm_ik.ik_fun(target_se3.homogeneous, gripper=0.0, motorstate=current_joint_angles)
    if not ok:
        print("IK retry: lowering orientation weight and retry once")
        args.ori_weight = max(0.2, args.ori_weight * 0.5)
        arm_ik = Arm_IK(args)
        sol_q, tau_ff, ok = arm_ik.ik_fun(target_se3.homogeneous, gripper=0.0, motorstate=current_joint_angles)
    if ok and sol_q is not None:
        # First 6 joints when lift disabled
        joint_angles = sol_q[:6]
        print("Solved joint angles (rad):", [f"{v:.6f}" for v in joint_angles])
        print("Solved joint angles (deg):", [f"{math.degrees(v):.3f}" for v in joint_angles])

        # Interpolation simulation relative to current joint angles feedback
        last_angles = current_joint_angles
        print_interpolated_sequence(last_angles, joint_angles)

        # Consistency check (position 0.3m, orientation 1rad)
        xyzrpy_fk = np.array(arm_ik.get_pose(joint_angles))
        is_consistent = check_consistency(xyzrpy_fk, np.array(target_xyzrpy))
        if not is_consistent:
            print("Result rejected by consistency thresholds (over_limit)")

        # FK position error for reference
        dist = arm_ik.get_dist(joint_angles, np.array(target_xyzrpy[:3]))
        print(f"FK position error to target: {dist:.4f} m")
    else:
        print("IK failed or collision detected. No solution.")


if __name__ == "__main__":
    main()
