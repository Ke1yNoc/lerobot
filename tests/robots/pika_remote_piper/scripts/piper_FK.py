#!/usr/bin/env python3
# piper_FK.py
# 功能：订阅机械臂关节状态（以及可选升降轴），计算并发布末端位姿（欧拉角与四元数两种形式）。
# 该节点是 teleop 流程中的“正解”发布者，供 teleop 与 IK 节点参考当前末端位姿。
import casadi
import meshcat.geometry as mg
import math
import numpy as np
import pinocchio as pin
from pinocchio import casadi as cpin
from pinocchio.visualize import MeshcatVisualizer
import os
import sys
import cv2
import rclpy
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import argparse
import threading
import rclpy
from rclpy.node import Node
import ast

# Add current script directory to Python path to find local modules
ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, ROOT_DIR)
sys.path.append(os.path.join(ROOT_DIR,'dist_pyarmor_FIK'))

from transformations import quaternion_from_euler, euler_from_quaternion, quaternion_from_matrix
from arm_FIK import Arm_FK

os.environ['MKL_NUM_THREADS'] = '1'
os.environ['NUMEXPR_NUM_THREADS'] = '1'
os.environ['OMP_NUM_THREADS'] = '1'
# 限制线程数以减少数值计算的抖动与资源占用（与 Pinocchio/Casadi 并存时更稳定）。


def matrix_to_xyzrpy(matrix):
    # 将 4x4 齐次变换矩阵转换为 [x, y, z, roll, pitch, yaw]
    # roll/pitch/yaw 按 ZYX 欧拉角顺序从旋转矩阵中解析。
    x = matrix[0, 3]
    y = matrix[1, 3]
    z = matrix[2, 3]
    roll = math.atan2(matrix[2, 1], matrix[2, 2])
    pitch = math.asin(-matrix[2, 0])
    yaw = math.atan2(matrix[1, 0], matrix[0, 0])
    return [x, y, z, roll, pitch, yaw]


def create_transformation_matrix(x, y, z, roll, pitch, yaw):
    # 根据位移 (x, y, z) 与欧拉角 (roll, pitch, yaw) 构造 4x4 齐次变换矩阵。
    transformation_matrix = np.eye(4)
    A = np.cos(yaw)
    B = np.sin(yaw)
    C = np.cos(pitch)
    D = np.sin(pitch)
    E = np.cos(roll)
    F = np.sin(roll)
    DE = D * E
    DF = D * F
    transformation_matrix[0, 0] = A * C
    transformation_matrix[0, 1] = A * DF - B * E
    transformation_matrix[0, 2] = B * F + A * DE
    transformation_matrix[0, 3] = x
    transformation_matrix[1, 0] = B * C
    transformation_matrix[1, 1] = A * E + B * DF
    transformation_matrix[1, 2] = B * DE - A * F
    transformation_matrix[1, 3] = y
    transformation_matrix[2, 0] = -D
    transformation_matrix[2, 1] = C * F
    transformation_matrix[2, 2] = C * E
    transformation_matrix[2, 3] = z
    transformation_matrix[3, 0] = 0
    transformation_matrix[3, 1] = 0
    transformation_matrix[3, 2] = 0
    transformation_matrix[3, 3] = 1
    return transformation_matrix


class RosOperator(Node):
    def __init__(self, args):
        super().__init__(f'piper_FK{args.index_name}')
        self.args = args
        self.arm_fk = Arm_FK(args)  # Arm_FK：基于 URDF 与工具偏置的前向运动学计算器
        self.lift_subscriber = None
        self.arm_end_pose_publisher = None
        self.arm_end_pose_orient_publisher = None
        self.arm_joint_state_subscriber = None
        self.arm_msg = None
        self.lift_msg = None
        self.calc_thread = None
        self.init_ros()  # 声明参数、建立订阅与发布、根据是否启用升降轴创建对应订阅

    def lift_callback(self, msg):
        # 升降轴状态回调（仅当 --lift 为真时订阅）
        self.lift_msg = msg

    def arm_joint_state_callback(self, msg):
        # 机械臂关节状态回调（joint_states_single{index}），取前 6 个关节用于末端位姿计算
        self.arm_msg = msg

    def start(self):
        # 启动计算线程，以固定频率发布末端位姿
        self.calc_thread = threading.Thread(target=self.calc)
        self.calc_thread.start()

    def calc(self):
        # 主循环：200Hz
        # - 若启用升降轴，需等待其消息与主臂关节消息；否则仅需主臂消息
        # - 通过 Arm_FK.get_pose 计算当前末端位姿（欧拉角形式）
        # - 发布两种形式的末端位姿：
        #   1) urdf_end_pose：欧拉角，orientation.w 复用为 gripper（第7关节）开合值
        #   2) urdf_end_pose_orient：四元数（从欧拉角转换）
        rate = self.create_rate(200)
        while rclpy.ok():
            if (self.args.lift and self.lift_msg is None) or self.arm_msg is None:
                rate.sleep()
                continue
            xyzrpy = self.arm_fk.get_pose((self.lift_msg.position if self.args.lift else []) + list(self.arm_msg.position[:6]))
            end_pose_msg = PoseStamped()
            end_pose_msg.header = Header()
            end_pose_msg.header.stamp = self.get_clock().now().to_msg()
            end_pose_msg.header.frame_id = "map"
            end_pose_msg.pose.position.x = xyzrpy[0]
            end_pose_msg.pose.position.y = xyzrpy[1]
            end_pose_msg.pose.position.z = xyzrpy[2]
            end_pose_msg.pose.orientation.x = xyzrpy[3]
            end_pose_msg.pose.orientation.y = xyzrpy[4]
            end_pose_msg.pose.orientation.z = xyzrpy[5]
            # 复用 orientation.w 作为夹爪开合量（第7关节），供欧拉角话题使用
            end_pose_msg.pose.orientation.w = self.arm_msg.position[6]
            self.arm_end_pose_publisher.publish(end_pose_msg)
            x, y, z, w = quaternion_from_euler(end_pose_msg.pose.orientation.x, end_pose_msg.pose.orientation.y, end_pose_msg.pose.orientation.z)
            end_pose_msg.pose.orientation.x = x
            end_pose_msg.pose.orientation.y = y
            end_pose_msg.pose.orientation.z = z
            end_pose_msg.pose.orientation.w = w
            # 四元数形式的末端位姿（orientation 为有效四元数）
            self.arm_end_pose_orient_publisher.publish(end_pose_msg)
            # print("end_pose:", xyzrpy)
            rate.sleep()

    def init_ros(self):
        # 声明参数：
        # - index_name：区分左右臂或单臂（例如 "", _l, _r）
        # - gripper_xyzrpy：工具坐标/夹爪相对第6关节的固定变换（与 Arm_FK 一致）
        self.declare_parameter('index_name', "")
        self.declare_parameter('gripper_xyzrpy', "[0.19, 0, 0, 0, 0, 0]")
        self.args.index_name = self.get_parameter('index_name').get_parameter_value().string_value
        self.args.gripper_xyzrpy = self.get_parameter('gripper_xyzrpy').get_parameter_value().string_value
        self.args.gripper_xyzrpy = ast.literal_eval(self.args.gripper_xyzrpy)
        # 订阅：升降轴（可选）与主臂关节状态
        if self.args.lift:
            self.lift_subscriber = self.create_subscription(JointState, f'/joint_states_single_lift', self.lift_callback, 1)
        self.arm_joint_state_subscriber = self.create_subscription(JointState, f'/joint_states_single{self.args.index_name}', self.arm_joint_state_callback, 1)
        # 发布：末端位姿（欧拉角话题与四元数话题）
        self.arm_end_pose_publisher = self.create_publisher(PoseStamped, f'/piper_FK{self.args.index_name}/urdf_end_pose', 1)
        self.arm_end_pose_orient_publisher = self.create_publisher(PoseStamped, f'/piper_FK{self.args.index_name}/urdf_end_pose_orient', 1)


def get_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument('--lift', action='store', type=bool, help='lift',
                        default=False, required=False)
    parser.add_argument('--index_name', action='store', type=str, help='index_name',
                        default="", required=False)
    parser.add_argument('--gripper_xyzrpy', action='store', nargs='+', type=float, help='gripper_xyzrpy',
                        default=[0.19, 0, 0, 0, 0, 0], required=False)
    # args = parser.parse_args()
    args, unknown = parser.parse_known_args()
    return args


def main():
    args = get_arguments()
    rclpy.init()
    ros_operator = RosOperator(args)
    ros_operator.start()
    rclpy.spin(ros_operator)


if __name__ == "__main__":
    main()
