#!/usr/bin/env python3
import casadi
import math
import numpy as np
import pinocchio as pin
from pinocchio import casadi as cpin
from transformations import quaternion_from_matrix
import os
import sys
from ament_index_python.packages import get_package_share_directory

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

os.environ['MKL_NUM_THREADS'] = '1'
os.environ['NUMEXPR_NUM_THREADS'] = '1'
os.environ['OMP_NUM_THREADS'] = '1'


def matrix_to_xyzrpy(matrix):
    x = matrix[0, 3]
    y = matrix[1, 3]
    z = matrix[2, 3]
    roll = math.atan2(matrix[2, 1], matrix[2, 2])
    pitch = math.asin(-matrix[2, 0])
    yaw = math.atan2(matrix[1, 0], matrix[0, 0])
    return [x, y, z, roll, pitch, yaw]


def create_transformation_matrix(x, y, z, roll, pitch, yaw):
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


class Arm_FK:
    def __init__(self, args):
        self.args = args

        np.set_printoptions(precision=5, suppress=True, linewidth=200)

        package_path = get_package_share_directory('piper_description')
        urdf_path = os.path.join(package_path, 'urdf', 'piper_description' + ('-lift.urdf' if args.lift else '.urdf'))

        self.robot = pin.RobotWrapper.BuildFromURDF(
            urdf_path,
            package_dirs=package_path
        )

        # self.robot = pin.RobotWrapper.BuildFromURDF(urdf_path)

        self.mixed_jointsToLockIDs = ["joint7",
                                      "joint8"
                                      ]

        self.reduced_robot = self.robot.buildReducedRobot(
            list_of_joints_to_lock=self.mixed_jointsToLockIDs,
            reference_configuration=np.array([0] * self.robot.model.nq),
        )

        # self.first_matrix = create_transformation_matrix(0, 0, 0, 0, -1.57, -1.57)
        self.first_matrix = create_transformation_matrix(0, 0, 0, 0, -1.57, 0)
        self.second_matrix = create_transformation_matrix(self.args.gripper_xyzrpy[0], self.args.gripper_xyzrpy[1], self.args.gripper_xyzrpy[2],
                                                          self.args.gripper_xyzrpy[3], self.args.gripper_xyzrpy[4], self.args.gripper_xyzrpy[5])
        self.last_matrix = np.dot(self.first_matrix, self.second_matrix)
        q = quaternion_from_matrix(self.last_matrix)
        self.reduced_robot.model.addFrame(
            pin.Frame('ee',
                      self.reduced_robot.model.getJointId('joint6'),
                      pin.SE3(
                          pin.Quaternion(q[3], q[0], q[1], q[2]),
                          np.array([self.last_matrix[0, 3], self.last_matrix[1, 3], self.last_matrix[2, 3]]),
                      ),
                      pin.FrameType.OP_FRAME)
        )

    def get_pose(self, q):
        index = 6 + (1 if self.args.lift else 0)
        pin.forwardKinematics(self.reduced_robot.model, self.reduced_robot.data, np.concatenate([q], axis=0))
        end_pose = create_transformation_matrix(self.reduced_robot.data.oMi[index].translation[0], self.reduced_robot.data.oMi[index].translation[1], self.reduced_robot.data.oMi[index].translation[2],
                                                math.atan2(self.reduced_robot.data.oMi[index].rotation[2, 1], self.reduced_robot.data.oMi[index].rotation[2, 2]),
                                                math.asin(-self.reduced_robot.data.oMi[index].rotation[2, 0]),
                                                math.atan2(self.reduced_robot.data.oMi[index].rotation[1, 0], self.reduced_robot.data.oMi[index].rotation[0, 0]))
        end_pose = np.dot(end_pose, self.last_matrix)
        return matrix_to_xyzrpy(end_pose)


class Arm_IK:
    def __init__(self, args):
        self.args = args
        np.set_printoptions(precision=5, suppress=True, linewidth=200)

        package_path = get_package_share_directory('piper_description')
        urdf_path = os.path.join(package_path, 'urdf', 'piper_description' + ('-lift.urdf' if args.lift else '.urdf'))

        self.robot = pin.RobotWrapper.BuildFromURDF(
            urdf_path,
            package_dirs=package_path,  # 添加包搜索路径
        )

        # self.robot = pin.RobotWrapper.BuildFromURDF(urdf_path)

        self.mixed_jointsToLockIDs = ["joint7",
                                      "joint8"
                                      ]

        self.reduced_robot = self.robot.buildReducedRobot(
            list_of_joints_to_lock=self.mixed_jointsToLockIDs,
            reference_configuration=np.array([0] * self.robot.model.nq),
        )

        # self.mixed_jointsToLockIDs = []
        # self.reduced_robot = self.robot

        # self.first_matrix = create_transformation_matrix(0, 0, 0, 0, -1.57, -1.57)
        self.first_matrix = create_transformation_matrix(0, 0, 0, 0, -1.57, 0)
        self.second_matrix = create_transformation_matrix(self.args.gripper_xyzrpy[0], self.args.gripper_xyzrpy[1], self.args.gripper_xyzrpy[2],
                                                          self.args.gripper_xyzrpy[3], self.args.gripper_xyzrpy[4], self.args.gripper_xyzrpy[5])
        self.last_matrix = np.dot(self.first_matrix, self.second_matrix)
        q = quaternion_from_matrix(self.last_matrix)
        self.reduced_robot.model.addFrame(
            pin.Frame('ee',
                      self.reduced_robot.model.getJointId('joint6'),
                      pin.SE3(
                          # pin.Quaternion(1, 0, 0, 0),
                          pin.Quaternion(q[3], q[0], q[1], q[2]),
                          np.array([self.last_matrix[0, 3], self.last_matrix[1, 3], self.last_matrix[2, 3]]),  # -y
                      ),
                      pin.FrameType.OP_FRAME)
        )

        self.geom_model = pin.buildGeomFromUrdf(self.robot.model, urdf_path, pin.GeometryType.COLLISION)
        for i in range(4, 10):
            for j in range(0, 3):
                self.geom_model.addCollisionPair(pin.CollisionPair(i, j))
        self.geometry_data = pin.GeometryData(self.geom_model)

        # print("self.robot.model.nq:", self.robot.model.nq)
        # for i, joint in enumerate(self.reduced_robot.model.joints):
        #     joint_name = self.reduced_robot.model.names[i]
        #     print(f"Joint {i}: {joint_name}, ID: {joint.id}")
        # for i in range(self.reduced_robot.model.nframes):
        #    frame = self.reduced_robot.model.frames[i]
        #    frame_id = self.reduced_robot.model.getFrameId(frame.name)
        #    print(f"Frame ID: {frame_id}, Name: {frame.name}")

        self.init_data = np.zeros(self.reduced_robot.model.nq)
        self.history_data = np.zeros(self.reduced_robot.model.nq)

        # Creating Casadi models and data for symbolic computing
        self.cmodel = cpin.Model(self.reduced_robot.model)
        self.cdata = self.cmodel.createData()

        # Creating symbolic variables
        self.cq = casadi.SX.sym("q", self.reduced_robot.model.nq, 1)
        self.cTf = casadi.SX.sym("tf", 4, 4)
        cpin.framesForwardKinematics(self.cmodel, self.cdata, self.cq)

        # # Get the hand joint ID and define the error function
        self.gripper_id = self.reduced_robot.model.getFrameId("ee")
        self.error = casadi.Function(
            "error",
            [self.cq, self.cTf],
            [
                casadi.vertcat(
                    cpin.log6(
                        self.cdata.oMf[self.gripper_id].inverse() * cpin.SE3(self.cTf)
                    ).vector,
                )
            ],
        )

        # Defining the optimization problem
        self.opti = casadi.Opti()
        self.var_q = self.opti.variable(self.reduced_robot.model.nq)
        # self.var_q_last = self.opti.parameter(self.reduced_robot.model.nq)   # for smooth
        self.param_tf = self.opti.parameter(4, 4)

        # self.totalcost = casadi.sumsqr(self.error(self.var_q, self.param_tf))
        # self.regularization = casadi.sumsqr(self.var_q)

        error_vec = self.error(self.var_q, self.param_tf)
        pos_error = error_vec[:3]  # 取前3个值为位置误差
        ori_error = error_vec[3:]  # 取后3个值为姿态误差
        # 设置位置和姿态的权重（可通过 args 配置）
        weight_position = getattr(self.args, 'pos_weight', 1.0)
        weight_orientation = getattr(self.args, 'ori_weight', 0.5)
        # 总成本函数
        self.totalcost = casadi.sumsqr(weight_position * pos_error) + casadi.sumsqr(weight_orientation * ori_error)
        # 正则化项
        self.regularization = casadi.sumsqr(self.var_q)

        # self.smooth_cost = casadi.sumsqr(self.var_q - self.var_q_last) # for smooth

        # Setting optimization constraints and goals
        self.opti.subject_to(self.opti.bounded(
            self.reduced_robot.model.lowerPositionLimit,
            self.var_q,
            self.reduced_robot.model.upperPositionLimit)
        )
        # print("self.reduced_robot.model.lowerPositionLimit:", self.reduced_robot.model.lowerPositionLimit)
        # print("self.reduced_robot.model.upperPositionLimit:", self.reduced_robot.model.upperPositionLimit)
        self.opti.minimize(20 * self.totalcost + 0.01 * self.regularization)
        # self.opti.minimize(20 * self.totalcost + 0.01 * self.regularization + 0.1 * self.smooth_cost) # for smooth

        opts = {
            'ipopt': {
                'print_level': 0,
                'max_iter': 600,
                'tol': 1e-8,
                'acceptable_tol': 1e-6,
                'acceptable_iter': 15,
                'mu_strategy': 'adaptive',
                'linear_solver': 'mumps',
                'hessian_approximation': 'limited-memory',
                'nlp_scaling_method': 'gradient-based',
                'bound_relax_factor': 0.0,
                'constr_viol_tol': 1e-6,
                'max_cpu_time': 1e3,
                'warm_start_init_point': 'yes',
            },
            'print_time': False
        }
        self.opti.solver("ipopt", opts)

    def ik_fun(self, target_pose, gripper=0, motorstate=None, motorV=None):
        gripper = np.array([gripper/2.0, -gripper/2.0])
        if motorstate is not None:
            self.init_data = motorstate
        self.opti.set_initial(self.var_q, self.init_data)

        # Visualization removed

        self.opti.set_value(self.param_tf, target_pose)
        # self.opti.set_value(self.var_q_last, self.init_data) # for smooth

        try:
            # 尝试有限求解，并依据状态判断成功与否
            sol = self.opti.solve_limited()
            stats = self.opti.stats() if hasattr(self.opti, 'stats') else {}
            return_status = stats.get('return_status', '')

            # 获取变量值（即使失败也可能有可用值，但标记为不成功）
            sol_q = self.opti.value(self.var_q)

            # 更新初始化数据以利于后续求解稳定
            if self.init_data is not None and sol_q is not None:
                max_diff = max(abs(self.history_data - sol_q))
                self.init_data = sol_q
                if max_diff > 30.0/180.0*3.1415:
                    self.init_data = np.zeros(self.reduced_robot.model.nq)
            else:
                self.init_data = sol_q

            self.history_data = sol_q if sol_q is not None else self.history_data

            # 前馈力矩估计（为保持接口一致，失败时也计算零速度）
            if motorV is not None:
                v = motorV * 0.0
            else:
                v = (sol_q - self.init_data) * 0.0 if sol_q is not None and self.init_data is not None else 0.0

            tau_ff = pin.rnea(self.reduced_robot.model, self.reduced_robot.data,
                              sol_q if sol_q is not None else np.zeros(self.reduced_robot.model.nq),
                              v,
                              np.zeros(self.reduced_robot.model.nv))

            # 碰撞检测与距离（失败时标记为不成功，不阻断函数）
            is_collision = False
            if sol_q is not None:
                is_collision = self.check_self_collision(sol_q, gripper)
                _ = self.get_dist(sol_q, target_pose[:3, 3])

            # 明确失败状态集合
            failed_status = {
                'Invalid_Number_Detected',
                'Infeasible_Problem_Detected',
                'Maximum_Iterations_Exceeded',
                'Error',
            }
            ok_flag = (return_status not in failed_status) and (sol_q is not None) and (not is_collision)
            return sol_q if sol_q is not None else None, tau_ff, ok_flag

        except Exception as e:
            # 收敛失败或数值异常，返回失败即可（减少冗余打印）
            return None, '', False

    def check_self_collision(self, q, gripper=np.array([0, 0])):
        pin.forwardKinematics(self.robot.model, self.robot.data, np.concatenate([q, gripper], axis=0))
        pin.updateGeometryPlacements(self.robot.model, self.robot.data, self.geom_model, self.geometry_data)
        collision = pin.computeCollisions(self.geom_model, self.geometry_data, False)
        # print("collision:", collision)
        return collision

    def get_dist(self, q, xyz):
        # Compute FK and measure distance at the EE pose (joint6 pose composed
        # with the fixed gripper offset 'last_matrix'), consistent with get_pose.
        index = 6 + (1 if self.args.lift else 0)
        pin.forwardKinematics(self.reduced_robot.model, self.reduced_robot.data, np.concatenate([q], axis=0))
        end_pose = create_transformation_matrix(
            self.reduced_robot.data.oMi[index].translation[0],
            self.reduced_robot.data.oMi[index].translation[1],
            self.reduced_robot.data.oMi[index].translation[2],
            math.atan2(self.reduced_robot.data.oMi[index].rotation[2, 1], self.reduced_robot.data.oMi[index].rotation[2, 2]),
            math.asin(-self.reduced_robot.data.oMi[index].rotation[2, 0]),
            math.atan2(self.reduced_robot.data.oMi[index].rotation[1, 0], self.reduced_robot.data.oMi[index].rotation[0, 0])
        )
        ee_pose = np.dot(end_pose, self.last_matrix)
        ee_pos = np.array([ee_pose[0, 3], ee_pose[1, 3], ee_pose[2, 3]])
        diff = ee_pos - np.array([xyz[0], xyz[1], xyz[2]])
        return float(np.linalg.norm(diff))

    def get_pose(self, q):
        index = 6 + (1 if self.args.lift else 0)
        pin.forwardKinematics(self.reduced_robot.model, self.reduced_robot.data, np.concatenate([q], axis=0))
        end_pose = create_transformation_matrix(self.reduced_robot.data.oMi[index].translation[0], self.reduced_robot.data.oMi[index].translation[1], self.reduced_robot.data.oMi[index].translation[2],
                                                math.atan2(self.reduced_robot.data.oMi[index].rotation[2, 1], self.reduced_robot.data.oMi[index].rotation[2, 2]),
                                                math.asin(-self.reduced_robot.data.oMi[index].rotation[2, 0]),
                                                math.atan2(self.reduced_robot.data.oMi[index].rotation[1, 0], self.reduced_robot.data.oMi[index].rotation[0, 0]))
        end_pose = np.dot(end_pose, self.last_matrix)
        return matrix_to_xyzrpy(end_pose)